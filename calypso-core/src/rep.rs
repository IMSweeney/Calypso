//! Rep detection and per-rep trace computation.

use crate::pb::msg;
use crate::sampler::Sample;

/// Linear-acceleration magnitude (m/s²) that counts as "moving".
pub const ACCEL_ON_THRESHOLD: f32 = 0.60;
/// Magnitude below which the sensor counts as at rest (hysteresis below [`ACCEL_ON_THRESHOLD`] so a rep doesn't chatter on/off near the threshold).
pub const ACCEL_REST_THRESHOLD: f32 = 0.25;
/// How long motion must persist before a rep starts.
pub const T_ON_MS: u32 = 100;
/// How long rest must persist before a rep closes.
pub const T_REST_MS: u32 = 250;
/// Shortest motion span still counted as a rep (noise/movement blips).
pub const MIN_REP_MS: u32 = 400;
/// Longest motion span that can be a rep; beyond this it's a set-up adjustment or an unrack, and the span is discarded.
pub const MAX_REP_MS: u32 = 8000;
/// Rest samples kept either side of the motion span so ZUPT has an at-rest window to anchor to, and the app can plot the rep's endpoints.
pub const MARGIN_MS: u32 = 200;
/// Sample times and velocities the detector keeps, so a rep can name the bar's velocity at its
/// *left edge* — `MARGIN_MS` before the motion, not the zero a detector that only knows "it is
/// moving now" would assume. The edge is that far behind motion being declared, plus headroom.
const VEL_HISTORY: usize = 48;
/// Smallest dominant-axis travel (metres) that counts as a rep. A hand on a rack, a bar being unracked, a bump: each crosses the motion thresholds, none of them moves the sensor far enough to be a rep.
pub const MIN_ROM_M: f32 = 0.10;

/// Speed (m/s) the bar must reach between reversals for a return to zero to be a reversal rather than noise.
pub const VEL_MIN: f32 = 0.15;
/// How long the bar's velocity must stay near zero for it to count as a reversal.
pub const T_TURN_MS: u32 = 40;
/// Time constant of the accelerometer's bias estimate. Longer than the slowest rep, so the rep is not taken out with the bias.
const BIAS_TAU_S: f32 = 3.0;
/// Longest integration step honoured, so a stalled sampler cannot turn a gap in the samples into a jump in velocity.
const MAX_DT_S: f32 = 0.1;
/// Time constant over which a velocity is pulled back to zero while the detector is idle and the bar is still — the velocity form of "it is not moving, so it is not going anywhere".
const V_TAU_S: f32 = 0.5;

/// Detector state (exposed for tests/logging).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Idle,
    Moving,
    Settle,
}

/// The motion span of a detected rep (milliseconds). Add [`MARGIN_MS`] at each end to get the trace window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RepSpan {
    pub start_ms: u32,
    pub end_ms: u32,
}

impl RepSpan {
    pub fn duration_ms(&self) -> u32 {
        self.end_ms.saturating_sub(self.start_ms)
    }
}

/// Motion/rest state machine:
///
/// ```text
/// IDLE ──|a| > ON for T_ON──► MOVING ──┬── |a| < REST for T_REST────────────► emit span, IDLE
///                                     └── second reversal of the bar ────────► emit span, IDLE
/// ```
///
/// **A rep is out and back, so it ends where the bar turns around, and a pause is only one of the ways that happens.** Stillness alone is not enough to go by: a set nobody pauses through has no stillness in it, and a detector that waits for one merges the whole set into a single span — which then exceeds [`MAX_REP_MS`] and is thrown away, taking every rep in it. The reversals are found from the velocity, which is zero at the bottom and at the top of every rep whether the bar dwells there or not.
pub struct Detector {
    state: State,
    /// Time |a| first exceeded the on-threshold (candidate motion start).
    above_since: Option<u32>,
    /// Time |a| first dropped below the rest-threshold.
    below_since: Option<u32>,
    /// Motion start of the rep in progress.
    start_ms: u32,
    /// Velocity of the bar, m/s: the acceleration integrated with its bias taken out (see [`Detector::integrate`]).
    v: [f32; 3],
    /// Slow estimate of the acceleration's bias, which is what would otherwise integrate into a velocity that never comes back to zero.
    bias: [f32; 3],
    /// Time the bar's velocity first fell to nothing, as a candidate reversal.
    still_since: Option<u32>,
    /// Whether the bar has moved since the last reversal. A return to zero only counts once it has.
    moved: bool,
    /// Reversals seen in this rep: out is one, back is two, and the second ends it.
    turns: u8,
    /// Timestamp of the previous sample, for the integration step.
    prev_t_ms: u32,
    /// Recent sample times and velocities, for [`Detector::velocity_at`].
    hist_t: [u32; VEL_HISTORY],
    hist_v: [[f32; 3]; VEL_HISTORY],
    hist_head: usize,
    /// The bar's velocity at the rep's left edge (`start_ms - MARGIN_MS`), captured as it began.
    v0: [f32; 3],
}

impl Detector {
    pub const fn new() -> Self {
        Detector {
            state: State::Idle,
            above_since: None,
            below_since: None,
            start_ms: 0,
            v: [0.0; 3],
            bias: [0.0; 3],
            still_since: None,
            moved: false,
            turns: 0,
            prev_t_ms: 0,
            hist_t: [0; VEL_HISTORY],
            hist_v: [[0.0; 3]; VEL_HISTORY],
            hist_head: 0,
            v0: [0.0; 3],
        }
    }

    pub fn state(&self) -> State {
        self.state
    }

    /// The bar's velocity at the left edge of the rep that just closed, captured when it began.
    pub fn rep_v0(&self) -> [f32; 3] {
        self.v0
    }

    /// Feed one sample; returns the span when a rep closes.
    pub fn update(&mut self, s: &Sample) -> Option<RepSpan> {
        let mag = accel_magnitude(s);
        self.advance(s, mag);
        match self.state {
            State::Idle => {
                if mag > ACCEL_ON_THRESHOLD {
                    let since = *self.above_since.get_or_insert(s.t_ms);
                    if s.t_ms.saturating_sub(since) >= T_ON_MS {
                        self.start_ms = since;
                        self.v0 = self.velocity_at(since.saturating_sub(MARGIN_MS));
                        self.state = State::Moving;
                        self.above_since = None;
                        self.below_since = None;
                    }
                } else {
                    self.above_since = None;
                }
                None
            }
            State::Moving | State::Settle => {
                if mag < ACCEL_REST_THRESHOLD {
                    let since = *self.below_since.get_or_insert(s.t_ms);
                    if s.t_ms.saturating_sub(since) >= T_REST_MS {
                        // ZUPT anchor: the sensor is at rest here, so the velocity integration is anchored to zero at this end of the rep (`compute_rep`).
                        let span = RepSpan {
                            start_ms: self.start_ms,
                            end_ms: since,
                        };
                        self.state = State::Idle;
                        self.below_since = None;
                        self.above_since = None;
                        self.turns = 0;
                        self.moved = false;
                        self.still_since = None;
                        if span.duration_ms() >= MIN_REP_MS && span.duration_ms() <= MAX_REP_MS {
                            return Some(span);
                        }
                        return None;
                    }
                } else {
                    self.below_since = None;
                }
                // The other way a rep ends: the bar has gone out and come back. Its velocity is zero at the top as surely as in a pause, and waiting for the pause is what merges a whole set into one span.
                let speed =
                    sqrtf(self.v[0] * self.v[0] + self.v[1] * self.v[1] + self.v[2] * self.v[2]);
                if speed > VEL_MIN {
                    self.moved = true;
                    self.still_since = None;
                } else if self.moved {
                    let since = *self.still_since.get_or_insert(s.t_ms);
                    if s.t_ms.saturating_sub(since) >= T_TURN_MS {
                        self.turns += 1;
                        self.moved = false;
                        self.still_since = None;
                        if self.turns >= 2 {
                            let span = RepSpan {
                                start_ms: self.start_ms,
                                end_ms: since,
                            };
                            self.state = State::Idle;
                            self.below_since = None;
                            self.above_since = None;
                            self.turns = 0;
                            if span.duration_ms() >= MIN_REP_MS && span.duration_ms() <= MAX_REP_MS
                            {
                                return Some(span);
                            }
                            return None;
                        }
                    }
                }
                None
            }
        }
    }

    /// Advance the velocity by this sample's acceleration, with the accelerometer's bias taken out.
    ///
    /// A biased acceleration integrated over a rep is a velocity that never returns to zero, so the reversals this detector looks for would never arrive. Both corrections need the bar to be *still*, which is why neither runs while it is moving: a bias learned from motion is the rep, taken out of itself, and a velocity damped during a rep is the same mistake one step later — the set stops splitting the moment either one runs between two reps of a set nobody pauses through.
    fn advance(&mut self, s: &Sample, mag: f32) {
        let dt = self.step_ms(s);
        if dt <= 0.0 {
            return;
        }
        let a = s.accel_f32();
        let quiet = mag < ACCEL_REST_THRESHOLD;
        for k in 0..3 {
            self.v[k] += (a[k] - self.bias[k]) * dt;
            if quiet {
                self.bias[k] += (a[k] - self.bias[k]) * (dt / BIAS_TAU_S);
                if self.state == State::Idle {
                    self.v[k] += (0.0 - self.v[k]) * (dt / V_TAU_S);
                }
            }
        }
        self.hist_t[self.hist_head] = s.t_ms;
        self.hist_v[self.hist_head] = self.v;
        self.hist_head = (self.hist_head + 1) % VEL_HISTORY;
    }

    /// The velocity at the newest sample at or before `t_ms`, or zero if it predates the ring
    /// (the session's start, where the bar was still).
    fn velocity_at(&self, t_ms: u32) -> [f32; 3] {
        let mut newest = None;
        for i in 0..VEL_HISTORY {
            let t = self.hist_t[i];
            if t <= t_ms && newest.map_or(true, |(bt, _)| t > bt) {
                newest = Some((t, self.hist_v[i]));
            }
        }
        newest.map_or([0.0; 3], |(_, v)| v)
    }

    /// Seconds since the previous sample, bounded so a stall in the sampler cannot become a jump in velocity.
    fn step_ms(&mut self, s: &Sample) -> f32 {
        let dt = (s.t_ms.saturating_sub(self.prev_t_ms) as f32 / 1000.0).min(MAX_DT_S);
        self.prev_t_ms = s.t_ms;
        dt
    }
}

impl Default for Detector {
    fn default() -> Self {
        Self::new()
    }
}

/// Linear-acceleration magnitude (m/s²) of a sample.
pub fn accel_magnitude(s: &Sample) -> f32 {
    let a = s.accel_f32();
    sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
}

/// Rotate a body-frame vector into the world frame by a unit quaternion `[w, x, y, z]`: `v' = q ⊗ v ⊗ q*`.
pub fn rotate_by_quat(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    // t = 2 * (q.xyz × v)
    let tx = 2.0 * (y * v[2] - z * v[1]);
    let ty = 2.0 * (z * v[0] - x * v[2]);
    let tz = 2.0 * (x * v[1] - y * v[0]);
    // v + w*t + (q.xyz × t)
    [
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    ]
}

/// Roll and pitch (degrees) from a unit quaternion `[w, x, y, z]`.
/// Yaw is deliberately not reported: it drifts without a magnetometer.
pub fn roll_pitch_deg(q: [f32; 4]) -> (f32, f32) {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let roll = atan2f(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let pitch_arg = clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    let pitch = asinf(pitch_arg);
    (roll.to_degrees(), pitch.to_degrees())
}

/// One point of a stored rep trace.
/// The raw `accel`/`quat` are kept alongside the computed `pos` so the app can re-derive traces with better algorithms later without a firmware update.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TracePoint {
    pub t_ms: u32,
    /// ZUPT-anchored displacement from the rep start (m).
    pub pos: [f32; 3],
    /// Raw linear acceleration (fixed point, `sampler::ACCEL_SCALE` LSB/m/s²).
    pub accel: [i16; 3],
    /// Raw orientation quaternion `[w, x, y, z]` (`sampler::QUAT_SCALE` LSB).
    pub quat: [i16; 4],
}

impl TracePoint {
    pub const ZERO: TracePoint = TracePoint {
        t_ms: 0,
        pos: [0.0; 3],
        accel: [0; 3],
        quat: [0; 4],
    };
}

/// Trace output buffer over caller-provided storage (no allocation, and the storage can live on the heap where a whole trace is too big for the stack).
pub struct TraceBuf<'a> {
    points: &'a mut [TracePoint],
    len: usize,
}

impl<'a> TraceBuf<'a> {
    pub fn new(points: &'a mut [TracePoint]) -> Self {
        TraceBuf { points, len: 0 }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn capacity(&self) -> usize {
        self.points.len()
    }

    pub fn get(&self, i: usize) -> Option<&TracePoint> {
        if i < self.len {
            self.points.get(i)
        } else {
            None
        }
    }

    pub fn iter(&self) -> core::slice::Iter<'_, TracePoint> {
        self.points[..self.len].iter()
    }

    /// The stored points, oldest first.
    pub fn as_slice(&self) -> &[TracePoint] {
        &self.points[..self.len]
    }

    /// Append a point; silently ignored once the buffer is full.
    pub fn push(&mut self, p: TracePoint) {
        if self.len < self.points.len() {
            self.points[self.len] = p;
            self.len += 1;
        }
    }

    fn clear(&mut self) {
        self.len = 0;
    }
}

/// [`msg::RepSummary::flags`] bit 0: the trace was decimated to fit device memory.
pub const REP_FLAG_DECIMATED: u32 = 1 << 0;
/// [`msg::RepSummary::flags`] bit 1: the accelerometer was calibrated as well as the gyroscope when the rep was recorded.
///
/// The recording gate asks only for the gyroscope, so this is the bit that says the rest of the calibration was there.
pub const REP_FLAG_CALIBRATED: u32 = 1 << 1;

impl msg::RepSummary {
    /// Whether this is a rep at all: the bar has to have gone up or down [`MIN_ROM_M`].
    ///
    /// The detector closes on motion and rest, which a vibration or a hand on the rack satisfies; what it cannot see is how far anything moved, because that costs two integrations. This is the check that costs nothing after the fact — and it is measured on the vertical, the only axis with a right answer.
    pub fn is_a_rep(&self) -> bool {
        self.rom >= MIN_ROM_M
    }

    pub const fn decimated(&self) -> bool {
        self.flags & REP_FLAG_DECIMATED != 0
    }

    pub const fn calibrated(&self) -> bool {
        self.flags & REP_FLAG_CALIBRATED != 0
    }
}

/// Compute the ZUPT-anchored world-frame trace and summary for one rep.
///
/// `samples` should be the detector's span plus margin at each end, and `v0` the bar's velocity
/// at the first sample (the detector's [`Detector::rep_v0`]), so the integration starts from the
/// true velocity rather than a forced zero. Any number of samples is accepted; if more than `N`
/// are supplied the trace is decimated uniformly to fit.
///
/// Each sample's linear acceleration is rotated into the world frame by its quaternion, integrated to velocity and position by trapezoid, then the linear drift is removed so the rep ends at rest.
/// That rejects constant bias exactly and bounds the remaining position error to within one rep, which is what bar-path plotting needs.
pub fn compute_rep(samples: &[Sample], v0: [f32; 3], out: &mut TraceBuf<'_>) -> msg::RepSummary {
    out.clear();
    let mut summary = msg::RepSummary {
        roll_min_deg: f32::MAX,
        roll_max_deg: f32::MIN,
        pitch_min_deg: f32::MAX,
        pitch_max_deg: f32::MIN,
        ..Default::default()
    };
    let n = samples.len();
    if n == 0 {
        summary.roll_min_deg = 0.0;
        summary.roll_max_deg = 0.0;
        summary.pitch_min_deg = 0.0;
        summary.pitch_max_deg = 0.0;
        return summary;
    }

    let capacity = out.capacity();
    let stride = if capacity == 0 {
        n
    } else {
        n.div_ceil(capacity).max(1)
    };
    if capacity > 0 && stride > 1 {
        summary.flags |= REP_FLAG_DECIMATED;
    }
    // Anchor the decimation grid at the *last* sample, so the rep's end is always stored (it is the most interesting point when plotting) and the count stays within capacity.
    let first = if capacity == 0 { 0 } else { (n - 1) % stride };

    let t0 = samples[0].t_ms;
    let t_end = samples[n - 1].t_ms;
    let total_dt = (t_end - t0) as f32 / 1000.0;
    let detrend = total_dt > 0.0;

    // Pass 1: world-frame accel → raw velocity, for the endpoint the detrend takes out and for the summary stats.
    let mut v_raw = v0;
    let mut prev_a = rotate_by_quat(samples[0].quat_f32(), samples[0].accel_f32());
    let mut a_max = 0.0f32;
    let mut v_sum = 0.0f32;
    for i in 0..n {
        let a = rotate_by_quat(samples[i].quat_f32(), samples[i].accel_f32());
        if i > 0 {
            let dt = (samples[i].t_ms - samples[i - 1].t_ms) as f32 / 1000.0;
            for k in 0..3 {
                v_raw[k] += 0.5 * (prev_a[k] + a[k]) * dt;
            }
        }
        prev_a = a;
        let amag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        if amag > a_max {
            a_max = amag;
        }
        v_sum += sqrtf(v_raw[0] * v_raw[0] + v_raw[1] * v_raw[1] + v_raw[2] * v_raw[2]);
    }
    let v_end = v_raw;

    // Pass 2: detrended velocity → the position the detrend ends at.
    let mut p_end = [0.0f32; 3];
    let mut v_max = 0.0f32;
    let mut walk = Velocity::new(samples, v_end, t0, total_dt, detrend, v0);
    let mut v_prev = [0.0f32; 3];
    for i in 0..n {
        let v = walk.step();
        if i > 0 {
            let dt = (samples[i].t_ms - samples[i - 1].t_ms) as f32 / 1000.0;
            for k in 0..3 {
                p_end[k] += 0.5 * (v_prev[k] + v[k]) * dt;
                if absf(v[k]) > v_max {
                    v_max = absf(v[k]);
                }
            }
        }
        v_prev = v;
    }

    // Pass 3: detrended position, emitting trace points and angle/ROM stats.
    let mut p = [0.0f32; 3];
    let mut roll_offset = 0.0f32;
    let mut roll_prev = 0.0f32;
    let mut roll_seen = false;
    let mut walk = Velocity::new(samples, v_end, t0, total_dt, detrend, v0);
    let mut v_prev = [0.0f32; 3];
    let mut lo = [f32::MAX; 3];
    let mut hi = [f32::MIN; 3];
    for i in 0..n {
        let v = walk.step();
        if i > 0 {
            let dt = (samples[i].t_ms - samples[i - 1].t_ms) as f32 / 1000.0;
            for k in 0..3 {
                p[k] += 0.5 * (v_prev[k] + v[k]) * dt;
            }
        }
        v_prev = v;
        let f = if detrend {
            (samples[i].t_ms - t0) as f32 / 1000.0 / total_dt
        } else {
            0.0
        };
        let mut pos = p;
        for k in 0..3 {
            pos[k] -= p_end[k] * f;
            if pos[k] < lo[k] {
                lo[k] = pos[k];
            }
            if pos[k] > hi[k] {
                hi[k] = pos[k];
            }
        }
        let (roll, pitch) = roll_pitch_deg(samples[i].quat_f32());
        // Roll is reported in (-180, 180], so a rep that crosses the seam reads as a full turn between its minimum and
        // its maximum. Unwrapping it first is what makes the range a rotation rather than an artefact of the branch cut.
        if roll_seen {
            let step = roll - roll_prev;
            if step > 180.0 {
                roll_offset -= 360.0;
            } else if step < -180.0 {
                roll_offset += 360.0;
            }
        }
        roll_seen = true;
        roll_prev = roll;
        let roll = roll + roll_offset;
        if roll < summary.roll_min_deg {
            summary.roll_min_deg = roll;
        }
        if roll > summary.roll_max_deg {
            summary.roll_max_deg = roll;
        }
        if pitch < summary.pitch_min_deg {
            summary.pitch_min_deg = pitch;
        }
        if pitch > summary.pitch_max_deg {
            summary.pitch_max_deg = pitch;
        }
        if i >= first && (i - first) % stride == 0 {
            out.push(TracePoint {
                t_ms: samples[i].t_ms,
                pos,
                accel: samples[i].accel,
                quat: samples[i].quat,
            });
        }
    }

    // Range of motion is the vertical travel, and Z is the only axis that can say what vertical is: the fusion pins it
    // to gravity, while X and Y are the mount's heading and drift with yaw. Whichever axis moved furthest is not the
    // same question — on the horizontal axes it is as often the estimate's error as the bar.
    summary.rom = hi[2] - lo[2];
    summary.t_start_ms = t0;
    summary.t_end_ms = t_end;
    summary.duration_ms = t_end - t0;
    summary.peak_accel = a_max;
    summary.peak_velocity = v_max;
    summary.mean_velocity = v_sum / n as f32;
    summary.trace_len = out.len() as u32;
    // The summary started here, so it carries the velocity the integration was seeded from: the app can tell a rep
    // that started mid-set from one that started at rest, and can re-derive the trace from the same origin.
    summary.set_v0(msg::TraceOrigin {
        vx: v0[0],
        vy: v0[1],
        vz: v0[2],
    });
    summary
}

/// The rep's velocity, integrated one sample at a time.
///
/// A pass over the rep costs one integration step per point, which is what keeps [`compute_rep`] linear: it needs the velocity at every point, and re-integrating from the start for each one is quadratic in the length of a rep.
struct Velocity<'a> {
    samples: &'a [Sample],
    /// Index of the next sample to read.
    i: usize,
    /// Raw velocity, before the endpoint drift is taken out.
    v: [f32; 3],
    prev_a: [f32; 3],
    /// Raw velocity at the last sample: taking it out proportionally is what makes both ends read zero.
    v_end: [f32; 3],
    t0: u32,
    total_dt: f32,
    detrend: bool,
}

impl<'a> Velocity<'a> {
    fn new(
        samples: &'a [Sample],
        v_end: [f32; 3],
        t0: u32,
        total_dt: f32,
        detrend: bool,
        v0: [f32; 3],
    ) -> Self {
        Velocity {
            samples,
            i: 0,
            v: v0,
            prev_a: rotate_by_quat(samples[0].quat_f32(), samples[0].accel_f32()),
            v_end,
            t0,
            total_dt,
            detrend,
        }
    }

    /// World-frame velocity at the next sample, oldest first.
    fn step(&mut self) -> [f32; 3] {
        let i = self.i;
        self.i += 1;
        let s = &self.samples[i];
        let a = rotate_by_quat(s.quat_f32(), s.accel_f32());
        if i > 0 {
            let dt = (s.t_ms - self.samples[i - 1].t_ms) as f32 / 1000.0;
            let (v, prev_a) = (self.v, self.prev_a);
            self.v = trapezoid(v, prev_a, a, dt);
        }
        self.prev_a = a;
        if self.detrend {
            let f = (s.t_ms - self.t0) as f32 / 1000.0 / self.total_dt;
            let (v, v_end) = (self.v, self.v_end);
            core::array::from_fn(|k| v[k] - v_end[k] * f)
        } else {
            self.v
        }
    }
}

// --- small no_std helpers -------------------------------------------------

/// One trapezoid step of the integration: `v + ½(a + b)·Δt`, on every axis.
fn trapezoid(v: [f32; 3], a: [f32; 3], b: [f32; 3], dt: f32) -> [f32; 3] {
    core::array::from_fn(|k| v[k] + 0.5 * (a[k] + b[k]) * dt)
}

fn sqrtf(x: f32) -> f32 {
    libm::sqrtf(x)
}

fn atan2f(y: f32, x: f32) -> f32 {
    libm::atan2f(y, x)
}

fn asinf(x: f32) -> f32 {
    libm::asinf(x)
}

fn absf(x: f32) -> f32 {
    if x < 0.0 {
        -x
    } else {
        x
    }
}

fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}
