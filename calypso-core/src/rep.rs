//! Rep detection and per-rep trace computation.
//!
//! Platform-independent and host-testable: the firmware feeds [`Detector`] one
//! sample at a time, and when a rep closes it slices the sample ring for the
//! reported span (plus [`MARGIN_MS`] each side) and calls [`compute_rep`].
//!
//! See `docs/rep-tracking/README.md` §3.

use crate::sampler::Sample;

/// Linear-acceleration magnitude (m/s²) that counts as "moving".
pub const ACCEL_ON_THRESHOLD: f32 = 0.60;
/// Magnitude below which the sensor counts as at rest (hysteresis below
/// [`ACCEL_ON_THRESHOLD`] so a rep doesn't chatter on/off near the threshold).
pub const ACCEL_REST_THRESHOLD: f32 = 0.25;
/// How long motion must persist before a rep starts.
pub const T_ON_MS: u32 = 100;
/// How long rest must persist before a rep closes.
pub const T_REST_MS: u32 = 250;
/// Shortest motion span still counted as a rep (noise/movement blips).
pub const MIN_REP_MS: u32 = 400;
/// Longest motion span that can be a rep; beyond this it's a set-up
/// adjustment or an unrack, and the span is discarded.
pub const MAX_REP_MS: u32 = 8000;
/// Rest samples kept either side of the motion span so ZUPT has an at-rest
/// window to anchor to, and the app can plot the rep's endpoints.
pub const MARGIN_MS: u32 = 200;

/// Detector state (exposed for tests/logging).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Idle,
    Moving,
    Settle,
}

/// The motion span of a detected rep (milliseconds). Add [`MARGIN_MS`] at
/// each end to get the trace window.
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
/// IDLE ──|a| > ON for T_ON──► MOVING ──|a| < REST for T_REST──► emit span, IDLE
/// ```
pub struct Detector {
    state: State,
    /// Time |a| first exceeded the on-threshold (candidate motion start).
    above_since: Option<u32>,
    /// Time |a| first dropped below the rest-threshold.
    below_since: Option<u32>,
    /// Motion start of the rep in progress.
    start_ms: u32,
}

impl Detector {
    pub const fn new() -> Self {
        Detector {
            state: State::Idle,
            above_since: None,
            below_since: None,
            start_ms: 0,
        }
    }

    pub fn state(&self) -> State {
        self.state
    }

    /// Feed one sample; returns the span when a rep closes.
    pub fn update(&mut self, s: &Sample) -> Option<RepSpan> {
        let mag = accel_magnitude(s);
        match self.state {
            State::Idle => {
                if mag > ACCEL_ON_THRESHOLD {
                    let since = *self.above_since.get_or_insert(s.t_ms);
                    if s.t_ms.saturating_sub(since) >= T_ON_MS {
                        self.start_ms = since;
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
                        // ZUPT anchor: the sensor is at rest here, so the
                        // velocity integration is anchored to zero at this
                        // end of the rep (`compute_rep`).
                        let span = RepSpan {
                            start_ms: self.start_ms,
                            end_ms: since,
                        };
                        self.state = State::Idle;
                        self.below_since = None;
                        self.above_since = None;
                        if span.duration_ms() >= MIN_REP_MS && span.duration_ms() <= MAX_REP_MS {
                            return Some(span);
                        }
                        return None;
                    }
                } else {
                    self.below_since = None;
                }
                None
            }
        }
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

/// Rotate a body-frame vector into the world frame by a unit quaternion
/// `[w, x, y, z]`: `v' = q ⊗ v ⊗ q*`.
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
///
/// These are the two angles that matter for bar tilt; yaw is deliberately not
/// reported (it drifts without a magnetometer).
pub fn roll_pitch_deg(q: [f32; 4]) -> (f32, f32) {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let roll = atan2f(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let pitch_arg = clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    let pitch = asinf(pitch_arg);
    (roll.to_degrees(), pitch.to_degrees())
}

/// One point of a stored rep trace.
///
/// The raw `accel`/`quat` are kept alongside the computed `pos` so the app can
/// re-derive traces with better algorithms later without a firmware update.
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

/// Trace output buffer over caller-provided storage (no allocation, and the
/// storage can live on the heap where a whole trace is too big for the stack).
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

    fn push(&mut self, p: TracePoint) {
        if self.len < self.points.len() {
            self.points[self.len] = p;
            self.len += 1;
        }
    }

    fn clear(&mut self) {
        self.len = 0;
    }
}

/// Summary statistics for one rep. Angles are degrees, distances metres,
/// velocities m/s.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RepSummary {
    pub t_start_ms: u32,
    pub t_end_ms: u32,
    pub duration_ms: u32,
    /// Range of motion: peak-to-peak displacement along the dominant axis.
    pub rom: f32,
    pub peak_velocity: f32,
    pub mean_velocity: f32,
    pub peak_accel: f32,
    pub roll_min_deg: f32,
    pub roll_max_deg: f32,
    pub pitch_min_deg: f32,
    pub pitch_max_deg: f32,
    /// Number of trace points stored (0 if the input or buffer was empty).
    pub trace_len: usize,
    /// True if the input didn't fit `out` and was decimated uniformly.
    pub decimated: bool,
}

/// Compute the ZUPT-anchored world-frame trace and summary for one rep.
///
/// `samples` should be the detector's span plus margin at each end (the rest
/// samples let the integration be anchored at zero velocity at both ends).
/// Any number of samples is accepted; if more than `N` are supplied the trace
/// is decimated uniformly to fit.
///
/// Integration: rotate each sample's linear acceleration into the world frame
/// by its quaternion, integrate to velocity and position by trapezoid, then
/// remove the linear drift in velocity and position so both start and end at
/// rest (ZUPT anchoring). This rejects constant bias exactly and bounds the
/// remaining position error to within one rep, which is what bar-path
/// plotting needs.
///
/// Runs in three passes with running scalars only (no scratch buffers), so it
/// needs no allocation on the device.
pub fn compute_rep(samples: &[Sample], out: &mut TraceBuf<'_>) -> RepSummary {
    out.clear();
    let mut summary = RepSummary {
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
    summary.decimated = capacity > 0 && stride > 1;

    let t0 = samples[0].t_ms;
    let t_end = samples[n - 1].t_ms;
    let total_dt = (t_end - t0) as f32 / 1000.0;
    let detrend = total_dt > 0.0;

    // Pass 1: world-frame accel → raw velocity; keep v_end and summary stats.
    let mut v_raw = [0.0f32; 3];
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

    // Pass 2: detrended velocity → raw position; keep p_end.
    let mut p_raw = [0.0f32; 3];
    let mut v_prev = detrended_velocity(samples, 0, v_end, t0, total_dt, detrend);
    let mut v_max = 0.0f32;
    for i in 1..n {
        let v = detrended_velocity(samples, i, v_end, t0, total_dt, detrend);
        let dt = (samples[i].t_ms - samples[i - 1].t_ms) as f32 / 1000.0;
        for k in 0..3 {
            p_raw[k] += 0.5 * (v_prev[k] + v[k]) * dt;
            if absf(v[k]) > v_max {
                v_max = absf(v[k]);
            }
        }
        v_prev = v;
    }
    let p_end = p_raw;

    // Pass 3: detrended position, emitting trace points and angle/ROM stats.
    let mut p = [0.0f32; 3];
    v_prev = detrended_velocity(samples, 0, v_end, t0, total_dt, detrend);
    let mut lo = [f32::MAX; 3];
    let mut hi = [f32::MIN; 3];
    for i in 0..n {
        let v = detrended_velocity(samples, i, v_end, t0, total_dt, detrend);
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
        if i % stride == 0 || i == n - 1 {
            out.push(TracePoint {
                t_ms: samples[i].t_ms,
                pos,
                accel: samples[i].accel,
                quat: samples[i].quat,
            });
        }
    }

    let range = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    summary.rom = if range[0] >= range[1] && range[0] >= range[2] {
        range[0]
    } else if range[1] >= range[2] {
        range[1]
    } else {
        range[2]
    };
    summary.t_start_ms = t0;
    summary.t_end_ms = t_end;
    summary.duration_ms = t_end - t0;
    summary.peak_accel = a_max;
    summary.peak_velocity = v_max;
    summary.mean_velocity = v_sum / n as f32;
    summary.trace_len = out.len();
    summary
}

/// Velocity at sample `i`, detrended so it starts and ends at zero.
fn detrended_velocity(
    samples: &[Sample],
    i: usize,
    v_end: [f32; 3],
    t0: u32,
    total_dt: f32,
    detrend: bool,
) -> [f32; 3] {
    // Re-integrate up to `i` (cheap: a rep is a few hundred samples).
    let mut v = [0.0f32; 3];
    let mut prev_a = rotate_by_quat(samples[0].quat_f32(), samples[0].accel_f32());
    for j in 1..=i {
        let a = rotate_by_quat(samples[j].quat_f32(), samples[j].accel_f32());
        let dt = (samples[j].t_ms - samples[j - 1].t_ms) as f32 / 1000.0;
        for k in 0..3 {
            v[k] += 0.5 * (prev_a[k] + a[k]) * dt;
        }
        prev_a = a;
    }
    if detrend {
        let f = (samples[i].t_ms - t0) as f32 / 1000.0 / total_dt;
        for k in 0..3 {
            v[k] -= v_end[k] * f;
        }
    }
    v
}

// --- small no_std helpers -------------------------------------------------
//
// `calypso-core` builds for a `no_std` target, so `f32`'s std-only methods
// are replaced by `libm`; the host test build uses the same code paths.

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
