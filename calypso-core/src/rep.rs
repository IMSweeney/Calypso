//! Rep detection and per-rep trace computation.

use crate::fusion::{magnitude, roll_pitch_deg, Filtered, Fusion, FusionState};
use crate::pb::msg;
use crate::sampler::{quantize, Sample, QUAT_SCALE};

/// Linear-acceleration magnitude (m/s²) that counts as "moving".
pub const ACCEL_ON_THRESHOLD: f32 = 0.20;
/// How long motion must persist before a rep starts.
pub const T_ON_MS: u32 = 50;
/// Shortest motion span still counted as a rep (noise/movement blips).
pub const MIN_REP_MS: u32 = 400;
/// Longest motion span that can be a rep; beyond this it's a set-up adjustment or an unrack, and the span is discarded.
pub const MAX_REP_MS: u32 = 8000;
/// Rest kept either side of the motion span, so the filter has an at-rest window to anchor the trace
/// to and the app can plot the rep's endpoints.
///
/// Half a second, because the filter only calls a bar at rest after it has held still for 400 ms
/// (a stroke's own shoulder reads the same for a third of a second). A shorter margin leaves the
/// trace with no anchor at its ends, and the position drifts with nothing to pull it back.
pub const MARGIN_MS: u32 = 500;
/// Sample times, filter states and heights the detector keeps, so a leg can name the point it began
/// at after the fact: at most `MARGIN_MS` behind a turn, or `T_ON_MS` behind a motion onset, plus
/// headroom.
const HIST: usize = 64;
/// Smallest dominant-axis travel (metres) that counts as a rep. A hand on a rack, a bar being unracked, a bump: each crosses the motion thresholds, none of them moves the sensor far enough to be a rep.
pub const MIN_ROM_M: f32 = 0.07;

/// Speed (m/s) the bar must reach between reversals for a return to zero to be a reversal rather than noise.
pub const VEL_MIN: f32 = 0.07;
/// How long the bar's velocity must stay near zero for it to count as a reversal.
pub const T_TURN_MS: u32 = 20;
/// Longest integration step honoured, so a stalled sampler cannot turn a gap in the samples into a jump in velocity.
const MAX_DT_S: f32 = 0.1;
/// Time constant over which the detector's velocity is pulled back to zero while it is idle and the
/// bar is still — the velocity form of "it is not moving, so it is not going anywhere".
///
/// The filter has its own, and deliberately: the detector goes on integrating the acceleration
/// itself so that a change in what the device stores can be attributed to the detection or to the
/// filter, and the two velocities disagreeing is the measurement.
const V_TAU_S: f32 = 0.5;

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

/// A leg: the bar's travel between two turns, and what the bar carried into it.
#[derive(Debug, Clone, Copy)]
struct Leg {
    start_ms: u32,
    end_ms: u32,
    /// Signed vertical displacement over the leg.
    disp_z: f32,
    /// Whether the bar came to rest before this leg moved — a hold, not a grind.
    had_rest: bool,
    /// The filter state one [`MARGIN_MS`] before the leg started, and the moment it was taken.
    state: (u32, FusionState),
}

/// Turning-point detector: **a rep is an out-and-back**.
///
/// Integrate the bar's velocity and watch for the moments it stalls. A stall is what the bottom
/// and the top of a rep are, whether the bar dwells there or not, so a set nobody pauses through
/// splits at each end just as a slow one does. The signed vertical travel between two consecutive
/// stalls is a *leg*, and a rep is two consecutive legs in opposite directions, each longer than
/// [`MIN_ROM_M`].
///
/// Asking the question in this order puts the travel check inside detection rather than after it:
/// a stroke too short to be half a rep can never pair with anything, so a bump, a re-rack and a
/// hand on the rack are rejected by the same measurement that accepts a press. A one-way push tall
/// enough to be a leg is dropped when the bar rests and then sets off the same way again — that is
/// an unrack, not a rep.
///
/// **What it cannot see.** A leg ends on a turn, and a turn is the speed falling below
/// [`VEL_MIN`]. A stroke that never reaches that speed never turns, so a very gentle return — below
/// 0.15 m/s at its peak — does not close a rep here, though a detector that keys on acceleration
/// could still catch it. Motion must also clear [`ACCEL_ON_THRESHOLD`] for [`T_ON_MS`] to open a
/// leg at all.
///
/// **Everything here is in the world frame**, because the filter supplies one worth the name: tilt
/// comes from the gyro, and the accelerometer is believed only while the bar is still. The
/// detector still integrates the acceleration into its own velocity rather than taking the
/// filter's, so that the two can be compared on a real session before either is trusted.
///
/// [`Detector::rep_state`] is the filter's state where the stored window begins, which is what
/// [`compute_rep`] replays from.
pub struct Detector {
    /// World-frame velocity, integrated here rather than taken from the filter.
    v: [f32; 3],
    /// Timestamp of the previous sample, for the integration step.
    prev_t_ms: u32,
    /// The previous step's vertical velocity, for the trapezoid.
    prev_vz: f32,
    /// Vertical displacement accumulated since the last rebase; only differences are ever used.
    pos_z: f32,
    /// The open leg: where it began, the height it began at, and the state the bar carried into it.
    leg_start: Option<u32>,
    leg_pos0_z: f32,
    leg_state: (u32, FusionState),
    leg_had_rest: bool,
    /// Whether the bar has moved since the leg opened. A turn only counts once it has.
    moved: bool,
    /// When the velocity first fell below [`VEL_MIN`], as a candidate turn.
    still_since: Option<u32>,
    /// When the acceleration first rose above [`ACCEL_ON_THRESHOLD`], as a candidate motion onset.
    above_since: Option<u32>,
    /// Whether a rest has been seen and not yet claimed by a leg.
    gap_rest: bool,
    /// The last completed leg not yet paired into a rep.
    pending: Option<Leg>,
    /// Recent sample times, filter states and heights, for naming a leg's start after the fact.
    hist_t: [u32; HIST],
    hist_state: [FusionState; HIST],
    hist_z: [f32; HIST],
    hist_head: usize,
    /// How much of the history has been written, so a lookup cannot read a slot that is still zero.
    hist_len: usize,
    /// The filter state at the left edge of the rep that just closed, and the moment it was taken.
    state: (u32, FusionState),
    /// The first sample the detector was ever fed, which is as far back as its history goes.
    first_t_ms: Option<u32>,
    /// Legs completed since the detector started. A leg is one stroke between two turns, and a rep is two of them, so this is the count that says whether detection got past its first stage.
    legs: u32,
}

impl Detector {
    pub const fn new() -> Self {
        Detector {
            v: [0.0; 3],
            prev_t_ms: 0,
            prev_vz: 0.0,
            pos_z: 0.0,
            leg_start: None,
            leg_pos0_z: 0.0,
            leg_state: (0, FusionState::LEVEL),
            leg_had_rest: false,
            moved: false,
            still_since: None,
            above_since: None,
            gap_rest: false,
            pending: None,
            hist_t: [0; HIST],
            hist_state: [FusionState::LEVEL; HIST],
            hist_z: [0.0; HIST],
            hist_head: 0,
            hist_len: 0,
            state: (0, FusionState::LEVEL),
            first_t_ms: None,
            legs: 0,
        }
    }

    /// The filter state at the left edge of the rep that just closed, captured as it began, and the
    /// sample it was taken at.
    ///
    /// That moment is the start of the stored window: everything before it describes motion the
    /// filter was not yet filtering, so a trace that reached further back would be replaying samples
    /// from a frame that does not exist.
    pub fn rep_state(&self) -> (u32, FusionState) {
        self.state
    }

    /// Legs completed since the detector started.
    pub fn legs(&self) -> u32 {
        self.legs
    }

    /// Speed the bar is taken to be moving at (m/s) — the number the turn test compares against [`VEL_MIN`].
    pub fn speed(&self) -> f32 {
        magnitude(self.v)
    }

    /// Whether a leg is open right now.
    pub fn leg_open(&self) -> bool {
        self.leg_start.is_some()
    }

    /// Feed one filtered sample; returns the span when a rep closes.
    pub fn update(&mut self, f: &Filtered) -> Option<RepSpan> {
        let dt = self.step_ms(f.t_ms);
        let a = f.linear_accel;
        let mag = magnitude(a);

        if dt > 0.0 {
            for k in 0..3 {
                self.v[k] += a[k] * dt;
            }
            if f.still && !self.moved {
                for k in 0..3 {
                    // Only anchor the velocity while the bar is still: at the midpoint of a stroke the acceleration crosses zero while the velocity is at its peak.
                    self.v[k] += (0.0 - self.v[k]) * (dt / V_TAU_S);
                }
            }
            self.pos_z += 0.5 * (self.prev_vz + self.v[2]) * dt;
        }
        self.prev_vz = self.v[2];

        self.first_t_ms.get_or_insert(f.t_ms);
        self.hist_t[self.hist_head] = f.t_ms;
        self.hist_state[self.hist_head] = f.state;
        self.hist_z[self.hist_head] = self.pos_z;
        self.hist_head = (self.hist_head + 1) % HIST;
        self.hist_len = (self.hist_len + 1).min(HIST);

        // A real rest: a hold, not a grind. `still` is the filter's, and it has already waited — it
        // means the reading has held at 1 g for long enough that a slow stroke's own shoulder
        // cannot be what is being seen.
        if f.still {
            self.gap_rest = true;
            if let Some(start) = self.leg_start.take() {
                let moved = self.moved;
                self.moved = false;
                self.still_since = None;
                if moved {
                    // **A rest ends a leg, and it is not only a kindness.** A stopped bar means the
                    // bar has stopped, whatever the integrated speed says; on a real capture the
                    // speed can sit above `VEL_MIN` for a whole window because the integration has
                    // drifted, and a leg that could only end on a speed would run from the first
                    // rep to the last and store nothing. The leg ends where the bar stopped, and
                    // the next one starts when it moves again.
                    let leg = Leg {
                        start_ms: start,
                        end_ms: f.t_ms,
                        disp_z: self.pos_at_z(f.t_ms) - self.leg_pos0_z,
                        had_rest: self.leg_had_rest,
                        state: self.leg_state,
                    };
                    self.legs += 1;
                    return self.pair(leg);
                }
            }
        }

        // Rebase the height while nothing depends on it, so a long session cannot walk it into the range where an f32 has no millimetres left.
        if f.still && self.leg_start.is_none() && self.pending.is_none() && absf(self.pos_z) > 2.0 {
            let shift = self.pos_z;
            self.pos_z -= shift;
            for z in self.hist_z.iter_mut() {
                *z -= shift;
            }
        }

        let speed = magnitude(self.v);

        if self.leg_start.is_none() {
            // Looking for a motion onset, which is only needed at the start of a stream: every leg after the first begins where the previous one turned.
            if mag > ACCEL_ON_THRESHOLD {
                let since = *self.above_since.get_or_insert(f.t_ms);
                if f.t_ms.saturating_sub(since) >= T_ON_MS {
                    self.open_leg(since);
                    self.leg_had_rest = self.gap_rest;
                    self.gap_rest = false;
                    self.above_since = None;
                }
            } else {
                self.above_since = None;
            }
            return None;
        }

        // A leg is open: it ends when the velocity has sat under VEL_MIN for T_TURN_MS.
        if speed > VEL_MIN {
            self.moved = true;
            self.still_since = None;
        } else if self.moved {
            let since = *self.still_since.get_or_insert(f.t_ms);
            if f.t_ms.saturating_sub(since) >= T_TURN_MS {
                let start = self.leg_start.unwrap();
                let leg = Leg {
                    start_ms: start,
                    end_ms: since,
                    disp_z: self.pos_at_z(since) - self.leg_pos0_z,
                    had_rest: self.leg_had_rest,
                    state: self.leg_state,
                };
                // The bar turns, it does not restart: the next leg begins where this one ended, so a turnaround with no pause either side is still two legs.
                self.open_leg(since);
                self.leg_had_rest = false;
                self.legs += 1;
                return self.pair(leg);
            }
        }
        None
    }

    /// Start a leg at `t_ms`, with the height and the filter state the bar had there.
    fn open_leg(&mut self, t_ms: u32) {
        self.leg_start = Some(t_ms);
        self.leg_pos0_z = self.pos_at_z(t_ms);
        self.leg_state = self.state_at(t_ms.saturating_sub(MARGIN_MS));
        self.moved = false;
        self.still_since = None;
    }

    /// Pair a completed leg with the pending one.
    ///
    /// Opposite directions, both past [`MIN_ROM_M`], is a rep. The same direction with no rest
    /// between them is one stroke split by a stall, so the halves fuse — the travel is the sum,
    /// which is what keeps a grind reading as one rep. The same direction *after* a rest is a
    /// one-way push (an unrack), and the older leg is the one to drop.
    fn pair(&mut self, leg: Leg) -> Option<RepSpan> {
        let mag = leg.disp_z.abs();
        let Some(p) = self.pending else {
            if mag >= MIN_ROM_M {
                self.pending = Some(leg);
            }
            return None;
        };
        let pmag = p.disp_z.abs();
        let opposite = p.disp_z * leg.disp_z < 0.0;

        if opposite && pmag >= MIN_ROM_M && mag >= MIN_ROM_M {
            let span = RepSpan {
                start_ms: p.start_ms,
                end_ms: leg.end_ms,
            };
            self.pending = None;
            self.state = p.state;
            if span.duration_ms() >= MIN_REP_MS && span.duration_ms() <= MAX_REP_MS {
                return Some(span);
            }
            return None;
        }

        let next = if !opposite && !leg.had_rest {
            Leg {
                start_ms: p.start_ms,
                end_ms: leg.end_ms,
                disp_z: p.disp_z + leg.disp_z,
                had_rest: p.had_rest,
                state: p.state,
            }
        } else {
            leg
        };
        self.pending = if next.disp_z.abs() >= MIN_ROM_M {
            Some(next)
        } else {
            None
        };
        None
    }

    /// The height at the newest sample at or before `t_ms`.
    fn pos_at_z(&self, t_ms: u32) -> f32 {
        let mut newest: Option<(u32, f32)> = None;
        for i in self.history() {
            let t = self.hist_t[i];
            if t <= t_ms && newest.is_none_or(|(bt, _)| t > bt) {
                newest = Some((t, self.hist_z[i]));
            }
        }
        newest.map_or(self.pos_z, |(_, z)| z)
    }

    /// The filter state at the newest sample at or before `t_ms`, or the oldest one held if
    /// `t_ms` is older than any of them.
    fn state_at(&self, t_ms: u32) -> (u32, FusionState) {
        let mut newest: Option<(u32, FusionState)> = None;
        let mut oldest: Option<(u32, FusionState)> = None;
        for i in self.history() {
            let t = self.hist_t[i];
            if oldest.is_none_or(|(bt, _)| t < bt) {
                oldest = Some((t, self.hist_state[i]));
            }
            if t <= t_ms && newest.is_none_or(|(bt, _)| t > bt) {
                newest = Some((t, self.hist_state[i]));
            }
        }
        newest.or(oldest).unwrap_or(self.state)
    }

    /// The slots the history holds, oldest first — the ring is not contiguous and may not be full.
    fn history(&self) -> impl Iterator<Item = usize> + '_ {
        let oldest = (self.hist_head + HIST - self.hist_len) % HIST;
        (0..self.hist_len).map(move |i| (oldest + i) % HIST)
    }

    /// Seconds since the previous sample, bounded so a stall in the sampler cannot become a jump in velocity.
    fn step_ms(&mut self, t_ms: u32) -> f32 {
        let dt = (t_ms.saturating_sub(self.prev_t_ms) as f32 / 1000.0).min(MAX_DT_S);
        self.prev_t_ms = t_ms;
        dt
    }
}

impl Default for Detector {
    fn default() -> Self {
        Self::new()
    }
}

/// One point of a stored rep trace.
/// The raw `accel` and the attitude are kept alongside the computed `pos` so the app can re-derive traces with better algorithms later without a firmware update.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TracePoint {
    pub t_ms: u32,
    /// ZUPT-anchored displacement from the rep start (m).
    pub pos: [f32; 3],
    /// Raw specific force (fixed point, `sampler::ACCEL_SCALE` LSB/m/s²), gravity included.
    pub accel: [i16; 3],
    /// The attitude the filter computed at this point `[w, x, y, z]` (`sampler::QUAT_SCALE` LSB).
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
    /// The detector already refuses to pair a leg shorter than [`MIN_ROM_M`], so this is the same question asked of a different measurement: the travel the stored trace computes, decimated, rather than the travel integrated on the way in. The two can disagree — a rep that spans a hole in the samples reads long — so the log holds only summaries where the number itself says the bar moved, and that is measured on the vertical, the only axis with a right answer.
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

/// Compute the trace and summary for one rep by replaying the filter over its window.
///
/// `samples` should be the detector's span plus margin at each end, and `from` the filter state the
/// detector captured where that window begins ([`Detector::rep_state`]). The filter is
/// deterministic from that state, so the same window and the same state always give the same
/// summary — which is what makes a stored trace re-derivable.
///
/// The trace's position is the filter's velocity integrated; the filter sets that velocity to zero
/// whenever the bar is still, and a lift guarantees stillness at both ends of every stroke, so the
/// window is anchored at both of them and the drift is bounded to within the rep. Any number of
/// samples is accepted; if more than `N` are supplied the trace is decimated uniformly to fit.
pub fn compute_rep(
    samples: &[Sample],
    from: &FusionState,
    out: &mut TraceBuf<'_>,
) -> msg::RepSummary {
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

    let mut fusion = Fusion::from_state(*from);
    let mut p = [0.0f32; 3];
    let mut v_prev = from.velocity;
    let mut a_max = 0.0f32;
    let mut v_max = 0.0f32;
    let mut v_sum = 0.0f32;
    let mut lo = [f32::MAX; 3];
    let mut hi = [f32::MIN; 3];
    let mut roll_offset = 0.0f32;
    let mut roll_prev = 0.0f32;
    let mut roll_seen = false;

    for i in 0..n {
        let filtered = fusion.update(&samples[i]);
        let v = filtered.state.velocity;
        if i > 0 {
            let dt = (samples[i].t_ms - samples[i - 1].t_ms) as f32 / 1000.0;
            for k in 0..3 {
                p[k] += 0.5 * (v_prev[k] + v[k]) * dt;
            }
        }
        v_prev = v;

        let amag = magnitude(filtered.linear_accel);
        if amag > a_max {
            a_max = amag;
        }
        let speed = magnitude(v);
        if speed > v_max {
            v_max = speed;
        }
        v_sum += speed;
        for k in 0..3 {
            if p[k] < lo[k] {
                lo[k] = p[k];
            }
            if p[k] > hi[k] {
                hi[k] = p[k];
            }
        }

        let (roll, pitch) = roll_pitch_deg(filtered.state.attitude);
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
                pos: p,
                accel: samples[i].accel,
                quat: filtered.state.attitude.map(|q| quantize(q, QUAT_SCALE)),
            });
        }
    }

    // Range of motion is the vertical travel, and Z is the only axis that can say what vertical is: the filter pins it
    // to gravity, while X and Y are the mount's heading and drift with yaw. Whichever axis moved furthest is not the
    // same question — on the horizontal axes it is as often the estimate's error as the bar.
    summary.rom = hi[2] - lo[2];
    summary.t_start_ms = samples[0].t_ms;
    summary.t_end_ms = samples[n - 1].t_ms;
    summary.duration_ms = summary.t_end_ms - summary.t_start_ms;
    summary.peak_accel = a_max;
    summary.peak_velocity = v_max;
    summary.mean_velocity = v_sum / n as f32;
    summary.trace_len = out.len() as u32;
    // The summary started here, so it carries the velocity the replay was seeded from: the app can tell a rep
    // that started mid-set from one that started at rest, and can re-derive the trace from the same origin.
    summary.set_v0(msg::TraceOrigin {
        vx: from.velocity[0],
        vy: from.velocity[1],
        vz: from.velocity[2],
    });
    summary
}

fn absf(x: f32) -> f32 {
    if x < 0.0 {
        -x
    } else {
        x
    }
}
