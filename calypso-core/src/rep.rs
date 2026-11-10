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
/// Rest kept either side of the motion span, so the filter has an at-rest window to anchor the trace to and the app can plot the rep's endpoints.
///
/// Half a second, because the filter only calls a bar at rest after it has held still for 400 ms (a stroke's own shoulder reads the same for a third of a second). A shorter margin leaves the trace with no anchor at its ends, and the position drifts with nothing to pull it back.
pub const MARGIN_MS: u32 = 500;
/// Sample times, filter states and heights the detector keeps, so a leg can name the point it began at after the fact: at most `MARGIN_MS` behind a turn, or `T_ON_MS` behind a motion onset, plus headroom.
const HIST: usize = 64;
/// How many points a rep's **plotted** trace keeps, however long the rep is.
///
/// The plot is the only consumer and a bar path is smooth, so this is every trace a screen can tell apart while costing a fraction of the log that the full rate would. Anything that has to be replayed from every sample is the raw trace, which is not decimated at all.
pub const TRACE_POINTS: usize = 128;
/// Smallest dominant-axis travel (metres) that counts as a rep. A hand on a rack, a bar being unracked, a bump: each crosses the motion thresholds, none of them moves the sensor far enough to be a rep.
pub const MIN_ROM_M: f32 = 0.07;

/// Speed (m/s) the bar must reach between reversals for a return to zero to be a reversal rather than noise.
pub const VEL_MIN: f32 = 0.07;
/// How long the bar's velocity must stay near zero for it to count as a reversal.
pub const T_TURN_MS: u32 = 20;
/// Vertical acceleration (m/s²) a turn's own extreme must reach before the acceleration can name it.
///
/// The one cue here that cannot drift, because it is a reading and not an integral: a set held at 1 m/s² of un-corrected tilt reached ten metres a second of integral and its turns still read 2–11 m/s² clear of it. A gentle stroke that never gets there is what [`VEL_MIN`] is for.
pub const TURN_ACCEL: f32 = 2.0;
/// How far (m/s²) the acceleration has to come back from its extreme before that extreme is a turn rather than one push still in progress.
pub const TURN_BACK: f32 = 1.0;
/// Longest integration step honoured, so a stalled sampler cannot turn a gap in the samples into a jump in velocity.
const MAX_DT_S: f32 = 0.1;
/// Time constant over which the detector's velocity is pulled back to zero while it is idle and the bar is still — the velocity form of "it is not moving, so it is not going anywhere".
///
/// The filter has its own, and deliberately: the detector goes on integrating the acceleration itself so that a change in what the device stores can be attributed to the detection or to the filter, and the two velocities disagreeing is the measurement.
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

/// What a replay needs to know about the turns the rep that just closed was measured between.
///
/// A turn is the one moment inside a stroke where the bar's vertical speed is genuinely zero — it is reversing — so it is where a replay can put its velocity back to zero without lying about the motion. Three of them delimit one rep: the two ends, and the turn between the legs.
///
/// **Only where a cue names that instant exactly.** The acceleration does: its extreme is where the bar stopped decelerating and started back. The speed does not: it knows the bar is under [`VEL_MIN`], which is a whole crowd of speed the bar still has, and zeroing there swaps a bounded drift for an error of the same size as the drift it was meant to bound. A turn with no instant is a turn the replay leaves alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RepTurns {
    /// The rep's own span: where its first leg began to move.
    pub start_ms: u32,
    /// Where its second leg turned — the rep's own end.
    pub end_ms: u32,
    /// Where the speed reached zero at each of the three instants that delimit the rep, oldest first, where a cue could say.
    pub zeros: [Option<Zero>; 3],
}

/// An instant a replay may put its velocity back to zero, and which side of its sample the step belongs to.
///
/// The distinction is one sample wide and it is not bookkeeping: the sample at an instant carries the acceleration of the step that *led* to it, so a bar that was already at rest as that step began has not been moved by it, and a bar that reached zero at it has.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Zero {
    /// The bar was already at rest when this sample's step began — an onset, or a hold. The step is the motion that follows the instant, so the zero goes in before it is integrated.
    Rest(u32),
    /// The bar's speed reached zero at this sample: it turned. The step carries the deceleration that led to the turn, so the zero goes in after it is integrated.
    Turn(u32),
}

/// Which way the bar turns next.
///
/// The vertical acceleration peaks where the bar stops falling and troughs where it stops rising, and a bar that has just stopped going one way has to come back before it can stop going the other: consecutive turns alternate. That alternation is also what keeps one push from naming two turns, which a plain "the acceleration turned around" test does whenever the extreme is followed by a slow slope.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Turn {
    /// The bar stops rising: a trough of the vertical acceleration.
    Top,
    /// The bar stops falling: a peak of it.
    Bottom,
}

impl Turn {
    const fn other(self) -> Turn {
        match self {
            Turn::Top => Turn::Bottom,
            Turn::Bottom => Turn::Top,
        }
    }
}

/// A leg: the bar's travel between two turns, and what the bar carried into it.
#[derive(Debug, Clone, Copy)]
struct Leg {
    start_ms: u32,
    /// Where the bar's vertical speed reached zero at the turn that began it, where the cue that named that turn could say.
    ///
    /// Not always [`Leg::start_ms`], because a cue names a turn as well as it can rather than exactly: the edge is where the motion was cut, and this is the instant a replay may put its velocity back to zero without throwing away speed the bar still had.
    start_zero: Option<Zero>,
    end_ms: u32,
    /// The same at the turn that ended it.
    end_zero: Option<Zero>,
    /// Signed vertical displacement over the leg.
    disp_z: f32,
    /// Whether the bar was rising over the leg.
    ///
    /// A turn the acceleration named says so by itself — it knows which way it stopped — and that is the answer to use, because the displacement is an integral and an integral is what drifted.
    /// A turn the velocity named does not, and there the displacement's sign is what is left.
    up: bool,
    /// Whether the bar came to rest before this leg moved — a hold, not a grind.
    had_rest: bool,
    /// The filter state one [`MARGIN_MS`] before the leg started, and the moment it was taken.
    state: (u32, FusionState),
}

/// Turning-point detector: **a rep is an out-and-back**.
///
/// A turn is where the bar reverses, and the bar's vertical acceleration reverses there too: it is a reading, and the one signal here that cannot walk away, so it is what names a turn. The vertical speed names one only when the stroke is too gentle to move the acceleration by [`TURN_ACCEL`], which a deliberate negative is.
///
/// Between two turns the bar went one way: that is a *leg*, and which way is known from the turn that ended it — a top ends an up leg — rather than from the travel, because the travel is an integral and an integral is the thing that drifts. A rep is two consecutive legs in opposite directions, each longer than [`MIN_ROM_M`].
///
/// Asking the question in this order puts the travel check inside detection rather than after it:
/// a stroke too short to be half a rep can never pair with anything, so a bump, a re-rack and a hand on the rack are rejected by the same measurement that accepts a press. A one-way push tall enough to be a leg is dropped when the bar rests and then sets off the same way again — that is an unrack, not a rep.
///
/// **What it cannot see.** A return that reaches neither cue — too gentle to move the acceleration by [`TURN_ACCEL`] and never above [`VEL_MIN`] at its peak — does not close a rep. Motion must also clear [`ACCEL_ON_THRESHOLD`] for [`T_ON_MS`] to open a leg at all. And an oscillation whose cycle is long enough to pass `MIN_REP_MS` and wide enough to pass `MIN_ROM_M` is an out-and-back like any other: nothing here can tell a bar being lifted from a hand moving one on the rack.
///
/// **Everything here is in the world frame**, because the filter supplies one worth the name: tilt comes from the gyro and is pulled onto gravity continuously. The detector still integrates the acceleration into its own velocity rather than taking the filter's, so that the two can be compared on a real session before either is trusted.
///
/// [`Detector::rep_state`] is the filter's state where the stored window begins, which is what [`compute_rep`] replays from.
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
    /// Where the bar's vertical speed reached zero at the turn the open leg began at, where the cue that named it could say.
    leg_zero: Option<Zero>,
    leg_pos0_z: f32,
    leg_state: (u32, FusionState),
    leg_had_rest: bool,
    /// Whether the bar has moved since the leg opened. A turn only counts once it has.
    moved: bool,
    /// Which way the bar turns next, and the acceleration's extreme in that direction since the last turn, with the moment it was reached.
    next_turn: Turn,
    ext: f32,
    ext_t: u32,
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
    /// The turns the rep that just closed was measured between.
    turns: RepTurns,
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
            leg_zero: None,
            leg_pos0_z: 0.0,
            leg_state: (0, FusionState::LEVEL),
            leg_had_rest: false,
            moved: false,
            next_turn: Turn::Top,
            ext: 0.0,
            ext_t: 0,
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
            turns: RepTurns {
                start_ms: 0,
                end_ms: 0,
                zeros: [None; 3],
            },
            first_t_ms: None,
            legs: 0,
        }
    }

    /// The filter state at the left edge of the rep that just closed, captured as it began, and the sample it was taken at.
    ///
    /// That moment is the start of the stored window: everything before it describes motion the filter was not yet filtering, so a trace that reached further back would be replaying samples from a frame that does not exist.
    pub fn rep_state(&self) -> (u32, FusionState) {
        self.state
    }

    /// The turns the rep that just closed was measured between.
    ///
    /// Taken with [`Detector::rep_state`]: the state is where the replay starts, and these are where it is told the bar stopped, so the two together are everything [`compute_rep`] needs to not carry an integral across a rep the bar turned inside.
    pub fn rep_turns(&self) -> RepTurns {
        self.turns
    }

    /// Legs completed since the detector started.
    pub fn legs(&self) -> u32 {
        self.legs
    }

    /// The bar's whole velocity (m/s), for the log. The turn test reads the vertical, not this.
    pub fn speed(&self) -> f32 {
        magnitude(self.v)
    }

    /// Vertical speed (m/s), which is the one a leg is delimited by.
    ///
    /// Every leg is a signed **vertical** travel, so the vertical is the axis a rep is measured on and the only one that can say the bar has stopped moving along it. The other two carry the mount's heading and whatever the frame has left in it — a couple of degrees of attitude is tenths of a metre per second squared, which over a stroke is a velocity no turnaround would ever fall below, so a turn read off the whole speed is a turn that never happens.
    pub fn vertical_speed(&self) -> f32 {
        absf(self.v[2])
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

        // A real rest: a hold, not a grind. `still` is the filter's, and it has already waited — it means the reading has held at 1 g for long enough that a slow stroke's own shoulder cannot be what is being seen.
        if f.still {
            self.gap_rest = true;
            if let Some(start) = self.leg_start.take() {
                let moved = self.moved;
                self.moved = false;
                self.still_since = None;
                if moved {
                    // **A rest ends a leg, and it is not only a kindness.** A stopped bar means the bar has stopped, whatever the integrated speed says; on a real capture the speed can sit above `VEL_MIN` for a whole window because the integration has drifted, and a leg that could only end on a speed would run from the first rep to the last and store nothing. The leg ends where the bar stopped, and the next one starts when it moves again.
                    let disp_z = self.pos_at_z(f.t_ms) - self.leg_pos0_z;
                    let leg = Leg {
                        start_ms: start,
                        start_zero: self.leg_zero,
                        end_ms: f.t_ms,
                        // No anchor: a rest is the one place the filter stops the velocity itself, and it has been pulling at it since long before this sample. An anchor here would add its own timing error to a zero the filter already has — and it lands on the first sample of the patch, where a stroke slow enough to look still for 400 ms is still genuinely moving.
                        end_zero: None,
                        disp_z,
                        up: disp_z > 0.0,
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

        let speed = self.vertical_speed();

        // The bar turns where its **vertical acceleration** does — a reading, and the one signal here that cannot walk away. The extreme comes first; the turn is named once the acceleration has come back far enough that the extreme is behind it.
        let w = self.on_side(a[2]);
        if w > self.ext {
            self.ext = w;
            self.ext_t = f.t_ms;
        }
        let turned = w <= self.ext - TURN_BACK && self.ext >= TURN_ACCEL;

        if self.leg_start.is_none() {
            // Looking for a motion onset, which is only needed at the start of a stream: every leg after the first begins where the previous one turned.
            if mag > ACCEL_ON_THRESHOLD {
                let since = *self.above_since.get_or_insert(f.t_ms);
                if f.t_ms.saturating_sub(since) >= T_ON_MS {
                    // An onset is where the bar started, not where it reversed, so the bar is still at rest as that sample's step begins — a different side of the sample from a turn, and the whole reason [`Zero`] has two of them.
                    self.open_leg(since, Some(Zero::Rest(since)));
                    self.leg_had_rest = self.gap_rest;
                    self.gap_rest = false;
                    self.above_since = None;
                    // Off a rest the first turnaround is the far end of this stroke: a stroke that begins by rising stops rising first. One that begins by falling names its own start as a trough and has that leg dropped for being too short to be half a rep, which costs the first leg and nothing else.
                    self.next_turn = Turn::Top;
                    self.ext = self.on_side(a[2]);
                    self.ext_t = f.t_ms;
                }
            } else {
                self.above_since = None;
            }
            return None;
        }

        if turned {
            let (at, ended) = (self.ext_t, self.next_turn);
            self.take_turn(a[2], f.t_ms);
            // The extreme of the vertical acceleration is where the bar stopped decelerating and started back, which is the instant its speed was zero: the cue that names that instant exactly, and the reason a replay is anchored here.
            return self.close_leg(at, Some(Zero::Turn(at)), Some(ended == Turn::Top));
        }

        // A leg also ends when the velocity has sat under VEL_MIN for T_TURN_MS, which is the only cue a stroke too gentle to move the acceleration gives.
        if speed > VEL_MIN {
            self.moved = true;
            self.still_since = None;
        } else if self.moved {
            let since = *self.still_since.get_or_insert(f.t_ms);
            if f.t_ms.saturating_sub(since) >= T_TURN_MS {
                self.take_turn(a[2], f.t_ms);
                // **This cue cannot say where the speed reached zero.** `since` is where it fell under [`VEL_MIN`] and the confirmation is `T_TURN_MS` later, and the stop is somewhere between: the bar still has a seventh of a metre per second at one end and may have turned at the other. Zeroing a replay's velocity anywhere in there would trade a bounded drift for an error of the same size, so it is not recorded and the replay is left unanchored across this turn.
                return self.close_leg(since, None, None);
            }
        }
        None
    }

    /// The vertical acceleration on the side the next turn is expected from.
    fn on_side(&self, a_z: f32) -> f32 {
        match self.next_turn {
            Turn::Top => -a_z,
            Turn::Bottom => a_z,
        }
    }

    /// Take a turn: the next one is the other way, and the acceleration starts again from where it is now, so the extreme just used cannot name a second turn.
    fn take_turn(&mut self, a_z: f32, t_ms: u32) {
        self.next_turn = self.next_turn.other();
        self.ext = self.on_side(a_z);
        self.ext_t = t_ms;
    }

    /// Close the open leg at `t_ms` and open the next one there, answering with a rep if this half finished one.
    ///
    /// `up` is the direction when the cue that found the turn knew it, and `None` when the travel is the only thing that can say.
    fn close_leg(&mut self, t_ms: u32, zero: Option<Zero>, up: Option<bool>) -> Option<RepSpan> {
        let start = self.leg_start.expect("a leg ends at a turn");
        let disp_z = self.pos_at_z(t_ms) - self.leg_pos0_z;
        let leg = Leg {
            start_ms: start,
            start_zero: self.leg_zero,
            end_ms: t_ms,
            end_zero: zero,
            disp_z,
            up: up.unwrap_or(disp_z > 0.0),
            had_rest: self.leg_had_rest,
            state: self.leg_state,
        };
        // The bar turns, it does not restart: the next leg begins where this one ended, so a turnaround with no pause either side is still two legs.
        self.open_leg(t_ms, zero);
        self.leg_had_rest = false;
        self.legs += 1;
        self.pair(leg)
    }

    /// Start a leg at `t_ms`, with the height and the filter state the bar had there.
    fn open_leg(&mut self, t_ms: u32, zero: Option<Zero>) {
        self.leg_start = Some(t_ms);
        self.leg_zero = zero;
        self.leg_pos0_z = self.pos_at_z(t_ms);
        self.leg_state = self.state_at(t_ms.saturating_sub(MARGIN_MS));
        self.moved = false;
        self.still_since = None;
    }

    /// Pair a completed leg with the pending one.
    ///
    /// Opposite directions, both past [`MIN_ROM_M`], is a rep. The same direction with no rest between them is one stroke split by a stall, so the halves fuse — the travel is the sum, which is what keeps a grind reading as one rep. The same direction *after* a rest is a one-way push (an unrack), and the older leg is the one to drop.
    fn pair(&mut self, leg: Leg) -> Option<RepSpan> {
        let mag = leg.disp_z.abs();
        let Some(p) = self.pending else {
            if mag >= MIN_ROM_M {
                self.pending = Some(leg);
            }
            return None;
        };
        let opposite = p.up != leg.up;

        // A stall that reaches no rest continues the stroke it interrupted, so its travel belongs to that stroke whatever size it is.
        if !opposite && !leg.had_rest {
            self.pending = Some(Leg {
                start_ms: p.start_ms,
                start_zero: p.start_zero,
                end_ms: leg.end_ms,
                end_zero: leg.end_zero,
                disp_z: p.disp_z + leg.disp_z,
                up: p.up,
                had_rest: p.had_rest,
                state: p.state,
            });
            return None;
        }

        // **A leg under [`MIN_ROM_M`] is not a stroke, and it does not consume one.** A bar settling a few centimetres at a turnaround is motion in the opposite direction and it can reach `VEL_MIN`, so it arrives here as a leg; letting it pair — or letting it replace the leg before it, which is what the same-direction case used to do — throws away the rep it followed. It is dropped, and what was pending keeps waiting.
        if mag < MIN_ROM_M {
            return None;
        }

        if opposite {
            let span = RepSpan {
                start_ms: p.start_ms,
                end_ms: leg.end_ms,
            };
            self.pending = None;
            self.state = p.state;
            if span.duration_ms() >= MIN_REP_MS && span.duration_ms() <= MAX_REP_MS {
                // The pending leg ended at the turn the second one began at, so its end is the turnaround — which is the instant a replay can put its velocity back to zero.
                self.turns = RepTurns {
                    start_ms: p.start_ms,
                    end_ms: leg.end_ms,
                    zeros: [p.start_zero, p.end_zero, leg.end_zero],
                };
                return Some(span);
            }
            return None;
        }

        // The same direction after a rest is a one-way push, and the older leg is the finished one.
        self.pending = Some(leg);
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

    /// The filter state at the newest sample at or before `t_ms`, or the oldest one held if `t_ms` is older than any of them.
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

/// One point of a rep's **plotted** trace: where the bar was, and which way the sensor faced.
///
/// Derived, and only ever used to draw. The inputs it was derived from are the rep's raw trace, which a device records on request (`REP_RAW`) and which is what a summary is re-derived from.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TracePoint {
    pub t_ms: u32,
    /// ZUPT-anchored displacement from the rep start (m).
    pub pos: [f32; 3],
    /// The attitude the filter computed at this point `[w, x, y, z]` (`sampler::QUAT_SCALE` LSB).
    pub quat: [i16; 4],
}

impl TracePoint {
    pub const ZERO: TracePoint = TracePoint {
        t_ms: 0,
        pos: [0.0; 3],
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

/// Which detection logic closed a rep, as it travels in that rep's `RepConfig`.
///
/// Bumped when a change makes a stored summary mean something else — a threshold that moves is a different measurement wearing the same name, and two numbers from two versions are then not comparable as if they were one.
pub const DETECTOR_VERSION: u32 = 4;

impl msg::RepSummary {
    /// Whether this is a rep at all: the bar has to have gone up or down [`MIN_ROM_M`].
    ///
    /// The detector already refuses to pair a leg shorter than [`MIN_ROM_M`], so this is the same question asked of a different measurement: the travel the stored trace computes, decimated, rather than the travel integrated on the way in. The two can disagree — a rep that spans a hole in the samples reads long — so the log holds only summaries where the number itself says the bar moved, and that is measured on the vertical, the only axis with a right answer.
    pub fn is_a_rep(&self) -> bool {
        self.rom >= MIN_ROM_M
    }
}

/// Compute the trace and summary for one rep by replaying the filter over its window.
///
/// `samples` should be the detector's span plus margin at each end, and `from` the filter state the detector captured where that window begins ([`Detector::rep_state`]). The filter is deterministic from that state, so the same window and the same state always give the same summary — which is what makes a stored trace re-derivable.
///
/// The trace's position is the filter's velocity integrated, and that integral is put back to zero twice over: wherever the filter sees the bar still, and at every turn `turns` names. Stillness alone is not enough on a real set — the window starts from a state that already carries the draft of the last one, and a stroke with no pause in it gives the filter nothing to anchor to between reps. A turn is the one instant the bar's vertical speed is genuinely zero whatever the estimate says, which is what bounds the drift to the inside of one leg.
///
/// Any number of samples is accepted; if more than `N` are supplied the plotted trace is decimated uniformly to fit (`TRACE_POINTS`, the plot's budget — the raw trace is never decimated and does not come from here).
pub fn compute_rep(
    samples: &[Sample],
    from: &FusionState,
    turns: Option<RepTurns>,
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
    // Anchor the decimation grid at the *last* sample, so the rep's end is always stored (it is the most interesting point when plotting) and the count stays within capacity.
    let first = if capacity == 0 { 0 } else { (n - 1) % stride };

    let mut fusion = Fusion::from_state(*from);
    let mut p = [0.0f32; 3];
    let mut v_prev = from.velocity;
    // The instants the replay is told the bar was not moving, split by which side of their sample the step belongs to: see [`Zero`]. Both are in the order they happen.
    let mut before = [0u32; 3];
    let mut before_len = 0usize;
    let mut after = [0u32; 3];
    let mut after_len = 0usize;
    if let Some(t) = turns {
        for zero in t.zeros.into_iter().flatten() {
            match zero {
                Zero::Rest(ms) => {
                    before[before_len] = ms;
                    before_len += 1;
                }
                Zero::Turn(ms) => {
                    after[after_len] = ms;
                    after_len += 1;
                }
            }
        }
    }
    let before = &before[..before_len];
    let after = &after[..after_len];
    let mut next_before = 0usize;
    let mut next_after = 0usize;
    // The range of motion is the rep's own travel, so the rest the window keeps either side of it — which the plot draws and the filter needs, and which is not motion the rep made — is not measured. Without a rep's span to measure, everything in the window is.
    let (span_start, span_end) = turns.map_or((u32::MIN, u32::MAX), |t| (t.start_ms, t.end_ms));
    let mut v_max = 0.0f32;
    let mut lo = [f32::MAX; 3];
    let mut hi = [f32::MIN; 3];
    let mut roll_offset = 0.0f32;
    let mut roll_prev = 0.0f32;
    let mut roll_seen = false;

    for i in 0..n {
        // A bar that was already at rest as this step began is not moved by it, so the zero goes in ahead of the integration: the step is the motion that follows the instant.
        while next_before < before.len() && samples[i].t_ms >= before[next_before] {
            fusion.zero_velocity();
            next_before += 1;
        }
        let filtered = fusion.update(&samples[i]);
        // A turn is the other way round: the step carries the deceleration that led to it, and the bar's speed is zero at the end of that step rather than the start.
        while next_after < after.len() && samples[i].t_ms >= after[next_after] {
            fusion.zero_velocity();
            next_after += 1;
        }
        let v = fusion.state().velocity;
        if i > 0 {
            let dt = (samples[i].t_ms - samples[i - 1].t_ms) as f32 / 1000.0;
            for k in 0..3 {
                p[k] += 0.5 * (v_prev[k] + v[k]) * dt;
            }
        }
        v_prev = v;

        let speed = magnitude(v);
        if speed > v_max {
            v_max = speed;
        }
        if samples[i].t_ms >= span_start && samples[i].t_ms <= span_end {
            for k in 0..3 {
                if p[k] < lo[k] {
                    lo[k] = p[k];
                }
                if p[k] > hi[k] {
                    hi[k] = p[k];
                }
            }
        }

        let (roll, pitch) = roll_pitch_deg(filtered.state.attitude);
        // Roll is reported in (-180, 180], so a rep that crosses the seam reads as a full turn between its minimum and its maximum. Unwrapping it first is what makes the range a rotation rather than an artefact of the branch cut.
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
                quat: filtered.state.attitude.map(|q| quantize(q, QUAT_SCALE)),
            });
        }
    }

    // Range of motion is the vertical travel, and Z is the only axis that can say what vertical is: the filter pins it to gravity, while X and Y are the mount's heading and drift with yaw. Whichever axis moved furthest is not the same question — on the horizontal axes it is as often the estimate's error as the bar.
    summary.rom = if hi[2] >= lo[2] { hi[2] - lo[2] } else { 0.0 };
    summary.t_start_ms = samples[0].t_ms;
    summary.t_end_ms = samples[n - 1].t_ms;
    summary.duration_ms = summary.t_end_ms - summary.t_start_ms;
    summary.peak_velocity = v_max;
    summary.computed_len = out.len() as u32;
    // The summary started here, so it carries the whole filter state the replay was seeded from: a filter started from it and fed the window lands exactly where this one did, which is what makes a stored summary re-derivable rather than merely reproducible.
    summary.set_state(msg::RepState {
        vx: from.velocity[0],
        vy: from.velocity[1],
        vz: from.velocity[2],
        qw: from.attitude[0],
        qx: from.attitude[1],
        qy: from.attitude[2],
        qz: from.attitude[3],
        accel_bias_x: from.accel_bias[0],
        accel_bias_y: from.accel_bias[1],
        accel_bias_z: from.accel_bias[2],
        gyro_bias_x: from.gyro_bias[0],
        gyro_bias_y: from.gyro_bias[1],
        gyro_bias_z: from.gyro_bias[2],
        settled: from.settled,
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
