//! Rep detection and per-rep trace computation.

use crate::fusion::{magnitude, roll_pitch_deg, Filtered, Fusion, FusionState};
use crate::pb::msg;
use crate::sampler::{quantize, Sample, QUAT_SCALE};

/// Speed (m/s) the bar has to reach going each way for a stroke to be a leg.
///
/// The one number that says a bar moved. A hand on the rack, a bump, a bar being slid into position: each reaches a fraction of a metre per second and no more.
pub const SWING_MPS: f32 = 0.30;
/// Time constant (s) over which the speed is pulled back toward zero.
///
/// Short enough that an acceleration bias cannot walk it away, long enough that a slow stroke is still a speed.
pub const SPEED_TAU_S: f32 = 1.0;
/// Shortest rep (ms): the turn it started at to the turn it ended at.
pub const MIN_SWING_MS: u32 = 400;
/// Longest rep (ms): past this the bar is being carried rather than lifted. The pauses at either end of a stroke are inside it, which is what makes it longer than a stroke.
pub const MAX_SWING_MS: u32 = 8000;
/// How long the bar has to be at rest before a stroke begins a rep rather than ending one.
///
/// Longer than any pause taken at the far end of a stroke — a hold on the chest, a reset on the floor — because a bar parked there is half a rep, and a shorter wait drops that half and puts every rep after it out of phase. Short enough to see the rest between sets, which is where the motion that is not a rep is.
pub const REST_MS: u32 = 10_000;
/// How far past the span a stored window runs.
///
/// Half a second, because the filter only calls a bar at rest after it has held still for 400 ms (a stroke's own shoulder reads the same for a third of a second). A shorter window leaves the trace with nothing to plot its end against.
pub const MARGIN_MS: u32 = 500;
/// Sample times and filter states the detector keeps, so the state a stored window is seeded from can be found after the fact: `MARGIN_MS` of samples, plus headroom.
const HIST: usize = 64;
/// How many points a rep's **plotted** trace keeps, however long the rep is.
///
/// The plot is the only consumer and a bar path is smooth, so this is every trace a screen can tell apart while costing a fraction of the log that the full rate would. Anything that has to be replayed from every sample is the raw trace, which is not decimated at all.
pub const TRACE_POINTS: usize = 128;
/// Smallest dominant-axis travel (metres) a stored summary can report and still be called a rep.
///
/// Nothing in the detector measures travel, so this does not accept a stroke: it is the same question asked of the trace a stored rep carries, which is what the app plots. A rep whose own trace says the bar moved less than this is a rep the trace does not support.
pub const MIN_ROM_M: f32 = 0.07;

/// Longest integration step honoured, so a stalled sampler cannot turn a gap in the samples into a jump in speed.
const MAX_DT_S: f32 = 0.1;

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
/// A turn is the one moment inside a stroke where the bar's vertical speed is genuinely zero — it is reversing — so it is where a replay can put its velocity back to zero without lying about the motion. Three of them delimit one rep: the two ends, and the turn between the halves.
///
/// All three are named the same way: the quietest moment in the stretch the bar is in, which is the speed's own minimum and the instant a replay can put its integral back to zero on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RepTurns {
    /// The rep's own span: the turn the bar came into it from.
    pub start_ms: u32,
    /// The rep's own end: the turn the bar came back to.
    pub end_ms: u32,
    /// Where the speed reached zero at each of the three instants that delimit the rep, oldest first, where a cue could say.
    pub zeros: [Option<Zero>; 3],
}

/// A detected rep: its span, the turns it was measured between, and where its window starts.
///
/// Everything the caller needs to store it, answered at the moment it closes rather than left in
/// the detector to be read back before the next sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DetectedRep {
    pub span: RepSpan,
    pub turns: RepTurns,
    /// Where the stored window starts, which is the turn the rep opened at: the samples before it are
    /// what the filter had to settle on, and the window's own margin is at its end.
    pub from_ms: u32,
    /// The filter state where that window starts, which is what the replay is seeded with.
    pub state: FusionState,
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

/// One sample as the detector keeps it: when it was, the bar's vertical speed on it, and the filter state there.
#[derive(Clone, Copy)]
struct Note {
    t_ms: u32,
    vz: f32,
    state: FusionState,
}

impl Note {
    const ZERO: Note = Note {
        t_ms: 0,
        vz: 0.0,
        state: FusionState::LEVEL,
    };
}

/// The last [`HIST`] samples, oldest first, over storage that wraps.
struct Hist {
    notes: [Note; HIST],
    head: usize,
    len: usize,
}

impl Hist {
    const fn new() -> Self {
        Hist {
            notes: [Note::ZERO; HIST],
            head: 0,
            len: 0,
        }
    }

    fn push(&mut self, note: Note) {
        self.notes[self.head] = note;
        self.head = (self.head + 1) % HIST;
        self.len = (self.len + 1).min(HIST);
    }

    /// The slots the history holds, oldest first — the ring is not contiguous and may not be full.
    fn slots(&self) -> impl DoubleEndedIterator<Item = usize> + '_ {
        let oldest = (self.head + HIST - self.len) % HIST;
        (0..self.len).map(move |i| (oldest + i) % HIST)
    }

    /// The quietest moment in the [`MARGIN_MS`] before `to_ms`, which is where a bar that was at rest has been since: the turn the first stroke of a run left.
    fn turn(&self, to_ms: u32) -> Option<Note> {
        let mut best: Option<Note> = None;
        for i in self.slots().rev() {
            let note = self.notes[i];
            if note.t_ms > to_ms {
                continue;
            }
            if to_ms.saturating_sub(note.t_ms) > MARGIN_MS {
                break;
            }
            if best.is_none_or(|had| absf(note.vz) < absf(had.vz)) {
                best = Some(note);
            }
        }
        best
    }

    /// The filter state at the newest sample at or before `t_ms`, or the oldest one held if `t_ms` is older than any of them.
    fn state_at(&self, t_ms: u32) -> FusionState {
        let mut newest: Option<(u32, FusionState)> = None;
        let mut oldest: Option<(u32, FusionState)> = None;
        for i in self.slots() {
            let note = self.notes[i];
            if oldest.is_none_or(|(bt, _)| note.t_ms < bt) {
                oldest = Some((note.t_ms, note.state));
            }
            if note.t_ms <= t_ms && newest.is_none_or(|(bt, _)| note.t_ms > bt) {
                newest = Some((note.t_ms, note.state));
            }
        }
        newest.or(oldest).map_or(FusionState::LEVEL, |(_, s)| s)
    }
}

/// The turn a stroke left, waiting for the turn that ends it.
#[derive(Clone, Copy)]
struct Turn {
    ms: u32,
    state: FusionState,
}

/// One stroke of the bar: from the turn it left to the turn it arrived at.
#[derive(Debug, Clone, Copy)]
struct Leg {
    start_ms: u32,
    end_ms: u32,
    /// The filter state where the stroke started, which is what a stored window is seeded with.
    state: FusionState,
}

/// The bar's speed, integrated from its vertical acceleration and pulled back toward zero with a
/// [`SPEED_TAU_S`] time constant, and the strokes the bar makes with it.
///
/// **The leak is the design.** An integral cannot know its own error, so an unfiltered one walks away
/// as soon as the acceleration carries a bias, and a working set is twenty seconds with no rest in it
/// long enough to correct that: a couple of degrees of tilt is `g·sin θ` for the whole set, and the
/// drift it leaves is larger than the stroke being measured. Pulling the integral toward zero makes
/// the speed a measurement of *now*, so a bar that is not moving has no speed — continuously, without
/// a rest having to be judged and without waiting for one.
///
/// A **leg** is one stroke, from the turn the bar left to the turn it arrived at. A turn is the
/// quietest moment in the stretch the bar is in — the speed's own minimum, which no threshold has to
/// announce — so a leg is named by the measurement rather than by a check on it. Nothing here says
/// whether a leg is half a rep: that is the next stage's question, and it is asked the same way of
/// every stroke the bar makes.
///
/// The bars are asked to know when a stroke *ended*, which is a turn: a crossing is only the middle of
/// one. So the crossing opens a wait, the bar's own speed is watched to its quietest moment, and the
/// leg is taken when the bar comes to rest on it — or when the next stroke starts, whichever is first.
///
/// What it cannot see. A stroke whose peak speed stays under [`SWING_MPS`] is not a leg to it, which is
/// the price of asking the question once. And two strokes in the same direction with a pause between
/// them are *one* leg, because the speed never changes side for the turn to be a turn — which is what
/// an unrack followed by a drive looks like.
struct Legs {
    /// The bar's speed on all three axes: the vertical is the one that decides, and the log reads the whole.
    v: [f32; 3],
    /// Timestamp of the previous sample, for the integration step.
    prev_t_ms: u32,
    /// The side of ±[`SWING_MPS`] the speed last cleared, so a wobble about one of them is not two crossings.
    side: i8,
    /// The turn the stroke in progress left, waiting for the turn that ends it.
    prev: Option<Turn>,
    /// The crossing that ended a stroke, and the quietest moment since it — the turn it ended on, which is only named once the bar has come to rest in it.
    ending: Option<Ending>,
    /// Recent samples, for the state a stored window is seeded from.
    hist: Hist,
    /// Strokes the bar has made since the detector started, which is the count that says whether detection got past its first stage.
    halves: u32,
}

impl Legs {
    const fn new() -> Self {
        Legs {
            v: [0.0; 3],
            prev_t_ms: 0,
            side: 0,
            prev: None,
            ending: None,
            hist: Hist::new(),
            halves: 0,
        }
    }

    /// Seconds since the previous sample, bounded so a stall in the sampler cannot become a jump in speed.
    fn step_ms(&mut self, t_ms: u32) -> f32 {
        let dt = (t_ms.saturating_sub(self.prev_t_ms) as f32 / 1000.0).min(MAX_DT_S);
        self.prev_t_ms = t_ms;
        dt
    }

    /// Take the stroke the open crossing ended, answering with the leg it made.
    ///
    /// The turn it ended on is the quietest moment in the wait, which is the instant the replay will
    /// put its own integral back to zero — the same question asked of the same velocity, so the two
    /// agree about where the bar stopped instead of each inventing an answer.
    fn take(&mut self) -> Option<Leg> {
        let ending = self.ending.take()?;
        let ended = Turn {
            ms: ending.quietest_ms,
            state: self.hist.state_at(ending.quietest_ms),
        };
        self.prev.replace(ended).map(|prev| Leg {
            start_ms: prev.ms,
            end_ms: ending.quietest_ms,
            state: prev.state,
        })
    }

    /// Feed one filtered sample; returns the leg when one ends.
    fn update(&mut self, f: &Filtered) -> Option<Leg> {
        let dt = self.step_ms(f.t_ms);
        if dt > 0.0 {
            for k in 0..3 {
                self.v[k] += f.linear_accel[k] * dt - self.v[k] * (dt / SPEED_TAU_S);
            }
        }
        let vz = self.v[2];
        let bar = f.state.velocity[2];
        self.hist.push(Note {
            t_ms: f.t_ms,
            vz: bar,
            state: f.state,
        });
        if let Some(ending) = &mut self.ending {
            if absf(bar) < ending.quietest {
                ending.quietest = absf(bar);
                ending.quietest_ms = f.t_ms;
            }
        }

        // A bar parked for longer than a set's rest is not still in the stroke that carried it there: the next stroke leaves a turn inside the rest, not back at the old one.
        // Only with no stroke in flight, because a turn is what that stroke's leg is cut from.
        if f.still
            && self.ending.is_none()
            && self
                .prev
                .is_some_and(|turn| f.t_ms.saturating_sub(turn.ms) > REST_MS)
        {
            self.prev = None;
        }

        // The stroke that has just cleared the threshold. A side cannot repeat without the speed passing zero in between, so a stroke is at most one of each.
        let side = if vz >= SWING_MPS {
            1
        } else if vz <= -SWING_MPS {
            -1
        } else {
            0
        };
        if side != 0 && side != self.side {
            self.side = side;
            self.halves += 1;
            // A stroke that has begun ends the one before it, whether or not that one was given its whole wait.
            let ended = self.take();
            if self.prev.is_none() {
                // No stroke has been taken yet, so the turn this one left is the quiet before it.
                self.prev = self.hist.turn(f.t_ms).map(|note| Turn {
                    ms: note.t_ms,
                    state: note.state,
                });
            }
            self.ending = Some(Ending {
                quietest_ms: f.t_ms,
                quietest: absf(bar),
            });
            return ended;
        }
        // The bar has come to rest, so the stroke the open crossing ended is over and its turn is the quietest moment of the rest.
        if f.still {
            return self.take();
        }
        None
    }
}

/// The crossing that ended a stroke, waiting for the quiet that says where the stroke stopped.
#[derive(Clone, Copy)]
struct Ending {
    /// The quietest moment since the crossing, and how quiet.
    quietest_ms: u32,
    quietest: f32,
}

/// Which legs are the two halves of a rep.
///
/// A rep is one leg and the leg that follows it: out, turn, and back. Which *pair* of legs is a rep is
/// the only real question, and it is a question about where the set started rather than about the
/// motion — both pairings are the same shape and give the same count, half a stroke apart. A set says
/// it the one way it can: the bar is parked at one end before the first rep, so the first stroke after
/// a rest begins a rep and the pairing follows from there. Anything carried across a set instead — a
/// parity that every stroke votes on — is a pairing a stray stroke flips for the rest of the session.
struct Reps {
    /// The leg waiting for its partner.
    pending: Option<Leg>,
    /// When the bar was last seen at rest, for how long the rest has held.
    resting_since: Option<u32>,
    /// Whether a rest long enough to end a set has happened since the last leg.
    rested: bool,
}

impl Reps {
    const fn new() -> Self {
        Reps {
            pending: None,
            resting_since: None,
            rested: false,
        }
    }

    /// Feed one sample's stillness and, when a stroke ended on it, that leg; returns the rep when one closes.
    fn update(&mut self, t_ms: u32, still: bool, leg: Option<Leg>) -> Option<DetectedRep> {
        if still {
            let since = *self.resting_since.get_or_insert(t_ms);
            if t_ms.saturating_sub(since) >= REST_MS {
                self.rested = true;
            }
        } else {
            self.resting_since = None;
        }
        let leg = leg?;
        if self.rested {
            self.pending = None;
            self.rested = false;
        }
        let Some(first) = self.pending.take() else {
            self.pending = Some(leg);
            return None;
        };
        match join(first, leg) {
            Some(rep) => Some(rep),
            // The leg that was waiting is not half of this one: it is a stroke the bar made and did not come back from, and this leg begins what follows it.
            None => {
                self.pending = Some(leg);
                None
            }
        }
    }
}

/// A rep, if these two legs are the halves of one: it has to take a rep's time.
fn join(first: Leg, second: Leg) -> Option<DetectedRep> {
    let span = RepSpan {
        start_ms: first.start_ms,
        end_ms: second.end_ms,
    };
    if !(MIN_SWING_MS..=MAX_SWING_MS).contains(&span.duration_ms()) {
        return None;
    }
    Some(DetectedRep {
        span,
        turns: RepTurns {
            start_ms: span.start_ms,
            end_ms: span.end_ms,
            zeros: [
                // The bar was parked where the rep started, so the zero goes in before the step that leaves it.
                Some(Zero::Rest(span.start_ms)),
                Some(Zero::Turn(first.end_ms)),
                Some(Zero::Turn(span.end_ms)),
            ],
        },
        from_ms: span.start_ms,
        state: first.state,
    })
}

/// The detector: the bar's strokes, and which of them are the halves of a rep.
///
/// Two stages, and the split is where the thresholds belong. The first measures a speed and cuts it at
/// its turns, which needs no opinion: every stroke the bar makes is a leg. The second decides what a
/// leg means — how long a rep takes, whether the bar came back, and where the next one begins.
///
/// Both run on the sample as it arrives, so a rep is decided a fraction of a stroke after it ended and
/// not a session later: the whole of the pending state is the one leg waiting for its partner.
pub struct Detector {
    legs: Legs,
    reps: Reps,
}

impl Detector {
    pub const fn new() -> Self {
        Detector {
            legs: Legs::new(),
            reps: Reps::new(),
        }
    }

    /// Start again from nothing at `t_ms`, as if nothing before it had been seen, with `t_ms` the moment the detector is about to be asked about again.
    ///
    /// The device pauses the detector while it streams its samples (`DevMode`), because a closed rep ends in a flash write and a flash write on the sampling task is a hole in the stream. What would carry across that pause is fiction: a stroke left open would close as one rep spanning the whole stream, a leg waiting for its partner would pair with one from the other side of it, and a step measured from the last sample before it would integrate the pause as motion. So the state goes, and the step into that next sample is zero-length rather than the gap.
    pub fn restart(&mut self, t_ms: u32) {
        *self = Self::new();
        self.legs.prev_t_ms = t_ms;
    }

    /// Strokes the bar has made since the detector started. Two of them make a rep, so this is the count that says whether detection got past its first stage.
    pub fn halves(&self) -> u32 {
        self.legs.halves
    }

    /// The bar's speed (m/s), which the legs are counted from and which the log prints beside the acceleration.
    pub fn speed(&self) -> f32 {
        magnitude(self.legs.v)
    }

    /// The bar's vertical speed (m/s), signed: the one axis the legs are counted on.
    ///
    /// [`Detector::speed`] is the whole of it, which is what the log prints; this is the axis a threshold means anything on.
    pub fn vertical(&self) -> f32 {
        self.legs.v[2]
    }

    /// Whether a rep is being built right now: one of its halves has been seen and the other has not.
    pub fn rep_open(&self) -> bool {
        self.reps.pending.is_some()
    }

    /// Feed one filtered sample; returns the rep when one closes.
    pub fn update(&mut self, f: &Filtered) -> Option<DetectedRep> {
        let leg = self.legs.update(f);
        self.reps.update(f.t_ms, f.still, leg)
    }
}

impl Default for Detector {
    fn default() -> Self {
        Self::new()
    }
}

fn absf(x: f32) -> f32 {
    if x < 0.0 {
        -x
    } else {
        x
    }
}

/// One point of a rep's **plotted** trace: where the bar was, and which way the sensor faced.
///
/// Derived, and only ever used to draw. The inputs it was derived from are the rep's raw trace, which a record-mode build stores with the rep and which is what a summary is re-derived from.
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

/// Which logic closed a rep — the filter its trace was measured with, and the detector that found it — as it travels in that rep's `RepConfig`.
///
/// Bumped when a change makes a stored summary mean something else — a threshold that moves is a different measurement wearing the same name, and two numbers from two versions are then not comparable as if they were one.
pub const DETECTOR_VERSION: u32 = 7;

impl msg::RepSummary {
    /// Whether this is a rep at all: the bar has to have gone up or down [`MIN_ROM_M`].
    ///
    /// What the detector counts is a pair of strokes that come back on each other, which says the bar moved and not how far. This is the question of how far, asked of the trace the rep carries: the log holds only summaries where that trace itself says the bar moved, and it is measured on the vertical, the only axis with a right answer.
    pub fn is_a_rep(&self) -> bool {
        self.rom >= MIN_ROM_M
    }
}

/// Compute the trace and summary for one rep by replaying the filter over its window.
///
/// `samples` should be the detector's span plus margin at each end, and `from` the state the detector answered with where that window begins ([`DetectedRep::state`]). The filter is deterministic from that state, so the same window and the same state always give the same summary — which is what makes a stored trace re-derivable.
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
