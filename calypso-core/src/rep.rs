//! Rep detection and per-rep trace computation.

use crate::fusion::{magnitude, roll_pitch_deg, Filtered, Fusion, FusionState};
use crate::pb::msg;
use crate::sampler::{quantize, Sample, QUAT_SCALE};

/// Speed (m/s) the bar has to reach going each way for a stroke to be a leg.
pub const SWING_MPS: f32 = 0.30;
/// Time constant (s) over which the speed is pulled back toward zero.
pub const SPEED_TAU_S: f32 = 1.0;
/// Shortest rep (ms): the turn it started at to the turn it ended at.
pub const MIN_SWING_MS: u32 = 400;
/// Longest rep (ms): past this the bar is being carried rather than lifted. The pauses at either end of a stroke are inside it, which is what makes it longer than a stroke.
pub const MAX_SWING_MS: u32 = 8000;
/// How long the bar has to be at rest before a stroke begins a rep rather than ending one.
pub const REST_MS: u32 = 10_000;
/// How far past the span a stored window runs.
pub const MARGIN_MS: u32 = 500;
/// Sample times and filter states the detector keeps, so the state a stored window is seeded from can be found after the fact: `MARGIN_MS` of samples, plus headroom.
const HIST: usize = 64;
/// How many points a rep's **plotted** trace keeps, however long the rep is.
pub const TRACE_POINTS: usize = 128;
/// Smallest dominant-axis travel (metres) a stored summary can report and still be called a rep.
pub const MIN_ROM_M: f32 = 0.07;
/// Longest integration step honoured, so a stalled sampler cannot turn a gap in the samples into a jump in speed.
const MAX_DT_S: f32 = 0.1;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RepTurns {
    /// The rep's own span: the turn the bar came into it from.
    pub start_ms: u32,
    /// The rep's own end: the turn the bar came back to.
    pub end_ms: u32,
    /// Where the speed reached zero at each of the three instants that delimit the rep, oldest first, where a cue could say.
    pub zeros: [Option<Zero>; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DetectedRep {
    pub span: RepSpan,
    pub turns: RepTurns,
    /// Where the stored window starts, which is the turn the rep opened at: the samples before it are what the filter had to settle on, and the window's own margin is at its end.
    pub from_ms: u32,
    /// The filter state where that window starts, which is what the replay is seeded with.
    pub state: FusionState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Zero {
    /// The bar was already at rest when this sample's step began — an onset, or a hold. The step is the motion that follows the instant, so the zero goes in before it is integrated.
    Rest(u32),
    /// The bar's speed reached zero at this sample: it turned. The step carries the deceleration that led to the turn, so the zero goes in after it is integrated.
    Turn(u32),
}

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

    fn slots(&self) -> impl DoubleEndedIterator<Item = usize> + '_ {
        let oldest = (self.head + HIST - self.len) % HIST;
        (0..self.len).map(move |i| (oldest + i) % HIST)
    }

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

#[derive(Clone, Copy)]
struct Turn {
    ms: u32,
    state: FusionState,
}

#[derive(Debug, Clone, Copy)]
struct Leg {
    start_ms: u32,
    end_ms: u32,
    /// The filter state where the stroke started, which is what a stored window is seeded with.
    state: FusionState,
}
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

    fn step_ms(&mut self, t_ms: u32) -> f32 {
        let dt = (t_ms.saturating_sub(self.prev_t_ms) as f32 / 1000.0).min(MAX_DT_S);
        self.prev_t_ms = t_ms;
        dt
    }

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

        if f.still
            && self.ending.is_none()
            && self
                .prev
                .is_some_and(|turn| f.t_ms.saturating_sub(turn.ms) > REST_MS)
        {
            self.prev = None;
        }

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
            let ended = self.take();
            if self.prev.is_none() {
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
        if f.still {
            return self.take();
        }
        None
    }
}

#[derive(Clone, Copy)]
struct Ending {
    quietest_ms: u32,
    quietest: f32,
}
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
            None => {
                self.pending = Some(leg);
                None
            }
        }
    }
}

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
                Some(Zero::Rest(span.start_ms)),
                Some(Zero::Turn(first.end_ms)),
                Some(Zero::Turn(span.end_ms)),
            ],
        },
        from_ms: span.start_ms,
        state: first.state,
    })
}

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

    pub fn restart(&mut self, t_ms: u32) {
        *self = Self::new();
        self.legs.prev_t_ms = t_ms;
    }

    pub fn halves(&self) -> u32 {
        self.legs.halves
    }

    pub fn speed(&self) -> f32 {
        magnitude(self.legs.v)
    }

    pub fn vertical(&self) -> f32 {
        self.legs.v[2]
    }

    pub fn rep_open(&self) -> bool {
        self.reps.pending.is_some()
    }

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

    pub fn as_slice(&self) -> &[TracePoint] {
        &self.points[..self.len]
    }

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

pub const DETECTOR_VERSION: u32 = 7;

impl msg::RepSummary {
    pub fn is_a_rep(&self) -> bool {
        self.rom >= MIN_ROM_M
    }
}

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
    let first = if capacity == 0 { 0 } else { (n - 1) % stride };

    let mut fusion = Fusion::from_state(*from);
    let mut p = [0.0f32; 3];
    let mut v_prev = from.velocity;
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
    let (span_start, span_end) = turns.map_or((u32::MIN, u32::MAX), |t| (t.start_ms, t.end_ms));
    let mut v_max = 0.0f32;
    let mut lo = [f32::MAX; 3];
    let mut hi = [f32::MIN; 3];
    let mut roll_offset = 0.0f32;
    let mut roll_prev = 0.0f32;
    let mut roll_seen = false;

    for i in 0..n {
        while next_before < before.len() && samples[i].t_ms >= before[next_before] {
            fusion.zero_velocity();
            next_before += 1;
        }
        let filtered = fusion.update(&samples[i]);
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

    summary.rom = if hi[2] >= lo[2] { hi[2] - lo[2] } else { 0.0 };
    summary.t_start_ms = samples[0].t_ms;
    summary.t_end_ms = samples[n - 1].t_ms;
    summary.duration_ms = summary.t_end_ms - summary.t_start_ms;
    summary.peak_velocity = v_max;
    summary.computed_len = out.len() as u32;
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
