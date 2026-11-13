//! Rep detection and set validation.

use crate::motion::cues::Cues;
use crate::motion::fusion::{Filtered, FusionState};
use crate::motion::integrate::{Anchor, Integrated};
use crate::pb::msg;

/// Speed (m/s) the bar has to reach going each way for a stroke to be a leg.
pub const SWING_MPS: f32 = 0.30;
/// Speed (m/s) the integrated velocity must get under for the crossing alone to confirm a turn.
const TURN_QUIET_MPS: f32 = SWING_MPS;
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
/// Smallest dominant-axis travel (metres) a stored summary can report and still be called a rep.
pub const MIN_ROM_M: f32 = 0.07;

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

/// What a replay of a window starts from: the filter's snapshot and the velocity the bar was carrying.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RepSeed {
    pub filter: FusionState,
    pub velocity: [f32; 3],
}

impl RepSeed {
    /// A level seed that has already found gravity, which is what a replay of a stored window starts from.
    pub const LEVEL: RepSeed = RepSeed {
        filter: FusionState::LEVEL_READY,
        velocity: [0.0; 3],
    };
}

impl Default for RepSeed {
    fn default() -> Self {
        Self::LEVEL
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RepTurns {
    /// The rep's own span: the turn the bar came into it from.
    pub start_ms: u32,
    /// The rep's own end: the turn the bar came back to.
    pub end_ms: u32,
    /// Where the speed reached zero at each of the three instants that delimit the rep, oldest first, where a cue could say.
    pub zeros: [Option<Anchor>; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DetectedRep {
    pub span: RepSpan,
    pub turns: RepTurns,
    /// Where the stored window starts, which is the turn the rep opened at: the samples before it are what the filter had to settle on, and the window's own margin is at its end.
    pub from_ms: u32,
    /// The state where that window starts, which is what the replay is seeded with.
    pub state: RepSeed,
}

#[derive(Clone, Copy)]
struct Note {
    t_ms: u32,
    vz: f32,
    state: RepSeed,
}

impl Note {
    const ZERO: Note = Note {
        t_ms: 0,
        vz: 0.0,
        state: RepSeed::LEVEL,
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

    fn state_at(&self, t_ms: u32) -> RepSeed {
        let mut newest: Option<(u32, RepSeed)> = None;
        let mut oldest: Option<(u32, RepSeed)> = None;
        for i in self.slots() {
            let note = self.notes[i];
            if oldest.is_none_or(|(bt, _)| note.t_ms < bt) {
                oldest = Some((note.t_ms, note.state));
            }
            if note.t_ms <= t_ms && newest.is_none_or(|(bt, _)| note.t_ms > bt) {
                newest = Some((note.t_ms, note.state));
            }
        }
        newest.or(oldest).map_or(RepSeed::LEVEL, |(_, s)| s)
    }
}

#[derive(Clone, Copy)]
struct Turn {
    ms: u32,
    state: RepSeed,
}

#[derive(Debug, Clone, Copy)]
struct Leg {
    start_ms: u32,
    end_ms: u32,
    /// The state where the stroke started, which is what a stored window is seeded with.
    state: RepSeed,
}
struct Legs {
    /// The last motion the detector saw, for the readouts a caller takes between samples.
    last: Integrated,
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
            last: Integrated::ZERO,
            side: 0,
            prev: None,
            ending: None,
            hist: Hist::new(),
            halves: 0,
        }
    }

    fn take(&mut self) -> Option<Leg> {
        let ending = self.ending.take()?;
        if ending.quietest > TURN_QUIET_MPS && ending.cue_ms.is_none() {
            return None;
        }
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

    fn update(&mut self, f: &Filtered, m: &Integrated, cues: &Cues) -> Option<Leg> {
        let vz = m.v_cross[2];
        let bar = m.velocity[2];
        self.last = *m;
        self.hist.push(Note {
            t_ms: f.t_ms,
            vz: bar,
            state: RepSeed {
                filter: f.state,
                velocity: m.velocity,
            },
        });
        if let Some(ending) = &mut self.ending {
            if cues.sign_change && ending.cue_ms.is_none() {
                ending.cue_ms = Some(f.t_ms);
            }
            if absf(bar) < ending.quietest {
                ending.quietest = absf(bar);
                ending.quietest_ms = f.t_ms;
            }
        }

        if f.resting
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
                cue_ms: None,
            });
            return ended;
        }
        if f.resting {
            return self.take();
        }
        None
    }
}

#[derive(Clone, Copy)]
struct Ending {
    quietest_ms: u32,
    quietest: f32,
    /// The first acceleration reversal since the crossing, which is the cue that confirms the turn.
    cue_ms: Option<u32>,
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

    fn update(&mut self, t_ms: u32, resting: bool, leg: Option<Leg>) -> Option<DetectedRep> {
        if resting {
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
                Some(Anchor::rest(span.start_ms)),
                Some(Anchor::turn(first.end_ms)),
                Some(Anchor::turn(span.end_ms)),
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

    pub fn restart(&mut self) {
        *self = Self::new();
    }

    pub fn halves(&self) -> u32 {
        self.legs.halves
    }

    pub fn vertical(&self) -> f32 {
        self.legs.last.velocity[2]
    }

    pub fn rep_open(&self) -> bool {
        self.reps.pending.is_some()
    }

    pub fn update(&mut self, f: &Filtered, m: &Integrated, cues: &Cues) -> Option<DetectedRep> {
        let leg = self.legs.update(f, m, cues);
        self.reps.update(f.t_ms, f.resting, leg)
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

pub const DETECTOR_VERSION: u32 = 8;

impl msg::RepSummary {
    pub fn is_a_rep(&self) -> bool {
        self.rom >= MIN_ROM_M
    }
}
