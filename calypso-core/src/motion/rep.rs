//! Rep detection and set validation.

use crate::motion::cues::Cues;
use crate::motion::fusion::{Filtered, FusionState};
use crate::motion::integrate::{Anchor, AnchorKind, Integrated};
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
/// Rotation rate (rad/s) a lifted bar does not reach, so time spent above it is the bar being carried.
const CARRIED_RATE: f32 = 1.0;
/// How much of a rep may be spent above [`CARRIED_RATE`] before it is the bar being carried. A lifted bar stays under this through every stroke of the recorded sets; loading a clip onto one does not.
const CARRIED_MS: u32 = 350;
/// Longest step a sample is credited with, so a gap in the stream cannot be counted as time spent turning.
const STEP_CAP_MS: u32 = 100;
/// How much of a rep off the ground has to be spent accelerating up before it is a lift rather than the bar being put somewhere.
///
/// A deadlift's five sets spend 1.4–4.3 s of each rep accelerating up; handling the bar between sets does not.
const LIFTED_MS: u32 = 500;

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

/// Which end of a stroke a turn is, and how the bar came to zero there.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TurnEnd {
    /// The bar was already still: a rest, and the bottom a stroke begins from.
    Still,
    /// The bar was stopped by what it landed on, which is the bottom of a lift off the floor.
    Landed,
    /// The bar turned in the air, which is the top of a stroke.
    Reversed,
}

impl TurnEnd {
    /// How the integration zeroes the bar at this turn.
    fn anchor(self) -> AnchorKind {
        match self {
            // A landing's deceleration is inside the step that lands it; anything else was still before its step began.
            TurnEnd::Landed => AnchorKind::Turn,
            _ => AnchorKind::Rest,
        }
    }
}

#[derive(Clone, Copy)]
struct Turn {
    ms: u32,
    state: RepSeed,
    /// How the bar came to zero here, which is also which end of a stroke it is.
    end: TurnEnd,
}

#[derive(Debug, Clone, Copy)]
struct Leg {
    start_ms: u32,
    end_ms: u32,
    /// The state where the stroke started, which is what a stored window is seeded with.
    state: RepSeed,
    /// How the stroke began, which is how the window it seeds is anchored.
    start_end: TurnEnd,
    /// How the stroke ended: on the ground, in the air, or at a rest.
    end_end: TurnEnd,
    /// Time the stroke spent turning faster than a lift does, which says whether the bar was carried through it.
    carried_ms: u32,
    /// Time the bar spent accelerating up through the stroke, which is what a lift has and handling does not.
    lifted_ms: u32,
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
    /// Time the stroke in progress has spent turning faster than a lift does.
    carried_ms: u32,
    /// Time the stroke in progress has spent accelerating up.
    lifted_ms: u32,
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
            carried_ms: 0,
            lifted_ms: 0,
        }
    }

    /// Forget the stroke in progress, so the next one starts from wherever the bar is now.
    fn start_afresh(&mut self) {
        self.prev = None;
        self.side = 0;
        self.carried_ms = 0;
        self.lifted_ms = 0;
        self.ending = None;
    }

    /// Close the stroke in progress on the turn it ended at, which `end` says how the bar arrived at.
    fn take(&mut self, end: TurnEnd) -> Option<Leg> {
        let carried_ms = core::mem::take(&mut self.carried_ms);
        let lifted_ms = core::mem::take(&mut self.lifted_ms);
        let ending = self.ending.take()?;
        if ending.quietest > TURN_QUIET_MPS && ending.cue_ms.is_none() {
            return None;
        }
        let ended = Turn {
            ms: ending.quietest_ms,
            state: self.hist.state_at(ending.quietest_ms),
            end,
        };
        let leg = self.prev.replace(ended).map(|prev| Leg {
            start_ms: prev.ms,
            end_ms: ending.quietest_ms,
            state: prev.state,
            start_end: prev.end,
            end_end: end,
            carried_ms,
            lifted_ms,
        });
        leg
    }

    fn update(&mut self, f: &Filtered, m: &Integrated, cues: &Cues) -> Option<Leg> {
        let vz = m.v_cross[2];
        let bar = m.velocity[2];
        let dt_ms = m.t_ms.saturating_sub(self.last.t_ms).min(STEP_CAP_MS);
        if cues.rate_mag > CARRIED_RATE {
            self.carried_ms = self.carried_ms.saturating_add(dt_ms);
        }
        if cues.sign_vert > 0 {
            self.lifted_ms = self.lifted_ms.saturating_add(dt_ms);
        }
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

        if let Some(landed_ms) = cues.landing {
            // The bar was stopped by what it landed on: the turn this stroke ended on is that instant, and the next begins there at zero.
            self.side = 0;
            self.ending = Some(Ending {
                quietest_ms: landed_ms,
                quietest: 0.0,
                cue_ms: Some(landed_ms),
            });
            return self.take(TurnEnd::Landed);
        }

        if f.resting
            && self.ending.is_none()
            && self
                .prev
                .is_some_and(|turn| f.t_ms.saturating_sub(turn.ms) > REST_MS)
        {
            // A rest ends the stroke too, so the next one counts whichever way it goes.
            self.start_afresh();
        }
        if self
            .prev
            .is_some_and(|turn| turn.end == TurnEnd::Landed && f.t_ms - turn.ms > REST_MS)
        {
            // The bar has been down on what it landed on long enough that the stroke off it is over.
            self.start_afresh();
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
            // A stroke that began on the ground only ends on the ground: what it passes through in the air is not a turn.
            let off_the_floor = self.prev.is_some_and(|turn| turn.end == TurnEnd::Landed);
            let ended = if off_the_floor {
                None
            } else {
                self.take(if side > 0 {
                    TurnEnd::Still
                } else {
                    TurnEnd::Reversed
                })
            };
            if self.prev.is_none() {
                self.prev = self.hist.turn(f.t_ms).map(|note| Turn {
                    ms: note.t_ms,
                    state: note.state,
                    end: TurnEnd::Still,
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
            if self.prev.is_some_and(|turn| turn.end == TurnEnd::Landed) {
                // The bar is on the ground it landed on: that stroke ends when it leaves and lands again.
                return None;
            }
            let ended = self.take(TurnEnd::Still);
            // The stroke ended here, so the next one starts at the rest rather than at the turn this one came from.
            if ended.is_some() {
                self.prev = Some(Turn {
                    ms: f.t_ms,
                    state: self.hist.state_at(f.t_ms),
                    end: TurnEnd::Still,
                });
            }
            return ended;
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
        if leg.start_end == TurnEnd::Landed && leg.end_end == TurnEnd::Landed {
            // A stroke from one landing to the next is a whole lift, so it waits for no partner.
            return whole(leg);
        }
        if leg.start_end == TurnEnd::Landed {
            // A stroke that came off the ground and did not land again is not a lift.
            return None;
        }
        let Some(first) = self.pending.take() else {
            self.pending = Some(leg);
            return None;
        };
        if first.carried_ms + leg.carried_ms > CARRIED_MS {
            // The bar was turned to get it there, so the strokes either side of that are not one lift.
            return None;
        }
        match join(first, leg) {
            Some(rep) => Some(rep),
            None => {
                self.pending = Some(leg);
                None
            }
        }
    }
}

/// The rep a stroke makes on its own: off the bottom of one landing and back onto the bottom of the next.
///
/// Its middle is not a turn the detector saw — the bar passes through the top of the lift without stopping — so the trace is left to find where it went.
fn whole(leg: Leg) -> Option<DetectedRep> {
    let span = RepSpan {
        start_ms: leg.start_ms,
        end_ms: leg.end_ms,
    };
    if leg.carried_ms > CARRIED_MS
        || leg.lifted_ms < LIFTED_MS
        || !(MIN_SWING_MS..=MAX_SWING_MS).contains(&span.duration_ms())
    {
        return None;
    }
    Some(DetectedRep {
        span,
        turns: RepTurns {
            start_ms: span.start_ms,
            end_ms: span.end_ms,
            zeros: [
                Some(Anchor::at(span.start_ms, leg.start_end.anchor())),
                None,
                Some(Anchor::at(span.end_ms, leg.end_end.anchor())),
            ],
        },
        from_ms: span.start_ms,
        state: leg.state,
    })
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
                Some(Anchor::at(span.start_ms, first.start_end.anchor())),
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

pub const DETECTOR_VERSION: u32 = 12;

impl msg::RepSummary {
    pub fn is_a_rep(&self) -> bool {
        self.rom >= MIN_ROM_M
    }
}
