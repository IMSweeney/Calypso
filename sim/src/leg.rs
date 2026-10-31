//! Prototype: a leg / turning-point rep detector, for comparison with `calypso_core::rep::Detector`.
//!
//! The model: a rep is an out-and-back excursion. The bar travels more than `MIN_ROM_M` one way,
//! turns, and travels more than `MIN_ROM_M` back. A *leg* is the displacement between two turns:
//! the bar's velocity drops below `VEL_MIN` for `T_TURN_MS` at each turn, and the signed vertical
//! displacement over the leg is what decides whether it is big enough to be half a rep.
//!
//! Pauses before and during a stroke are flat spots — the velocity is simply low — so they do not
//! split a leg. A one-way push (an unrack) is one leg and never becomes a rep. A stall mid-stroke
//! splits a leg, but the two same-direction halves are merged unless the bar came to a real rest
//! between them (a hold, not a grind), which is the one place this model still needs `T_REST_MS`.
//!
//! This mirrors `Detector`'s interface (`update` -> `Option<RepSpan>`, `rep_v0`) and is a simulation
//! prototype: no fixed-point containers, no flash, no firmware constraints.

use calypso_core::pb::msg;
use calypso_core::rep::{
    accel_magnitude, compute_rep, rotate_by_quat, RepSpan, TraceBuf, TracePoint,
    ACCEL_ON_THRESHOLD, ACCEL_REST_THRESHOLD, MARGIN_MS, MAX_REP_MS, MIN_REP_MS, MIN_ROM_M,
    T_ON_MS, T_REST_MS, T_TURN_MS, VEL_MIN,
};
use calypso_core::sampler::{Sample, SampleRing};

use crate::motion::{Motion, TRACE_POINTS};

/// Ring size used when replaying samples, matching `fixtures`.
const REPLAY_RING: usize = 4096;
/// One window big enough for the longest rep plus both margins, matching `fixtures`.
const WINDOW_SAMPLES: usize = 900;

/// Time constant of the accelerometer's bias estimate, matching the firmware detector.
const BIAS_TAU_S: f32 = 3.0;
/// Longest integration step honoured, matching the firmware detector.
const MAX_DT_S: f32 = 0.1;
/// Time constant over which a resting velocity is pulled to zero (the ZUPT anchor).
const V_TAU_S: f32 = 0.5;
/// Samples of history kept so a leg can name its displacement between two past instants.
const HIST: usize = 64;

#[derive(Debug, Clone, Copy)]
struct Leg {
    start_ms: u32,
    end_ms: u32,
    /// Signed vertical displacement over the leg.
    disp_z: f32,
    /// Whether the bar came to rest in the gap before this leg (a hold, not a grind).
    had_rest: bool,
    /// The bar's velocity at the leg's left edge, less one margin.
    v0: [f32; 3],
}

pub struct LegDetector {
    v: [f32; 3],
    bias: [f32; 3],
    prev_t_ms: u32,
    /// Displacement accumulated since boot; only differences are ever used.
    pos: [f32; 3],
    /// The open leg, if any, and where it began.
    leg_start: Option<u32>,
    leg_pos0: [f32; 3],
    leg_v0: [f32; 3],
    leg_had_rest: bool,
    // Turn detection: velocity below VEL_MIN for T_TURN_MS after having moved.
    moved: bool,
    still_since: Option<u32>,
    above_since: Option<u32>,
    // A real rest in the gap between legs (a hold, not a grind).
    rest_since: Option<u32>,
    gap_rest: bool,
    /// The last completed leg not yet paired into a rep.
    pending: Option<Leg>,
    // History for looking up velocity/displacement at a past sample.
    hist_t: [u32; HIST],
    hist_v: [[f32; 3]; HIST],
    hist_pos: [[f32; 3]; HIST],
    hist_head: usize,
    v0: [f32; 3],
}

impl LegDetector {
    pub const fn new() -> Self {
        LegDetector {
            v: [0.0; 3],
            bias: [0.0; 3],
            prev_t_ms: 0,
            pos: [0.0; 3],
            leg_start: None,
            leg_pos0: [0.0; 3],
            leg_v0: [0.0; 3],
            leg_had_rest: false,
            moved: false,
            still_since: None,
            above_since: None,
            rest_since: None,
            gap_rest: false,
            pending: None,
            hist_t: [0; HIST],
            hist_v: [[0.0; 3]; HIST],
            hist_pos: [[0.0; 3]; HIST],
            hist_head: 0,
            v0: [0.0; 3],
        }
    }

    /// The bar's velocity at the left edge of the rep that just closed.
    pub fn rep_v0(&self) -> [f32; 3] {
        self.v0
    }

    /// Feed one sample; returns the span when a rep closes.
    pub fn update(&mut self, s: &Sample) -> Option<RepSpan> {
        let mag = accel_magnitude(s);
        let dt = ((s.t_ms.saturating_sub(self.prev_t_ms)) as f32 / 1000.0).min(MAX_DT_S);
        self.prev_t_ms = s.t_ms;
        let v_prev = self.v;

        let a = rotate_by_quat(s.quat_f32(), s.accel_f32());
        let quiet = mag < ACCEL_REST_THRESHOLD;
        if dt > 0.0 {
            for k in 0..3 {
                self.v[k] += (a[k] - self.bias[k]) * dt;
            }
            if quiet {
                for k in 0..3 {
                    self.bias[k] += (a[k] - self.bias[k]) * (dt / BIAS_TAU_S);
                    // Only anchor the velocity while the bar is still: at the midpoint of a stroke
                    // the acceleration crosses zero while the velocity is at its peak.
                    if !self.moved {
                        self.v[k] += (0.0 - self.v[k]) * (dt / V_TAU_S);
                    }
                }
            }
            for k in 0..3 {
                self.pos[k] += 0.5 * (v_prev[k] + self.v[k]) * dt;
            }
        }

        self.hist_t[self.hist_head] = s.t_ms;
        self.hist_v[self.hist_head] = self.v;
        self.hist_pos[self.hist_head] = self.pos;
        self.hist_head = (self.hist_head + 1) % HIST;

        // A real rest: a hold, not a grind.
        if quiet {
            let since = *self.rest_since.get_or_insert(s.t_ms);
            if s.t_ms.saturating_sub(since) >= T_REST_MS {
                if self.leg_start.take().is_some() {
                    // The bar has not moved yet, so a hold means the next leg starts when it does.
                    self.moved = false;
                    self.still_since = None;
                }
                self.gap_rest = true;
            }
        } else {
            self.rest_since = None;
        }

        let speed =
            libm::sqrtf(self.v[0] * self.v[0] + self.v[1] * self.v[1] + self.v[2] * self.v[2]);

        if self.leg_start.is_none() {
            // Looking for a motion onset.
            if mag > ACCEL_ON_THRESHOLD {
                let since = *self.above_since.get_or_insert(s.t_ms);
                if s.t_ms.saturating_sub(since) >= T_ON_MS {
                    self.leg_start = Some(since);
                    self.leg_pos0 = self.pos_at(since);
                    self.leg_v0 = self.velocity_at(since.saturating_sub(MARGIN_MS));
                    self.leg_had_rest = self.gap_rest;
                    self.gap_rest = false;
                    self.moved = false;
                    self.still_since = None;
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
            let since = *self.still_since.get_or_insert(s.t_ms);
            if s.t_ms.saturating_sub(since) >= T_TURN_MS {
                let start = self.leg_start.take().unwrap();
                let leg = Leg {
                    start_ms: start,
                    end_ms: since,
                    disp_z: self.pos_at(since)[2] - self.leg_pos0[2],
                    had_rest: self.leg_had_rest,
                    v0: self.leg_v0,
                };
                // The bar turns, it does not restart: the next leg begins where this one ended.
                self.leg_start = Some(since);
                self.leg_pos0 = self.pos_at(since);
                self.leg_v0 = self.velocity_at(since.saturating_sub(MARGIN_MS));
                self.leg_had_rest = false;
                self.moved = false;
                self.still_since = None;
                return self.pair(leg);
            }
        }
        None
    }

    /// Pair a completed leg with the pending one: opposite directions, both past `MIN_ROM_M`, is
    /// a rep. Same direction with no rest between them is one stroke split by a stall (merge);
    /// same direction after a rest is a stale one-way push (drop it).
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
            // Captured where the leg opened: by now the history has moved past it.
            self.v0 = p.v0;
            if span.duration_ms() >= MIN_REP_MS && span.duration_ms() <= MAX_REP_MS {
                return Some(span);
            }
            return None;
        }

        let next = if !opposite && !leg.had_rest {
            // A stall split the stroke: fuse the halves.
            Leg {
                start_ms: p.start_ms,
                end_ms: leg.end_ms,
                disp_z: p.disp_z + leg.disp_z,
                had_rest: p.had_rest,
                v0: p.v0,
            }
        } else {
            leg
        };
        if next.disp_z.abs() >= MIN_ROM_M {
            self.pending = Some(next);
        } else {
            self.pending = None;
        }
        None
    }

    fn pos_at(&self, t_ms: u32) -> [f32; 3] {
        let mut newest: Option<(u32, [f32; 3])> = None;
        for i in 0..HIST {
            let t = self.hist_t[i];
            if t <= t_ms && newest.map_or(true, |(bt, _)| t > bt) {
                newest = Some((t, self.hist_pos[i]));
            }
        }
        newest.map_or([0.0; 3], |(_, p)| p)
    }

    fn velocity_at(&self, t_ms: u32) -> [f32; 3] {
        let mut newest: Option<(u32, [f32; 3])> = None;
        for i in 0..HIST {
            let t = self.hist_t[i];
            if t <= t_ms && newest.map_or(true, |(bt, _)| t > bt) {
                newest = Some((t, self.hist_v[i]));
            }
        }
        newest.map_or([0.0; 3], |(_, v)| v)
    }
}

/// The summaries the leg detector would store for this motion — the [`crate::motion::stored`]
/// question, asked of the other pipeline.
pub fn leg_stored(motion: &Motion) -> Vec<msg::RepSummary> {
    leg_detect_and_trace(&motion.samples(), TRACE_POINTS)
        .into_iter()
        .map(|(summary, _)| summary)
        .filter(|s| s.is_a_rep())
        .collect()
}

/// Replay samples through the leg detector, then the same ZUPT-anchored trace the firmware uses.
/// Mirrors `crate::fixtures::detect_and_trace` so a scenario can be run through both detectors.
pub fn leg_detect_and_trace(
    samples: &[Sample],
    trace_capacity: usize,
) -> Vec<(msg::RepSummary, Vec<TracePoint>)> {
    let mut ring_buf = vec![Sample::ZERO; REPLAY_RING];
    let mut ring = SampleRing::new(&mut ring_buf);
    let mut det = LegDetector::new();
    let mut window = vec![Sample::ZERO; WINDOW_SAMPLES];
    let mut storage = vec![TracePoint::ZERO; trace_capacity];
    let mut out = Vec::new();

    for s in samples {
        ring.push(*s);
        if let Some(span) = det.update(s) {
            let n = ring.copy_window(
                span.start_ms,
                span.end_ms,
                MARGIN_MS,
                MARGIN_MS,
                &mut window,
            );
            if n == 0 {
                continue;
            }
            let mut trace = TraceBuf::new(&mut storage);
            let summary = compute_rep(&window[..n], det.rep_v0(), &mut trace);
            out.push((summary, trace.as_slice().to_vec()));
        }
    }
    out
}
