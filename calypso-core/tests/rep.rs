//! What counts as a rep.
//!
//! A stroke is a stroke because the bar has to clear `SWING_MPS` the other way to leave it, and a bench press leaves in the same direction it arrives: down to the chest, up, down again. A detector that carries that direction across a rest reads the first descent of a set as the tail of the stroke before it, so the first rep is stored a stroke late: ascent then descent, starting at the bottom.
//!
//! A rep is also the bar being lifted, and a bar that is being carried, loaded or adjusted is turned through angles a lift does not reach. Those strokes are the ones that read as reps when the bar is handled between sets.

use calypso_core::motion::cues::Cues;
use calypso_core::motion::fusion::{Filtered, FusionState};
use calypso_core::motion::integrate::Integrated;
use calypso_core::motion::rep::{DetectedRep, Detector};

const DT_MS: u32 = 10;

struct Script {
    detector: Detector,
    t_ms: u32,
    reps: Vec<DetectedRep>,
    /// The speed the sample before carried, so the change in it is the acceleration's sign.
    last_velocity: f32,
}

impl Script {
    fn new() -> Self {
        Script {
            detector: Detector::new(),
            t_ms: 0,
            reps: Vec::new(),
            last_velocity: 0.0,
        }
    }

    /// `resting` is the filter's own gate: a bar held under load is not at rest.
    fn still(&mut self, ms: u32, resting: bool) {
        for _ in 0..ms / DT_MS {
            self.sample(0.0, resting);
        }
    }

    /// One half-cosine stroke, out and back to rest, the way a bar actually travels.
    fn stroke(&mut self, ms: u32, peak: f32) {
        self.stroke_with(ms, peak, |_| 0.0);
    }

    /// One stroke that turns at `rate_mag` for `wobble_ms` about its middle and holds level either side.
    fn stroke_wobbling(&mut self, ms: u32, peak: f32, wobble_ms: u32, rate_mag: f32) {
        self.stroke_with(ms, peak, move |u| {
            let at_ms = u * ms as f32;
            let mid = ms as f32 * 0.5;
            if (at_ms - mid).abs() <= wobble_ms as f32 * 0.5 {
                rate_mag
            } else {
                0.0
            }
        });
    }

    /// One stroke, with the rate each sample carries given by `rate` against the stroke's own progress.
    fn stroke_with(&mut self, ms: u32, peak: f32, rate: impl Fn(f32) -> f32) {
        let steps = ms / DT_MS;
        for i in 0..=steps {
            let u = i as f32 / steps as f32;
            let velocity = peak * (1.0 - (2.0 * u - 1.0).powi(2));
            self.sample_at(velocity, false, rate(u));
        }
    }

    fn sample(&mut self, velocity: f32, resting: bool) {
        self.sample_at(velocity, resting, 0.0);
    }

    /// One sample of the bar being stopped by what it landed on: the filter sees a shock here.
    fn landing(&mut self) {
        let at = self.t_ms;
        self.sample_with(0.0, false, 0.0, Some(at));
    }

    fn sample_at(&mut self, velocity: f32, resting: bool, rate_mag: f32) {
        self.sample_with(velocity, resting, rate_mag, None);
    }

    fn sample_with(&mut self, velocity: f32, resting: bool, rate_mag: f32, landing: Option<u32>) {
        let f = Filtered {
            t_ms: self.t_ms,
            linear_accel: [0.0; 3],
            rate: [0.0; 3],
            resting,
            ready: true,
            state: FusionState::LEVEL_READY,
        };
        let m = Integrated {
            t_ms: self.t_ms,
            velocity: [0.0, 0.0, velocity],
            v_cross: [0.0, 0.0, velocity],
        };
        let accel = velocity - self.last_velocity;
        self.last_velocity = velocity;
        let sign_vert = if accel > 0.0 {
            1
        } else if accel < 0.0 {
            -1
        } else {
            0
        };
        let cues = Cues {
            sign_vert,
            rate_mag,
            tilt: 0.0,
            sign_change: false,
            landing,
        };
        if let Some(rep) = self.detector.update(&f, &m, &cues) {
            self.reps.push(rep);
        }
        self.t_ms += DT_MS;
    }
}

#[test]
fn the_first_stroke_after_a_rest_counts_whichever_way_it_goes() {
    let mut s = Script::new();
    s.still(300, false);
    s.stroke(1000, -0.5);
    s.still(300, false);
    s.stroke(1000, 0.5);
    s.still(300, false);
    s.stroke(1000, -0.5);
    s.still(400, false);

    let set_began = s.t_ms;
    s.still(11_000, true);

    let top = s.t_ms;
    s.stroke(1000, -0.5);
    s.still(400, false);
    s.stroke(1000, 0.5);
    s.still(400, false);
    s.stroke(1000, -0.5);
    s.still(400, false);
    s.stroke(1000, 0.5);
    s.still(500, false);

    let first = s
        .reps
        .iter()
        .find(|rep| rep.span.start_ms >= set_began)
        .expect("the set made a rep");

    assert!(
        first.span.start_ms < top + 200,
        "the set opens at the top ({top} ms) with a descent, so its first rep starts there, not at {} ms",
        first.span.start_ms
    );
    let middle = first.turns.zeros[1].expect("a rep has a middle").t_ms;
    assert!(
        (top + 700..=top + 1_300).contains(&middle),
        "the middle of the first rep is the bottom of its descent, {}-{} ms, not {middle} ms",
        top + 700,
        top + 1_300
    );
}

/// Three strokes, so two legs and one rep, each carrying the rate `rate` says.
fn three_strokes(rate: impl Fn(f32) -> f32) -> Script {
    let mut s = Script::new();
    s.still(300, false);
    for peak in [-0.5, 0.5, -0.5] {
        s.stroke_with(1000, peak, &rate);
        s.still(300, false);
    }
    s.still(500, false);
    s
}

#[test]
fn a_stroke_the_bar_is_turned_through_is_not_a_lift() {
    let carried = three_strokes(|_| 4.0);
    assert!(
        carried.reps.is_empty(),
        "a bar turned through the whole stroke is being carried, so it made no rep"
    );

    let lifted = three_strokes(|_| 0.0);
    assert_eq!(
        lifted.reps.len(),
        1,
        "the same three strokes held level are one rep"
    );
}

#[test]
fn a_pause_at_the_top_begins_the_next_rep() {
    let mut s = Script::new();
    s.still(300, false);
    s.stroke(1000, -0.5);
    s.still(300, false);
    s.stroke(1000, 0.5);
    s.still(300, false);

    let pause = s.t_ms;
    s.still(5_000, true);

    s.stroke(1000, -0.5);
    s.still(300, false);
    s.stroke(1000, 0.5);
    s.still(300, false);
    s.still(2_000, true);

    assert_eq!(
        s.reps.len(),
        2,
        "two strokes either side of a pause are two reps"
    );
    let second = s.reps[1];
    assert!(
        second.span.start_ms >= pause,
        "the rep after the pause starts at {pause} ms, where the bar came to rest, not at {} ms, the turn before it",
        second.span.start_ms
    );
}

#[test]
fn a_bar_put_down_turns_where_it_lands() {
    let mut s = Script::new();
    s.still(300, false);
    // A lift: up, and down fast enough that the detector sees the top.
    s.stroke(1000, 0.5);
    s.stroke(1000, -0.5);
    // And a lowering too slow for the swing threshold, which ends by the bar reaching the ground.
    s.stroke(400, -0.15);
    let landing = s.t_ms;
    s.landing();
    s.still(300, false);
    // The next lift, landing the same way.
    s.stroke(1000, 0.5);
    s.stroke(1000, -0.5);
    s.stroke(400, -0.15);
    let next_landing = s.t_ms;
    s.landing();
    s.still(500, false);

    let first = s.reps.first().expect("two lifts made a rep");
    assert_eq!(
        first.span.end_ms, landing,
        "the rep ends at {landing} ms, where the bar landed, not at {} ms",
        first.span.end_ms
    );
    assert_eq!(
        first.turns.zeros.last().and_then(|a| *a).map(|a| a.t_ms),
        Some(landing),
        "the landing is the turn the rep is anchored on"
    );

    let second = s.reps.get(1).expect("the lift after a landing made a rep");
    assert_eq!(
        second.span.start_ms, landing,
        "the next rep starts where the bar landed, not at {} ms",
        second.span.start_ms
    );
    assert_eq!(
        second.span.end_ms, next_landing,
        "and ends where it landed again"
    );
}

#[test]
fn a_momentary_wobble_inside_a_stroke_is_still_a_lift() {
    let mut s = Script::new();
    s.still(300, false);
    for peak in [-0.5, 0.5, -0.5] {
        s.stroke_wobbling(1000, peak, 100, 4.0);
        s.still(300, false);
    }
    s.still(500, false);

    assert_eq!(
        s.reps.len(),
        1,
        "a tenth of a second of turning inside a stroke is a wobble, not handling"
    );
}
