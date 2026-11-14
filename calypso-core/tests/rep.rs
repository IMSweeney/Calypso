//! A rest ends the stroke as well as the rep: a set may open the way the last one closed.
//!
//! A stroke is a stroke because the bar has to clear `SWING_MPS` the other way to leave it, and a bench press leaves in the same direction it arrives: down to the chest, up, down again. A detector that carries that direction across a rest reads the first descent of a set as the tail of the stroke before it, so the first rep is stored a stroke late: ascent then descent, starting at the bottom.

use calypso_core::motion::cues::Cues;
use calypso_core::motion::fusion::{Filtered, FusionState};
use calypso_core::motion::integrate::Integrated;
use calypso_core::motion::rep::{DetectedRep, Detector};

const DT_MS: u32 = 10;

struct Script {
    detector: Detector,
    t_ms: u32,
    reps: Vec<DetectedRep>,
}

impl Script {
    fn new() -> Self {
        Script {
            detector: Detector::new(),
            t_ms: 0,
            reps: Vec::new(),
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
        let steps = ms / DT_MS;
        for i in 0..=steps {
            let u = i as f32 / steps as f32;
            self.sample(peak * (1.0 - (2.0 * u - 1.0).powi(2)), false);
        }
    }

    fn sample(&mut self, velocity: f32, resting: bool) {
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
        let cues = Cues {
            sign_vert: 0,
            rate_mag: 0.0,
            tilt: 0.0,
            sign_change: false,
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
