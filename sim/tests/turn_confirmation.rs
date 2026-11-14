//! A crossing that never gets quiet is a turn only when the acceleration says it is.

use calypso_core::motion::cues::Cues;
use calypso_core::motion::fusion::{Filtered, FusionState};
use calypso_core::motion::integrate::Integrated;
use calypso_core::motion::rep::Detector;

const DT_MS: u32 = 10;

fn sample(t_ms: u32, velocity: f32, cue: bool) -> (Filtered, Integrated, Cues) {
    let f = Filtered {
        t_ms,
        linear_accel: [0.0; 3],
        rate: [0.0; 3],
        resting: false,
        ready: true,
        state: FusionState::LEVEL,
    };
    let m = Integrated {
        t_ms,
        velocity: [0.0, 0.0, velocity],
        v_cross: [0.0, 0.0, velocity],
    };
    let cues = Cues {
        sign_vert: 0,
        rate_mag: 0.0,
        tilt: 0.0,
        sign_change: cue,
        landing: None,
    };
    (f, m, cues)
}

/// Two strokes whose speed never falls under the swing threshold are not a rep unless the acceleration reverses.
///
/// The integrated speed can carry a bias through a stroke and never reach zero; the acceleration's sign change is then the only evidence the bar turned. Without it, the crossing is a bump, not a turn.
#[test]
fn a_turn_the_speed_never_confirms_needs_the_acceleration_to() {
    for cue in [false, true] {
        let mut det = Detector::new();
        let mut t = 0u32;
        let mut reps = 0;

        // Two seconds of rest, then a stroke each way without the speed ever getting quiet.
        for _ in 0..200 {
            let (f, m, c) = sample(t, 0.0, false);
            det.update(&f, &m, &c);
            t += DT_MS;
        }
        for v in [0.5f32, -0.5, 0.5] {
            for _ in 0..60 {
                let (f, m, c) = sample(t, v, cue);
                reps += usize::from(det.update(&f, &m, &c).is_some());
                t += DT_MS;
            }
        }
        for _ in 0..50 {
            let (f, m, c) = sample(t, 0.0, false);
            reps += usize::from(det.update(&f, &m, &c).is_some());
            t += DT_MS;
        }

        assert_eq!(
            reps,
            usize::from(cue),
            "cue {cue}: {reps} reps from two strokes that never got quiet"
        );
    }
}
