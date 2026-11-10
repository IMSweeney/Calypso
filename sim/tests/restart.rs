//! What the detector does when the device stops asking it: `DevMode` pauses detection for as long as it streams, and what the detector picks up afterwards must not describe the pause.
//!
//! The two things that would otherwise cross it are a leg left open — which would close as one rep spanning the whole stream — and a first step measured from the last sample before it, which would integrate minutes of nothing as motion.

use calypso_core::fusion::{Filtered, FusionState};
use calypso_core::rep::Detector;

const DT_MS: u32 = 10;

/// One filtered sample as the detector reads it: `linear` is the bar's own acceleration, gravity already out.
fn sample(t_ms: u32, linear: [f32; 3], still: bool) -> Filtered {
    Filtered {
        t_ms,
        linear_accel: linear,
        still,
        state: FusionState::LEVEL,
    }
}

fn rest(det: &mut Detector, t: &mut u32, ms: u32) {
    for _ in 0..ms / DT_MS {
        det.update(&sample(*t, [0.0; 3], true));
        *t += DT_MS;
    }
}

#[test]
fn a_pause_ends_the_leg_it_interrupted_instead_of_stretching_it() {
    let mut det = Detector::new();
    let mut t = 0;
    rest(&mut det, &mut t, 2000);
    // A drive, still moving when the stream is asked for.
    for _ in 0..300 / DT_MS {
        det.update(&sample(t, [0.0, 0.0, 4.0], false));
        t += DT_MS;
    }
    assert!(det.leg_open(), "the drive is a leg");

    det.restart(t);

    assert!(!det.leg_open(), "the leg ended with the pause");
    assert_eq!(det.legs(), 0, "and it was not closed as a rep of its own");
}

#[test]
fn the_first_sample_after_a_pause_is_not_a_step() {
    let mut det = Detector::new();
    // The device restarts on the tick that notices the switch, so the sample it is about to be asked about carries that tick's own timestamp.
    det.restart(600_000);
    det.update(&sample(600_000, [0.0, 0.0, 4.0], false));

    assert_eq!(
        det.speed(),
        0.0,
        "the pause is not a step between the last sample before it and the first after"
    );
}
