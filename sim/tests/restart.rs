//! What the detector does when the device stops asking it: `DevMode` pauses detection for as long as it streams, and what the detector picks up afterwards must not describe the pause.
//!
//! The two things that would otherwise cross it are a stroke left open — whose turn would pair with a stroke from the far side of the stream as one rep spanning it — and a first step measured from the last sample before it, which would integrate minutes of nothing as motion.

use calypso_core::motion::fusion::{Filtered, FusionState};
use calypso_core::motion::integrate::Integration;
use calypso_core::motion::rep::Detector;

const DT_MS: u32 = 10;

/// One filtered sample as the detector reads it: `linear` is the bar's own acceleration, gravity already out.
fn sample(t_ms: u32, linear: [f32; 3], resting: bool) -> Filtered {
    Filtered {
        t_ms,
        linear_accel: linear,
        rate: [0.0; 3],
        resting,
        ready: true,
        state: FusionState::LEVEL,
    }
}

fn rest(det: &mut Detector, integration: &mut Integration, t: &mut u32, ms: u32) {
    for _ in 0..ms / DT_MS {
        let f = sample(*t, [0.0; 3], true);
        let m = integration.step(f.t_ms, f.linear_accel, f.resting, f.ready, &[]);
        det.update(&f, &m);
        *t += DT_MS;
    }
}

#[test]
fn a_pause_ends_the_stroke_it_interrupted_instead_of_stretching_it() {
    let mut det = Detector::new();
    let mut integration = Integration::new();
    let mut t = 0;
    rest(&mut det, &mut integration, &mut t, 2000);
    // A drive, still moving when the stream is asked for.
    for _ in 0..300 / DT_MS {
        let f = sample(t, [0.0, 0.0, 4.0], false);
        let m = integration.step(f.t_ms, f.linear_accel, f.resting, f.ready, &[]);
        det.update(&f, &m);
        t += DT_MS;
    }
    assert_eq!(det.halves(), 1, "the drive is a stroke the bar made");

    det.restart();
    integration.restart(t);

    assert_eq!(det.halves(), 0, "the stroke ended with the pause");
    assert!(
        !det.rep_open(),
        "and nothing was left waiting for a partner across it"
    );
}

#[test]
fn the_first_sample_after_a_pause_is_not_a_step() {
    let mut det = Detector::new();
    let mut integration = Integration::new();
    // The device restarts on the tick that notices the switch, so the sample it is about to be asked about carries that tick's own timestamp.
    det.restart();
    integration.restart(600_000);
    let f = sample(600_000, [0.0, 0.0, 4.0], false);
    let m = integration.step(f.t_ms, f.linear_accel, f.resting, f.ready, &[]);
    det.update(&f, &m);

    assert_eq!(
        det.vertical(),
        0.0,
        "the pause is not a step between the last sample before it and the first after"
    );
}
