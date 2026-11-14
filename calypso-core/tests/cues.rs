//! The cues on their own: what the filter's output says without an integrator behind it.

use calypso_core::motion::cues::CueReader;
use calypso_core::motion::fusion::{quat_from_rotvec, Filtered, FusionState};

fn filtered(accel: [f32; 3], rate: [f32; 3], attitude: [f32; 4]) -> Filtered {
    Filtered {
        t_ms: 0,
        linear_accel: accel,
        rate,
        resting: false,
        ready: true,
        state: FusionState {
            attitude,
            ..FusionState::LEVEL
        },
    }
}

/// The same sample at `t_ms`, with `vert` as its vertical acceleration: the clock is what a contact is timed on.
fn vert_at(t_ms: u32, vert: f32) -> Filtered {
    Filtered {
        t_ms,
        ..filtered([0.0, 0.0, vert], [0.0; 3], [1.0, 0.0, 0.0, 0.0])
    }
}

#[test]
fn a_sign_change_is_the_vertical_acceleration_reversing() {
    let mut reader = CueReader::new();
    let up = filtered([0.0, 0.0, 1.0], [0.0; 3], [1.0, 0.0, 0.0, 0.0]);
    let down = filtered([0.0, 0.0, -1.0], [0.0; 3], [1.0, 0.0, 0.0, 0.0]);

    assert!(!reader.update(&up).sign_change);
    assert!(!reader.update(&up).sign_change);
    let cue = reader.update(&down);
    assert!(cue.sign_change);
    assert_eq!(cue.sign_vert, -1);
}

#[test]
fn a_deadband_is_not_a_sign_change() {
    let mut reader = CueReader::new();
    let up = filtered([0.0, 0.0, 1.0], [0.0; 3], [1.0, 0.0, 0.0, 0.0]);
    let near_zero = filtered([0.0, 0.0, 0.01], [0.0; 3], [1.0, 0.0, 0.0, 0.0]);
    let down = filtered([0.0, 0.0, -1.0], [0.0; 3], [1.0, 0.0, 0.0, 0.0]);

    assert!(!reader.update(&up).sign_change);
    assert!(!reader.update(&near_zero).sign_change);
    assert!(reader.update(&down).sign_change);
}

#[test]
fn a_landing_is_a_contact_and_a_lift_is_not() {
    let mut reader = CueReader::new();
    let mut t_ms = 1_000;

    // A stroke: vertical acceleration through a few m/s², the way a bar loaded for a lift moves.
    let mut landed = None;
    for i in 0..=100 {
        let vert = 3.0 * (i as f32 / 100.0);
        landed = landed.or(reader.update(&vert_at(t_ms, vert)).landing);
        t_ms += 10;
    }
    assert!(landed.is_none(), "a stroke is not a landing");

    // The bar reaching the ground: one sample of shock.
    let peak = t_ms;
    assert!(
        reader.update(&vert_at(peak, 27.5)).landing.is_none(),
        "a shock is only known once its acceleration has come back off"
    );
    t_ms += 10;
    assert_eq!(
        reader.update(&vert_at(t_ms, -5.0)).landing,
        Some(peak),
        "a sample that jumps 24 m/s² in 10 ms and falls 32 is the bar being stopped"
    );

    // The bounce off it is the same landing, not a second one.
    t_ms += 10;
    assert!(reader.update(&vert_at(t_ms, 25.0)).landing.is_none());
    assert!(reader.update(&vert_at(t_ms + 190, 0.0)).landing.is_none());
    let next = t_ms + 290;
    reader.update(&vert_at(next, 20.0));
    assert_eq!(
        reader.update(&vert_at(next + 10, -5.0)).landing,
        Some(next),
        "the next rep's landing is a landing"
    );
}

/// A stroke does not start from rest: the bar's acceleration is already at its peak on the first sample, and stays there. A cosine drive reads 6–15 m/s² on its first sample in the synthetic sets, which is why the cue has to see the acceleration come back off.
#[test]
fn a_stroke_that_opens_at_its_peak_is_not_a_landing() {
    let mut reader = CueReader::new();
    let mut t_ms = 1_000;
    reader.update(&vert_at(t_ms, 0.0));
    let mut landed = None;
    for i in 0..20 {
        t_ms += 10;
        let vert = 15.1 * (std::f32::consts::PI * i as f32 / 20.0).cos();
        landed = landed.or(reader.update(&vert_at(t_ms, vert)).landing);
    }
    assert!(
        landed.is_none(),
        "a drive opening at 15 m/s² and holding it is not the bar being stopped"
    );
}

#[test]
fn a_gap_in_the_stream_is_not_a_shock() {
    let mut reader = CueReader::new();
    reader.update(&vert_at(1_000, 0.0));
    reader.update(&vert_at(1_010, 0.0));
    reader.update(&vert_at(1_020, 20.0));
    assert!(
        reader.update(&vert_at(2_020, -5.0)).landing.is_none(),
        "a second of missing samples is not the bar landing"
    );
}

#[test]
fn rate_magnitude_is_the_corrected_rate_length() {
    let mut reader = CueReader::new();
    let cue = reader.update(&filtered([0.0; 3], [3.0, 4.0, 0.0], [1.0, 0.0, 0.0, 0.0]));
    assert!((cue.rate_mag - 5.0).abs() < 1e-6);
}

#[test]
fn tilt_is_the_angle_between_the_bar_and_up() {
    for deg in [0.0f32, 15.0, 45.0, 90.0] {
        let attitude = quat_from_rotvec([deg.to_radians(), 0.0, 0.0]);
        let mut reader = CueReader::new();
        let cue = reader.update(&filtered([0.0; 3], [0.0; 3], attitude));
        assert!(
            (cue.tilt.to_degrees() - deg).abs() < 0.5,
            "{deg}° tilt read as {:.1}°",
            cue.tilt.to_degrees()
        );
    }
}
