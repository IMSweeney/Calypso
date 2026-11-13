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
