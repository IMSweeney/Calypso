//! The integrator on its own: acceleration in, velocity out, and what an anchor does to it.

use calypso_core::motion::integrate::{Anchor, Integration};

const DT_MS: u32 = 10;

/// Feed `ms` of `accel` and return the last motion.
fn drive(integration: &mut Integration, from_ms: u32, ms: u32, accel: f32) -> f32 {
    let mut t = from_ms;
    for _ in 0..ms / DT_MS {
        t += DT_MS;
        integration.step(t, [accel, 0.0, 0.0], false, true, &[]);
    }
    integration.velocity()[0]
}

#[test]
fn velocity_is_the_integral_of_acceleration() {
    let mut integration = Integration::new();
    // The first sample starts the clock rather than covering a step.
    integration.step(0, [0.0; 3], false, true, &[]);
    let v = drive(&mut integration, 0, 500, 2.0);
    assert!((v - 1.0).abs() < 1e-4, "half a second at 2 m/s² is {v}");
}

#[test]
fn a_hole_is_one_capped_step() {
    let mut integration = Integration::new();
    integration.step(0, [0.0; 3], false, true, &[]);
    // Ten seconds without a sample is not ten seconds of acceleration.
    let m = integration.step(10_000, [1.0, 0.0, 0.0], false, true, &[]);
    assert!(
        (m.velocity[0] - 0.1).abs() < 1e-4,
        "a ten second gap integrated {} m/s",
        m.velocity[0]
    );
}

#[test]
fn a_rest_anchor_zeroes_velocity_before_the_step() {
    let mut integration = Integration::new();
    integration.step(0, [0.0; 3], false, true, &[]);
    drive(&mut integration, 0, 1_000, 1.0);

    // The motion that follows a resting instant is integrated from rest, so the step reads one sample of acceleration.
    let m = integration.step(1_010, [1.0, 0.0, 0.0], false, true, &[Anchor::rest(1_010)]);
    assert!(
        (m.velocity[0] - 0.01).abs() < 1e-5,
        "a step after a rest anchor left {:.3} m/s",
        m.velocity[0]
    );
}

#[test]
fn a_turn_anchor_zeroes_velocity_after_the_step() {
    let mut integration = Integration::new();
    integration.step(0, [0.0; 3], false, true, &[]);
    drive(&mut integration, 0, 1_000, 1.0);

    // The step carries the deceleration that led to the turn, and the zero lands on top of it.
    let m = integration.step(1_010, [1.0, 0.0, 0.0], false, true, &[Anchor::turn(1_010)]);
    assert_eq!(m.velocity[0], 0.0, "a turn left the bar moving");
}

#[test]
fn an_anchor_only_applies_at_its_own_sample() {
    let mut integration = Integration::new();
    integration.step(0, [0.0; 3], false, true, &[]);
    drive(&mut integration, 0, 1_000, 1.0);

    let m = integration.step(1_010, [1.0, 0.0, 0.0], false, true, &[Anchor::rest(1_000)]);
    assert!(
        m.velocity[0] > 1.0,
        "an anchor from an earlier sample zeroed this one: {:.3}",
        m.velocity[0]
    );
}
