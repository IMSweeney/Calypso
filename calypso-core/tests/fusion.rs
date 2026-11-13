//! The filter, on its own: what the accelerometer and gyroscope say about where the bar is and which way is down.
//!
//! Everything here is a stream fed straight to [`Fusion`], so a failure names the filter rather than the detector or the trace. Velocity is the integrator's, and is tested there. What the *pipeline* stores is `calypso-sim`'s.

use calypso_core::motion::fusion::{
    conjugate, magnitude, quat_from_rotvec, roll_pitch_deg, rotate_by_quat, Fusion, FusionState,
    GRAVITY,
};
use calypso_core::motion::integrate::Integration;
use calypso_core::sampler::{quantize, Sample, ACCEL_SCALE, GYRO_SCALE};

const DT_MS: u32 = 10;

/// A sample of a sensor at rest, mounted so that its own up axis is `mount` away from the world's.
///
/// At rest the accelerometer measures gravity and nothing else, so this is exactly `1 g` along the sensor's up axis — the whole point of the raw path: a still sensor reads a number with a known meaning, and anything else is the sensor's own offset.
fn rest(t_ms: u32, mount: [f32; 3], offset: [f32; 3], gyro_dps: [f32; 3]) -> Sample {
    let up = rotate_by_quat(quat_from_rotvec(mount), [0.0, 0.0, 1.0]);
    Sample {
        t_ms,
        accel: [
            up[0] * GRAVITY + offset[0],
            up[1] * GRAVITY + offset[1],
            up[2] * GRAVITY + offset[2],
        ]
        .map(|a| quantize(a, ACCEL_SCALE)),
        gyro: gyro_dps.map(|g| quantize(g, GYRO_SCALE)),
    }
}

/// Feed `ms` of rest and return the filter it left and its last reading.
fn settle(mount: [f32; 3], offset: [f32; 3], gyro_dps: [f32; 3], ms: u32) -> (Fusion, [f32; 3]) {
    let mut fusion = Fusion::new();
    let mut last = [f32::MAX; 3];
    for t in (0..ms).step_by(DT_MS as usize) {
        last = fusion
            .update(&rest(t, mount, offset, gyro_dps))
            .linear_accel;
    }
    (fusion, last)
}

/// A sensor at rest reads nothing, whatever angle it is bolted on at.
///
/// This is the property the fused path could not offer: "linear acceleration" there is zero at rest by definition, so it cannot be checked. Here the accelerometer reads gravity and the filter takes its direction from it, so every mount is the same answer — including the ones beyond a quarter turn, where the attitude has to be turned right over.
#[test]
fn a_still_sensor_reads_nothing_at_any_attitude() {
    for axis in 0..3 {
        for deg in [0.0f32, 30.0, 90.0, 120.0, 180.0, 270.0] {
            let mut mount = [0.0; 3];
            mount[axis] = deg.to_radians();
            let (fusion, last) = settle(mount, [0.0; 3], [0.0; 3], 3000);
            assert!(
                fusion.ready(),
                "{deg}° about axis {axis}: the filter never found gravity"
            );
            assert!(
                magnitude(last) < 0.05,
                "{deg}° about axis {axis}: a resting sensor reads {last:?} m/s²"
            );
        }
    }
}

/// A still sensor at a 90° mount whose up axis reads 9.5 g, not 9.81 g, still finds its frame.
///
/// A chip read raw carries per-axis offsets of a few hundred mg, so `still`'s ±0.12 g magnitude check fails on a sensor whose up axis is short by 0.3 m/s². The frame must still converge on gravity's *direction*, which an offset does not corrupt: a still 90° mount left wrong by this double-integrates metres of travel. The filter starts already wrong, as a replay of a window whose stored attitude is, so it has to pull onto the direction rather than be handed it.
#[test]
fn a_ninety_degree_mount_with_a_magnitude_offset_recovers_its_frame() {
    // Gravity along the sensor's Y axis, and a 0.3 m/s² shortfall on that same axis: the reading the capture's broken window held, |f| ≈ 9.5 g.
    let mount = [core::f32::consts::FRAC_PI_2, 0.0, 0.0];
    let up = rotate_by_quat(quat_from_rotvec(mount), [0.0, 0.0, 1.0]);
    let offset = up.map(|c| c * -0.3);

    // Ready but not resting, with no window behind it: what a stored state replays from.
    let mut fusion = Fusion::from_state(FusionState::LEVEL_READY);
    let mut last = [0.0f32; 3];
    for t in (0..10000).step_by(DT_MS as usize) {
        last = fusion
            .update(&rest(t, mount, offset, [0.0; 3]))
            .linear_accel;
    }

    // The frame pulled onto gravity's direction: the filter's up is the mount's up.
    let frame_up = rotate_by_quat(conjugate(fusion.state().attitude), [0.0, 0.0, 1.0]);
    let aligned = frame_up[0] * up[0] + frame_up[1] * up[1] + frame_up[2] * up[2];
    assert!(
        aligned > 0.9,
        "the frame is {:.0}° off gravity at a still 90° mount (dot {aligned:.2})",
        aligned.min(1.0).acos().to_degrees()
    );
    assert!(
        fusion.state().resting,
        "a still bar at a 90° mount never passed the stillness gate"
    );
    assert!(
        magnitude(last) < 0.05,
        "a still bar at a 90° mount still reads {last:?} m/s²"
    );
}

/// An accelerometer offset is learned while the bar is still, and a still bar does not accelerate because of it.
///
/// The offset arrives in the sensor's own frame, so which axis it is on must not matter — and a z offset is the interesting one: a sideways one is indistinguishable from a tilt at one orientation, while the vertical component has nowhere to hide.
#[test]
fn a_constant_offset_is_learned_from_the_still_window() {
    for offset in [
        [0.0f32, 0.0, 0.35],
        [0.35, 0.0, 0.0],
        [0.0, -0.3, 0.0],
        [0.05, -0.35, 0.04],
    ] {
        let (fusion, last) = settle([0.0; 3], offset, [0.0; 3], 3000);
        assert!(
            fusion.ready(),
            "offset {offset:?}: the filter never found gravity"
        );
        assert!(
            magnitude(last) < 0.05,
            "offset {offset:?}: a resting sensor reads {last:?} m/s²"
        );
    }
}

/// A confirmed rest window is enough to replay from: it fixes the frame and both offsets before the first step.
///
/// Detection confirms rest once; the filter should not have to spend the replay's first second relearning what that window already says. The last samples are used, so a longer window adds nothing and a short one is still a seed.
#[test]
fn a_confirmed_rest_window_seeds_the_frame_and_both_offsets() {
    let mount = [core::f32::consts::FRAC_PI_2, 0.0, 0.0];
    let offset = [0.21, -0.3, 0.12];
    let gyro = [2.0, -1.0, 0.5];

    for window_ms in [150, 400] {
        let mut samples = Vec::new();
        for t in (0..window_ms).step_by(DT_MS as usize) {
            samples.push(rest(t, mount, offset, gyro));
        }
        let mut fusion = Fusion::from_rest(&samples).expect("a rest window seeds the filter");
        let next = fusion.update(&rest(window_ms, mount, offset, gyro));

        assert!(next.ready, "{window_ms} ms: the seed is not ready");
        assert!(
            magnitude(next.linear_accel) < 0.05,
            "{window_ms} ms: a still bar reads {:?} m/s² after the seed",
            next.linear_accel
        );
        let learned = fusion.state().gyro_bias;
        for (k, dps) in gyro.iter().enumerate() {
            assert!(
                (learned[k] - dps.to_radians()).abs() < 1e-3,
                "{window_ms} ms: axis {k} learned {:.3} dps, not {dps}",
                learned[k].to_degrees()
            );
        }
    }
}

/// A window with no gravity in it is not a rest seed.
#[test]
fn a_window_without_gravity_is_not_a_rest_seed() {
    assert!(Fusion::from_rest(&[]).is_none(), "no samples, no seed");
    let no_force = [Sample {
        t_ms: 0,
        accel: [0; 3],
        gyro: [0; 3],
    }];
    assert!(
        Fusion::from_rest(&no_force).is_none(),
        "a window reading no force cannot say which way is down"
    );
}

/// A gyroscope that reports a rate while nothing is turning does not rotate the ground.
///
/// The attitude is integrated from the gyro, so an offset in it is a slow tilt that never stops — and a tilt is gravity leaking into the horizontal, which is the acceleration a detector would read as a bar being carried. The offset is learned from the same still window as the accelerometer's.
#[test]
fn a_gyro_offset_does_not_rotate_gravity() {
    for axis in 0..3 {
        let mut gyro = [0.0; 3];
        gyro[axis] = 3.0;
        let (fusion, last) = settle([0.0; 3], [0.0; 3], gyro, 10000);
        assert!(
            fusion.ready(),
            "3 dps on axis {axis}: the filter never found gravity"
        );
        assert!(
            magnitude(last) < 0.05,
            "3 dps on axis {axis}: ten seconds of rest reads {last:?} m/s²"
        );
    }
}

/// A gyro bias that arrives after boot is learned during the set, not only at its rests.
///
/// A bar moving up and down has no angular rate and leaves gravity pointing the same way in the sensor's frame, so a set teaches the gyro even between reps. Without that, a degree per second left in the attitude walks the vertical over a set, and the horizontal velocity that leaks from it is metres per second the bar never had.
#[test]
fn a_gyro_bias_that_arrives_during_a_set_does_not_walk_the_ground() {
    let mut fusion = Fusion::new();
    let mut integration = Integration::new();
    let mut t = 0u32;
    let mut max_h = 0.0f32;
    let mut max_tilt = 0.0f32;

    // 1.6 s of lead-in rest, then twelve vertical reps with the pauses a bench has.
    for _ in 0..160 {
        feed(
            &mut fusion,
            &mut integration,
            t,
            0.0,
            0.0,
            &mut max_h,
            &mut max_tilt,
        );
        t += DT_MS;
    }
    for _ in 0..12 {
        let bias = if t < 5000 { 0.0 } else { 1.0 };
        let travel = 0.19f32;
        let speed = 0.55f32;
        let duration = core::f32::consts::PI * travel / (2.0 * speed);
        let steps = (duration / (DT_MS as f32 / 1000.0)).round() as u32;
        let peak = 2.0 * speed * speed / travel;
        for dir in [1.0f32, -1.0] {
            for i in 0..=steps {
                let accel =
                    dir * peak * libm::cosf(core::f32::consts::PI * i as f32 / steps as f32);
                feed(
                    &mut fusion,
                    &mut integration,
                    t,
                    accel,
                    bias,
                    &mut max_h,
                    &mut max_tilt,
                );
                t += DT_MS;
            }
            let pause = if dir > 0.0 { 150 } else { 300 };
            for _ in 0..(pause / DT_MS) {
                let bias = if t < 5000 { 0.0 } else { 1.0 };
                feed(
                    &mut fusion,
                    &mut integration,
                    t,
                    0.0,
                    bias,
                    &mut max_h,
                    &mut max_tilt,
                );
                t += DT_MS;
            }
        }
    }

    assert!(
        max_h < 1.0,
        "a vertical set with a 1 dps bias carried {max_h:.2} m/s sideways"
    );
    assert!(max_tilt < 2.0, "the ground tilted {max_tilt:.1}°");
    let learned = fusion.state().gyro_bias[0].to_degrees();
    assert!(
        (learned - 1.0).abs() < 0.2,
        "the bias was learned as {learned:.2} dps, not 1"
    );
}

/// Feed one vertical sample and record the sideways speed and the tilt it left.
fn feed(
    fusion: &mut Fusion,
    integration: &mut Integration,
    t_ms: u32,
    accel: f32,
    gyro_dps: f32,
    max_h: &mut f32,
    max_tilt: &mut f32,
) {
    let s = Sample::from_f32(t_ms, [0.0, 0.0, GRAVITY + accel], [gyro_dps, 0.0, 0.0]);
    let f = fusion.update(&s);
    let m = integration.step(f.t_ms, f.linear_accel, f.resting, f.ready, &[]);
    let v = m.velocity;
    *max_h = max_h.max((v[0] * v[0] + v[1] * v[1]).sqrt());
    let (roll, pitch) = roll_pitch_deg(f.state.attitude);
    *max_tilt = max_tilt.max((roll * roll + pitch * pitch).sqrt());
}

/// A boot in the middle of a rep is not ready, and becomes ready at the first rest — never before.
///
/// There is nothing to integrate before the filter has seen gravity, so a bar that starts moving at a boot has no frame at all. What matters is that it says so rather than inventing one: the gate the device records through is this answer.
#[test]
fn a_boot_in_motion_is_not_ready_until_the_bar_stops() {
    let mut fusion = Fusion::new();
    for t in (0..3000).step_by(DT_MS as usize) {
        // A stroke's own acceleration on top of a tilted gravity vector.
        let up = rotate_by_quat(quat_from_rotvec([0.3, 0.0, 0.0]), [0.0, 0.0, 1.0]);
        let stroke = 4.0 * libm::cosf(t as f32 / 300.0);
        let s = Sample {
            t_ms: t,
            accel: [up[0] * GRAVITY, up[1] * GRAVITY, up[2] * GRAVITY + stroke]
                .map(|a| quantize(a, ACCEL_SCALE)),
            gyro: [0; 3],
        };
        let f = fusion.update(&s);
        assert!(
            !f.ready,
            "a filtered sample was ready during motion at {t} ms"
        );
        assert!(!fusion.ready(), "ready during motion at {t} ms");
    }

    for t in (3000..6000).step_by(DT_MS as usize) {
        fusion.update(&rest(t, [0.3, 0.0, 0.0], [0.0; 3], [0.0; 3]));
    }
    assert!(fusion.ready(), "not ready after three seconds at rest");
    let last = fusion.update(&rest(6000, [0.3, 0.0, 0.0], [0.0; 3], [0.0; 3]));
    assert!(
        last.ready,
        "a filtered sample stayed unready after the rest"
    );
    assert!(
        magnitude(last.linear_accel) < 0.05,
        "after the rest a still bar reads {:?}",
        last.linear_accel
    );
}

/// A stored trace is re-derivable: the state is the filter's memory, so the same window and the same state give the same run, and a run that started earlier lands in the same place.
///
/// Two claims. **Determinism** is the one the archive depends on: a summary recomputed from a trace and its header is the summary that was stored, bit for bit. **Fidelity** is the one that says the state is enough: a filter started from a recorded state tracks the one that never stopped. The offsets are the exception and are allowed a hair — a replay has to refill its stillness window, which pauses the slow tracking for 400 ms, and nothing the trace is integrated from can see it.
#[test]
fn a_replay_from_a_state_reproduces_the_run() {
    let mut samples = Vec::new();
    for t in (0..6000).step_by(DT_MS as usize) {
        // Still, then a lift, then still: the shape of a rep, so the state has a velocity to carry.
        let stroke = if (2000..4000).contains(&t) {
            3.0 * libm::cosf((t - 2000) as f32 / 300.0)
        } else {
            0.0
        };
        samples.push(Sample {
            t_ms: t,
            accel: [0.0, 0.0, GRAVITY + stroke].map(|a| quantize(a, ACCEL_SCALE)),
            gyro: [0; 3],
        });
    }

    // Past the boot, because a replay is not a boot: `from_state` takes the attitude as learned, so the state a stored window carries is always one taken after the filter was ready.
    let at = 150;
    let mut live = Fusion::new();
    let mut live_motion = Integration::new();
    for s in &samples[..=at] {
        let f = live.update(s);
        live_motion.step(f.t_ms, f.linear_accel, f.resting, f.ready, &[]);
    }
    let state = live.state();
    let velocity = live_motion.velocity();

    // The window starts at the sample the state was taken at, so the replay is fed that one too: it is the replay's starting point rather than a step from one, so it changes nothing.
    let mut again = Fusion::from_state(state);
    let mut again_motion = Integration::from_velocity(velocity);
    let mut replay = Fusion::from_state(state);
    let mut replay_motion = Integration::from_velocity(velocity);
    for s in &samples[at..] {
        let a = again.update(s);
        again_motion.step(a.t_ms, a.linear_accel, a.resting, a.ready, &[]);
        let r = replay.update(s);
        let rm = replay_motion.step(r.t_ms, r.linear_accel, r.resting, r.ready, &[]);
        assert_eq!(a.state, r.state, "two replays diverged at {} ms", s.t_ms);

        let b = live.update(s);
        let bm = live_motion.step(b.t_ms, b.linear_accel, b.resting, b.ready, &[]);
        if s.t_ms > samples[at].t_ms {
            assert_eq!(r.state.attitude, b.state.attitude, "at {} ms", s.t_ms);
            assert_eq!(r.state.resting, b.state.resting, "at {} ms", s.t_ms);
            assert!(
                magnitude([
                    rm.velocity[0] - bm.velocity[0],
                    rm.velocity[1] - bm.velocity[1],
                    rm.velocity[2] - bm.velocity[2],
                ]) < 1e-4,
                "the replay's velocity left the run's at {} ms",
                s.t_ms
            );
        }
    }
    assert!(
        magnitude(replay_motion.velocity()) < 0.05,
        "a replay that ended at rest carries velocity {:?}",
        replay_motion.velocity()
    );
}

#[test]
fn a_sample_the_sensor_cannot_have_read_does_not_move_the_bar() {
    let mut fusion = Fusion::new();
    let mut integration = Integration::new();
    let mut t = 0u32;
    for _ in 0..300 {
        let f = fusion.update(&rest(t, [0.0; 3], [0.0; 3], [0.0; 3]));
        integration.step(f.t_ms, f.linear_accel, f.resting, f.ready, &[]);
        t += DT_MS;
    }

    // The chip is configured for 8 g, so a specific force of 20 g is a corrupt register or a corrupt chunk rather than a bar.
    let spike = Sample {
        t_ms: t,
        accel: [0, 0, quantize(20.0 * GRAVITY, ACCEL_SCALE)],
        gyro: [0; 3],
    };
    let f = fusion.update(&spike);
    integration.step(f.t_ms, f.linear_accel, f.resting, f.ready, &[]);
    assert!(
        magnitude(integration.velocity()) < 0.05,
        "one unreadable sample left the bar moving at {:.2} m/s",
        magnitude(integration.velocity())
    );
    let f = fusion.update(&rest(t + DT_MS, [0.0; 3], [0.0; 3], [0.0; 3]));
    integration.step(f.t_ms, f.linear_accel, f.resting, f.ready, &[]);
    assert!(
        f.resting,
        "an unreadable sample broke the stillness of the window around it"
    );
}

#[test]
fn magnitude_matches_vector_length() {
    assert!((magnitude([3.0, 4.0, 0.0]) - 5.0).abs() < 1e-6);
    assert!((magnitude([0.0, 0.0, -9.0]) - 9.0).abs() < 1e-6);
    assert_eq!(magnitude([0.0; 3]), 0.0);
}

#[test]
fn a_rotation_of_no_angle_is_the_identity() {
    assert_eq!(quat_from_rotvec([0.0, 0.0, 0.0]), [1.0, 0.0, 0.0, 0.0]);
}

#[test]
fn rotate_by_quat_maps_body_to_world() {
    let s = core::f32::consts::FRAC_1_SQRT_2;
    let q = [s, s, 0.0, 0.0];
    let v = rotate_by_quat(q, [0.0, 1.0, 0.0]);
    assert!((v[0]).abs() < 1e-5);
    assert!((v[1]).abs() < 1e-5);
    assert!((v[2] - 1.0).abs() < 1e-5);
    assert_eq!(
        rotate_by_quat([1.0, 0.0, 0.0, 0.0], [1.0, 2.0, 3.0]),
        [1.0, 2.0, 3.0]
    );
}

#[test]
fn roll_pitch_from_quaternion() {
    let r = 30f32.to_radians() / 2.0;
    let (roll, pitch) = roll_pitch_deg([libm::cosf(r), libm::sinf(r), 0.0, 0.0]);
    assert!((roll - 30.0).abs() < 0.5, "roll {roll}");
    assert!(pitch.abs() < 0.5, "pitch {pitch}");
    let p = 20f32.to_radians() / 2.0;
    let (roll, pitch) = roll_pitch_deg([libm::cosf(p), 0.0, libm::sinf(p), 0.0]);
    assert!(roll.abs() < 0.5, "roll {roll}");
    assert!((pitch - 20.0).abs() < 0.5, "pitch {pitch}");
}
