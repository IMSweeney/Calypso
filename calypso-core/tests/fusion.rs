//! The filter, on its own: what the accelerometer and gyroscope say about where the bar is and
//! which way is down.
//!
//! Everything here is a stream fed straight to [`Fusion`], so a failure names the filter rather than
//! the detector or the trace. What the *pipeline* stores is `calypso-sim`'s.

use calypso_core::fusion::{
    magnitude, quat_from_rotvec, roll_pitch_deg, rotate_by_quat, Fusion, FusionState, GRAVITY,
};
use calypso_core::sampler::{quantize, Sample, ACCEL_SCALE, GYRO_SCALE};

const DT_MS: u32 = 10;

/// A sample of a sensor at rest, mounted so that its own up axis is `mount` away from the world's.
///
/// At rest the accelerometer measures gravity and nothing else, so this is exactly `1 g` along the
/// sensor's up axis — the whole point of the raw path: a still sensor reads a number with a known
/// meaning, and anything else is the sensor's own offset.
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
/// This is the property the fused path could not offer: "linear acceleration" there is zero at rest
/// by definition, so it cannot be checked. Here the accelerometer reads gravity and the filter takes
/// its direction from it, so every mount is the same answer — including the ones beyond a quarter
/// turn, where the attitude has to be turned right over.
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
            assert!(
                magnitude(fusion.state().velocity) < 0.05,
                "{deg}° about axis {axis}: a resting bar has velocity {:?}",
                fusion.state().velocity
            );
        }
    }
}

/// An accelerometer offset is learned while the bar is still, and a still bar does not accelerate
/// because of it.
///
/// The offset arrives in the sensor's own frame, so which axis it is on must not matter — and a z
/// offset is the interesting one: a sideways one is indistinguishable from a tilt at one
/// orientation, while the vertical component has nowhere to hide.
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

/// A gyroscope that reports a rate while nothing is turning does not rotate the ground.
///
/// The attitude is integrated from the gyro, so an offset in it is a slow tilt that never stops —
/// and a tilt is gravity leaking into the horizontal, which is the acceleration a detector would
/// read as a bar being carried. The offset is learned from the same still window as the
/// accelerometer's.
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

/// A boot in the middle of a rep is not ready, and becomes ready at the first rest — never before.
///
/// There is nothing to integrate before the filter has seen gravity, so a bar that starts moving at
/// a boot has no frame at all. What matters is that it says so rather than inventing one: the gate
/// the device records through is this answer.
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
        fusion.update(&s);
        assert!(!fusion.ready(), "ready during motion at {t} ms");
    }

    for t in (3000..6000).step_by(DT_MS as usize) {
        fusion.update(&rest(t, [0.3, 0.0, 0.0], [0.0; 3], [0.0; 3]));
    }
    assert!(fusion.ready(), "not ready after three seconds at rest");
    let last = fusion.update(&rest(6000, [0.3, 0.0, 0.0], [0.0; 3], [0.0; 3]));
    assert!(
        magnitude(last.linear_accel) < 0.05,
        "after the rest a still bar reads {:?}",
        last.linear_accel
    );
}

/// A stored trace is re-derivable: the state is the filter's memory, so the same window and the same
/// state give the same run, and a run that started earlier lands in the same place.
///
/// Two claims. **Determinism** is the one the archive depends on: a summary recomputed from a trace
/// and its header is the summary that was stored, bit for bit. **Fidelity** is the one that says the
/// state is enough: a filter started from a recorded state tracks the one that never stopped. The
/// offsets are the exception and are allowed a hair — a replay has to refill its stillness window,
/// which pauses the slow tracking for 400 ms, and nothing the trace is integrated from can see it.
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

    // Past the boot, because a replay is not a boot: `from_state` takes the attitude as learned, so
    // the state a stored window carries is always one taken after the filter was ready.
    let at = 150;
    let mut live = Fusion::new();
    for s in &samples[..=at] {
        live.update(s);
    }
    let state = live.state();

    // The window starts at the sample the state was taken at, so the replay is fed that one too: it
    // is the replay's starting point rather than a step from one, so it changes nothing.
    let mut again = Fusion::from_state(state);
    let mut replay = Fusion::from_state(state);
    for s in &samples[at..] {
        let a = again.update(s);
        let r = replay.update(s);
        assert_eq!(a.state, r.state, "two replays diverged at {} ms", s.t_ms);

        let b = live.update(s);
        if s.t_ms > samples[at].t_ms {
            assert_eq!(r.state.attitude, b.state.attitude, "at {} ms", s.t_ms);
            assert_eq!(r.state.settled, b.state.settled, "at {} ms", s.t_ms);
            assert!(
                magnitude([
                    r.state.velocity[0] - b.state.velocity[0],
                    r.state.velocity[1] - b.state.velocity[1],
                    r.state.velocity[2] - b.state.velocity[2],
                ]) < 1e-4,
                "the replay's velocity left the run's at {} ms",
                s.t_ms
            );
        }
    }
    assert!(
        magnitude(replay.state().velocity) < 0.05,
        "a replay that ended at rest carries velocity {:?}",
        replay.state().velocity
    );
    assert!(matches!(state, FusionState { .. }));
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
