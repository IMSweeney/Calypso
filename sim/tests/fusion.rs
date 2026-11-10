//! Pins for the filter's effect on what the device stores.
//!
//! These are the things the fused orientation could not do and the raw path must: a mount is never square to the bar, a sensor carries offsets, and a boot can happen anywhere — including mid-rep.
//! Each scenario is synthetic motion plus one impairment, replayed through the whole pipeline, so a failure here is a stored summary changing rather than a filter internals changing.
//!
//! The filter's own arithmetic is pinned in `calypso-core/tests/fusion.rs`; what is here is what reaches the log.

use calypso_core::sampler::Sample;
use calypso_sim::fixtures::{at, detect_and_trace, LEAD_IN_MS, SAMPLE_DT_MS};
use calypso_sim::motion::{Impairments, Rep, Rng, TRACE_POINTS};

/// The summaries the device would store for a stream: those that clear the travel a rep needs.
fn stored(samples: &[Sample]) -> Vec<calypso_core::pb::msg::RepSummary> {
    detect_and_trace(samples, TRACE_POINTS)
        .into_iter()
        .map(|(summary, _)| summary)
        .filter(|s| s.is_a_rep())
        .collect()
}

fn impaired(samples: &[Sample], i: Impairments) -> Vec<Sample> {
    let mut out = samples.to_vec();
    let mut rng = Rng::new(0x5eed);
    i.apply(&mut out, &mut rng);
    out
}

/// A sensor bolted to the bar at an angle measures the same motion in another frame, and the travel it reads back has to be the bar's, not the mount's.
///
/// A fusion in IMU mode got this wrong in the direction that costs the most: roll and pitch came partly from the accelerometer, so a stroke's own acceleration pulled the estimate off, and five of fourteen captured reps had less vertical travel than sideways. Here the gyro carries the tilt and the accelerometer only supplies gravity's direction, so a mount at 90° costs nothing: it is one constant attitude the filter learns at boot and integrates from.
#[test]
fn a_mount_tilt_does_not_change_the_travel() {
    let travel = Rep::bench().travel;
    let upright = Rep::bench().samples();
    for axis in 0..3 {
        for deg in [0.0f32, 30.0, 90.0] {
            let mut tilt = [0.0; 3];
            tilt[axis] = deg;
            let samples = impaired(
                &upright,
                Impairments {
                    tilt_deg: tilt,
                    ..Default::default()
                },
            );
            let reps = stored(&samples);
            assert_eq!(
                reps.len(),
                1,
                "tilt {deg}° about axis {axis}: {} reps stored",
                reps.len()
            );
            let rom = reps[0].rom;
            assert!(
                (rom - travel).abs() < travel * 0.10,
                "tilt {deg}° about axis {axis}: rom {rom:.3} for {travel:.3} m of travel"
            );
        }
    }
}

/// A gyroscope that reports a rate while the bar is still does not turn the ground: the offset is learned at rest and taken out, so a rep after a long still period reads its travel.
///
/// This is the failure mode the fused part of the old path hid: an attitude that drifts while the bar rests leaks gravity into the horizontal axes, and a detector integrating that sees a bar that is moving when it is on the rack.
#[test]
fn a_gyro_bias_does_not_rotate_the_ground() {
    let travel = Rep::bench().travel;
    let bench = Rep::bench().samples();
    for dps in [1.0f32, 2.0] {
        for axis in 0..3 {
            let mut gyro = [0.0; 3];
            gyro[axis] = dps;
            let samples = impaired(
                &bench,
                Impairments {
                    gyro_bias_dps: gyro,
                    ..Default::default()
                },
            );
            let reps = stored(&samples);
            assert_eq!(
                reps.len(),
                1,
                "{dps} dps on axis {axis}: {} reps",
                reps.len()
            );
            let rom = reps[0].rom;
            assert!(
                (rom - travel).abs() < travel * 0.10,
                "{dps} dps on axis {axis}: rom {rom:.3} for {travel:.3} m"
            );
        }
    }
}

/// An accelerometer offset is the sensor's zero, not motion, and it arrives from a still window rather than from a seed taken at boot.
#[test]
fn an_accelerometer_bias_is_the_sensor_zero_not_motion() {
    let travel = Rep::bench().travel;
    let bench = Rep::bench().samples();
    for bias in [
        [0.0f32, -0.35, 0.0],
        [0.35, 0.0, 0.0],
        [0.0, 0.0, 0.35],
        [0.05, -0.35, 0.04],
    ] {
        let samples = impaired(
            &bench,
            Impairments {
                accel_bias: bias,
                ..Default::default()
            },
        );
        let reps = stored(&samples);
        assert_eq!(reps.len(), 1, "bias {bias:?}: {} reps stored", reps.len());
        let rom = reps[0].rom;
        assert!(
            (rom - travel).abs() < travel * 0.10,
            "bias {bias:?}: rom {rom:.3} for {travel:.3} m"
        );
    }
}

/// An offset that arrives *after* boot is absorbed by the next still window.
///
/// The sensor's offsets move with temperature, so a value learned at boot is wrong an hour later.
/// What makes this testable is that nothing announces the change: the reading simply sits somewhere else, which is exactly what a bar being held at an angle looks like too — so the filter learns the accelerometer's offset only while the bar is still, and only from stillness.
#[test]
fn an_offset_that_arrives_after_boot_is_absorbed() {
    for settle_ms in [3000u32, 5000, 10000] {
        let before = [0.0f32, -0.05, 0.0];
        let after = [0.0f32, -0.35, 0.0];
        let mut all = Vec::new();
        // Boot, then a while at rest with the old offset, then the same with the new one.
        for i in 0..(3000 + settle_ms) / SAMPLE_DT_MS {
            let t = i * SAMPLE_DT_MS;
            let bias = if t < 3000 { before } else { after };
            all.push(
                impaired(
                    &[at(t, [0.0; 3])],
                    Impairments {
                        accel_bias: bias,
                        ..Default::default()
                    },
                )[0],
            );
        }
        let offset = all.last().unwrap().t_ms + SAMPLE_DT_MS;
        for s in Rep::bench().samples() {
            let mut s = s;
            s.t_ms += offset;
            all.push(
                impaired(
                    &[s],
                    Impairments {
                        accel_bias: after,
                        ..Default::default()
                    },
                )[0],
            );
        }
        let reps = stored(&all);
        assert_eq!(
            reps.len(),
            1,
            "{settle_ms} ms after the offset moved: {} reps stored",
            reps.len()
        );
        let travel = Rep::bench().travel;
        let rom = reps[0].rom;
        assert!(
            (rom - travel).abs() < travel * 0.10,
            "{settle_ms} ms after the offset moved: rom {rom:.3}"
        );
    }
}

/// A boot in the middle of a rep stores nothing for it, and comes back at the first rest.
///
/// The filter has no idea which way gravity points until it has been still, so a boot that starts moving has no frame to integrate in. What it must not do is invent one: the motion before the first rest is discarded, and the next rep — after the bar has rested — is recorded normally.
#[test]
fn a_boot_into_motion_stores_nothing_until_it_rests() {
    let travel = Rep::bench().travel;

    // Boot, and move: no lead-in at all. The motion is the only thing in the stream.
    let mut motion = Vec::new();
    Rep::bench().strokes(&mut motion, 0);
    assert!(
        stored(&motion).is_empty(),
        "a boot into motion stored {} rep(s): the filter cannot know gravity yet",
        stored(&motion).len()
    );

    // Boot, move, then rest, then a rep: the first motion is lost and the second is not.
    let mut all = motion.clone();
    let end = all.last().map(|s| s.t_ms).unwrap_or(0) + SAMPLE_DT_MS;
    let mut t = end;
    while t < end + LEAD_IN_MS {
        all.push(at(t, [0.0; 3]));
        t += SAMPLE_DT_MS;
    }
    let mut second = Rep::bench();
    second.pause_bottom_ms = LEAD_IN_MS;
    let mut stream = all.clone();
    second.strokes(&mut stream, t);

    let reps = stored(&stream);
    assert_eq!(
        reps.len(),
        1,
        "after the first rest the rep should be stored, got {}",
        reps.len()
    );
    let rom = reps[0].rom;
    assert!(
        (rom - travel).abs() < travel * 0.10,
        "rom {rom:.3} for {travel:.3} m"
    );
}
