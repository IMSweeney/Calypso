//! The arithmetic inside a rep: what `compute_rep` makes of a known motion, and the pieces a stored trace and summary are built from — the position it integrates, the angles it reports, and the decimation that fits it into the device's buffer.
//!
//! **The detector is not in this file.** Whether a given motion is a rep is pinned in `calypso-sim`, against the motion generators (`sim/src/motion.rs`) and the sets they assemble, because that question needs a stream rather than a sample list and because a detector change is measured on the sweeps there. Nothing here should grow one: these tests are the seam below it, where a failure says the integration or the angles are wrong rather than that the thresholds moved.
//!
//! The motion here is written out by hand (`known_motion`), not generated: each one is a case with an answer worked out on paper — a cosine out-and-back whose travel is known, a still sensor whose offset must come out as nothing.

use calypso_core::motion::fusion::{roll_pitch_deg, Fusion, GRAVITY};
use calypso_core::motion::rep::{compute_rep, RepSeed, TraceBuf, TracePoint, MARGIN_MS};
use calypso_core::sampler::{quantize, Sample, ACCEL_SCALE, QUAT_SCALE};

const HZ: u32 = 100;
const DT_MS: u32 = 1000 / HZ;
const TRACE_CAP: usize = 512;

/// A sample of a level sensor: `linear` is the bar's own acceleration, and a sensor reads that plus gravity.
fn sample(t_ms: u32, linear: [f32; 3], gyro_dps: [f32; 3]) -> Sample {
    Sample::from_f32(t_ms, [linear[0], linear[1], linear[2] + GRAVITY], gyro_dps)
}

/// Stillness, then one out-and-back of `t_ms` that travels `a` metres on Z, then stillness.
///
/// The acceleration is a cosine and its second derivative is chosen so the bar leaves and arrives at rest: a rep that travels out and never comes back is a push, and there is no position to check one against.
fn known_motion(a: f32, t_ms: u32, margin_ms: u32) -> Vec<Sample> {
    let mut out = Vec::new();
    let mut t = 0u32;
    while t < margin_ms {
        out.push(sample(t, [0.0, 0.0, 0.0], [0.0; 3]));
        t += DT_MS;
    }
    let steps = t_ms / DT_MS;
    let tt = t_ms as f32 / 1000.0;
    let peak_accel = 2.0 * core::f32::consts::PI.powi(2) * a / (tt * tt);
    for i in 0..=steps {
        let time = i as f32 * DT_MS as f32 / 1000.0;
        let accel = -peak_accel * libm::cosf(2.0 * core::f32::consts::PI * time / tt);
        out.push(sample(t, [0.0, 0.0, accel], [0.0; 3]));
        t += DT_MS;
    }
    for _ in 0..(margin_ms / DT_MS) {
        out.push(sample(t, [0.0, 0.0, 0.0], [0.0; 3]));
        t += DT_MS;
    }
    out
}

fn peak_to_peak_z(buf: &TraceBuf<'_>) -> f32 {
    let mut lo = f32::MAX;
    let mut hi = f32::MIN;
    for p in buf.iter() {
        if p.pos[2] < lo {
            lo = p.pos[2];
        }
        if p.pos[2] > hi {
            hi = p.pos[2];
        }
    }
    hi - lo
}

#[test]
fn compute_rep_recovers_known_displacement() {
    let a = 0.4f32;
    let samples = known_motion(a, 1500, MARGIN_MS);
    let mut storage = [TracePoint::ZERO; TRACE_CAP];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, &RepSeed::LEVEL, None, &mut buf);

    assert_eq!(sum.computed_len, samples.len() as u32);
    assert_eq!(sum.duration_ms, 1500 + 2 * MARGIN_MS);
    let peak_to_peak = peak_to_peak_z(&buf);
    assert!(
        (peak_to_peak - a).abs() < 0.3 * a,
        "peak-to-peak z {peak_to_peak}, expected ~{a}"
    );
    assert!(sum.peak_velocity > 0.0);
    assert!(
        sum.peak_velocity < 1.0,
        "peak velocity {}",
        sum.peak_velocity
    );
    // The window has rest at both ends, so the filter's velocity is anchored at both of them and the bar is back where it started — to within what the integration leaves, which is a few centimetres over a rep this long: nothing is detrended to zero here, so an endpoint reads what the sensor and the anchor leave it at.
    let first = buf.get(0).unwrap();
    let last = buf.get(buf.len() - 1).unwrap();
    assert!(first.pos[2].abs() < 1e-3);
    assert!(
        last.pos[2].abs() < 0.15 * a,
        "a rep that came back to where it started ends {} m away",
        last.pos[2]
    );
}

/// A still sensor carrying an offset integrates nothing once that offset is known — which is what a trace's own header says it is.
///
/// This is the property the whole trace rests on, and it is the one the raw accelerometer can state plainly: a fused "linear acceleration" reads zero at rest whatever the truth is, so a constant offset there is only distinguishable from motion by its constancy. Here the offset is what has to be explained, the seed says what it is, and a window of a sensor that never moved is nothing.
#[test]
fn a_still_window_seeded_with_its_offset_integrates_nothing() {
    let mut samples = Vec::new();
    let mut t = 0;
    for _ in 0..800 {
        // 1 g up, and 0.3 m/s² of offset on the same axis.
        samples.push(Sample::from_f32(t, [0.0, 0.0, GRAVITY + 0.3], [0.0; 3]));
        t += DT_MS;
    }

    // A boot at rest: the filter learns the offset from the still window, and that is the state a stored rep carries.
    let mut boot = Fusion::new();
    for s in &samples[..300] {
        boot.update(s);
    }
    assert!(boot.ready(), "the boot never settled");

    let mut storage = [TracePoint::ZERO; TRACE_CAP];
    let mut buf = TraceBuf::new(&mut storage);
    let seed = RepSeed {
        filter: boot.state(),
        velocity: [0.0; 3],
    };
    let sum = compute_rep(&samples[300..], &seed, None, &mut buf);
    assert!(sum.rom < 0.01, "rom {}", sum.rom);
    assert!(peak_to_peak_z(&buf) < 0.02);
    assert!(sum.peak_velocity < 0.02);
}

/// A rep that swung sideways further than it went up: what the range of motion must not be measured on.
#[test]
fn rom_is_the_vertical_travel_not_the_biggest_excursion() {
    let vertical = 0.4f32;
    let mut samples = known_motion(vertical, 1500, MARGIN_MS);
    // A sideways push of five times the vertical acceleration: same shape, five times the excursion, so the largest range in the trace is on X and the answer has to ignore it.
    for s in samples.iter_mut() {
        let z = s.specific_force_f32()[2] - GRAVITY;
        s.accel[0] = quantize(z * 5.0, ACCEL_SCALE);
    }
    let mut storage = [TracePoint::ZERO; TRACE_CAP];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, &RepSeed::LEVEL, None, &mut buf);

    let range = |f: &dyn Fn(&TracePoint) -> f32| {
        let values: Vec<f32> = buf.as_slice().iter().map(f).collect();
        values.iter().cloned().fold(f32::MIN, f32::max)
            - values.iter().cloned().fold(f32::MAX, f32::min)
    };
    let lateral = range(&|p| p.pos[0]);
    assert!(
        lateral > 2.0 * vertical,
        "the test's own sideways excursion is bigger: {lateral}"
    );
    assert!(
        (sum.rom - vertical).abs() < 0.3 * vertical,
        "rom {} for {vertical} m of lift and {lateral} m of swing",
        sum.rom
    );
}

#[test]
fn trace_is_decimated_when_it_does_not_fit() {
    let samples = known_motion(0.4, 3000, MARGIN_MS);
    let mut storage = [TracePoint::ZERO; 64];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, &RepSeed::LEVEL, None, &mut buf);
    assert!(sum.computed_len <= 64);
    assert!(sum.computed_len >= 50, "computed_len {}", sum.computed_len);
    // Anchored at the last sample, so a stored rep always ends where the motion did.
    assert_eq!(
        buf.get(buf.len() - 1).unwrap().t_ms,
        samples.last().unwrap().t_ms
    );
}

/// The angles a summary reports come from the attitude the filter integrates, not from a sensor's own answer: a rep during which the clip rolls reports that roll.
#[test]
fn roll_range_is_reported_in_summary() {
    // 25° over four seconds, about the sensor's x axis. Gravity turns in the sensor's frame as the sensor turns, which is what a real roll at rest looks like.
    let mut samples = Vec::new();
    let mut t = 0;
    for i in 0..400 {
        let roll = 6.25f32.to_radians() * (i as f32 * DT_MS as f32 / 1000.0);
        let body = [0.0, GRAVITY * libm::sinf(roll), GRAVITY * libm::cosf(roll)];
        samples.push(Sample::from_f32(t, body, [6.25, 0.0, 0.0]));
        t += DT_MS;
    }
    let mut storage = [TracePoint::ZERO; TRACE_CAP];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, &RepSeed::LEVEL, None, &mut buf);
    assert!(
        sum.roll_min_deg.abs() < 1.0,
        "roll min {}",
        sum.roll_min_deg
    );
    assert!(
        (sum.roll_max_deg - 25.0).abs() < 1.5,
        "roll max {}",
        sum.roll_max_deg
    );
    assert!(sum.pitch_max_deg.abs() < 1.0);
    // The stored attitude is the same one the angles came from.
    let last = buf.get(buf.len() - 1).unwrap();
    let (roll, _) = roll_pitch_deg(last.quat.map(|q| q as f32 / QUAT_SCALE as f32));
    assert!((roll - sum.roll_max_deg).abs() < 0.1, "trace roll {roll}");
}

#[test]
fn empty_input_is_handled() {
    let mut storage = [TracePoint::ZERO; 64];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&[], &RepSeed::LEVEL, None, &mut buf);
    assert_eq!(sum.computed_len, 0);
    assert!(buf.is_empty());
    assert_eq!(sum.duration_ms, 0);
}
