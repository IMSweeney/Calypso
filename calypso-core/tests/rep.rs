//! The arithmetic inside a rep: the rotation into the world frame, the angles, and what `compute_rep` makes of a
//! known motion — the pieces a stored trace and summary are built from.
//!
//! **The detector is not in this file.** Whether a given motion is a rep is pinned in `calypso-sim`, against the
//! motion generators (`sim/src/motion.rs`) and the sets they assemble, because that question needs a stream rather
//! than a sample list and because a detector change is measured on the sweeps there. Nothing here should grow one:
//! these tests are the seam below it, where a failure says the integration or the angles are wrong rather than that
//! the thresholds moved.
//!
//! The motion here is written out by hand (`known_motion`), not generated: each one is a case with an answer worked
//! out on paper — a cosine out-and-back whose travel is known, a constant bias that must come out as nothing.

use calypso_core::rep::{
    accel_magnitude, compute_rep, roll_pitch_deg, rotate_by_quat, TraceBuf, TracePoint, MARGIN_MS,
};
use calypso_core::sampler::{quantize, Sample, ACCEL_SCALE, QUAT_SCALE};

const HZ: u32 = 100;
const DT_MS: u32 = 1000 / HZ;
const TRACE_CAP: usize = 512;

const Q_ID: [i16; 4] = [QUAT_SCALE as i16, 0, 0, 0];

fn sample(t_ms: u32, accel: [f32; 3], quat: [i16; 4]) -> Sample {
    Sample {
        t_ms,
        accel: accel.map(|a| quantize(a, ACCEL_SCALE)),
        quat,
    }
}

/// Stillness, then one out-and-back of `t_ms` that travels `a` metres on Z, then stillness.
///
/// The acceleration is a cosine and its second derivative is chosen so the bar leaves and arrives at rest: a rep that
/// travels out and never comes back is a push, and there is no position to check one against.
fn known_motion(a: f32, t_ms: u32, margin_ms: u32) -> Vec<Sample> {
    let mut out = Vec::new();
    let mut t = 0u32;
    while t < margin_ms {
        out.push(sample(t, [0.0, 0.0, 0.0], Q_ID));
        t += DT_MS;
    }
    let steps = t_ms / DT_MS;
    let tt = t_ms as f32 / 1000.0;
    let peak_accel = 2.0 * core::f32::consts::PI.powi(2) * a / (tt * tt);
    for i in 0..=steps {
        let time = i as f32 * DT_MS as f32 / 1000.0;
        let accel = -peak_accel * libm::cosf(2.0 * core::f32::consts::PI * time / tt);
        out.push(sample(t, [0.0, 0.0, accel], Q_ID));
        t += DT_MS;
    }
    for _ in 0..(margin_ms / DT_MS) {
        out.push(sample(t, [0.0, 0.0, 0.0], Q_ID));
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

#[test]
fn accel_magnitude_matches_vector_length() {
    let s = sample(0, [3.0, 4.0, 0.0], Q_ID);
    assert!((accel_magnitude(&s) - 5.0).abs() < 0.02);
}

#[test]
fn compute_rep_recovers_known_displacement() {
    let a = 0.4f32;
    let samples = known_motion(a, 1500, MARGIN_MS);
    let mut storage = [TracePoint::ZERO; TRACE_CAP];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, [0.0; 3], &mut buf);

    assert!(!sum.decimated());
    assert_eq!(sum.trace_len, samples.len() as u32);
    assert_eq!(sum.duration_ms, 1500 + 2 * MARGIN_MS);
    let peak_to_peak = peak_to_peak_z(&buf);
    assert!(
        (peak_to_peak - a).abs() < 0.3 * a,
        "peak-to-peak z {peak_to_peak}, expected ~{a}"
    );
    assert!(sum.peak_velocity > 0.0);
    assert!(
        sum.peak_velocity < 1.5,
        "peak velocity {}",
        sum.peak_velocity
    );
    let first = buf.get(0).unwrap();
    let last = buf.get(buf.len() - 1).unwrap();
    assert!(first.pos[2].abs() < 1e-3);
    assert!(last.pos[2].abs() < 1e-3);
}

/// A constant acceleration that never changes is bias, not motion: the detrend has to take out all of it.
///
/// This is the property the whole trace rests on — a rep's displacement is read off a velocity that has had its
/// endpoint drift removed, so an offset that is constant over the rep must leave nothing behind on any axis.
#[test]
fn constant_bias_is_fully_rejected() {
    let mut samples = Vec::new();
    let mut t = 0;
    for _ in 0..400 {
        samples.push(sample(t, [0.0, 0.0, 0.3], Q_ID));
        t += DT_MS;
    }
    let mut storage = [TracePoint::ZERO; TRACE_CAP];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, [0.0; 3], &mut buf);
    assert!(sum.rom < 1e-3, "rom {}", sum.rom);
    assert!(peak_to_peak_z(&buf) < 1e-3);
    assert!(sum.peak_velocity < 1e-3);
}

/// A rep that swung sideways further than it went up: what the range of motion must not be measured on.
#[test]
fn rom_is_the_vertical_travel_not_the_biggest_excursion() {
    let vertical = 0.4f32;
    let mut samples = known_motion(vertical, 1500, MARGIN_MS);
    // A sideways push of five times the vertical acceleration: same shape, five times the excursion, so the largest
    // range in the trace is on X and the answer has to ignore it.
    for s in samples.iter_mut() {
        s.accel[0] = s.accel[2] * 5;
    }
    let mut storage = [TracePoint::ZERO; TRACE_CAP];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, [0.0; 3], &mut buf);

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
    let sum = compute_rep(&samples, [0.0; 3], &mut buf);
    assert!(sum.decimated());
    assert!(sum.trace_len <= 64);
    assert!(sum.trace_len >= 50, "trace_len {}", sum.trace_len);
    // Anchored at the last sample, so a stored rep always ends where the motion did.
    assert_eq!(
        buf.get(buf.len() - 1).unwrap().t_ms,
        samples.last().unwrap().t_ms
    );
}

#[test]
fn roll_pitch_range_is_reported_in_summary() {
    let mut samples = Vec::new();
    let mut t = 0;
    for i in 0..400 {
        let roll = 25.0f32.to_radians() * (i as f32 / 400.0);
        let q = [libm::cosf(roll / 2.0), libm::sinf(roll / 2.0), 0.0, 0.0];
        samples.push(sample(
            t,
            [0.0, 0.0, 0.5],
            [quantize(q[0], QUAT_SCALE), quantize(q[1], QUAT_SCALE), 0, 0],
        ));
        t += DT_MS;
    }
    let mut storage = [TracePoint::ZERO; TRACE_CAP];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, [0.0; 3], &mut buf);
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
}

#[test]
fn empty_input_is_handled() {
    let mut storage = [TracePoint::ZERO; 64];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&[], [0.0; 3], &mut buf);
    assert_eq!(sum.trace_len, 0);
    assert!(buf.is_empty());
    assert_eq!(sum.duration_ms, 0);
}
