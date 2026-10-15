//! Host tests for rep detection and ZUPT-anchored trace computation, driven
//! by synthetic sample streams (no device needed).

use calypso_core::rep::{
    accel_magnitude, compute_rep, roll_pitch_deg, rotate_by_quat, Detector, RepSpan, State,
    TraceBuf, TracePoint, MARGIN_MS, T_REST_MS,
};
use calypso_core::sampler::{quantize, Sample, ACCEL_SCALE, QUAT_SCALE};

const HZ: u32 = 100;
const DT_MS: u32 = 1000 / HZ;
/// Trace storage used by most tests.
const TRACE_CAP: usize = 512;

/// Identity quaternion in fixed point.
const Q_ID: [i16; 4] = [QUAT_SCALE as i16, 0, 0, 0];

fn sample(t_ms: u32, accel: [f32; 3], quat: [i16; 4]) -> Sample {
    Sample {
        t_ms,
        accel: accel.map(|a| quantize(a, ACCEL_SCALE)),
        quat,
    }
}

/// A synthetic workout: `reps` squat-like reps separated by rests, sampled at
/// 100 Hz. Returns (samples, expected motion spans).
///
/// Each rep: `up_ms` of motion (sinusoidal accel, peak `peak_accel`) then rest.
/// Body frame == world frame (identity quaternion).
fn squat_set(reps: usize, up_ms: u32, rest_ms: u32, peak_accel: f32) -> (Vec<Sample>, Vec<RepSpan>) {
    let mut out = Vec::new();
    let mut spans = Vec::new();
    let mut t = 0u32;

    // Lead-in rest.
    for _ in 0..(HZ / 2) {
        out.push(sample(t, [0.02, 0.0, 0.01], Q_ID));
        t += DT_MS;
    }

    for _ in 0..reps {
        let start = t;
        let steps = up_ms / DT_MS;
        for i in 0..steps {
            // One cycle of vertical accel (down, then up).
            let phase = i as f32 / steps as f32;
            let a = peak_accel * libm::sinf(2.0 * core::f32::consts::PI * phase);
            out.push(sample(t, [0.0, 0.0, a], Q_ID));
            t += DT_MS;
        }
        // The last *motion* sample: the sinusoid returns to ~0 accel here.
        spans.push(RepSpan {
            start_ms: start,
            end_ms: t - DT_MS,
        });
        for _ in 0..(rest_ms / DT_MS) {
            out.push(sample(t, [0.02, 0.01, 0.0], Q_ID));
            t += DT_MS;
        }
    }
    (out, spans)
}

#[test]
fn detector_finds_each_rep_in_a_set() {
    let (samples, expected) = squat_set(3, 1500, 800, 3.0);
    let mut det = Detector::new();
    let mut found = Vec::new();
    for s in &samples {
        if let Some(span) = det.update(s) {
            found.push(span);
        }
    }
    assert_eq!(found.len(), 3, "expected 3 reps, got {:?}", found);
    for (got, want) in found.iter().zip(expected.iter()) {
        // Motion start is detected after T_ON debounce; end after T_REST.
        assert!(
            got.start_ms >= want.start_ms && got.start_ms <= want.start_ms + 250,
            "start {:?} vs {:?}",
            got,
            want
        );
        // The low-accel tail of the sinusoid can trip the rest debounce a
        // little early; the rep must still end near the true motion end.
        assert!(
            got.end_ms + 150 >= want.end_ms && got.end_ms <= want.end_ms + 400,
            "end {:?} vs {:?}",
            got,
            want
        );
    }
    assert_eq!(det.state(), State::Idle);
}

#[test]
fn short_noise_blip_is_not_a_rep() {
    let mut det = Detector::new();
    let mut t = 0;
    // Settle, then a 50 ms spike (shorter than T_ON), then settle.
    for _ in 0..30 {
        assert!(det.update(&sample(t, [0.01, 0.0, 0.0], Q_ID)).is_none());
        t += DT_MS;
    }
    for _ in 0..5 {
        assert!(det.update(&sample(t, [0.0, 0.0, 9.0], Q_ID)).is_none());
        t += DT_MS;
    }
    for _ in 0..100 {
        assert!(det.update(&sample(t, [0.01, 0.0, 0.0], Q_ID)).is_none());
        t += DT_MS;
    }
    assert_eq!(det.state(), State::Idle);
}

#[test]
fn very_long_span_is_rejected() {
    // Motion that never rests for longer than MAX_REP_MS is a set-up
    // adjustment, not a rep.
    let mut det = Detector::new();
    let mut t = 0;
    let mut reps = 0;
    for _ in 0..1200 {
        // 12 s of continuous motion
        if det.update(&sample(t, [0.0, 0.0, 4.0], Q_ID)).is_some() {
            reps += 1;
        }
        t += DT_MS;
    }
    for _ in 0..60 {
        if det.update(&sample(t, [0.01, 0.0, 0.0], Q_ID)).is_some() {
            reps += 1;
        }
        t += DT_MS;
    }
    assert_eq!(reps, 0);
}

#[test]
fn rotate_by_quat_maps_body_to_world() {
    let s = core::f32::consts::FRAC_1_SQRT_2;
    // 90° about x: body +y becomes world +z.
    let q = [s, s, 0.0, 0.0];
    let v = rotate_by_quat(q, [0.0, 1.0, 0.0]);
    assert!((v[0]).abs() < 1e-5);
    assert!((v[1]).abs() < 1e-5);
    assert!((v[2] - 1.0).abs() < 1e-5);
    // Identity is a no-op.
    assert_eq!(rotate_by_quat([1.0, 0.0, 0.0, 0.0], [1.0, 2.0, 3.0]), [1.0, 2.0, 3.0]);
}

#[test]
fn roll_pitch_from_quaternion() {
    // 30° roll about x: q = [cos15, sin15, 0, 0].
    let r = 30f32.to_radians() / 2.0;
    let (roll, pitch) = roll_pitch_deg([libm::cosf(r), libm::sinf(r), 0.0, 0.0]);
    assert!((roll - 30.0).abs() < 0.5, "roll {roll}");
    assert!(pitch.abs() < 0.5, "pitch {pitch}");
    // 20° pitch about y: q = [cos10, 0, sin10, 0].
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

/// Build samples for a known squat-like vertical motion: the sensor travels
/// from z=0 down to z=-A and back to z=0 over `t_ms` (a squat), i.e.
/// `z(t) = -(A/2)(1 - cos(2*pi*t/T))`. Its analytic second derivative is
/// `a(t) = -(2*pi^2*A/T^2) * cos(2*pi*t/T)`, which has zero mean and zero
/// first moment, so it starts and ends at rest at the same place — exactly
/// what ZUPT anchoring assumes. Rest samples are added either side.
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
fn compute_rep_recovers_known_displacement() {
    // 0.4 m amplitude, 1.5 s: the trace should show ~0.4 m of travel on z and
    // come back to the origin, anchored at rest at both ends.
    let a = 0.4f32;
    let samples = known_motion(a, 1500, MARGIN_MS);
    let mut storage = [TracePoint::ZERO; TRACE_CAP];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, &mut buf);

    assert!(!sum.decimated);
    assert_eq!(sum.trace_len, samples.len());
    assert_eq!(sum.duration_ms, 1500 + 2 * MARGIN_MS);
    let peak_to_peak = peak_to_peak_z(&buf);
    assert!(
        (peak_to_peak - a).abs() < 0.3 * a,
        "peak-to-peak z {peak_to_peak}, expected ~{a}"
    );
    assert!(sum.peak_velocity > 0.0);
    assert!(sum.peak_velocity < 1.5, "peak velocity {}", sum.peak_velocity);
    let first = buf.get(0).unwrap();
    let last = buf.get(buf.len() - 1).unwrap();
    assert!(first.pos[2].abs() < 1e-3);
    assert!(last.pos[2].abs() < 1e-3);
}

#[test]
fn constant_bias_is_fully_rejected() {
    // A stationary sensor with a 0.3 m/s² bias on z: ZUPT detrending should
    // remove it exactly, leaving ~zero displacement.
    let mut samples = Vec::new();
    let mut t = 0;
    for _ in 0..400 {
        samples.push(sample(t, [0.0, 0.0, 0.3], Q_ID));
        t += DT_MS;
    }
    let mut storage = [TracePoint::ZERO; TRACE_CAP];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, &mut buf);
    assert!(sum.rom < 1e-3, "rom {}", sum.rom);
    assert!(peak_to_peak_z(&buf) < 1e-3);
    assert!(sum.peak_velocity < 1e-3);
}

#[test]
fn trace_is_decimated_when_it_does_not_fit() {
    let samples = known_motion(0.4, 3000, MARGIN_MS); // 340 samples
    let mut storage = [TracePoint::ZERO; 64];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, &mut buf);
    assert!(sum.decimated);
    assert!(sum.trace_len <= 64);
    // stride 6 over 340 samples, plus the always-included last sample.
    assert!(sum.trace_len >= 50, "trace_len {}", sum.trace_len);
    // The last emitted point is the true last sample.
    assert_eq!(
        buf.get(buf.len() - 1).unwrap().t_ms,
        samples.last().unwrap().t_ms
    );
}

#[test]
fn roll_pitch_range_is_reported_in_summary() {
    // A rep with a tilting sensor: roll sweeps 0..25°.
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
    let sum = compute_rep(&samples, &mut buf);
    assert!(sum.roll_min_deg.abs() < 1.0, "roll min {}", sum.roll_min_deg);
    assert!(
        (sum.roll_max_deg - 25.0).abs() < 1.5,
        "roll max {}",
        sum.roll_max_deg
    );
    assert!(sum.pitch_max_deg.abs() < 1.0);
}

#[test]
fn detect_then_compute_from_the_ring_pipeline() {
    // End-to-end: samples go into the ring, the detector runs over each new
    // sample, and a closed rep is sliced out of the ring and computed.
    let mut ring = calypso_core::sampler::SampleRing::<1024>::new();
    let mut det = Detector::new();
    let mut window = [Sample::ZERO; 600];
    let mut rep = None;

    let (samples, _) = squat_set(1, 1500, 600, 3.0);
    for s in &samples {
        ring.push(*s);
        if let Some(span) = det.update(s) {
            let n = ring.copy_window(
                span.start_ms,
                span.end_ms,
                MARGIN_MS,
                MARGIN_MS,
                &mut window,
            );
            assert!(n > 0, "window slice was empty");
            // The window starts one margin before the motion span.
            assert!(window[0].t_ms + 50 >= span.start_ms.saturating_sub(MARGIN_MS));
            let mut storage = [TracePoint::ZERO; TRACE_CAP];
            let mut buf = TraceBuf::new(&mut storage);
            rep = Some(compute_rep(&window[..n], &mut buf));
            assert!(rep.is_some());
            assert!(!buf.is_empty());
        }
    }

    let sum = rep.expect("no rep detected");
    // Motion span (~1500 ms) plus margins, minus the debounce delays.
    assert!(
        sum.duration_ms >= 1500 && sum.duration_ms <= 1500 + 2 * MARGIN_MS + 200,
        "duration {}",
        sum.duration_ms
    );
    assert!(sum.peak_accel > 1.0, "peak accel {}", sum.peak_accel);
    assert!(sum.rom > 0.0);
}

#[test]
fn empty_input_is_handled() {
    let mut storage = [TracePoint::ZERO; 64];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&[], &mut buf);
    assert_eq!(sum.trace_len, 0);
    assert!(buf.is_empty());
    assert_eq!(sum.duration_ms, 0);
}

#[test]
fn rest_debounce_prevents_split_reps() {
    // A brief dip below the rest threshold mid-rep must not close the rep.
    let mut det = Detector::new();
    let mut t = 0;
    let mut closed = 0;
    for _ in 0..20 {
        det.update(&sample(t, [0.0, 0.0, 3.0], Q_ID));
        t += DT_MS;
    }
    // 100 ms "dip" (shorter than T_REST).
    for _ in 0..(T_REST_MS / DT_MS - 2) {
        det.update(&sample(t, [0.0, 0.0, 0.0], Q_ID));
        t += DT_MS;
    }
    assert_eq!(det.state(), State::Moving);
    for _ in 0..20 {
        det.update(&sample(t, [0.0, 0.0, 3.0], Q_ID));
        t += DT_MS;
    }
    for _ in 0..(T_REST_MS / DT_MS + 10) {
        if det.update(&sample(t, [0.0, 0.0, 0.0], Q_ID)).is_some() {
            closed += 1;
        }
        t += DT_MS;
    }
    assert_eq!(closed, 1, "expected one merged rep");
}
