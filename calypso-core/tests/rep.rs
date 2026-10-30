use calypso_core::rep::{
    accel_magnitude, compute_rep, roll_pitch_deg, rotate_by_quat, Detector, RepSpan, State,
    TraceBuf, TracePoint, MARGIN_MS, T_REST_MS,
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

/// `reps` out-and-backs of `up_ms` each, `rest_ms` of stillness between them, over a sensor that reads a small bias while it is still.
///
/// The acceleration is a cosine, so the velocity is zero where the bar turns around as well as at the ends: a rep that travels out and never comes back is a push, and a detector tuned on one has nothing to learn about reps.
fn squat_set(
    reps: usize,
    up_ms: u32,
    rest_ms: u32,
    peak_accel: f32,
) -> (Vec<Sample>, Vec<RepSpan>) {
    let mut out = Vec::new();
    let mut spans = Vec::new();
    let mut t = 0u32;

    for _ in 0..(HZ / 2) {
        out.push(sample(t, [0.02, 0.0, 0.01], Q_ID));
        t += DT_MS;
    }

    for _ in 0..reps {
        let start = t;
        let steps = up_ms / DT_MS;
        for i in 0..steps {
            let phase = i as f32 / steps as f32;
            let a = peak_accel * libm::cosf(2.0 * core::f32::consts::PI * phase);
            out.push(sample(t, [0.0, 0.0, a], Q_ID));
            t += DT_MS;
        }
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
        assert!(
            got.start_ms >= want.start_ms && got.start_ms <= want.start_ms + 250,
            "start {:?} vs {:?}",
            got,
            want
        );
        assert!(
            got.end_ms + 150 >= want.end_ms && got.end_ms <= want.end_ms + 400,
            "end {:?} vs {:?}",
            got,
            want
        );
    }
    assert_eq!(det.state(), State::Idle);
}

/// A set nobody pauses through: the reps run back to back, and the only rest in the whole thing is after the last one.
fn no_pause_set(reps: usize, up_ms: u32, peak_accel: f32) -> (Vec<Sample>, Vec<RepSpan>) {
    let (mut samples, spans) = squat_set(reps, up_ms, 0, peak_accel);
    let mut t = samples.last().expect("a set has samples").t_ms + DT_MS;
    for _ in 0..(2 * HZ) {
        samples.push(sample(t, [0.01, 0.0, 0.0], Q_ID));
        t += DT_MS;
    }
    (samples, spans)
}

#[test]
fn detector_finds_each_rep_in_a_set_nobody_pauses_through() {
    let (samples, expected) = no_pause_set(5, 1500, 3.0);
    let mut det = Detector::new();
    let mut found = Vec::new();
    for s in &samples {
        if let Some(span) = det.update(s) {
            found.push(span);
        }
    }
    assert_eq!(
        found.len(),
        expected.len(),
        "five reps with no rest between them are five reps, not {:?}",
        found
    );
    for (got, want) in found.iter().zip(expected.iter()) {
        // Symmetric, because nothing stops between the reps: the bar is moving at the boundary, so the first sample of a rep can be one either side of it.
        assert!(
            (got.start_ms as i64 - want.start_ms as i64).abs() <= 250,
            "start {:?} vs {:?}",
            got,
            want
        );
        assert!(
            (got.end_ms as i64 - want.end_ms as i64).abs() <= 250,
            "end {:?} vs {:?}",
            got,
            want
        );
        assert!(
            got.duration_ms() >= 1000 && got.duration_ms() <= 2000,
            "a rep that lasts {:?} is not one of five",
            got
        );
    }
}

/// What the one real capture was full of: acceleration enough to start and stop a rep, on a sensor that never went anywhere — 0.03-0.09 m of travel against 0.21 m for the smallest real rep.
#[test]
fn a_wobble_is_not_a_rep() {
    let mut samples = Vec::new();
    let mut t = 0;
    for _ in 0..30 {
        samples.push(sample(t, [0.01, 0.0, 0.0], Q_ID));
        t += DT_MS;
    }
    // Three seconds of a 6 Hz shake: acceleration to spare, and nowhere to go — 2.5 m/s^2 at that rate is 3 mm of travel.
    for _ in 0..300 {
        let a = 2.5 * libm::sinf(2.0 * core::f32::consts::PI * 6.0 * t as f32 / 1000.0);
        samples.push(sample(t, [0.0, 0.0, a], Q_ID));
        t += DT_MS;
    }
    for _ in 0..100 {
        samples.push(sample(t, [0.01, 0.0, 0.0], Q_ID));
        t += DT_MS;
    }
    let mut storage = [TracePoint::ZERO; TRACE_CAP];
    let mut trace = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, &mut trace);
    assert!(!sum.is_a_rep(), "{} m of wobble was called a rep", sum.rom);

    let (samples, _) = squat_set(1, 3000, 800, 3.0);
    let mut trace = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, &mut trace);
    assert!(sum.is_a_rep(), "a real rep was thrown away: {} m", sum.rom);
}

#[test]
fn short_noise_blip_is_not_a_rep() {
    let mut det = Detector::new();
    let mut t = 0;
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
    let mut det = Detector::new();
    let mut t = 0;
    let mut reps = 0;
    for _ in 0..1200 {
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
    let sum = compute_rep(&samples, &mut buf);

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
fn compute_rep_recovers_known_displacement() {
    let a = 0.4f32;
    let samples = known_motion(a, 1500, MARGIN_MS);
    let mut storage = [TracePoint::ZERO; TRACE_CAP];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, &mut buf);

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
    let sum = compute_rep(&samples, &mut buf);
    assert!(sum.rom < 1e-3, "rom {}", sum.rom);
    assert!(peak_to_peak_z(&buf) < 1e-3);
    assert!(sum.peak_velocity < 1e-3);
}

#[test]
fn trace_is_decimated_when_it_does_not_fit() {
    let samples = known_motion(0.4, 3000, MARGIN_MS);
    let mut storage = [TracePoint::ZERO; 64];
    let mut buf = TraceBuf::new(&mut storage);
    let sum = compute_rep(&samples, &mut buf);
    assert!(sum.decimated());
    assert!(sum.trace_len <= 64);
    assert!(sum.trace_len >= 50, "trace_len {}", sum.trace_len);
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
    let sum = compute_rep(&samples, &mut buf);
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
fn detect_then_compute_from_the_ring_pipeline() {
    let mut ring_buf = vec![calypso_core::sampler::Sample::ZERO; 1024];
    let mut ring = calypso_core::sampler::SampleRing::new(&mut ring_buf);
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
            assert!(window[0].t_ms + 50 >= span.start_ms.saturating_sub(MARGIN_MS));
            let mut storage = [TracePoint::ZERO; TRACE_CAP];
            let mut buf = TraceBuf::new(&mut storage);
            rep = Some(compute_rep(&window[..n], &mut buf));
            assert!(rep.is_some());
            assert!(!buf.is_empty());
        }
    }

    let sum = rep.expect("no rep detected");
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
    let mut det = Detector::new();
    let mut t = 0;
    let mut closed = 0;
    for _ in 0..20 {
        det.update(&sample(t, [0.0, 0.0, 3.0], Q_ID));
        t += DT_MS;
    }
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
