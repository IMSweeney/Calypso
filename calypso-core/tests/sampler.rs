use calypso_core::sampler::{Sample, SampleRing, QUAT_SCALE};

fn sample(t_ms: u32) -> Sample {
    Sample {
        t_ms,
        accel: [1, 2, 3],
        quat: [QUAT_SCALE as i16, 0, 0, 0],
    }
}

#[test]
fn ring_holds_samples_oldest_first() {
    let mut ring_buf = vec![Sample::ZERO; 4];
    let mut ring = SampleRing::new(&mut ring_buf);
    assert!(ring.is_empty());
    for t in 0..3 {
        ring.push(sample(t * 10));
    }
    assert_eq!(ring.len(), 3);
    assert!(!ring.is_empty());
    for i in 0..3 {
        assert_eq!(ring.get(i).unwrap().t_ms, i as u32 * 10);
    }
    assert!(ring.get(3).is_none());
    assert_eq!(ring.latest().unwrap().t_ms, 20);
}

#[test]
fn ring_evicts_oldest_on_overflow() {
    let mut ring_buf = vec![Sample::ZERO; 4];
    let mut ring = SampleRing::new(&mut ring_buf);
    for t in 0..10 {
        ring.push(sample(t));
    }
    assert_eq!(ring.len(), 4);
    assert_eq!(ring.pushed(), 10);
    for (i, t) in (6..10).enumerate() {
        assert_eq!(ring.get(i).unwrap().t_ms, t);
    }
    assert_eq!(ring.latest().unwrap().t_ms, 9);
}

#[test]
fn sample_fixed_point_round_trip() {
    let s = Sample::from_f32(1234, [1.23, -9.81, 0.0], [1.0, 0.0, 0.0, -0.5]);
    assert_eq!(s.accel, [123, -981, 0]);
    assert_eq!(s.quat, [16384, 0, 0, -8192]);
    let a = s.accel_f32();
    assert!((a[0] - 1.23).abs() < 1e-6);
    assert!((a[1] + 9.81).abs() < 1e-6);
    let q = s.quat_f32();
    assert_eq!(q, [1.0, 0.0, 0.0, -0.5]);
}

#[test]
fn sample_clamps_out_of_range() {
    let s = Sample::from_f32(0, [1000.0, -1000.0, 0.0], [2.0, -2.0, 0.0, 0.0]);
    assert_eq!(s.accel, [i16::MAX, i16::MIN, 0]);
    assert_eq!(s.quat, [i16::MAX, i16::MIN, 0, 0]);
}

#[test]
fn copy_window_slices_by_time_with_margins() {
    let mut ring_buf = vec![Sample::ZERO; 8];
    let mut ring = SampleRing::new(&mut ring_buf);
    for i in 0..10u32 {
        ring.push(sample(i * 10));
    }
    assert_eq!(ring.index_at_or_after(20).unwrap(), 0);
    assert_eq!(ring.index_at_or_after(95), None);

    let mut out = [Sample::ZERO; 16];
    let n = ring.copy_window(50, 70, 10, 10, &mut out);
    assert_eq!(n, 5);
    let times: Vec<u32> = out[..n].iter().map(|s| s.t_ms).collect();
    assert_eq!(times, vec![40, 50, 60, 70, 80]);

    let mut small = [Sample::ZERO; 2];
    let n = ring.copy_window(50, 70, 10, 10, &mut small);
    assert_eq!(n, 2);
    assert_eq!(small[0].t_ms, 40);

    let mut out = [Sample::ZERO; 16];
    let n = ring.copy_window(20, 80, 200, 200, &mut out);
    assert_eq!(n, 8);
    assert_eq!(out[0].t_ms, 20);
}
