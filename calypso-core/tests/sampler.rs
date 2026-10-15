//! Host tests for the sampler sample format and ring buffer.

use calypso_core::sampler::{Sample, SampleRing, ACCEL_SCALE, QUAT_SCALE};

fn sample(t_ms: u32) -> Sample {
    Sample {
        t_ms,
        accel: [1, 2, 3],
        quat: [QUAT_SCALE as i16, 0, 0, 0],
    }
}

#[test]
fn ring_holds_samples_oldest_first() {
    let mut ring = SampleRing::<4>::new();
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
    let mut ring = SampleRing::<4>::new();
    for t in 0..10 {
        ring.push(sample(t));
    }
    assert_eq!(ring.len(), 4);
    assert_eq!(ring.pushed(), 10);
    // Live window is the newest 4, in order.
    for (i, t) in (6..10).enumerate() {
        assert_eq!(ring.get(i).unwrap().t_ms, t);
    }
    assert_eq!(ring.latest().unwrap().t_ms, 9);
}

#[test]
fn ring_for_each_visits_in_order() {
    let mut ring = SampleRing::<8>::new();
    for t in 0..12 {
        ring.push(sample(t));
    }
    let mut seen = Vec::new();
    ring.for_each(|_, s| seen.push(s.t_ms));
    assert_eq!(seen, vec![4, 5, 6, 7, 8, 9, 10, 11]);
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
    assert_eq!(ACCEL_SCALE, 100);
}

#[test]
fn sample_clamps_out_of_range() {
    let s = Sample::from_f32(0, [1000.0, -1000.0, 0.0], [2.0, -2.0, 0.0, 0.0]);
    assert_eq!(s.accel, [i16::MAX, i16::MIN, 0]);
    assert_eq!(s.quat, [i16::MAX, i16::MIN, 0, 0]);
}

#[test]
fn sample_is_18_bytes() {
    assert_eq!(core::mem::size_of::<Sample>(), Sample::BYTE_SIZE);
}
