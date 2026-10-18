//! End-to-end host test of the Phase 4 chain: synthetic motion → detector →
//! ZUPT trace → protobuf records → flash log → read back and decode. No
//! device needed.

mod common;

use calypso_core::fixtures::{detect_and_trace, squat_set};
use calypso_core::rep::{RepSummary, TraceBuf, TracePoint};
use calypso_core::sampler::{Sample, QUAT_SCALE};
use calypso_core::store::{self, RECORD_BUF_BYTES};
use calypso_core::uuid::{fixture_uuid, RepUuid};
use common::{mount, BigFlash, MockFlash};

const Q_ID: [i16; 4] = [QUAT_SCALE as i16, 0, 0, 0];

/// Runs the shared fixture pipeline and splits it the way these tests want.
fn pipeline(samples: &[Sample]) -> (Vec<Vec<TracePoint>>, Vec<RepSummary>) {
    let reps = detect_and_trace(samples, 512);
    (
        reps.iter().map(|(_, trace)| trace.clone()).collect(),
        reps.iter().map(|(summary, _)| *summary).collect(),
    )
}

#[test]
fn rep_survives_the_round_trip_through_flash() {
    let (samples, _) = squat_set(3, 1500, 800, 0.4);
    let (traces, summaries) = pipeline(&samples);
    assert_eq!(traces.len(), 3);
    assert_eq!(summaries.len(), 3);

    // Real 4 KB sectors, so a full trace chunk fits one log record.
    let mut flash = BigFlash::new();
    let mut buf = [0u8; RECORD_BUF_BYTES];
    let stored: Vec<(RepUuid, calypso_core::rep::RepSummary, Vec<TracePoint>)> = summaries
        .iter()
        .zip(traces.iter())
        .enumerate()
        .map(|(i, (sum, trace))| (fixture_uuid(i as u32), *sum, trace.clone()))
        .collect();

    {
        let mut log = mount(&mut flash);
        for (rep_uuid, sum, trace) in &stored {
            let written = store::append_rep(&mut log, &mut buf, *rep_uuid, sum, trace).unwrap();
            let expected_chunks = 1 + calypso_core::pb::chunk_count_for(trace.len());
            assert_eq!(written, expected_chunks);
        }
    }

    // Remount (as the device does at boot) and read it all back.
    let mut log = mount(&mut flash);
    // Scratch the caller owns: the decoded record is ~2 KB, so it is not built
    // inside the scan (see `pb::decode_record_into`).
    let mut rec = calypso_core::pb::msg::RepRecord::default();
    let mut summaries_back = Vec::new();
    store::for_each_summary(&mut log, &mut buf, &mut rec, |s| summaries_back.push(s.clone()))
        .unwrap();
    assert_eq!(summaries_back.len(), 3, "all three summaries should be in the log");
    for (i, s) in summaries_back.iter().enumerate() {
        let (rep_uuid, want, _) = &stored[i];
        assert_eq!(calypso_core::pb::summary_uuid(s), Some(*rep_uuid));
        assert_eq!(s.t_start_ms, want.t_start_ms);
        assert_eq!(s.duration_ms, want.duration_ms);
        assert_eq!(s.trace_len as usize, want.trace_len);
        assert!((s.rom - want.rom).abs() < 1e-6);
        assert!((s.peak_velocity - want.peak_velocity).abs() < 1e-6);
    }

    // Traces come back with the same shape as what was computed.
    for (rep_uuid, _, want_trace) in &stored {
        let mut storage = vec![TracePoint::ZERO; 512];
        let mut out = TraceBuf::new(&mut storage);
        let n = store::read_trace(&mut log, &mut buf, &mut rec, *rep_uuid, &mut out).unwrap();
        assert_eq!(n, want_trace.len(), "trace length for {rep_uuid:?}");
        for (got, want) in out.iter().zip(want_trace.iter()) {
            // Timestamps are stored relative to the rep's first point.
            assert_eq!(got.t_ms, want.t_ms - want_trace[0].t_ms);
            assert!((got.pos[0] - want.pos[0]).abs() < 1e-6);
            assert!((got.pos[2] - want.pos[2]).abs() < 1e-6);
            assert_eq!(got.accel, want.accel);
            assert_eq!(got.quat, want.quat);
        }
    }

    // A rep that isn't in the log reads back empty.
    let mut storage = vec![TracePoint::ZERO; 512];
    let mut out = TraceBuf::new(&mut storage);
    assert_eq!(
        store::read_trace(&mut log, &mut buf, &mut rec, fixture_uuid(99), &mut out).unwrap(),
        0
    );
}

#[test]
fn oldest_reps_fall_off_when_the_ring_wraps() {
    // Small mock sectors (256 B) so a handful of reps wrap the ring; the
    // trace is short so each record still fits a 236-byte record payload.
    let (samples, _) = squat_set(1, 1500, 800, 0.4);
    let summary = pipeline(&samples).1.remove(0);
    let trace: Vec<TracePoint> = (0..4)
        .map(|i| TracePoint {
            t_ms: i * 10,
            pos: [0.0, 0.0, -0.1 * i as f32],
            accel: [0, 0, -100],
            quat: Q_ID,
        })
        .collect();

    let mut flash = MockFlash::new();
    let mut buf = [0u8; RECORD_BUF_BYTES];
    {
        let mut log = mount(&mut flash);
        for i in 1..=20 {
            store::append_rep(&mut log, &mut buf, fixture_uuid(i), &summary, &trace).unwrap();
        }
    }

    let mut log = mount(&mut flash);
    let mut rec = calypso_core::pb::msg::RepRecord::default();
    let mut surviving = Vec::new();
    store::for_each_summary(&mut log, &mut buf, &mut rec, |s| {
        surviving.push(calypso_core::pb::summary_uuid(s).unwrap())
    })
    .unwrap();

    // UUIDs carry no order, so this asserts on membership rather than sequence:
    // some reps were dropped, the newest survived, and every survivor is one we
    // actually wrote.
    assert!(!surviving.is_empty());
    assert!(surviving.len() < 20, "the ring should have dropped some: {surviving:?}");
    assert!(surviving.contains(&fixture_uuid(20)), "newest rep survives");
    let written: Vec<RepUuid> = (1..=20).map(fixture_uuid).collect();
    assert!(surviving.iter().all(|uuid| written.contains(uuid)), "only our reps");

    // A ring drops from the oldest end, so what survives is a suffix of what was
    // written: the earliest survivor, and then every rep after it.
    let first_surviving = (1..=20)
        .find(|i| surviving.contains(&fixture_uuid(*i)))
        .expect("something survived");
    assert!(first_surviving > 1, "the oldest reps should have been overwritten");
    for i in first_surviving..=20 {
        assert!(surviving.contains(&fixture_uuid(i)), "rep {i} should have survived");
    }
    assert_eq!(surviving.len(), (20 - first_surviving + 1) as usize);
}

#[test]
fn summary_and_chunks_share_the_rep_uuid() {
    let (samples, _) = squat_set(1, 1500, 800, 0.4);
    let (traces, summaries) = pipeline(&samples);
    let trace = &traces[0];
    let summary = summaries[0];

    let mut flash = BigFlash::new();
    let mut buf = [0u8; RECORD_BUF_BYTES];
    let mut log = mount(&mut flash);
    let want = fixture_uuid(5);
    store::append_rep(&mut log, &mut buf, want, &summary, trace).unwrap();

    // Walk the raw records: one summary, then chunks 0..n share its uuid.
    let short = String::from_utf8_lossy(&calypso_core::uuid::short(&want)).to_string();
    let mut kinds = Vec::new();
    log.for_each(&mut buf, |_, payload| {
        match calypso_core::pb::decode_record(payload).unwrap() {
            calypso_core::pb::Record::Summary(s) => {
                kinds.push(format!("summary:{short}:chunks={}", s.chunk_count));
            }
            calypso_core::pb::Record::Trace(t) => {
                kinds.push(format!("trace:{short}:{}", t.index));
            }
        }
    })
    .unwrap();

    let expected_chunks = calypso_core::pb::chunk_count_for(trace.len());
    assert_eq!(kinds.len(), 1 + expected_chunks as usize);
    assert_eq!(kinds[0], format!("summary:{short}:chunks={expected_chunks}"));
    for i in 0..expected_chunks {
        assert_eq!(kinds[1 + i as usize], format!("trace:{short}:{i}"));
    }
}
