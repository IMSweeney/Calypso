//! End-to-end host test of the Phase 4 chain: synthetic motion → detector →
//! ZUPT trace → protobuf records → flash log → read back and decode. No
//! device needed.

mod common;

use calypso_core::rep::{compute_rep, Detector, TraceBuf, TracePoint, MARGIN_MS};
use calypso_core::sampler::{quantize, Sample, ACCEL_SCALE, QUAT_SCALE};
use calypso_core::store::{self, RECORD_BUF_BYTES};
use common::{mount, BigFlash, MockFlash};

const DT_MS: u32 = 10;
const Q_ID: [i16; 4] = [QUAT_SCALE as i16, 0, 0, 0];

fn sample(t_ms: u32, accel: [f32; 3]) -> Sample {
    Sample {
        t_ms,
        accel: accel.map(|a| quantize(a, ACCEL_SCALE)),
        quat: Q_ID,
    }
}

/// Squat-like motion: down to -`a` and back to 0 over `t_ms`, with rest
/// either side (see `tests/rep.rs` for the derivation).
fn squat_set(reps: usize, t_ms: u32, rest_ms: u32, a: f32) -> Vec<Sample> {
    let mut out = Vec::new();
    let mut t = 0u32;
    for _ in 0..(300 / DT_MS) {
        out.push(sample(t, [0.0, 0.0, 0.0]));
        t += DT_MS;
    }
    for _ in 0..reps {
        let tt = t_ms as f32 / 1000.0;
        let peak = 2.0 * core::f32::consts::PI.powi(2) * a / (tt * tt);
        for i in 0..=(t_ms / DT_MS) {
            let time = i as f32 * DT_MS as f32 / 1000.0;
            let accel = -peak * libm::cosf(2.0 * core::f32::consts::PI * time / tt);
            out.push(sample(t, [0.0, 0.0, accel]));
            t += DT_MS;
        }
        for _ in 0..(rest_ms / DT_MS) {
            out.push(sample(t, [0.0, 0.0, 0.0]));
            t += DT_MS;
        }
    }
    out
}

/// Run the full sampler pipeline over `samples` and return each rep's
/// trace, the way the firmware's sampler task does.
fn detect_and_trace(samples: &[Sample]) -> (Vec<Vec<TracePoint>>, Vec<calypso_core::rep::RepSummary>) {
    let mut ring = calypso_core::sampler::SampleRing::<4096>::new();
    let mut det = Detector::new();
    let mut window = [Sample::ZERO; 900];
    let mut traces = Vec::new();
    let mut summaries = Vec::new();
    for s in samples {
        ring.push(*s);
        if let Some(span) = det.update(s) {
            let n = ring.copy_window(span.start_ms, span.end_ms, MARGIN_MS, MARGIN_MS, &mut window);
            assert!(n > 0);
            let mut storage = vec![TracePoint::ZERO; 512];
            let mut buf = TraceBuf::new(&mut storage);
            let sum = compute_rep(&window[..n], &mut buf);
            traces.push(buf.iter().copied().collect());
            summaries.push(sum);
        }
    }
    (traces, summaries)
}

#[test]
fn rep_survives_the_round_trip_through_flash() {
    let samples = squat_set(3, 1500, 800, 0.4);
    let (traces, summaries) = detect_and_trace(&samples);
    assert_eq!(traces.len(), 3);
    assert_eq!(summaries.len(), 3);

    // Real 4 KB sectors, so a full trace chunk fits one log record.
    let mut flash = BigFlash::new();
    let mut buf = [0u8; RECORD_BUF_BYTES];
    let stored: Vec<(u32, calypso_core::rep::RepSummary, Vec<TracePoint>)> = summaries
        .iter()
        .zip(traces.iter())
        .enumerate()
        .map(|(i, (sum, trace))| ((i + 1) as u32, *sum, trace.clone()))
        .collect();

    {
        let mut log = mount(&mut flash);
        for (rep_id, sum, trace) in &stored {
            let written = store::append_rep(&mut log, &mut buf, *rep_id, sum, trace).unwrap();
            let expected_chunks = 1 + calypso_core::pb::chunk_count_for(trace.len());
            assert_eq!(written, expected_chunks);
        }
    }

    // Remount (as the device does at boot) and read it all back.
    let mut log = mount(&mut flash);
    let mut summaries_back = Vec::new();
    store::for_each_summary(&mut log, &mut buf, |s| summaries_back.push(s)).unwrap();
    assert_eq!(summaries_back.len(), 3, "all three summaries should be in the log");
    for (i, s) in summaries_back.iter().enumerate() {
        let (rep_id, want, _) = &stored[i];
        assert_eq!(s.rep_id, *rep_id);
        assert_eq!(s.t_start_ms, want.t_start_ms);
        assert_eq!(s.duration_ms, want.duration_ms);
        assert_eq!(s.trace_len as usize, want.trace_len);
        assert!((s.rom - want.rom).abs() < 1e-6);
        assert!((s.peak_velocity - want.peak_velocity).abs() < 1e-6);
    }

    // Traces come back with the same shape as what was computed.
    for (rep_id, _, want_trace) in &stored {
        let mut storage = vec![TracePoint::ZERO; 512];
        let mut out = TraceBuf::new(&mut storage);
        let n = store::read_trace(&mut log, &mut buf, *rep_id, &mut out).unwrap();
        assert_eq!(n, want_trace.len(), "trace length for rep {rep_id}");
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
    assert_eq!(store::read_trace(&mut log, &mut buf, 99, &mut out).unwrap(), 0);
}

#[test]
fn oldest_reps_fall_off_when_the_ring_wraps() {
    // Small mock sectors (256 B) so a handful of reps wrap the ring; the
    // trace is short so each record still fits a 236-byte record payload.
    let summary = detect_and_trace(&squat_set(1, 1500, 800, 0.4)).1.remove(0);
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
        for rep_id in 1..=20 {
            store::append_rep(&mut log, &mut buf, rep_id, &summary, &trace).unwrap();
        }
    }

    let mut log = mount(&mut flash);
    let mut ids = Vec::new();
    store::for_each_summary(&mut log, &mut buf, |s| ids.push(s.rep_id)).unwrap();
    assert!(!ids.is_empty());
    assert_eq!(*ids.last().unwrap(), 20, "newest rep survives");
    assert!(ids[0] > 1, "oldest reps were overwritten: {ids:?}");
    // The surviving reps are the newest, in order.
    assert!(ids.windows(2).all(|w| w[1] == w[0] + 1), "ids contiguous: {ids:?}");
}

#[test]
fn summary_and_chunks_share_the_rep_id() {
    let samples = squat_set(1, 1500, 800, 0.4);
    let (traces, summaries) = detect_and_trace(&samples);
    let trace = &traces[0];
    let summary = summaries[0];

    let mut flash = BigFlash::new();
    let mut buf = [0u8; RECORD_BUF_BYTES];
    let mut log = mount(&mut flash);
    store::append_rep(&mut log, &mut buf, 5, &summary, trace).unwrap();

    // Walk the raw records: one summary, then chunks 0..n with rep_id 5.
    let mut kinds = Vec::new();
    log.for_each(&mut buf, |_, payload| {
        match calypso_core::pb::decode_record(payload).unwrap() {
            calypso_core::pb::Record::Summary(s) => {
                kinds.push(format!("summary:{}:chunks={}", s.rep_id, s.chunk_count));
            }
            calypso_core::pb::Record::Trace(t) => {
                kinds.push(format!("trace:{}:{}", t.rep_id, t.index));
            }
        }
    })
    .unwrap();

    let expected_chunks = calypso_core::pb::chunk_count_for(trace.len());
    assert_eq!(kinds.len(), 1 + expected_chunks as usize);
    assert_eq!(kinds[0], format!("summary:5:chunks={expected_chunks}"));
    for i in 0..expected_chunks {
        assert_eq!(kinds[1 + i as usize], format!("trace:5:{i}"));
    }
}
