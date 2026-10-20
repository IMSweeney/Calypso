//! Fixture generation: synthetic reps and `.pblog` log dumps.
//!
//! The client's fixtures (and, later, its history files) are **log dumps**: the
//! `RepRecord` messages the device stores, framed for a byte stream as
//! `[u32 little-endian length][record bytes]`. Records come from the same
//! `pb::encode_*` calls the firmware uses, so a fixture cannot drift from the
//! wire format.
//!
//! This is tooling support — the firmware never calls it. It is used by
//! `examples/gen_fixtures.rs` (which writes `calypso-app/assets/fixtures/`) and
//! by the store tests.

use alloc::vec::Vec;

use crate::pb::{self, msg};
use crate::rep::{compute_rep, Detector, RepSpan, TraceBuf, TracePoint, MARGIN_MS};
use crate::sampler::{quantize, Sample, SampleRing, ACCEL_SCALE, QUAT_SCALE};
use crate::store::RECORD_BUF_BYTES;
use crate::uuid;

/// Sample interval used by the synthetic fixtures (100 Hz, like the device).
pub const SAMPLE_DT_MS: u32 = 10;

/// Ring size used when replaying synthetic motion through the sampler pipeline.
const REPLAY_RING: usize = 4096;
/// Detector window (max rep + ZUPT margins).
const WINDOW_SAMPLES: usize = 900;

const Q_ID: [i16; 4] = [QUAT_SCALE as i16, 0, 0, 0];

/// One synthetic sample with the identity orientation.
pub fn sample(t_ms: u32, accel: [f32; 3]) -> Sample {
    Sample {
        t_ms,
        accel: accel.map(|a| quantize(a, ACCEL_SCALE)),
        quat: Q_ID,
    }
}

/// Squat-like motion: the sensor travels from 0 down to `-a` and back, in
/// `reps` repetitions separated by `rest_ms` of rest, sampled at 100 Hz.
///
/// `z(t) = -(a/2)(1 - cos(2*pi*t/T))`, whose second derivative has zero mean
/// and zero first moment — so the movement genuinely starts and ends at rest in
/// the same place, which is what ZUPT anchoring assumes. Returns the samples and
/// the motion spans the detector should find.
pub fn squat_set(reps: usize, t_ms: u32, rest_ms: u32, a: f32) -> (Vec<Sample>, Vec<RepSpan>) {
    let mut out = Vec::new();
    let mut spans = Vec::new();
    let mut t = 0u32;

    // Lead-in rest, so the detector starts from a settled state.
    for _ in 0..(300 / SAMPLE_DT_MS) {
        out.push(sample(t, [0.0, 0.0, 0.0]));
        t += SAMPLE_DT_MS;
    }

    for _ in 0..reps {
        let start = t;
        let tt = t_ms as f32 / 1000.0;
        let tau2 = 2.0 * core::f32::consts::PI * core::f32::consts::PI;
        let peak = tau2 * a / (tt * tt);
        for i in 0..=(t_ms / SAMPLE_DT_MS) {
            let time = i as f32 * SAMPLE_DT_MS as f32 / 1000.0;
            let accel = -peak * libm::cosf(2.0 * core::f32::consts::PI * time / tt);
            out.push(sample(t, [0.0, 0.0, accel]));
            t += SAMPLE_DT_MS;
        }
        spans.push(RepSpan {
            start_ms: start,
            end_ms: t - SAMPLE_DT_MS,
        });
        for _ in 0..(rest_ms / SAMPLE_DT_MS) {
            out.push(sample(t, [0.0, 0.0, 0.0]));
            t += SAMPLE_DT_MS;
        }
    }
    (out, spans)
}

/// Replay samples through the sampler pipeline the firmware runs: ring buffer,
/// detector, then the ZUPT-anchored trace per detected rep.
///
/// `trace_capacity` caps stored trace points as on the device, so a long rep
/// comes back decimated with `summary.decimated == true`.
pub fn detect_and_trace(
    samples: &[Sample],
    trace_capacity: usize,
) -> Vec<(msg::RepSummary, Vec<TracePoint>)> {
    let mut ring_buf = alloc::vec![Sample::ZERO; REPLAY_RING];
    let mut ring = SampleRing::new(&mut ring_buf);
    let mut det = Detector::new();
    let mut window = alloc::vec![Sample::ZERO; WINDOW_SAMPLES];
    let mut storage = alloc::vec![TracePoint::ZERO; trace_capacity];
    let mut out = Vec::new();

    for s in samples {
        ring.push(*s);
        if let Some(span) = det.update(s) {
            let n = ring.copy_window(
                span.start_ms,
                span.end_ms,
                MARGIN_MS,
                MARGIN_MS,
                &mut window,
            );
            if n == 0 {
                continue;
            }
            let mut trace = TraceBuf::new(&mut storage);
            let summary = compute_rep(&window[..n], &mut trace);
            out.push((summary, trace.as_slice().to_vec()));
        }
    }
    out
}

/// Frame one encoded record into a dump: `[u32 LE length][record bytes]`.
pub fn push_record(dump: &mut Vec<u8>, record: &[u8]) {
    dump.extend_from_slice(&(record.len() as u32).to_le_bytes());
    dump.extend_from_slice(record);
}

/// Append a rep (summary record + trace chunk records) to a dump. Returns the
/// number of records written.
pub fn push_rep(
    dump: &mut Vec<u8>,
    rec_buf: &mut [u8],
    summary: &mut msg::RepSummary,
    trace: &[TracePoint],
) -> Result<u32, pb::Error> {
    // The chunks carry the same identity as the summary, and that identity is the
    // summary's — deriving it here is what keeps them together.
    let rep_uuid = pb::summary_uuid(summary).ok_or(pb::Error::Decode)?;
    summary.chunk_count = pb::chunk_count_for(trace.len());
    let n = pb::encode_summary(summary, rec_buf)?;
    push_record(dump, &rec_buf[..n]);

    let t0_ms = trace.first().map(|p| p.t_ms).unwrap_or(0);
    let mut written = 1;
    for (index, chunk) in trace.chunks(pb::TRACE_CHUNK_SAMPLES).enumerate() {
        // Fixtures are stored records, not a stream: total_chunks stays 0.
        let n = pb::encode_trace_chunk(rep_uuid, index as u32, t0_ms, 0, chunk, rec_buf)?;
        push_record(dump, &rec_buf[..n]);
        written += 1;
    }
    Ok(written)
}

/// Convenience: generate a squat set and write it as a dump, returning the
/// bytes. `reps` reps at 100 Hz with 0.8 s rests.
pub fn squat_dump(reps: usize, a: f32) -> Vec<u8> {
    let (samples, _) = squat_set(reps, 1500, 800, a);
    let reps = detect_and_trace(&samples, 512);
    let mut rec_buf = [0u8; RECORD_BUF_BYTES];
    let mut dump = Vec::new();
    for (i, (mut summary, trace)) in reps.into_iter().enumerate() {
        // Deterministic identities: fixtures are regenerated and compared.
        summary.rep_uuid = pb::uuid_to_field(&uuid::fixture_uuid(i as u32));
        push_rep(&mut dump, &mut rec_buf, &mut summary, &trace).unwrap();
    }
    dump
}

/// Parse a dump back into records using the `[u32 LE length][bytes]` framing.
///
/// Returns the records and whether the final one was cut short (a torn write,
/// or a capture that ended mid-record). This is the reference implementation of
/// the framing the client's Dart reader must match.
pub fn read_dump(dump: &[u8]) -> (Vec<&[u8]>, bool) {
    let mut records = Vec::new();
    let mut pos = 0usize;
    while pos + 4 <= dump.len() {
        let len = u32::from_le_bytes(dump[pos..pos + 4].try_into().unwrap()) as usize;
        let start = pos + 4;
        if start + len > dump.len() {
            return (records, true);
        }
        records.push(&dump[start..start + len]);
        pos = start + len;
    }
    let truncated = pos != dump.len();
    (records, truncated)
}

/// A squat set where the bar drifts to one side on the way down — the case the
/// 3D plot exists to show.
///
/// Built by tilting the lateral acceleration on a normal squat set, so the path
/// bows: the vertical travel is unchanged (so the detector and the summary are
/// unaffected) but the trace visibly leaves the vertical plane.
pub fn dipped_squat_set(reps: usize, lateral_fraction: f32) -> (Vec<Sample>, Vec<RepSpan>) {
    let (mut samples, spans) = squat_set(reps, 1500, 800, 0.4);
    for s in samples.iter_mut() {
        // Lateral push proportional to the *magnitude* of the vertical
        // acceleration, so it always pushes the same way: the bar drifts left
        // through the whole rep instead of returning to centre. That leaves the
        // descent and the ascent as two distinct paths — a bow you can see —
        // rather than one straight line traversed twice.
        let lateral = (f32::from(s.accel[2].abs()) * lateral_fraction) as i16;
        s.accel[0] = lateral;
    }
    (samples, spans)
}
