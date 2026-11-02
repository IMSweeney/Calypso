//! Fixture generation: synthetic reps and `.pblog` log dumps.
//!
//! A dump is the `RepRecord` messages the device stores, framed as `[u32 little-endian length][record bytes]`.
//! Records come from the same `pb::encode_*` calls the firmware uses, so a fixture cannot drift from the wire format.
//!
//! The firmware never calls any of this: the generators live outside `calypso-core` so that the device's library carries nothing it does not use.

use calypso_core::fusion::{Fusion, GRAVITY};
use calypso_core::pb::{self, msg};
use calypso_core::rep::{compute_rep, Detector, RepSpan, TraceBuf, TracePoint, MARGIN_MS};
use calypso_core::sampler::{quantize, Sample, SampleRing, ACCEL_SCALE};
use calypso_core::store::RECORD_BUF_BYTES;
use calypso_core::uuid;

/// Sample interval used by the synthetic fixtures (100 Hz, like the device).
pub const SAMPLE_DT_MS: u32 = 10;

/// Milliseconds of rest every synthetic stream starts with.
///
/// Not politeness: the filter reports no linear acceleration until it has been still long enough to
/// know which way gravity points, and the detector is not fed before that, so a stream that starts
/// moving is not recordable anywhere in it.
pub const LEAD_IN_MS: u32 = 1600;

/// The wall clock the fixtures are dated from: 2026-01-01T00:00:00Z.
///
/// A fixed second rather than the time of generation, so a regenerated fixture is byte-identical, and one late enough that the device would believe it (`calypso_core::clock::EARLIEST_S`).
pub const FIXTURE_EPOCH_S: u32 = 1_767_225_600;

/// A dump that starts the way a log does: with the boot's clock.
///
/// Every boot writes one at mount (`calypso_core::clock`), so a dump without one is a log whose reps can never be dated — and the app labels and orders its list by that time.
pub fn new_dump(rec_buf: &mut [u8]) -> Vec<u8> {
    let mut dump = Vec::new();
    let n =
        pb::encode_clock(FIXTURE_EPOCH_S, rec_buf).expect("a clock record fits a record buffer");
    push_record(&mut dump, &rec_buf[..n]);
    dump
}

/// Ring size used when replaying synthetic motion through the sampler pipeline.
const REPLAY_RING: usize = 4096;
/// Detector window (max rep + ZUPT margins).
const WINDOW_SAMPLES: usize = 900;

/// One synthetic sample of level motion: `accel` is the bar's own acceleration, and the sensor
/// reads that plus gravity.
///
/// The addition is the whole difference between a generator and a sensor, so it happens here
/// rather than in each scenario: a stroke's acceleration is what a rep is made of, and `g` is
/// only ever the platform.
pub fn at(t_ms: u32, accel: [f32; 3]) -> Sample {
    Sample {
        t_ms,
        accel: [accel[0], accel[1], accel[2] + GRAVITY].map(|a| quantize(a, ACCEL_SCALE)),
        gyro: [0; 3],
    }
}

/// Squat-like motion: the sensor travels from 0 down to `-a` and back, in `reps` repetitions separated by `rest_ms` of rest, sampled at 100 Hz.
/// `z(t) = -(a/2)(1 - cos(2*pi*t/T))`, whose second derivative has zero mean and zero first moment — so the movement genuinely starts and ends at rest in the same place, which is what ZUPT anchoring assumes.
/// Returns the samples and the motion spans the detector should find.
pub fn squat_set(reps: usize, t_ms: u32, rest_ms: u32, a: f32) -> (Vec<Sample>, Vec<RepSpan>) {
    let mut out = Vec::new();
    let mut spans = Vec::new();
    let mut t = 0u32;

    // Lead-in rest, so the filter has found gravity before the motion starts.
    for _ in 0..(LEAD_IN_MS / SAMPLE_DT_MS) {
        out.push(at(t, [0.0, 0.0, 0.0]));
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
            out.push(at(t, [0.0, 0.0, accel]));
            t += SAMPLE_DT_MS;
        }
        spans.push(RepSpan {
            start_ms: start,
            end_ms: t - SAMPLE_DT_MS,
        });
        for _ in 0..(rest_ms / SAMPLE_DT_MS) {
            out.push(at(t, [0.0, 0.0, 0.0]));
            t += SAMPLE_DT_MS;
        }
    }
    (out, spans)
}

/// Replay samples through the pipeline the firmware runs: ring buffer, filter, detector, then the trace per detected rep.
/// `trace_capacity` caps stored trace points as on the device, so a long rep comes back with the decimated flag set.
pub fn detect_and_trace(
    samples: &[Sample],
    trace_capacity: usize,
) -> Vec<(msg::RepSummary, Vec<TracePoint>)> {
    let mut ring_buf = vec![Sample::ZERO; REPLAY_RING];
    let mut ring = SampleRing::new(&mut ring_buf);
    let mut fusion = Fusion::new();
    let mut det = Detector::new();
    let mut window = vec![Sample::ZERO; WINDOW_SAMPLES];
    let mut storage = vec![TracePoint::ZERO; trace_capacity];
    let mut out = Vec::new();

    for s in samples {
        ring.push(*s);
        let filtered = fusion.update(s);
        // The device records nothing until the filter knows which way is down, so a boot's first
        // motion never reaches the detector. The ring still holds it, so a rep that starts right
        // after readiness has its margin.
        if !fusion.ready() {
            continue;
        }
        if let Some(span) = det.update(&filtered) {
            // The window starts where the filter state does, which is not always `MARGIN_MS` before
            // the rep: one close behind the boot has no filtered history that far back.
            let (from_ms, state) = det.rep_state();
            let n = ring.copy_window(from_ms, span.end_ms, 0, MARGIN_MS, &mut window);
            if n == 0 {
                continue;
            }
            let mut trace = TraceBuf::new(&mut storage);
            let mut summary = compute_rep(&window[..n], &state, &mut trace);
            // Synthetic motion is fully calibrated by construction, and a fixture that read as "recorded while the accelerometer wandered" would put that caveat on every row of a demo.
            summary.flags |= calypso_core::rep::REP_FLAG_CALIBRATED;
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

/// Append a rep (summary record + trace chunk records) to a dump. Returns the number of records written.
pub fn push_rep(
    dump: &mut Vec<u8>,
    rec_buf: &mut [u8],
    summary: &mut msg::RepSummary,
    trace: &[TracePoint],
) -> Result<u32, pb::Error> {
    // The chunks carry the same identity as the summary, and that identity is the summary's — deriving it here is what keeps them together.
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

/// Convenience: generate a squat set and write it as a dump, returning the bytes. `reps` reps at 100 Hz with 0.8 s rests.
pub fn squat_dump(reps: usize, a: f32) -> Vec<u8> {
    let (samples, _) = squat_set(reps, 1500, 800, a);
    let reps = detect_and_trace(&samples, 512);
    let mut rec_buf = [0u8; RECORD_BUF_BYTES];
    let mut dump = new_dump(&mut rec_buf);
    for (i, (mut summary, trace)) in reps.into_iter().enumerate() {
        // Deterministic identities: fixtures are regenerated and compared.
        summary.rep_uuid = pb::uuid_to_field(&uuid::fixture_uuid(i as u32));
        push_rep(&mut dump, &mut rec_buf, &mut summary, &trace).unwrap();
    }
    dump
}

/// Parse a dump back into records using the `[u32 LE length][bytes]` framing.
/// Returns the records and whether the final one was cut short (a torn write, or a capture that ended mid-record).
/// This is the reference implementation of the framing the client's Dart reader must match.
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

/// A squat set where the bar drifts to one side on the way down — the case the 3D plot exists to show.
/// Built by tilting the lateral acceleration on a normal squat set, so the path bows: the vertical travel is unchanged (so the detector and the summary are unaffected) but the trace visibly leaves the vertical plane.
pub fn dipped_squat_set(reps: usize, lateral_fraction: f32) -> (Vec<Sample>, Vec<RepSpan>) {
    let (mut samples, spans) = squat_set(reps, 1500, 800, 0.4);
    let rest = quantize(GRAVITY, ACCEL_SCALE);
    for s in samples.iter_mut() {
        // Lateral push proportional to the magnitude of the vertical acceleration, so it always pushes the same way: the bar drifts left through the whole rep instead of returning to centre.
        // That leaves the descent and the ascent as two distinct paths — a bow you can see — rather than one straight line traversed twice.
        // Measured against gravity rather than against zero, because a still sensor reads 1 g and a constant sideways push at rest is not a bow.
        let motion = s.accel[2].abs().saturating_sub(rest);
        s.accel[0] = (f32::from(motion) * lateral_fraction) as i16;
    }
    (samples, spans)
}
