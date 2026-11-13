//! Fixture dumps, and the replay plumbing the fixtures and the tests are built on.
//!
//! A dump is the `RepRecord` messages the device stores, framed as `[u32 little-endian length][record bytes]`.
//! Records come from the same `pb::encode_*` calls the firmware uses, so a fixture cannot drift from the wire format.
//!
//! The firmware never calls any of this: the generators live outside `calypso-core` so that the device's library carries nothing it does not use.

use calypso_core::motion::fusion::{Fusion, GRAVITY};
use calypso_core::motion::rep::{RepSeed, DETECTOR_VERSION};
use calypso_core::motion::trace::TracePoint;
use calypso_core::pb::{self, msg};
use calypso_core::sampler::{quantize, RepConfig, Sample, ACCEL_SCALE, SAMPLE_RATE_HZ};
use calypso_core::store::RECORD_BUF_BYTES;
use calypso_core::uuid;

use crate::motion::Set;
use crate::replay;

/// Sample interval used by the synthetic fixtures (100 Hz, like the device).
pub const SAMPLE_DT_MS: u32 = 10;

/// Milliseconds of rest every synthetic stream starts with.
///
/// Not politeness: the filter reports no linear acceleration until it has been still long enough to know which way gravity points, and the detector is not fed before that, so a stream that starts moving is not recordable anywhere in it.
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

/// One synthetic sample of level motion: `accel` is the bar's own acceleration, and the sensor reads that plus gravity.
///
/// The addition is the whole difference between a generator and a sensor, so it happens here rather than in each scenario: a stroke's acceleration is what a rep is made of, and `g` is only ever the platform.
pub fn at(t_ms: u32, accel: [f32; 3]) -> Sample {
    Sample {
        t_ms,
        accel: [accel[0], accel[1], accel[2] + GRAVITY].map(|a| quantize(a, ACCEL_SCALE)),
        gyro: [0; 3],
    }
}

/// Replay samples through the pipeline the firmware runs and pair each rep with its trace, dated by the build that produced it.
/// `trace_capacity` caps stored trace points as on the device, so a long rep comes back with the decimated flag set.
pub fn detect_and_trace(
    samples: &[Sample],
    trace_capacity: usize,
) -> Vec<(msg::RepSummary, Vec<TracePoint>)> {
    let out = replay::run(
        samples,
        Fusion::new(),
        [0.0; 3],
        trace_capacity,
        &mut |_, _, _, _| {},
    );
    out.summaries
        .into_iter()
        .zip(out.traces)
        .map(|(mut summary, trace)| {
            summary.set_config(pb::config_message(&RepConfig::of(
                SAMPLE_RATE_HZ,
                DETECTOR_VERSION,
            )));
            (summary, trace)
        })
        .collect()
}

/// Frame one encoded record into a dump: `[u32 LE length][record bytes]`.
pub fn push_record(dump: &mut Vec<u8>, record: &[u8]) {
    dump.extend_from_slice(&(record.len() as u32).to_le_bytes());
    dump.extend_from_slice(record);
}

/// Append a rep (summary record, plotted chunks, and the raw trace when one is given) to a dump.
/// Returns the number of records written. The order is the log's, so a reader finds either trace from the summary.
pub fn push_rep(
    dump: &mut Vec<u8>,
    rec_buf: &mut [u8],
    summary: &mut msg::RepSummary,
    computed: &[TracePoint],
    raw: Option<&[Sample]>,
) -> Result<u32, pb::Error> {
    // The chunks carry the same identity as the summary, and that identity is the summary's — deriving it here is what keeps them together.
    let rep_uuid = pb::summary_uuid(summary).ok_or(pb::Error::Decode)?;
    summary.computed_len = computed.len() as u32;
    summary.computed_chunks = pb::chunk_count_for(computed.len());
    summary.raw_len = raw.map_or(0, |r| r.len()) as u32;
    summary.raw_chunks = pb::chunk_count_for(summary.raw_len as usize);
    let n = pb::encode_summary(summary, rec_buf)?;
    push_record(dump, &rec_buf[..n]);

    let t0_ms = summary.t_start_ms;
    let mut written = 1;
    for (index, chunk) in computed.chunks(pb::TRACE_CHUNK_SAMPLES).enumerate() {
        // Fixtures are stored records, not a stream: total_chunks stays 0.
        let n = pb::encode_computed_chunk(rep_uuid, index as u32, t0_ms, 0, chunk, rec_buf)?;
        push_record(dump, &rec_buf[..n]);
        written += 1;
    }
    for (index, chunk) in raw
        .unwrap_or(&[])
        .chunks(pb::TRACE_CHUNK_SAMPLES)
        .enumerate()
    {
        let n = pb::encode_raw_chunk(rep_uuid, index as u32, t0_ms, 0, chunk, rec_buf)?;
        push_record(dump, &rec_buf[..n]);
        written += 1;
    }
    Ok(written)
}

/// A set's stored reps as a dump, with fixture identities in the order they were detected.
pub fn set_dump(set: &Set, trace_capacity: usize) -> Vec<u8> {
    let mut rec_buf = [0u8; RECORD_BUF_BYTES];
    let mut dump = new_dump(&mut rec_buf);
    for (i, (mut summary, trace)) in detect_and_trace(&set.samples(), trace_capacity)
        .into_iter()
        .enumerate()
    {
        // Deterministic identities: fixtures are regenerated and compared.
        summary.rep_uuid = pb::uuid_to_field(&uuid::fixture_uuid(i as u32));
        push_rep(&mut dump, &mut rec_buf, &mut summary, &trace, None).unwrap();
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

/// One stored window out of a `.pblog`, with the samples the device recorded and the filter state the stored summary says its replay starts from.
pub struct Window {
    /// The rep's uuid, lower-case hex, so a run can name one.
    pub uuid: String,
    pub t_start_ms: u32,
    /// The travel the device's own summary reports, for comparison with what a replay makes of it.
    pub rom: f32,
    pub state: RepSeed,
    pub samples: Vec<Sample>,
}

/// Every window in a `.pblog` that carries a raw trace, in log order.
pub fn capture_windows(dump: &[u8]) -> (Vec<Window>, bool) {
    let (records, truncated) = read_dump(dump);
    let mut windows: Vec<Window> = Vec::new();
    for record in &records {
        match pb::decode_record(record).expect("a record in the dump decodes") {
            pb::Record::Summary(s) => {
                if s.raw_len == 0 {
                    continue;
                }
                windows.push(Window {
                    uuid: s.rep_uuid.iter().map(|b| format!("{b:02x}")).collect(),
                    t_start_ms: s.t_start_ms,
                    rom: s.rom,
                    state: pb::state_from_message(&s.state),
                    samples: Vec::with_capacity(s.raw_len as usize),
                });
            }
            pb::Record::Raw(r) => {
                let Some(w) = windows.last_mut() else {
                    panic!("a raw chunk before any summary")
                };
                // A point's time is an offset from the rep's first sample, which is the summary's `t_start_ms` — the same origin the plotted trace counts from.
                for p in &r.samples {
                    let mut s = pb::raw_point_from_message(p);
                    s.t_ms = s.t_ms.saturating_add(w.t_start_ms);
                    w.samples.push(s);
                }
            }
            _ => {}
        }
    }
    (windows, truncated)
}
