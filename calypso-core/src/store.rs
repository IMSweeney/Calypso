//! Storing reps in the flash log and reading them back.
//!
//! The record buffer is the caller's: it is too large for the device's static RAM, so the firmware keeps one on the heap and reuses it.

use crate::flashlog::{Error as LogError, FlashLog};
use crate::motion::rep::{TraceBuf, TracePoint};
use crate::pb::{self, msg, TRACE_CHUNK_SAMPLES};
use crate::sampler::Sample;
use crate::uuid::RepUuid;
use embedded_storage::nor_flash::NorFlash;

/// Size of the caller's record buffer.
/// A full trace chunk of 48 samples encodes to at most 2459 bytes even with worst-case field values (asserted by a test), and the flash log's record limit for a 4 KB sector is 4076 bytes, so this covers every record we can produce with margin.
pub const RECORD_BUF_BYTES: usize = 3072;

#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Log(LogError<E>),
    Encode(pb::Error),
}

impl<E> From<LogError<E>> for Error<E> {
    fn from(e: LogError<E>) -> Self {
        Error::Log(e)
    }
}

impl<E> From<pb::Error> for Error<E> {
    fn from(e: pb::Error) -> Self {
        Error::Encode(e)
    }
}

/// Append one rep (summary, plotted trace, and the raw trace when one was recorded) to the log.
///
/// The order is part of the format: a rep's summary, then every computed chunk, then every raw chunk.
/// That is what lets a trace be found from the summary's record id alone, and what makes [`crate::sync::RepPlace`] three numbers instead of four.
///
/// `rec_buf` must be at least [`RECORD_BUF_BYTES`]; it is scratch reused for each record.
/// Returns the number of records written.
pub fn append_rep<'a, N: NorFlash>(
    log: &mut FlashLog<'a, N>,
    rec_buf: &mut [u8],
    summary: &mut msg::RepSummary,
    computed: &[TracePoint],
    raw: Option<&[Sample]>,
) -> Result<u32, Error<N::Error>> {
    let rep_uuid = pb::summary_uuid(summary).ok_or(Error::Encode(pb::Error::Decode))?;
    summary.computed_len = computed.len() as u32;
    summary.computed_chunks = pb::chunk_count_for(computed.len());
    summary.raw_len = raw.map_or(0, |r| r.len()) as u32;
    summary.raw_chunks = pb::chunk_count_for(summary.raw_len as usize);
    let n = pb::encode_summary(summary, rec_buf)?;
    log.append(&rec_buf[..n])?;

    // One origin for both traces: the window's first sample, which is what the summary's `t_start_ms` is.
    let t0_ms = summary.t_start_ms;
    let mut written = 1;
    for (index, chunk) in computed.chunks(TRACE_CHUNK_SAMPLES).enumerate() {
        let n = pb::encode_computed_chunk(rep_uuid, index as u32, t0_ms, 0, chunk, rec_buf)?;
        log.append(&rec_buf[..n])?;
        written += 1;
    }
    for (index, chunk) in raw.unwrap_or(&[]).chunks(TRACE_CHUNK_SAMPLES).enumerate() {
        let n = pb::encode_raw_chunk(rep_uuid, index as u32, t0_ms, 0, chunk, rec_buf)?;
        log.append(&rec_buf[..n])?;
        written += 1;
    }
    Ok(written)
}

/// Decode a record the log has already checked the CRC of.
/// Panics rather than skipping: a CRC-valid record that will not decode means the schema and the log disagree, not a torn write, and skipping it would hide that.
fn decode_or_panic(rec: &mut pb::msg::RepRecord, id: u32, payload: &[u8]) {
    if let Err(e) = pb::decode_record_into(rec, payload) {
        panic!(
            "log record {id} ({} bytes) is not a RepRecord: {e:?}",
            payload.len()
        );
    }
}

/// Append a boot's clock record, returning its id.
///
/// This only writes it: *when* it is written is the caller's, and the rule for reading it back is `crate::clock`'s.
pub fn append_clock<'a, N: NorFlash>(
    log: &mut FlashLog<'a, N>,
    buf: &mut [u8],
    epoch_s: u32,
) -> Result<u32, Error<N::Error>> {
    let n = pb::encode_clock(epoch_s, buf)?;
    Ok(log.append(&buf[..n])?)
}

/// Visit every record in the log, oldest first, with the id the log gave it.
/// `buf` must be large enough for the record being looked for (larger records are skipped by the log itself).
///
/// The id is what dates a record: a boot's clock is written *after* the reps it applies to, and the clock record names the id its boot started at (`crate::clock`).
///
/// The record is borrowed from `rec`, so it is only valid for the duration of the call.
pub fn for_each_record<'a, N: NorFlash>(
    log: &mut FlashLog<'a, N>,
    buf: &mut [u8],
    rec: &mut pb::msg::RepRecord,
    mut f: impl FnMut(u32, &pb::msg::RepRecord),
) -> Result<(), Error<N::Error>> {
    log.for_each(buf, |id, payload| {
        decode_or_panic(rec, id, payload);
        f(id, rec);
    })?;
    Ok(())
}

/// Visit every rep summary in the log, oldest first.
///
/// The summary is borrowed from `rec`, so it is only valid for the duration of the call.
pub fn for_each_summary<'a, N: NorFlash>(
    log: &mut FlashLog<'a, N>,
    buf: &mut [u8],
    rec: &mut pb::msg::RepRecord,
    mut f: impl FnMut(&pb::msg::RepSummary),
) -> Result<(), Error<N::Error>> {
    for_each_record(log, buf, rec, |_, record| {
        if let Some(pb::msg::RepRecord_::Body::Summary(s)) = &record.body {
            f(s);
        }
    })
}

/// How many records past a rep's chunks its walk may look before giving up.
///
/// The chunks follow the summary because `append_rep` writes them together under the log's lock, so this is headroom for the one record that can legitimately sit among them — nothing, in fact — rather than a budget: it bounds a walk that would otherwise keep reading after a rep whose chunks are not all there.
const TRACE_WALK_SLACK: u32 = 2;

/// Read one rep's trace back into `out` given where its summary sits in the log: the record after it is the rep's first chunk, and `chunk_count` chunks follow.
///
/// This is the read a trace request takes when the device knows where the rep's records are (the summary stream recorded it when it read the summary). It gives the same answer as `read_trace` for a walk of a dozen records instead of the whole log.
/// Returns the number of trace points recovered (0 if none of the rep's chunks are there any more).
pub fn read_trace_from<'a, N: NorFlash>(
    log: &mut FlashLog<'a, N>,
    buf: &mut [u8],
    rec: &mut pb::msg::RepRecord,
    first_chunk_id: u32,
    chunk_count: u32,
    rep_uuid: RepUuid,
    out: &mut TraceBuf<'_>,
) -> Result<usize, Error<N::Error>> {
    let mut expected_index = 0u32;
    log.for_each_from(
        first_chunk_id,
        chunk_count.saturating_add(TRACE_WALK_SLACK),
        buf,
        |id, payload| {
            if expected_index >= chunk_count {
                return false; // the whole trace, and nothing after it
            }
            decode_or_panic(rec, id, payload);
            let Some(pb::msg::RepRecord_::Body::Computed(chunk)) = &rec.body else {
                return true; // a clock, a summary, or the raw trace: not this rep's plotted one
            };
            if pb::computed_uuid(chunk) != Some(rep_uuid) || chunk.index != expected_index {
                return true;
            }
            for s in chunk.samples.iter() {
                out.push(pb::computed_point_from_message(s));
            }
            expected_index += 1;
            true
        },
    )?;
    Ok(out.len())
}

/// Read one rep's trace back into `out`, in chunk order, oldest point first, by walking the whole log.
/// Returns the number of trace points recovered (0 if the rep isn't in the log any more).
///
/// The fallback for a trace request this device cannot place: a rep whose summary it has not read this connection.
/// Prefer [`read_trace_from`], which is the same answer without the walk.
pub fn read_trace<'a, N: NorFlash>(
    log: &mut FlashLog<'a, N>,
    buf: &mut [u8],
    rec: &mut pb::msg::RepRecord,
    rep_uuid: RepUuid,
    out: &mut TraceBuf<'_>,
) -> Result<usize, Error<N::Error>> {
    let mut expected_index = 0u32;
    log.for_each(buf, |id, payload| {
        decode_or_panic(rec, id, payload);
        let Some(pb::msg::RepRecord_::Body::Computed(chunk)) = &rec.body else {
            return;
        };
        if pb::computed_uuid(chunk) != Some(rep_uuid) || chunk.index != expected_index {
            return;
        }
        for s in chunk.samples.iter() {
            out.push(pb::computed_point_from_message(s));
        }
        expected_index += 1;
    })?;
    Ok(out.len())
}

/// Read one rep's **raw** trace back into `out`, in chunk order, oldest sample first, starting at a known record.
/// Returns the number of samples recovered.
pub fn read_raw_from<'a, N: NorFlash>(
    log: &mut FlashLog<'a, N>,
    buf: &mut [u8],
    rec: &mut pb::msg::RepRecord,
    first_chunk_id: u32,
    chunk_count: u32,
    rep_uuid: RepUuid,
    out: &mut [Sample],
) -> Result<usize, Error<N::Error>> {
    let mut expected_index = 0u32;
    let mut len = 0usize;
    log.for_each_from(
        first_chunk_id,
        chunk_count.saturating_add(TRACE_WALK_SLACK),
        buf,
        |id, payload| {
            if expected_index >= chunk_count {
                return false;
            }
            decode_or_panic(rec, id, payload);
            let Some(pb::msg::RepRecord_::Body::Raw(chunk)) = &rec.body else {
                return true;
            };
            if pb::raw_uuid(chunk) != Some(rep_uuid) || chunk.index != expected_index {
                return true;
            }
            for s in chunk.samples.iter() {
                if len < out.len() {
                    out[len] = pb::raw_point_from_message(s);
                    len += 1;
                }
            }
            expected_index += 1;
            true
        },
    )?;
    Ok(len)
}

/// The same by walking the whole log: the fallback for a rep this connection has not placed.
pub fn read_raw<'a, N: NorFlash>(
    log: &mut FlashLog<'a, N>,
    buf: &mut [u8],
    rec: &mut pb::msg::RepRecord,
    rep_uuid: RepUuid,
    out: &mut [Sample],
) -> Result<usize, Error<N::Error>> {
    let mut expected_index = 0u32;
    let mut len = 0usize;
    log.for_each(buf, |id, payload| {
        decode_or_panic(rec, id, payload);
        let Some(pb::msg::RepRecord_::Body::Raw(chunk)) = &rec.body else {
            return;
        };
        if pb::raw_uuid(chunk) != Some(rep_uuid) || chunk.index != expected_index {
            return;
        }
        for s in chunk.samples.iter() {
            if len < out.len() {
                out[len] = pb::raw_point_from_message(s);
                len += 1;
            }
        }
        expected_index += 1;
    })?;
    Ok(len)
}
