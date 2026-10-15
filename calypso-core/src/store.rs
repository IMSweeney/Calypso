//! Storing reps in the flash log and reading them back.
//!
//! One rep becomes one `RepSummary` record followed by `chunk_count`
//! `TraceChunk` records with the same `rep_id` (see `proto/calypso.proto`).
//! Records are chunked because a log record must fit inside one erase sector
//! (see [`crate::flashlog`]).
//!
//! The record buffer is passed in by the caller: it is too large for the
//! device's static RAM region, so the firmware allocates one on the heap and
//! reuses it for every record.

use crate::flashlog::{Error as LogError, FlashLog};
use crate::pb::{self, TRACE_CHUNK_SAMPLES};
use crate::rep::{RepSummary, TraceBuf, TracePoint};
use embedded_storage::nor_flash::NorFlash;

/// Size of the caller's record buffer. A full trace chunk of 48 samples
/// encodes to at most 2459 bytes even with worst-case field values (asserted
/// by a test), and the flash log's record limit for a 4 KB sector is 4076
/// bytes, so this covers every record we can produce with margin.
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

/// Append one rep (summary + trace chunks) to the log.
///
/// `rec_buf` must be at least [`RECORD_BUF_BYTES`]; it is scratch space
/// reused for each record. Returns the number of records written.
pub fn append_rep<'a, N: NorFlash>(
    log: &mut FlashLog<'a, N>,
    rec_buf: &mut [u8],
    rep_id: u32,
    summary: &RepSummary,
    trace: &[TracePoint],
) -> Result<u32, Error<N::Error>> {
    let chunk_count = pb::chunk_count_for(trace.len());
    let n = pb::encode_summary(rep_id, summary, chunk_count, rec_buf)?;
    log.append(&rec_buf[..n])?;

    let t0_ms = trace.first().map(|p| p.t_ms).unwrap_or(0);
    let mut written = 1;
    for (index, chunk) in trace.chunks(TRACE_CHUNK_SAMPLES).enumerate() {
        let n = pb::encode_trace_chunk(rep_id, index as u32, t0_ms, chunk, rec_buf)?;
        log.append(&rec_buf[..n])?;
        written += 1;
    }
    Ok(written)
}

/// Visit every rep summary in the log, oldest first. `buf` must be large
/// enough for a summary record (a few hundred bytes); larger records are
/// skipped by the log itself.
pub fn for_each_summary<'a, N: NorFlash>(
    log: &mut FlashLog<'a, N>,
    buf: &mut [u8],
    mut f: impl FnMut(pb::msg::RepSummary),
) -> Result<(), Error<N::Error>> {
    log.for_each(buf, |_, payload| {
        if let Ok(pb::Record::Summary(s)) = pb::decode_record(payload) {
            f(s);
        }
    })?;
    Ok(())
}

/// Read one rep's trace back into `out`, in chunk order, oldest point first.
/// Returns the number of trace points recovered (0 if the rep isn't in the
/// log any more).
pub fn read_trace<'a, N: NorFlash>(
    log: &mut FlashLog<'a, N>,
    buf: &mut [u8],
    rep_id: u32,
    out: &mut TraceBuf<'_>,
) -> Result<usize, Error<N::Error>> {
    let mut expected_index = 0u32;
    log.for_each(buf, |_, payload| {
        let Ok(pb::Record::Trace(chunk)) = pb::decode_record(payload) else {
            return;
        };
        if chunk.rep_id != rep_id || chunk.index != expected_index {
            return;
        }
        for s in chunk.samples.iter() {
            out.push(pb::trace_point_from_message(s));
        }
        expected_index += 1;
    })?;
    Ok(out.len())
}
