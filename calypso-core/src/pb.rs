//! Protobuf wire format for rep records and the BLE sync protocol.
//!
//! The schema lives in `proto/calypso.proto` (shared with the app); the Rust
//! types are generated from it at build time by `micropb-gen` (see
//! `build.rs`). This module wraps them with conversions to and from the
//! platform-independent types in [`crate::rep`] and [`crate::sampler`], plus
//! record-level encode/decode into caller-provided byte buffers.
//!
//! Log layout: each rep is one `RepSummary` record followed by
//! `chunk_count` `TraceChunk` records carrying the same `rep_uuid`. Records are
//! chunked because a flash log record must fit one 4 KB erase sector (see
//! [`crate::flashlog`]).

use crate::calibration::Calibration;
use crate::rep::TracePoint;
use crate::uuid::{RepUuid, UUID_LEN};
use micropb::MessageDecode;

/// Generated Protobuf types.
pub mod proto {
    #![allow(clippy::all, nonstandard_style, unused, irrefutable_let_patterns)]
    include!(concat!(env!("OUT_DIR"), "/calypso.rs"));
}

/// Generated message types, for callers that need them directly.
pub use proto::calypso_ as msg;

/// Trace samples per [`msg::TraceChunk`]. Must match the `max_len`
/// configured in `build.rs` (asserted by a test).
pub const TRACE_CHUNK_SAMPLES: usize = 48;

/// Errors from the record layer.
#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    /// The output buffer was too small for the encoded record.
    BufferTooSmall,
    /// Decoding failed: the bytes aren't a valid record.
    Decode,
    /// Decoded cleanly, but carried no body.
    Empty,
}

/// A decoded log record.
///
/// The `Trace` variant is large (a full chunk of samples is stored inline, so
/// the enum is ~2 KB) — bounded by [`TRACE_CHUNK_SAMPLES`]. Callers that
/// decode in a memory-tight context should keep the value short-lived; the
/// `#[allow]` is deliberate rather than boxing, which would need an allocator.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, PartialEq)]
pub enum Record {
    Summary(msg::RepSummary),
    Trace(msg::TraceChunk),
}

/// Number of trace chunks needed for `trace_len` points.
pub fn chunk_count_for(trace_len: usize) -> u32 {
    if trace_len == 0 {
        return 0;
    }
    trace_len.div_ceil(TRACE_CHUNK_SAMPLES) as u32
}

/// A rep UUID in its wire form: 16 bytes in a fixed-capacity container.
pub fn uuid_to_field(rep_uuid: &RepUuid) -> heapless::Vec<u8, UUID_LEN> {
    let mut field = heapless::Vec::new();
    // Capacity is UUID_LEN, so this cannot fail.
    let _ = field.extend_from_slice(rep_uuid);
    field
}

/// A rep UUID read back off the wire, or `None` if it isn't 16 bytes.
pub fn uuid_from_field(field: &[u8]) -> Option<RepUuid> {
    if field.len() != UUID_LEN {
        return None;
    }
    let mut uuid = [0u8; UUID_LEN];
    uuid.copy_from_slice(field);
    Some(uuid)
}

/// The UUID of a decoded summary record.
pub fn summary_uuid(s: &msg::RepSummary) -> Option<RepUuid> {
    uuid_from_field(&s.rep_uuid)
}

/// The UUID of a decoded trace chunk.
pub fn chunk_uuid(c: &msg::TraceChunk) -> Option<RepUuid> {
    uuid_from_field(&c.rep_uuid)
}

/// Convert calibration levels into their wire form.
///
/// `ready` is the device's own gate (see [`Calibration::is_ready`]) rather than
/// something the app re-derives, so the two cannot disagree about whether reps
/// are being recorded.
pub fn calibration_message(c: &Calibration) -> msg::CalibrationStatus {
    msg::CalibrationStatus {
        sys: c.sys as u32,
        accel: c.accel as u32,
        gyro: c.gyro as u32,
        mag: c.mag as u32,
        ready: c.is_ready(),
    }
}

/// Convert one trace point into its wire form, with timestamps relative to
/// `t0_ms` (the rep's first trace point).
pub fn trace_sample_message(p: &TracePoint, t0_ms: u32) -> msg::TraceSample {
    msg::TraceSample {
        t_offset_ms: p.t_ms.saturating_sub(t0_ms),
        px: p.pos[0],
        py: p.pos[1],
        pz: p.pos[2],
        ax: p.accel[0] as i32,
        ay: p.accel[1] as i32,
        az: p.accel[2] as i32,
        qw: p.quat[0] as i32,
        qx: p.quat[1] as i32,
        qy: p.quat[2] as i32,
        qz: p.quat[3] as i32,
    }
}

/// Back-convert a wire sample. `t_ms` is relative to the rep's first point
/// (see `TraceChunk::first_t_offset_ms`).
pub fn trace_point_from_message(s: &msg::TraceSample) -> TracePoint {
    TracePoint {
        t_ms: s.t_offset_ms,
        pos: [s.px, s.py, s.pz],
        accel: [s.ax as i16, s.ay as i16, s.az as i16],
        quat: [s.qw as i16, s.qx as i16, s.qy as i16, s.qz as i16],
    }
}

/// Build a chunk message from trace points (at most [`TRACE_CHUNK_SAMPLES`]
/// are taken), timestamps relative to `t0_ms`.
///
/// `total_chunks` is the number of chunks in the stream this record belongs to,
/// or 0 for a record being stored in the log (see the field's comment in
/// `proto/calypso.proto`).
pub fn chunk_message(
    rep_uuid: RepUuid,
    index: u32,
    t0_ms: u32,
    total_chunks: u32,
    points: &[TracePoint],
) -> msg::TraceChunk {
    let mut chunk = msg::TraceChunk {
        rep_uuid: uuid_to_field(&rep_uuid),
        index,
        first_t_offset_ms: points
            .first()
            .map(|p| p.t_ms.saturating_sub(t0_ms))
            .unwrap_or(0),
        total_chunks,
        samples: heapless::Vec::new(),
    };
    for p in points.iter().take(TRACE_CHUNK_SAMPLES) {
        // Capacity is TRACE_CHUNK_SAMPLES, so this cannot fail.
        let _ = chunk.samples.push(trace_sample_message(p, t0_ms));
    }
    chunk
}

/// Encode a rep summary as a log record. Returns the encoded length.
///
/// The summary is the wire type, so there is nothing to convert: the caller fills
/// in `rep_uuid` (minted when the rep closed) and `chunk_count` ([`chunk_count_for`]
/// of the trace it is about to store), and this puts it in the log's record
/// envelope.
pub fn encode_summary(summary: &msg::RepSummary, out: &mut [u8]) -> Result<usize, Error> {
    record(msg::RepRecord_::Body::Summary(summary.clone()), out)
}

/// Encode a trace chunk as a log record. Returns the encoded length.
pub fn encode_trace_chunk(
    rep_uuid: RepUuid,
    index: u32,
    t0_ms: u32,
    total_chunks: u32,
    points: &[TracePoint],
    out: &mut [u8],
) -> Result<usize, Error> {
    record(
        msg::RepRecord_::Body::Trace(chunk_message(rep_uuid, index, t0_ms, total_chunks, points)),
        out,
    )
}

/// Encode a sync message (device -> app).
pub fn encode_sync(body: msg::SyncMessage_::Body, out: &mut [u8]) -> Result<usize, Error> {
    let message = msg::SyncMessage { body: Some(body) };
    encode_message(&message, out)
}

fn record(body: msg::RepRecord_::Body, out: &mut [u8]) -> Result<usize, Error> {
    let message = msg::RepRecord { body: Some(body) };
    encode_message(&message, out)
}

fn encode_message<M: micropb::MessageEncode>(message: &M, out: &mut [u8]) -> Result<usize, Error> {
    let capacity = out.len();
    let mut writer: &mut [u8] = out;
    {
        let mut encoder = micropb::PbEncoder::new(&mut writer);
        message.encode(&mut encoder).map_err(|_| Error::BufferTooSmall)?;
    }
    Ok(capacity - writer.len())
}

/// Decode a log record.
///
/// This builds the message on the caller's stack — ~2 KB, because a full trace
/// chunk is stored inline — so it is for host code and tests. Firmware should
/// hold one [`msg::RepRecord`] (on the heap) and call [`decode_record_into`]: a
/// 2 KB stack frame in an async task ends up inside the task's future, and on a
/// 4 KB-sector device that is real RAM.
pub fn decode_record(bytes: &[u8]) -> Result<Record, Error> {
    let mut message = msg::RepRecord::default();
    decode_record_into(&mut message, bytes)?;
    match message.body {
        Some(msg::RepRecord_::Body::Summary(s)) => Ok(Record::Summary(s)),
        Some(msg::RepRecord_::Body::Trace(t)) => Ok(Record::Trace(t)),
        None => Err(Error::Empty),
    }
}

/// Decode a log record into a caller-owned message, which is what makes scanning
/// a whole log allocate nothing and keep nothing large on the stack.
///
/// The buffer is reused as-is: whatever the last record left in it is dropped
/// first. Returns [`Error::Empty`] if the bytes decoded cleanly but carried no
/// body, so `is_ok()` still means "there is a record here".
pub fn decode_record_into(message: &mut msg::RepRecord, bytes: &[u8]) -> Result<(), Error> {
    // Reset through the oneof rather than `*message = RepRecord::default()`:
    // the latter builds a whole ~2 KB record as a temporary, which is the very
    // thing this function exists to avoid.
    message.body = None;
    message.decode_from_bytes(bytes).map_err(|_| Error::Decode)?;
    if message.body.is_none() {
        return Err(Error::Empty);
    }
    Ok(())
}

/// Encode a trace request inside the `SyncRequest` envelope the app writes, which
/// is one encoding rather than two: the envelope is what the write characteristic
/// carries, and what a client has to produce to ask for anything.
pub fn encode_trace_request(request: &msg::TraceRequest, out: &mut [u8]) -> Result<usize, Error> {
    let envelope = msg::SyncRequest {
        body: Some(msg::SyncRequest_::Body::Trace(request.clone())),
    };
    encode_message(&envelope, out)
}

/// Decode a `SyncRequest` (app -> device).
pub fn decode_sync_request(bytes: &[u8]) -> Result<msg::SyncRequest, Error> {
    let mut message = msg::SyncRequest::default();
    message.decode_from_bytes(bytes).map_err(|_| Error::Decode)?;
    Ok(message)
}

/// Decode a sync message (app -> device or device -> app).
pub fn decode_sync(bytes: &[u8]) -> Result<msg::SyncMessage, Error> {
    let mut message = msg::SyncMessage::default();
    message.decode_from_bytes(bytes).map_err(|_| Error::Decode)?;
    Ok(message)
}

/// The trace request inside a `SyncRequest`, or [`Error::Empty`] if the bytes
/// carry some other request (or none).
pub fn decode_trace_request(bytes: &[u8]) -> Result<msg::TraceRequest, Error> {
    Ok(match decode_sync_request(bytes)?.body {
        Some(msg::SyncRequest_::Body::Trace(trace)) => trace,
        None => return Err(Error::Empty),
    })
}
