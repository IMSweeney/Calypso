//! Protobuf wire format for rep records and the BLE sync protocol.
//!
//! The schema is `proto/calypso.proto` (shared with the app) and the Rust types are generated from it at build time (see `build.rs`); those types **are** the records and messages.
//! What is left here is the framing above them — the log's `[u32 len][record]` frame, the BLE envelope, and encode/decode into a caller's buffer, since the device has no allocator — and the boundary types where the on-chip representation is not the wire one.
//!
//! Log layout: each rep is one `RepSummary` record followed by its `computed_chunks` `TraceChunk` records and then, on a device that recorded them, its `raw_chunks` `RawChunk` records — all carrying the same `rep_uuid` — because a log record must fit one 4 KB erase sector (see [`crate::flashlog`]). The order is what lets a trace be found from the summary's record id alone.

use crate::motion::fusion::FusionState;
use crate::motion::rep::RepSeed;
use crate::motion::trace::TracePoint;
use crate::sampler::{quantize, RepConfig, Sample, QUAT_SCALE};
use crate::uuid::{RepUuid, UUID_LEN};
use micropb::MessageDecode;

/// Generated Protobuf types.
pub mod proto {
    #![allow(clippy::all, nonstandard_style, unused, irrefutable_let_patterns)]
    include!(concat!(env!("OUT_DIR"), "/calypso.rs"));
}

pub use proto::calypso_ as msg;

/// Trace samples per [`msg::TraceChunk`]. Must match the `max_len` configured in `build.rs` (asserted by a test).
pub const TRACE_CHUNK_SAMPLES: usize = 48;

/// Samples per live [`msg::ImuChunk`], which is one notification on the 244-byte characteristic.
/// Must match the `max_len` configured in `build.rs`; a test pins the worst-case encoding against the characteristic's size.
pub const IMU_CHUNK_SAMPLES: usize = 8;

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
/// The two chunk kinds store a whole chunk inline (~2 KB, bounded by [`TRACE_CHUNK_SAMPLES`]); the `#[allow]` is deliberate rather than a `Box`, which would need an allocator.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, PartialEq)]
pub enum Record {
    Summary(msg::RepSummary),
    Computed(msg::TraceChunk),
    Raw(msg::RawChunk),
    Clock(msg::BootClock),
    /// A live IMU chunk from a phone's recording. The device never writes one.
    Imu(msg::ImuChunk),
}

/// How many records a trace of `len` points takes, either kind: the two chunk messages hold the same number of samples.
pub fn chunk_count_for(len: usize) -> u32 {
    if len == 0 {
        return 0;
    }
    len.div_ceil(TRACE_CHUNK_SAMPLES) as u32
}

/// A rep UUID in its wire form: 16 bytes in a fixed-capacity container.
pub fn uuid_to_field(rep_uuid: &RepUuid) -> heapless::Vec<u8, UUID_LEN> {
    let mut field = heapless::Vec::new();
    // Capacity is UUID_LEN, so this cannot fail.
    let _ = field.extend_from_slice(rep_uuid);
    field
}

pub fn uuid_from_field(field: &[u8]) -> Option<RepUuid> {
    if field.len() != UUID_LEN {
        return None;
    }
    let mut uuid = [0u8; UUID_LEN];
    uuid.copy_from_slice(field);
    Some(uuid)
}

pub fn summary_uuid(s: &msg::RepSummary) -> Option<RepUuid> {
    uuid_from_field(&s.rep_uuid)
}

pub fn computed_uuid(c: &msg::TraceChunk) -> Option<RepUuid> {
    uuid_from_field(&c.rep_uuid)
}

pub fn raw_uuid(c: &msg::RawChunk) -> Option<RepUuid> {
    uuid_from_field(&c.rep_uuid)
}

/// A rep's sensor configuration in its wire form.
pub fn config_message(c: &RepConfig) -> msg::RepConfig {
    msg::RepConfig {
        accel_range_g: c.accel_range_g,
        gyro_range_dps: c.gyro_range_dps,
        accel_bandwidth_hz: c.accel_bandwidth_hz,
        gyro_bandwidth_hz: c.gyro_bandwidth_hz,
        sample_rate_hz: c.sample_rate_hz,
        detector_version: c.detector_version,
    }
}

/// Back-convert a wire state, which is what a stored rep is replayed from.
pub fn state_from_message(s: &msg::RepState) -> RepSeed {
    RepSeed {
        filter: FusionState {
            attitude: [s.qw, s.qx, s.qy, s.qz],
            accel_bias: [s.accel_bias_x, s.accel_bias_y, s.accel_bias_z],
            gyro_bias: [s.gyro_bias_x, s.gyro_bias_y, s.gyro_bias_z],
            resting: s.settled,
            ready: true,
        },
        velocity: [s.vx, s.vy, s.vz],
    }
}

/// One point of the plotted trace, with the time relative to `t0_ms` (the rep's first sample).
fn computed_point_message(p: &TracePoint, t0_ms: u32) -> msg::TraceComputedPoint {
    msg::TraceComputedPoint {
        t_offset_ms: p.t_ms.saturating_sub(t0_ms),
        px: p.pos[0],
        py: p.pos[1],
        pz: p.pos[2],
        qw: p.quat[0] as f32 / QUAT_SCALE as f32,
        qx: p.quat[1] as f32 / QUAT_SCALE as f32,
        qy: p.quat[2] as f32 / QUAT_SCALE as f32,
        qz: p.quat[3] as f32 / QUAT_SCALE as f32,
    }
}

/// One sample of the raw trace, with the time relative to `t0_ms` (the rep's first sample).
fn raw_point_message(s: &Sample, t0_ms: u32) -> msg::TraceRawPoint {
    msg::TraceRawPoint {
        t_offset_ms: s.t_ms.saturating_sub(t0_ms),
        ax: s.accel[0] as i32,
        ay: s.accel[1] as i32,
        az: s.accel[2] as i32,
        gx: s.gyro[0] as i32,
        gy: s.gyro[1] as i32,
        gz: s.gyro[2] as i32,
    }
}

/// Back-convert one point of the plotted trace. `t_ms` is relative to the rep's first sample (see `TraceChunk::first_t_offset_ms`).
pub fn computed_point_from_message(s: &msg::TraceComputedPoint) -> TracePoint {
    TracePoint {
        t_ms: s.t_offset_ms,
        pos: [s.px, s.py, s.pz],
        quat: [s.qw, s.qx, s.qy, s.qz].map(|q| quantize(q, QUAT_SCALE)),
    }
}

/// Back-convert one sample of the raw trace, ready to replay: the same fixed point the device read.
pub fn raw_point_from_message(s: &msg::TraceRawPoint) -> Sample {
    Sample {
        t_ms: s.t_offset_ms,
        accel: [s.ax as i16, s.ay as i16, s.az as i16],
        gyro: [s.gx as i16, s.gy as i16, s.gz as i16],
    }
}

/// Build a chunk of the plotted trace (at most [`TRACE_CHUNK_SAMPLES`] points are taken), times relative to `t0_ms`.
///
/// `total_chunks` is the number of chunks in the stream this record belongs to, or 0 for a record being stored in the log (see the field's comment in `proto/calypso.proto`).
pub fn computed_chunk_message(
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
        let _ = chunk.samples.push(computed_point_message(p, t0_ms));
    }
    chunk
}

/// Build a chunk of the raw trace, the same way.
pub fn raw_chunk_message(
    rep_uuid: RepUuid,
    index: u32,
    t0_ms: u32,
    total_chunks: u32,
    samples: &[Sample],
) -> msg::RawChunk {
    let mut chunk = msg::RawChunk {
        rep_uuid: uuid_to_field(&rep_uuid),
        index,
        first_t_offset_ms: samples
            .first()
            .map(|s| s.t_ms.saturating_sub(t0_ms))
            .unwrap_or(0),
        total_chunks,
        samples: heapless::Vec::new(),
    };
    for s in samples.iter().take(TRACE_CHUNK_SAMPLES) {
        let _ = chunk.samples.push(raw_point_message(s, t0_ms));
    }
    chunk
}

/// Fill `chunk` with one live slice of `samples`, whose times are the device's uptime.
///
/// `seq` numbers the chunks of one connection's stream, and `first_t_ms` is the origin each point's `t_offset_ms` is relative to.
/// Takes the message rather than returning one, so the device can reuse its heap buffer instead of building a 200-byte value on the stack every 80 ms.
pub fn fill_imu_chunk(chunk: &mut msg::ImuChunk, seq: u32, samples: &[Sample]) {
    let first_t_ms = samples.first().map(|s| s.t_ms).unwrap_or(0);
    chunk.first_t_ms = first_t_ms;
    chunk.sample_rate_hz = crate::sampler::SAMPLE_RATE_HZ;
    chunk.seq = seq;
    chunk.samples.clear();
    for s in samples.iter().take(IMU_CHUNK_SAMPLES) {
        // Capacity is IMU_CHUNK_SAMPLES, so this cannot fail.
        let _ = chunk.samples.push(raw_point_message(s, first_t_ms));
    }
}

/// Build one live chunk from `samples`; the same thing as [`fill_imu_chunk`] for callers without a buffer.
pub fn imu_chunk_message(seq: u32, samples: &[Sample]) -> msg::ImuChunk {
    let mut chunk = msg::ImuChunk::default();
    fill_imu_chunk(&mut chunk, seq, samples);
    chunk
}

/// Encode a rep summary as a log record. Returns the encoded length.
/// The caller fills in `rep_uuid` and `chunk_count`; the summary is the wire type, so nothing is converted.
pub fn encode_summary(summary: &msg::RepSummary, out: &mut [u8]) -> Result<usize, Error> {
    record(msg::RepRecord_::Body::Summary(summary.clone()), out)
}

/// Encode a chunk of the plotted trace as a log record. Returns the encoded length.
pub fn encode_computed_chunk(
    rep_uuid: RepUuid,
    index: u32,
    t0_ms: u32,
    total_chunks: u32,
    points: &[TracePoint],
    out: &mut [u8],
) -> Result<usize, Error> {
    record(
        msg::RepRecord_::Body::Computed(computed_chunk_message(
            rep_uuid,
            index,
            t0_ms,
            total_chunks,
            points,
        )),
        out,
    )
}

/// Encode a chunk of the raw trace as a log record. Returns the encoded length.
pub fn encode_raw_chunk(
    rep_uuid: RepUuid,
    index: u32,
    t0_ms: u32,
    total_chunks: u32,
    samples: &[Sample],
    out: &mut [u8],
) -> Result<usize, Error> {
    record(
        msg::RepRecord_::Body::Raw(raw_chunk_message(
            rep_uuid,
            index,
            t0_ms,
            total_chunks,
            samples,
        )),
        out,
    )
}

/// Encode a boot's clock as a log record. Returns the encoded length.
///
/// Written once per boot at mount (`epoch_s` 0, so the boot before it is not read as this one) and once when the client sets the clock (`crate::clock`).
pub fn encode_clock(epoch_s: u32, out: &mut [u8]) -> Result<usize, Error> {
    record(
        msg::RepRecord_::Body::Clock(msg::BootClock { epoch_s }),
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
        message
            .encode(&mut encoder)
            .map_err(|_| Error::BufferTooSmall)?;
    }
    Ok(capacity - writer.len())
}

/// Decode a log record, building the message on the caller's stack (~2 KB). Host code and tests; firmware should use [`decode_record_into`].
pub fn decode_record(bytes: &[u8]) -> Result<Record, Error> {
    let mut message = msg::RepRecord::default();
    decode_record_into(&mut message, bytes)?;
    match message.body {
        Some(msg::RepRecord_::Body::Summary(s)) => Ok(Record::Summary(s)),
        Some(msg::RepRecord_::Body::Computed(t)) => Ok(Record::Computed(t)),
        Some(msg::RepRecord_::Body::Raw(r)) => Ok(Record::Raw(r)),
        Some(msg::RepRecord_::Body::Clock(c)) => Ok(Record::Clock(c)),
        Some(msg::RepRecord_::Body::Imu(i)) => Ok(Record::Imu(i)),
        None => Err(Error::Empty),
    }
}

/// Decode a log record into a caller-owned message.
/// The buffer is reused as-is: whatever the last record left in it is dropped first.
/// Returns [`Error::Empty`] if the bytes decoded cleanly but carried no body, so `is_ok()` still means "there is a record here".
pub fn decode_record_into(message: &mut msg::RepRecord, bytes: &[u8]) -> Result<(), Error> {
    // Reset through the oneof rather than `*message = RepRecord::default()`: the latter builds a whole ~2 KB record as a temporary, which is the very thing this function exists to avoid.
    message.body = None;
    message
        .decode_from_bytes(bytes)
        .map_err(|_| Error::Decode)?;
    if message.body.is_none() {
        return Err(Error::Empty);
    }
    Ok(())
}

/// Encode a trace request inside the `SyncRequest` envelope the app writes — the envelope is what the write characteristic carries.
pub fn encode_trace_request(request: &msg::TraceRequest, out: &mut [u8]) -> Result<usize, Error> {
    let envelope = msg::SyncRequest {
        body: Some(msg::SyncRequest_::Body::Trace(request.clone())),
    };
    encode_message(&envelope, out)
}

pub fn decode_sync_request(bytes: &[u8]) -> Result<msg::SyncRequest, Error> {
    let mut message = msg::SyncRequest::default();
    message
        .decode_from_bytes(bytes)
        .map_err(|_| Error::Decode)?;
    Ok(message)
}

pub fn decode_sync(bytes: &[u8]) -> Result<msg::SyncMessage, Error> {
    let mut message = msg::SyncMessage::default();
    message
        .decode_from_bytes(bytes)
        .map_err(|_| Error::Decode)?;
    Ok(message)
}

/// The trace request inside a `SyncRequest`, or [`Error::Empty`] if the bytes carry some other request (or none).
pub fn decode_trace_request(bytes: &[u8]) -> Result<msg::TraceRequest, Error> {
    Ok(match decode_sync_request(bytes)?.body {
        Some(msg::SyncRequest_::Body::Trace(trace)) => trace,
        _ => return Err(Error::Empty),
    })
}

/// Encode the dev switch inside the `SyncRequest` envelope. Its arm of `SyncRequest` is an operation rather than a value, so it travels on the pipe like a trace request does.
pub fn encode_dev_mode(enabled: bool, out: &mut [u8]) -> Result<usize, Error> {
    let envelope = msg::SyncRequest {
        body: Some(msg::SyncRequest_::Body::DevMode(msg::DevMode { enabled })),
    };
    encode_message(&envelope, out)
}
