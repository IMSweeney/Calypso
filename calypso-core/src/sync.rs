//! BLE sync streaming: turning stored reps into `SyncMessage` payloads.
//!
//! The device's ATT MTU caps a notification at [`BLE_NOTIFY_BYTES`] of
//! payload, while a stored trace is one `TraceChunk` per 24 samples (see
//! `pb::TRACE_CHUNK_SAMPLES`). So traces are re-chunked on the way out:
//! [`TraceStreamer`] walks the stored points and emits one `SyncMessage` per
//! [`BLE_CHUNK_SAMPLES`] points, which the app reassembles by `index`.
//!
//! Platform-independent, so the whole flow is host-tested (see
//! `tests/sync.rs`) without a radio.

use crate::calibration::Calibration;
use crate::pb::{self, msg, TRACE_CHUNK_SAMPLES};
use crate::rep::TracePoint;
use crate::uuid::RepUuid;

/// Usable notification payload with a 247-byte ATT MTU (the value BLE stacks
/// commonly negotiate): MTU - 3 bytes of ATT header.
pub const BLE_NOTIFY_BYTES: usize = 244;

/// Trace points per notification.
///
/// Three, not four, deliberately: Apple platforms manage the ATT MTU themselves
/// and historically negotiated ~185 bytes (182-byte payload), where four samples
/// (~200 bytes with the wrapper) would not fit. Three stays inside that with
/// room to spare. The right long-term fix is for the *peripheral* to size chunks
/// to the MTU it actually negotiated — it learns that per connection, and
/// `TraceChunk.total_chunks` means the client copes with any chunk size — but
/// until the device does that, the conservative number is the correct one.
pub const BLE_CHUNK_SAMPLES: usize = 3;

/// Streams a rep's trace to the app, one `SyncMessage` at a time.
pub struct TraceStreamer<'a> {
    rep_uuid: RepUuid,
    t0_ms: u32,
    points: &'a [TracePoint],
    /// Next BLE chunk index to emit.
    next_index: u32,
    /// When set, emit only this chunk index and then stop.
    only: Option<u32>,
}

impl<'a> TraceStreamer<'a> {
    /// Stream every chunk of the trace (a `TraceRequest { all_chunks: true }`).
    pub fn all(rep_uuid: RepUuid, t0_ms: u32, points: &'a [TracePoint]) -> Self {
        TraceStreamer {
            rep_uuid,
            t0_ms,
            points,
            next_index: 0,
            only: None,
        }
    }

    /// Stream a single chunk (`TraceRequest { all_chunks: false, chunk_index }`).
    pub fn one(rep_uuid: RepUuid, t0_ms: u32, points: &'a [TracePoint], index: u32) -> Self {
        TraceStreamer {
            rep_uuid,
            t0_ms,
            points,
            next_index: index,
            only: Some(index),
        }
    }

    /// Number of BLE chunks a trace of `points` points will produce.
    pub fn chunk_count(points: usize) -> u32 {
        if points == 0 {
            return 0;
        }
        points.div_ceil(BLE_CHUNK_SAMPLES) as u32
    }

    /// Encode the next chunk into `out` as a `SyncMessage`, returning its
    /// length. `None` when the stream is finished (including when a
    /// single-chunk request points past the end of the trace).
    pub fn next_message(&mut self, out: &mut [u8]) -> Option<usize> {
        if self.only.is_some_and(|i| i != self.next_index) {
            return None;
        }
        let start = self.next_index as usize * BLE_CHUNK_SAMPLES;
        if start >= self.points.len() {
            return None;
        }
        let end = (start + BLE_CHUNK_SAMPLES).min(self.points.len());
        // Every chunk carries the stream's total, so the app knows which one is
        // the last and can stop waiting for more.
        let chunk = pb::chunk_message(
            self.rep_uuid,
            self.next_index,
            self.t0_ms,
            Self::chunk_count(self.points.len()),
            &self.points[start..end],
        );
        let n = pb::encode_sync(msg::SyncMessage_::Body::Trace(chunk), out).ok()?;
        self.next_index += 1;
        if self.only.is_some() {
            self.only = None;
            self.next_index = Self::chunk_count(self.points.len());
        }
        Some(n)
    }
}

/// Summaries to send when the app subscribes.
///
/// The device reads summaries from the log (one flash record at a time) and
/// feeds them in; `index` tracks how far the subscriber has been brought up
/// to date, so re-subscribing only sends what's new.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct SummaryCursor {
    /// Index of the next summary the app hasn't seen.
    next: usize,
}

impl SummaryCursor {
    pub const fn new() -> Self {
        SummaryCursor { next: 0 }
    }

    /// Index of the next summary to send.
    pub fn next(&self) -> usize {
        self.next
    }

    /// Advance past `count` summaries that were sent (or skipped).
    pub fn advance(&mut self, count: usize) {
        self.next += count;
    }

    /// Rewind if the log has fewer summaries than we've already sent, i.e.
    /// the ring dropped old reps since the subscriber last synced.
    pub fn clamp_to(&mut self, total: usize) {
        if self.next > total {
            self.next = total;
        }
    }

    /// How many summaries with index >= `next` remain in a log of `total`.
    pub fn remaining(&self, total: usize) -> usize {
        total.saturating_sub(self.next)
    }
}

/// Encode one summary as a `SyncMessage` for the summary characteristic.
pub fn encode_summary_message(summary: msg::RepSummary, out: &mut [u8]) -> Option<usize> {
    pb::encode_sync(msg::SyncMessage_::Body::Summary(summary), out).ok()
}

/// Encode a status message (rep counts etc.) for the summary characteristic.
pub fn encode_status_message(status: msg::SyncStatus, out: &mut [u8]) -> Option<usize> {
    pb::encode_sync(msg::SyncMessage_::Body::Status(status), out).ok()
}

/// Encode an IMU calibration status message for the summary characteristic.
pub fn encode_calibration_message(calibration: &Calibration, out: &mut [u8]) -> Option<usize> {
    pb::encode_sync(
        msg::SyncMessage_::Body::Calibration(pb::calibration_message(calibration)),
        out,
    )
    .ok()
}

/// Decode a `TraceRequest` written by the app, rejecting anything absurd.
pub fn decode_trace_request(bytes: &[u8]) -> Option<msg::TraceRequest> {
    let request = pb::decode_trace_request(bytes).ok()?;
    // A short uuid is not an identity.
    pb::uuid_from_field(&request.rep_uuid)?;
    if !request.all_chunks && request.chunk_index >= 64 {
        return None;
    }
    Some(request)
}

/// Largest encoded `SyncMessage` we may need to build: a full BLE chunk.
pub const SYNC_CHUNK_MAX: usize = 1 + BLE_CHUNK_SAMPLES * 64;

/// Compile-time sanity: the BLE chunk max is what the record buffer holds.
const _: () = assert!(SYNC_CHUNK_MAX <= crate::store::RECORD_BUF_BYTES);
const _: () = assert!(TRACE_CHUNK_SAMPLES >= BLE_CHUNK_SAMPLES);
