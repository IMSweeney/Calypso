//! BLE sync: the messages the device pushes to the app.
//!
//! Both characteristics are halves of one pipe ([`crate::pipe`]), so nothing here
//! is about *how* a message travels any more: a summary, a calibration status and
//! a trace chunk are each one transfer, and their sizes are the pipe's business.
//! What is left here is what the device says — the envelope it puts a summary or a
//! calibration status in, and the request it will serve — plus the cursor that
//! keeps track of how much of the log the app has been sent.
//!
//! Platform-independent, so the whole flow is host-tested (see `tests/sync.rs`)
//! without a radio.

use crate::calibration::Calibration;
use crate::pb::{self, msg};

/// Summaries to send when the app subscribes.
///
/// The device reads summaries from the log (one flash record at a time) and
/// feeds them in; `index` tracks how far the subscriber has been brought up to
/// date, so re-subscribing only sends what's new.
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

/// One summary as the `SyncMessage` the notify characteristic carries.
pub fn summary_message(summary: msg::RepSummary) -> msg::SyncMessage {
    msg::SyncMessage {
        body: Some(msg::SyncMessage_::Body::Summary(summary)),
    }
}

/// The IMU's calibration status as a `SyncMessage`.
pub fn calibration_message(calibration: &Calibration) -> msg::SyncMessage {
    msg::SyncMessage {
        body: Some(msg::SyncMessage_::Body::Calibration(
            pb::calibration_message(calibration),
        )),
    }
}

/// The trace request inside a `SyncRequest` the app wrote, rejecting anything
/// absurd.
///
/// The envelope is why the write characteristic carries one kind of message and can
/// carry more later — a request to clear the log, say — without a characteristic,
/// and a UUID, of its own. Taking the decoded message rather than its bytes is the
/// pipe's doing: [`crate::pipe`] reassembles the write into a `SyncRequest`, and
/// this decides whether the request inside it is one this device will serve.
pub fn trace_request(request: &msg::SyncRequest) -> Option<msg::TraceRequest> {
    let trace = match &request.body {
        Some(msg::SyncRequest_::Body::Trace(trace)) => trace.clone(),
        None => return None,
    };
    // A short uuid is not an identity.
    pb::uuid_from_field(&trace.rep_uuid)?;
    if !trace.all_chunks && trace.chunk_index >= 64 {
        return None;
    }
    Some(trace)
}
