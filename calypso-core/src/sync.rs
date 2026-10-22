//! BLE sync: what the device says on the pipe — the envelope a summary goes in, the request it will serve, and the cursor tracking how much of the log the app has been sent.
//!
//! Calibration is not here: the app reads it off its own characteristic (`proto/calypso_gatt.proto`).

use crate::pb::{self, msg};

/// How far the subscriber has been brought up to date, so re-subscribing only sends what is new.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct SummaryCursor {
    next: usize,
}

impl SummaryCursor {
    pub const fn new() -> Self {
        SummaryCursor { next: 0 }
    }

    pub fn next(&self) -> usize {
        self.next
    }

    /// Advance past `count` summaries that were sent (or skipped).
    pub fn advance(&mut self, count: usize) {
        self.next += count;
    }

    /// Rewind if the ring dropped old reps since the subscriber last synced.
    pub fn clamp_to(&mut self, total: usize) {
        if self.next > total {
            self.next = total;
        }
    }

    pub fn remaining(&self, total: usize) -> usize {
        total.saturating_sub(self.next)
    }
}

pub fn summary_message(summary: msg::RepSummary) -> msg::SyncMessage {
    msg::SyncMessage {
        body: Some(msg::SyncMessage_::Body::Summary(summary)),
    }
}

/// The trace request inside a `SyncRequest` the app wrote, or `None` if it is not one this device will serve.
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
