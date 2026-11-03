//! BLE sync: what the device says on the pipe — the envelope a summary goes in, the request it will serve, and the cursor tracking how much of the log the app has been sent.

use crate::pb::{self, msg};
use crate::uuid::RepUuid;

/// How many reps the device remembers the place of. One batch of summaries, which is the order the app asks for
/// their traces in, so the requests a batch causes are answered from the index rather than by reading the log.
pub const REP_INDEX_LEN: usize = 16;

/// Where one rep's records sit in the log: the record after its summary, and how many chunk records of each kind
/// follow it.
///
/// `append_rep` writes a rep's summary, its plotted chunks and then its raw chunks together under the log's lock, so
/// both runs are the records after the summary with none of anyone else's among them — and the raw run starts
/// exactly where the plotted one ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RepPlace {
    pub rep_uuid: RepUuid,
    pub first_chunk_id: u32,
    pub computed_chunks: u32,
    /// 0 when this rep has no raw trace.
    pub raw_chunks: u32,
}

impl RepPlace {
    /// Where the raw run starts, given that the plotted one came first.
    pub fn first_raw_chunk_id(&self) -> u32 {
        self.first_chunk_id + self.computed_chunks
    }
}

/// Where the reps the summary stream has handed out sit in the log.
///
/// A trace request names a rep by uuid, and the log can only answer that by reading records — every record, in the
/// worst case, which is seconds the device is not sampling. The summary collect has just read the summary and knows
/// its record id, so it records that here, and the trace read becomes a walk of that rep's own records.
///
/// A cache, never the record of what exists: a miss — a rep whose summary this connection never read, or one pushed
/// out of the ring of entries — falls back to the walk, which is why this can be small and forget things.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct RepIndex {
    entries: [Option<RepPlace>; REP_INDEX_LEN],
    next: usize,
}

impl RepIndex {
    pub const fn new() -> Self {
        RepIndex {
            entries: [None; REP_INDEX_LEN],
            next: 0,
        }
    }

    /// Record where a rep is, overwriting the oldest entry when full: the entry the app is least likely to ask for
    /// next is the one it was sent longest ago.
    pub fn record(
        &mut self,
        rep_uuid: RepUuid,
        first_chunk_id: u32,
        computed_chunks: u32,
        raw_chunks: u32,
    ) {
        self.entries[self.next] = Some(RepPlace {
            rep_uuid,
            first_chunk_id,
            computed_chunks,
            raw_chunks,
        });
        self.next = (self.next + 1) % REP_INDEX_LEN;
    }

    pub fn find(&self, rep_uuid: &RepUuid) -> Option<RepPlace> {
        self.entries
            .iter()
            .flatten()
            .find(|place| place.rep_uuid == *rep_uuid)
            .copied()
    }
}

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
///
/// Both kinds are served: the plotted trace from the chunks every rep has, and the raw one from the chunks a dev
/// session recorded. Which of them a rep has is its summary's business (`computed_chunks`, `raw_chunks`), not this
/// end's — a request for a trace a rep does not have reads back empty, and that is the answer either way.
pub fn trace_request(request: &msg::SyncRequest) -> Option<msg::TraceRequest> {
    let trace = match &request.body {
        Some(msg::SyncRequest_::Body::Trace(trace)) => trace.clone(),
        _ => return None,
    };
    // A short uuid is not an identity.
    pb::uuid_from_field(&trace.rep_uuid)?;
    if !trace.all_chunks && trace.chunk_index >= 64 {
        return None;
    }
    Some(trace)
}

/// Whether a `SyncRequest` is the dev switch, and which way it sets it.
pub fn dev_mode(request: &msg::SyncRequest) -> Option<bool> {
    match &request.body {
        Some(msg::SyncRequest_::Body::DevMode(dev)) => Some(dev.enabled),
        _ => None,
    }
}
