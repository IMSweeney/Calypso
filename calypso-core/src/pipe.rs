//! The device's end of the sync pipe.
//!
//! `tools/gatt-pipe` is the framing and the two state machines; this is Calypso's
//! use of them, and it is what turns a write into a `SyncRequest` and a
//! `SyncMessage` into notifications. Neither characteristic carries a value any
//! more (see `proto/calypso_gatt.proto`): each carries chunks, so the two ends
//! agree on a frame rather than on a message fitting in one write.
//!
//! No embassy, no radio, and no clock of its own — the caller passes the time in —
//! which is what lets the whole of it be host-tested (`tests/pipe.rs`).
//!
//! Three rules of the pipe the caller has to live with, rather than this module's
//! judgement:
//!
//! - **One transfer at a time.** [`Session::send`] does nothing until the peer has
//!   acknowledged the transfer before it, so a caller with more than one message
//!   to send keeps a queue of its own and asks again later.
//! - **An acknowledgement arrives as a write on the other characteristic.** That
//!   is why [`Session::written`] is the only place the sender learns anything, and
//!   why the device's send path has to be pumped by the connection's event loop
//!   rather than driven in a loop of its own.
//! - **A stalled transfer is the caller's to notice.** [`Session::tick`] is what
//!   retries one and eventually gives up on it.
//!
//! The trace cursor lives here too, for two reasons: a trace is a *sequence* of
//! transfers, so it is exactly the state that must survive between event loop
//! iterations, and [`Session::trace_storage`] is where `store::read_trace` reads a
//! rep into, so the points and the cursor that walks them stay together.

use crate::gatt::pipe_sync;
use crate::pb::{self, msg, TRACE_CHUNK_SAMPLES};
use crate::rep::TracePoint;
use crate::uuid::RepUuid;
use gatt_pipe::{flags, Header, Received, Receiver, Send, Sender, MAX_ATTEMPTS};

/// Largest message one transfer may carry, both ways.
///
/// The buffer a message is reassembled in, so it is the schema's number
/// (`pipe_msg_max`, see `tool/gen_gatt.sh`) rather than this module's, and the
/// generated table asserts that `micropb`'s worst case for a `SyncMessage` fits it.
pub const MESSAGE_MAX: usize = pipe_sync::MESSAGE_MAX;

/// Bytes of payload one chunk carries, before its frame: the characteristic's
/// backing buffer, and therefore the most one write or notification holds.
pub const CHUNK_MAX: usize = pipe_sync::CHUNK_MAX;

/// The largest frame this end writes, which is [`CHUNK_MAX`]: the payload is
/// clamped so that the frame — header, length and CRC — fits the buffer.
pub const FRAME_MAX: usize = CHUNK_MAX;

/// No progress for this long (milliseconds) abandons a transfer.
pub const XFER_TIMEOUT_MS: u32 = pipe_sync::XFER_TIMEOUT_MS;

/// What a write from the app left behind.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Written {
    /// A whole message arrived: [`Session::message`] holds it.
    pub message: bool,
    /// The app is owed an acknowledgement, so the caller notifies
    /// [`Session::reply`] back without delay.
    pub reply: bool,
}

/// What a transfer that stopped making progress did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stalled {
    /// It is moving, or nothing is in flight.
    No,
    /// Sent again from the beginning: `attempt` of [`MAX_ATTEMPTS`].
    Retrying { attempt: u8 },
    /// Given up on, so the message is lost and the pipe is free for the next one.
    Abandoned { attempt: u8 },
}

/// What a message is, which is what the caller wants to log when one is lost, and
/// what decides whether a trace stream survives it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// A rep summary the app has not seen.
    Summary,
    /// The IMU's calibration state.
    Calibration,
    /// One chunk of a rep's trace.
    Trace,
}

impl Kind {
    /// This kind's name, for a log line.
    pub fn name(self) -> &'static str {
        match self {
            Kind::Summary => "summary",
            Kind::Calibration => "calibration",
            Kind::Trace => "trace chunk",
        }
    }
}

/// The device's end of the pipe.
pub struct Session<'a> {
    sender: Sender,
    receiver: Receiver,
    /// The message being sent: the codec writes it here, the sender cuts chunks
    /// out of it.
    outgoing: &'a mut [u8],
    outgoing_len: usize,
    /// What is being sent, for the log line when it is lost.
    kind: Kind,
    /// The calibration levels the app has been told on this connection, so a status
    /// that says the same thing is not sent again.
    calibration_sent: Option<u32>,
    /// The message being received.
    incoming: &'a mut [u8],
    incoming_len: usize,
    /// The frame the app is owed for the write that just arrived.
    reply: Option<Reply>,
    /// A trace being streamed back, and the points it comes from.
    points: &'a mut [TracePoint],
    points_len: usize,
    trace: Option<Trace>,
}

/// The acknowledgement owed to the app, and the next number it should expect.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Reply {
    Ack(u8),
    Nak(u8),
}

/// A trace being streamed: one `TraceChunk` per transfer, in chunk order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Trace {
    rep_uuid: RepUuid,
    t0_ms: u32,
    /// Chunks in the whole trace, which every chunk carries so the app knows when
    /// it has them all.
    total_chunks: u32,
    next: u32,
    /// Set when the app asked for one chunk by index rather than for the stream.
    only: Option<u32>,
}

impl Trace {
    /// The next chunk index to send, or `None` when the trace is done.
    fn take_next(&mut self) -> Option<u32> {
        if self.next >= self.total_chunks {
            return None;
        }
        let index = self.next;
        // A single-chunk request is satisfied by the one chunk, whatever else is
        // in the trace.
        self.next = if self.only.is_some() {
            self.total_chunks
        } else {
            self.next + 1
        };
        Some(index)
    }
}

impl<'a> Session<'a> {
    /// `outgoing` and `incoming` hold one message each ([`MESSAGE_MAX`] bytes
    /// apiece; the caller owns them, on the heap, because nothing that size belongs
    /// on a task stack) and `points` is where a trace being streamed is read into.
    pub fn new(
        outgoing: &'a mut [u8],
        incoming: &'a mut [u8],
        points: &'a mut [TracePoint],
    ) -> Self {
        Session {
            sender: Sender::new(pipe_sync::WINDOW),
            receiver: Receiver::new(pipe_sync::WINDOW),
            outgoing,
            outgoing_len: 0,
            kind: Kind::Summary,
            calibration_sent: None,
            incoming,
            incoming_len: 0,
            reply: None,
            points,
            points_len: 0,
            trace: None,
        }
    }

    /// Whether a message can be started: no transfer in flight, and none waiting
    /// for an acknowledgement.
    pub fn is_idle(&self) -> bool {
        self.sender.is_idle()
    }

    /// Whether a trace is being streamed (`next_trace_message` has more).
    pub fn streaming(&self) -> bool {
        self.trace.is_some()
    }

    /// What is in flight, for the log line when a transfer is retried or given up
    /// on. [`Kind::Summary`] when nothing is — the only caller that asks is writing
    /// about a transfer that exists.
    pub fn kind(&self) -> Kind {
        self.kind
    }

    /// Send a summary, a message built here rather than by the caller.
    ///
    /// The three `send_*` methods exist because a `SyncMessage` is 2 KB *by value*
    /// — one of its variants is a whole trace chunk — and a firmware task's future
    /// is the wrong place for that: a value built and encoded inside a plain method
    /// call is a stack frame that is gone before the notification is awaited. The
    /// general `send` below is for callers whose message is not one of Calypso's
    /// three, and for tests.
    pub fn send_summary(&mut self, summary: &msg::RepSummary) -> bool {
        self.send(
            Kind::Summary,
            &crate::sync::summary_message(summary.clone()),
        )
    }

    /// Send the IMU's calibration status, unless this connection has already sent a
    /// status that says the same thing.
    ///
    /// A status is a *state*, not an event: the app draws one hint from it, and the
    /// device's own 1 Hz poll is what changes it. An unchanged one is not merely
    /// idle traffic — the app acknowledges every message it receives, an
    /// acknowledgement is a write, and a write is where the send path is asked what
    /// to send next, so "status, acknowledgement, status, acknowledgement" is a loop
    /// the summaries and traces never get the pipe out of.
    ///
    /// `false` if it was not worth sending, or if a transfer is still in flight.
    pub fn send_calibration(&mut self, calibration: &crate::calibration::Calibration) -> bool {
        let levels = calibration.to_bits();
        if self.calibration_sent == Some(levels) {
            return false;
        }
        let sent = self.send(
            Kind::Calibration,
            &crate::sync::calibration_message(calibration),
        );
        if sent {
            self.calibration_sent = Some(levels);
        }
        sent
    }

    /// Send the next chunk of the trace being streamed, if there is one: the cursor
    /// moves here, so this is only called when the pipe is free.
    ///
    /// `false` when the trace is finished (or was never started), which is also when
    /// the stream is forgotten.
    pub fn send_next_trace_chunk(&mut self) -> bool {
        let Some(index) = self.take_trace_chunk() else {
            self.trace = None;
            return false;
        };
        let (rep_uuid, t0_ms, total_chunks) = match self.trace {
            Some(trace) => (trace.rep_uuid, trace.t0_ms, trace.total_chunks),
            None => return false,
        };
        let start = (index as usize) * TRACE_CHUNK_SAMPLES;
        let end = (start + TRACE_CHUNK_SAMPLES).min(self.points_len);
        let Some(points) = self.points.get(start..end) else {
            return false;
        };
        let chunk = pb::chunk_message(rep_uuid, index, t0_ms, total_chunks, points);
        self.send(
            Kind::Trace,
            &msg::SyncMessage {
                body: Some(msg::SyncMessage_::Body::Trace(chunk)),
            },
        )
    }

    /// The next chunk index of the trace being streamed, moving the cursor.
    fn take_trace_chunk(&mut self) -> Option<u32> {
        let mut trace = self.trace?;
        let index = trace.take_next()?;
        self.trace = Some(trace);
        Some(index)
    }

    /// Begin sending `message`, which is one transfer whatever its size.
    ///
    /// `false` if a transfer is still in flight (the caller tries again when the
    /// acknowledgement arrives) or if the message does not encode into the pipe's
    /// buffer, which the generated table's assertion is there to make impossible.
    pub fn send(&mut self, kind: Kind, message: &msg::SyncMessage) -> bool {
        if !self.sender.is_idle() {
            return false;
        }
        let Some(len) = pipe_sync::encode_response(message, self.outgoing) else {
            return false;
        };
        if self.sender.start(len, MESSAGE_MAX).is_err() {
            return false;
        }
        self.outgoing_len = len;
        self.kind = kind;
        true
    }

    /// The next chunk of the message in flight, written into `out` (which must
    /// hold [`FRAME_MAX`] bytes), or `None` when there is nothing to send — the
    /// whole message is out, or the window is full (it cannot be: the window is
    /// wider than any message this device sends).
    ///
    /// `mtu` is the negotiated ATT MTU, read from the connection: the chunk is
    /// clamped to it, so the notification fits the link as well as the buffer.
    pub fn next_chunk(&mut self, mtu: usize, out: &mut [u8]) -> Option<usize> {
        let capacity = gatt_pipe::capacity(mtu, CHUNK_MAX, self.outgoing_len as u32)?;
        match self
            .sender
            .next(&self.outgoing[..self.outgoing_len], capacity, out)
        {
            Ok(Send::Chunk(len)) => Some(len),
            _ => None,
        }
    }

    /// Take a write from the app: a chunk of a request, or the acknowledgement of
    /// a message this end sent.
    pub fn written(&mut self, bytes: &[u8], now_ms: u32) -> Written {
        self.incoming_len = 0;

        // A frame with an acknowledgement flag says where the app is in *our*
        // transfer, so it is the sender's business. The flag says which it is
        // without guessing, and it is unambiguous: an acknowledgement carries no
        // payload, so it is never a chunk in disguise.
        if let Ok((header, _)) = gatt_pipe::decode(bytes) {
            if header.is_reply() {
                if header.is_ack() {
                    self.sender.on_ack(header.seq);
                } else {
                    self.sender.on_nak();
                }
                // The peer is alive and talking, whatever it said.
                self.sender.note_progress(now_ms);
                return Written::default();
            }
        }

        match self.receiver.on_chunk(bytes, self.incoming, MESSAGE_MAX) {
            Received::Continue => Written::default(),
            Received::Ack { next_expected } => {
                self.reply = Some(Reply::Ack(next_expected));
                Written {
                    message: false,
                    reply: true,
                }
            }
            Received::Nak => {
                self.reply = Some(Reply::Nak(self.receiver.next_expected()));
                Written {
                    message: false,
                    reply: true,
                }
            }
            // The end of a message is also the place an acknowledgement is owed,
            // whatever the window's cadence says: the app's sender is waiting on it
            // to start anything else.
            Received::Delivered { len } => {
                self.incoming_len = len;
                self.reply = Some(Reply::Ack(self.receiver.next_expected()));
                Written {
                    message: true,
                    reply: true,
                }
            }
            // A frame that did not check out, dropped whole rather than
            // half-believed: the app's timeout is the recovery, because a
            // corrupted acknowledgement is no better than none.
            Received::Dropped(_) => Written::default(),
        }
    }

    /// The message a write assembled, valid until the next write.
    pub fn message(&self) -> &[u8] {
        &self.incoming[..self.incoming_len]
    }

    /// The request that message is, or `None` if the bytes are not one.
    ///
    /// Nothing here decides whether the device will *serve* it: that is
    /// [`crate::sync::trace_request`], which knows what the device can do. This
    /// only knows what the schema says a `SyncRequest` is.
    pub fn request(&self) -> Option<msg::SyncRequest> {
        pipe_sync::decode_request(self.message())
    }

    /// The frame the app is owed, written into `out`, or `None` if it is owed
    /// nothing.
    ///
    /// Notified on the *other* characteristic from the one the write arrived on:
    /// an acknowledgement travels beside the data it is about, which is what makes
    /// both directions usable and neither of them privileged.
    pub fn reply(&mut self, out: &mut [u8]) -> Option<usize> {
        let (seq, flags) = match self.reply.take()? {
            Reply::Ack(seq) => (seq, flags::ACK),
            Reply::Nak(seq) => (seq, flags::NAK),
        };
        gatt_pipe::encode(
            Header {
                seq,
                flags,
                total_len: None,
            },
            &[],
            out,
        )
        .ok()
    }

    /// Notice a transfer that has stopped moving: send it again, or give up.
    ///
    /// The three attempts are [`MAX_ATTEMPTS`], and they are counted per message
    /// rather than per chunk: the pipe has no selective retransmit, so a retry is
    /// the whole transfer.
    pub fn tick(&mut self, now_ms: u32) -> Stalled {
        if !self.sender.stalled(now_ms, XFER_TIMEOUT_MS) {
            return Stalled::No;
        }
        let attempt = self.sender.attempts();
        // Everything that was in flight is abandoned either way: the peer has
        // said nothing, so there is no telling how much of it arrived.
        self.sender.abort();
        if attempt >= MAX_ATTEMPTS {
            // A trace whose chunk never arrived is a trace the app cannot
            // assemble, so the rest of it is not worth sending.
            if self.kind == Kind::Trace {
                self.trace = None;
            }
            // A status the app never acknowledged is one it has not been told, so
            // the next request for it goes out rather than being remembered as sent.
            if self.kind == Kind::Calibration {
                self.calibration_sent = None;
            }
            return Stalled::Abandoned { attempt };
        }
        // A new attempt, which is what the limit above counts: `start` is where
        // the attempt number comes from, so a stall that only rewound the offset
        // would retry for ever (see the reference driver in `tools/gatt-pipe`).
        if self.sender.start(self.outgoing_len, MESSAGE_MAX).is_err() {
            return Stalled::Abandoned { attempt };
        }
        self.sender.note_progress(now_ms);
        Stalled::Retrying {
            attempt: self.sender.attempts(),
        }
    }

    /// Forget everything: a new connection, where whatever was in flight is gone.
    /// Half a message is not a message, and the app's ends are reset with ours.
    pub fn reset(&mut self) {
        self.sender.reset();
        self.receiver.reset();
        self.reply = None;
        self.incoming_len = 0;
        self.outgoing_len = 0;
        self.calibration_sent = None;
        self.trace = None;
        self.points_len = 0;
    }

    /// The storage a trace being streamed is read into, for `store::read_trace`.
    ///
    /// Handed out mutably rather than kept as a `TraceBuf`: the read fills it, and
    /// the cursor that walks it afterwards is this end's.
    pub fn trace_storage(&mut self) -> &mut [TracePoint] {
        self.points_len = 0;
        self.trace = None;
        &mut self.points[..]
    }

    /// Start streaming the trace in [`Session::trace_storage`], which is now
    /// `points` long. `only` is the one chunk index to send, for a request that
    /// asked for a single chunk rather than for the trace.
    ///
    /// A trace already streaming is replaced: the app asks for one at a time, and
    /// the newer request is the one that will be waited for.
    pub fn start_trace(&mut self, rep_uuid: RepUuid, t0_ms: u32, points: usize, only: Option<u32>) {
        self.points_len = points.min(self.points.len());
        self.trace = Some(Trace {
            rep_uuid,
            t0_ms,
            total_chunks: pb::chunk_count_for(self.points_len),
            next: only.unwrap_or(0),
            only,
        });
    }

    /// Stop streaming a trace, whatever is left of it.
    pub fn stop_trace(&mut self) {
        self.trace = None;
    }
}
