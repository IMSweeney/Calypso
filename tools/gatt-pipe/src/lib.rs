//! The chunked pipe: the framing both ends of a BLE link share, and the two state
//! machines that use it.
//!
//! What a BLE characteristic cannot carry in one value travels as a transfer: a
//! message cut into chunks, each with a header and a CRC, reassembled at the
//! other end. `SPEC.md` in `tools/gatt-gen` describes the protocol and why it
//! acknowledges itself.
//!
//! The generator writes the parts only the schema knows — which characteristic
//! carries which direction, how big a chunk and a message may be, how to turn
//! bytes into a message — and this crate is the rest: no buffers of its own, no
//! clock, no BLE, no embassy. Bytes in, decisions out, which is what makes the
//! failure modes that matter (a chunk lost, duplicated, corrupted, an
//! acknowledgement that never comes) testable on a host.
//!
//! # Frame
//!
//! ```text
//! | seq u8 | flags u8 | [total_len varint, START only] | payload | crc16 LE |
//! ```
//!
//! `flags`: `START=0x01`, `END=0x02`, `ACK=0x04`, `NAK=0x08`. A chunk may be START
//! and END at once, which is how a message that fits in one chunk travels —
//! including an *empty* message (`total_len` 0 and no payload: a protobuf `Empty`
//! encodes to nothing, and that is a message, not an aborted transfer).
//!
//! `crc16` is CRC-16/CCITT-FALSE over every preceding byte of the chunk. A chunk
//! that does not check out is dropped rather than interpreted.
//!
//! # What the spec left open, and what this decides
//!
//! - **`seq` has two roles, chosen by the flag.** On a data chunk it is that
//!   chunk's number, incremented per chunk and wrapping at 256. On an ACK or a NAK
//!   it is *cumulative*: the next in-order number its sender expects. One field
//!   rather than two because the role is unambiguous — an acknowledgement carries
//!   no payload — and it keeps two bytes off every chunk.
//! - **The window is capped at [`MAX_WINDOW`].** A cumulative acknowledgement
//!   against a wrapping `u8` is only unambiguous while the unacknowledged window is
//!   under half the sequence space.
//! - **A START during a transfer restarts it** rather than NAKing for ever: that is
//!   what a peer retrying after its own timeout looks like.
//! - **A disconnect resets both ends** ([`Sender::reset`], [`Receiver::reset`]).
//!   Whatever was in flight is gone, and half a message is not a message.
//! - **Retrying resends the whole transfer.** There is no selective retransmit
//!   here; the caller decides whether to retry at all, and this counts the tries.
//!
//! # Flow control
//!
//! Notifications are not acknowledged and the plugins serialise writes, so the pipe
//! acknowledges itself: the sender keeps at most `window` chunks in flight, and the
//! receiver acknowledges at least every `window / 2` chunks and always on END. Both
//! directions use the same frame and the same rules, and an acknowledgement travels
//! on the opposite characteristic from the data it is about.

use core::fmt;

/// Frame flags.
pub mod flags {
    pub const START: u8 = 0x01;
    pub const END: u8 = 0x02;
    pub const ACK: u8 = 0x04;
    pub const NAK: u8 = 0x08;

    /// The bits that mean something. Anything else is a frame this protocol does
    /// not know, and is refused rather than guessed at.
    pub const KNOWN: u8 = START | END | ACK | NAK;
}

/// `seq` and `flags`.
const HEADER: usize = 2;
/// CRC-16, little endian.
const CRC: usize = 2;
/// The ATT header a notification or a write costs: opcode plus a 2-byte handle.
const ATT_HEADER: usize = 3;

/// The largest window a wrapping `u8` sequence number can acknowledge
/// unambiguously.
pub const MAX_WINDOW: u8 = 127;

/// The largest chunk any link is asked to carry: a pipe characteristic's backing
/// buffer is a compile-time constant on the device, so the frame has a ceiling of
/// its own whatever the MTU turns out to be.
pub const DEFAULT_CHUNK_MAX: usize = 244;

/// How many transfers this side will send before giving up on one.
pub const MAX_ATTEMPTS: u8 = 3;

/// CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no reflection.
pub fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for byte in bytes {
        crc ^= (*byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Bytes a varint needs for `value`.
pub fn varint_len(value: u32) -> usize {
    match value {
        0..=0x7F => 1,
        0x80..=0x3FFF => 2,
        0x4000..=0x1FFFFF => 3,
        0x200000..=0x0FFF_FFFF => 4,
        _ => 5,
    }
}

/// Bytes of a chunk that are not payload.
pub fn overhead(total_len: Option<u32>) -> usize {
    HEADER + CRC + total_len.map_or(0, varint_len)
}

/// How much payload fits in one chunk of a link whose ATT MTU is `mtu`, given
/// `chunk_max`.
///
/// The MTU is the negotiated one, read from the connection: assuming 247 is how a
/// notification becomes a silent no-op on a link that negotiated less.
pub fn capacity(mtu: usize, chunk_max: usize, total_len: u32) -> Option<usize> {
    let usable = mtu.checked_sub(ATT_HEADER + overhead(Some(total_len)))?;
    let limit = if chunk_max < usable {
        chunk_max
    } else {
        usable
    };
    if limit == 0 {
        None
    } else {
        Some(limit)
    }
}

/// A decoded frame header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    pub seq: u8,
    pub flags: u8,
    /// The message's encoded length, on a START.
    pub total_len: Option<u32>,
}

impl Header {
    pub fn is_start(&self) -> bool {
        self.flags & flags::START != 0
    }

    pub fn is_end(&self) -> bool {
        self.flags & flags::END != 0
    }

    pub fn is_ack(&self) -> bool {
        self.flags & flags::ACK != 0
    }

    pub fn is_nak(&self) -> bool {
        self.flags & flags::NAK != 0
    }

    /// Whether this is an acknowledgement of some kind rather than data.
    pub fn is_reply(&self) -> bool {
        self.flags & (flags::ACK | flags::NAK) != 0
    }
}

/// Why a chunk could not be read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Fewer bytes than a frame needs.
    Short,
    /// The CRC does not match: these bytes are not this frame, or not any frame.
    Crc,
    /// A flag bit that means nothing, or a length where one cannot be.
    BadFrame,
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DecodeError::Short => write!(f, "chunk is too short to be a frame"),
            DecodeError::Crc => write!(f, "chunk failed its crc"),
            DecodeError::BadFrame => write!(f, "chunk is not a frame this protocol knows"),
        }
    }
}

/// Read a frame, or say why not.
pub fn decode(chunk: &[u8]) -> Result<(Header, &[u8]), DecodeError> {
    if chunk.len() < HEADER + CRC {
        return Err(DecodeError::Short);
    }
    let (body, crc_bytes) = chunk.split_at(chunk.len() - CRC);
    let expected = u16::from_le_bytes([crc_bytes[0], crc_bytes[1]]);
    if crc16(body) != expected {
        return Err(DecodeError::Crc);
    }

    let seq = body[0];
    // A chunk in the middle of a transfer carries no flags at all, so zero is a
    // value rather than an error: what is refused is a bit nobody defined.
    let flags = body[1];
    if flags & !flags::KNOWN != 0 {
        return Err(DecodeError::BadFrame);
    }

    let mut rest = &body[HEADER..];
    let total_len = if flags & flags::START != 0 {
        let (value, used) = read_varint(rest).ok_or(DecodeError::BadFrame)?;
        rest = &rest[used..];
        Some(value)
    } else {
        None
    };

    // An acknowledgement says where the receiver is, not what it received.
    if flags & (flags::ACK | flags::NAK) != 0 && !rest.is_empty() {
        return Err(DecodeError::BadFrame);
    }

    Ok((
        Header {
            seq,
            flags,
            total_len,
        },
        rest,
    ))
}

/// Write a frame into `out`, returning its length.
pub fn encode(header: Header, payload: &[u8], out: &mut [u8]) -> Result<usize, EncodeError> {
    let length = header.total_len.map_or(0, varint_len);
    let total = HEADER + length + payload.len() + CRC;
    if out.len() < total {
        return Err(EncodeError::BufferTooSmall);
    }

    out[0] = header.seq;
    out[1] = header.flags;
    let mut at = HEADER;
    if let Some(value) = header.total_len {
        at += write_varint(value, &mut out[at..]);
    }
    out[at..at + payload.len()].copy_from_slice(payload);
    let end = at + payload.len();
    let crc = crc16(&out[..end]).to_le_bytes();
    out[end..end + CRC].copy_from_slice(&crc);
    Ok(total)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    BufferTooSmall,
}

fn read_varint(bytes: &[u8]) -> Option<(u32, usize)> {
    let mut value = 0u32;
    let mut shift = 0;
    for (index, byte) in bytes.iter().enumerate() {
        if shift > 28 {
            return None;
        }
        value |= ((byte & 0x7F) as u32) << shift;
        if byte & 0x80 == 0 {
            return Some((value, index + 1));
        }
        shift += 7;
    }
    None
}

fn write_varint(mut value: u32, out: &mut [u8]) -> usize {
    let mut at = 0;
    loop {
        let byte = (value & 0x7F) as u8;
        value >>= 7;
        if value == 0 {
            out[at] = byte;
            return at + 1;
        }
        out[at] = byte | 0x80;
        at += 1;
    }
}

/// What a sender wants to do next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Send {
    /// Send this many bytes from the caller's buffer, then call again.
    Chunk(usize),
    /// The window is full: nothing goes out until an acknowledgement arrives.
    WindowFull,
    /// The whole message is on the wire (the peer may still not have it).
    Done,
}

/// What an acknowledgement did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Acked {
    /// Nothing new: an acknowledgement of chunks already counted, or of a
    /// transfer this end is not in.
    Ignored,
    /// Chunks were acknowledged; this many remain in flight.
    Advanced { in_flight: u8 },
    /// Every chunk was acknowledged: the peer has the message.
    Complete,
}

/// The sending half: one message at a time, at most `window` chunks in flight.
#[derive(Debug)]
pub struct Sender {
    window: u8,
    /// The next sequence number to use.
    seq: u8,
    /// How far into the message the next chunk starts.
    offset: usize,
    /// The message being sent.
    message_len: usize,
    /// Chunks sent and not yet acknowledged.
    in_flight: u8,
    /// Whether a transfer is in progress. Tracked separately from `message_len`
    /// because an empty message is a message.
    active: bool,
    /// Set once the END chunk has gone out.
    end_sent: bool,
    /// Transfers started, for the caller's retry policy.
    attempts: u8,
    /// When something last happened, for [`Sender::stalled`].
    progress_ms: u32,
}

impl Sender {
    /// `window` chunks in flight at most; see [`MAX_WINDOW`].
    pub fn new(window: u8) -> Self {
        assert!(
            (1..=MAX_WINDOW).contains(&window),
            "a window of {window} cannot be acknowledged by a u8 sequence number"
        );
        Sender {
            window,
            seq: 0,
            offset: 0,
            message_len: 0,
            in_flight: 0,
            active: false,
            end_sent: false,
            attempts: 0,
            progress_ms: 0,
        }
    }

    pub fn window(&self) -> u8 {
        self.window
    }

    pub fn attempts(&self) -> u8 {
        self.attempts
    }

    /// Whether no transfer is in progress and nothing is outstanding.
    pub fn is_idle(&self) -> bool {
        !self.active && self.in_flight == 0
    }

    /// Begin a transfer of `message_len` bytes, giving up if the peer's
    /// reassembly buffer could not hold it.
    pub fn start(&mut self, message_len: usize, max_message: usize) -> Result<(), SendError> {
        if !self.is_idle() {
            return Err(SendError::Busy);
        }
        if message_len > max_message {
            return Err(SendError::TooLarge {
                len: message_len,
                max: max_message,
            });
        }
        self.message_len = message_len;
        self.offset = 0;
        self.in_flight = 0;
        self.active = true;
        self.end_sent = false;
        self.attempts = self.attempts.saturating_add(1);
        Ok(())
    }

    /// The next chunk to send, written into `out`.
    ///
    /// `capacity` is what the link carries ([`capacity`], from the negotiated
    /// MTU), and the flags say whether the chunk starts, ends or *is* the message.
    pub fn next(
        &mut self,
        message: &[u8],
        capacity: usize,
        out: &mut [u8],
    ) -> Result<Send, SendError> {
        if !self.active {
            return Ok(Send::Done);
        }
        if self.in_flight >= self.window {
            return Ok(Send::WindowFull);
        }

        let remaining = self.message_len - self.offset;
        let take = if capacity < remaining {
            capacity
        } else {
            remaining
        };
        let start = self.offset == 0;
        let end = self.offset + take == self.message_len;

        let mut bits = 0u8;
        if start {
            bits |= flags::START;
        }
        if end {
            bits |= flags::END;
        }

        let written = encode(
            Header {
                seq: self.seq,
                flags: bits,
                total_len: if start {
                    Some(self.message_len as u32)
                } else {
                    None
                },
            },
            &message[self.offset..self.offset + take],
            out,
        )
        .map_err(|_| SendError::BufferTooSmall)?;

        self.offset += take;
        self.seq = self.seq.wrapping_add(1);
        self.in_flight += 1;
        if end {
            self.end_sent = true;
        }
        Ok(Send::Chunk(written))
    }

    /// An acknowledgement naming the next in-order sequence the peer expects.
    pub fn on_ack(&mut self, next_expected: u8) -> Acked {
        if self.in_flight == 0 {
            return Acked::Ignored;
        }
        // How many of our chunks the peer has taken: it names the next one it
        // wants, and the next one we would send is `seq`.
        let unacked = self.seq.wrapping_sub(next_expected);
        if unacked >= self.in_flight {
            // It is telling us something we already knew, or about a transfer this
            // end has left.
            return Acked::Ignored;
        }
        self.in_flight = unacked;
        if self.in_flight == 0 && self.end_sent {
            self.message_len = 0;
            self.offset = 0;
            self.end_sent = false;
            self.active = false;
            return Acked::Complete;
        }
        Acked::Advanced {
            in_flight: self.in_flight,
        }
    }

    /// The peer refused the transfer: send it again from the beginning.
    pub fn on_nak(&mut self) {
        self.offset = 0;
        self.in_flight = 0;
        self.end_sent = false;
    }

    /// Note that something happened, so the no-progress timeout restarts.
    pub fn note_progress(&mut self, now_ms: u32) {
        self.progress_ms = now_ms;
    }

    /// Whether the transfer has made no progress for `timeout_ms`.
    pub fn stalled(&self, now_ms: u32, timeout_ms: u32) -> bool {
        !self.is_idle() && now_ms.wrapping_sub(self.progress_ms) >= timeout_ms
    }

    /// Give up on the transfer, keeping the count of attempts it took.
    pub fn abort(&mut self) {
        self.message_len = 0;
        self.offset = 0;
        self.in_flight = 0;
        self.end_sent = false;
        self.active = false;
    }

    /// Forget everything, including the attempt count: a new connection.
    pub fn reset(&mut self) {
        self.abort();
        self.attempts = 0;
        self.seq = 0;
        self.progress_ms = 0;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SendError {
    /// A transfer is already in flight.
    Busy,
    /// The message is larger than the peer's reassembly buffer.
    TooLarge { len: usize, max: usize },
    /// The caller's buffer cannot hold a chunk.
    BufferTooSmall,
}

impl fmt::Display for SendError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SendError::Busy => write!(f, "a transfer is already in flight"),
            SendError::TooLarge { len, max } => {
                write!(f, "a {len}-byte message does not fit a {max}-byte pipe")
            }
            SendError::BufferTooSmall => write!(f, "the chunk buffer is too small"),
        }
    }
}

/// What a receiver made of a chunk.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Received {
    /// In order, and no acknowledgement is due yet.
    Continue,
    /// Answer with an ACK naming the sequence this end expects next.
    Ack { next_expected: u8 },
    /// Answer with a NAK: the chunk was not the one expected, or promised more
    /// than this end can hold.
    Nak,
    /// A whole message: `len` bytes are in the caller's buffer.
    Delivered { len: usize },
    /// A frame that could not be read, dropped. The sender's timeout is the
    /// recovery, because a corrupted acknowledgement is no better than none.
    Dropped(DecodeError),
}

/// The receiving half: reassembles one message at a time into a caller's buffer.
#[derive(Debug)]
pub struct Receiver {
    window: u8,
    /// The next sequence number expected.
    seq: u8,
    /// The length the START declared, while a transfer is in progress.
    expected: Option<usize>,
    used: usize,
    /// Chunks taken since the last acknowledgement.
    since_ack: u8,
    /// Whether a repeat arrived, which is worth answering.
    repeated: bool,
}

impl Receiver {
    /// `window` is the number the peer's sender was built with: it decides how
    /// often an acknowledgement has to go out.
    pub fn new(window: u8) -> Self {
        assert!(
            (1..=MAX_WINDOW).contains(&window),
            "a window of {window} cannot be acknowledged by a u8 sequence number"
        );
        Receiver {
            window,
            seq: 0,
            expected: None,
            used: 0,
            since_ack: 0,
            repeated: false,
        }
    }

    pub fn in_transfer(&self) -> bool {
        self.expected.is_some()
    }

    /// Take one chunk. `buffer` is the reassembly buffer and `max_message` its
    /// size: a START that promises more is refused outright rather than filling
    /// the buffer and failing at the end.
    pub fn on_chunk(&mut self, chunk: &[u8], buffer: &mut [u8], max_message: usize) -> Received {
        let (header, payload) = match decode(chunk) {
            Ok(frame) => frame,
            Err(error) => {
                self.discard();
                return Received::Dropped(error);
            }
        };

        if header.is_reply() {
            // The peer is telling us where it is. That is the sender's business.
            return Received::Continue;
        }

        if header.is_start() {
            let expected = header.total_len.unwrap_or(0) as usize;
            if expected > max_message || expected > buffer.len() {
                self.discard();
                return Received::Nak;
            }
            self.expected = Some(expected);
            self.used = 0;
            self.seq = header.seq;
            self.since_ack = 0;
            self.repeated = false;
        } else if self.expected.is_none() {
            // The middle of a transfer this end never saw start: the peer is
            // ahead of us, or the START is the chunk that was lost.
            self.repeated = true;
            return Received::Nak;
        }

        if header.seq != self.seq {
            // A repeat of the chunk before the one expected is acknowledged again
            // — our ACK may be the thing that was lost — but a *gap* is not:
            // nothing here can fill one.
            if header.seq.wrapping_add(1) == self.seq {
                self.repeated = true;
                return self.acknowledge();
            }
            self.discard();
            return Received::Nak;
        }

        let expected = self.expected.unwrap_or(0);
        if self.used + payload.len() > expected {
            self.discard();
            return Received::Nak;
        }
        buffer[self.used..self.used + payload.len()].copy_from_slice(payload);
        self.used += payload.len();
        self.seq = self.seq.wrapping_add(1);
        self.since_ack += 1;

        if header.is_end() {
            if self.used != expected {
                self.discard();
                return Received::Nak;
            }
            let len = self.used;
            self.discard();
            return Received::Delivered { len };
        }

        self.acknowledge()
    }

    /// Give up on the transfer in progress. The sequence number stays: the next
    /// START says where the peer is.
    fn discard(&mut self) {
        self.expected = None;
        self.used = 0;
        self.since_ack = 0;
        self.repeated = false;
    }

    /// Acknowledge when the window says to, or when the peer has asked twice.
    fn acknowledge(&mut self) -> Received {
        let cadence = (self.window / 2).max(1);
        if self.repeated || self.since_ack >= cadence {
            self.since_ack = 0;
            self.repeated = false;
            return Received::Ack {
                next_expected: self.seq,
            };
        }
        Received::Continue
    }

    /// Forget everything: a new connection, or a transfer this end is abandoning.
    pub fn reset(&mut self) {
        self.discard();
        self.seq = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(seq: u8, bits: u8, total_len: Option<u32>, payload: &[u8], out: &mut [u8]) -> usize {
        encode(
            Header {
                seq,
                flags: bits,
                total_len,
            },
            payload,
            out,
        )
        .unwrap()
    }

    #[test]
    fn a_frame_round_trips() {
        let mut out = [0u8; 64];
        let header = Header {
            seq: 3,
            flags: flags::START | flags::END,
            total_len: Some(5),
        };
        let len = encode(header, b"hello", &mut out).unwrap();

        let (read, payload) = decode(&out[..len]).unwrap();
        assert_eq!(read, header);
        assert_eq!(payload, b"hello");
    }

    #[test]
    fn a_corrupted_frame_is_refused() {
        let mut out = [0u8; 64];
        let len = frame(1, flags::START, Some(2), b"ab", &mut out);
        out[3] ^= 0x40;
        assert_eq!(decode(&out[..len]), Err(DecodeError::Crc));
    }

    #[test]
    fn a_flag_that_means_nothing_is_refused() {
        let mut out = [0u8; 64];
        let len = frame(1, flags::START, Some(0), &[], &mut out);
        out[1] = 0x80;
        let crc = crc16(&out[..len - CRC]).to_le_bytes();
        out[len - CRC..len].copy_from_slice(&crc);
        assert_eq!(decode(&out[..len]), Err(DecodeError::BadFrame));
    }

    #[test]
    fn an_acknowledgement_carrying_payload_is_refused() {
        let mut out = [0u8; 64];
        let len = frame(4, flags::ACK, None, b"x", &mut out);
        assert_eq!(decode(&out[..len]), Err(DecodeError::BadFrame));
    }

    #[test]
    fn capacity_leaves_room_for_the_frame_and_the_att_header() {
        // A 23-byte MTU is 20 bytes of ATT payload, and a START spends five of
        // them on the frame, so 15 bytes of message travel first.
        assert_eq!(capacity(23, DEFAULT_CHUNK_MAX, 100), Some(15));
        // A 247-byte MTU leaves 244, and the chunk ceiling is 244 whatever the
        // MTU: the rest is the frame.
        assert_eq!(
            capacity(247, DEFAULT_CHUNK_MAX, 100),
            Some(244 - overhead(Some(100)))
        );
        // A link that cannot carry a frame is not a link.
        assert_eq!(capacity(6, DEFAULT_CHUNK_MAX, 0), None);
    }
}
