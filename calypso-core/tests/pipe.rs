//! The device's end of the pipe, against the app's.
//!
//! `gatt_pipe`'s own tests drive two bare state machines over a hostile link;
//! these drive [`Session`], which is what the firmware actually runs — the
//! encoding, the trace cursor, the reply frames and the retry policy — against the
//! app's half (`Receiver` here, `calypso-app/lib/gatt/pipe.dart` there), over a
//! link that is a `Vec` rather than a radio.
//!
//! No radio and no embassy, so every case a bench session would cost to find is
//! reproducible here: a chunk lost, an acknowledgement swallowed, a transfer that
//! stalls until it is given up on.

use calypso_core::gatt::pipe_sync;
use calypso_core::pb::{self, msg, TRACE_CHUNK_SAMPLES};
use calypso_core::pipe::{
    Kind, Session, Stalled, ATT_MTU_DEFAULT, FRAME_MAX, MESSAGE_MAX, XFER_TIMEOUT_MS,
};
use calypso_core::rep::TracePoint;
use calypso_core::sync::{self, SummaryCursor};
use calypso_core::uuid::{fixture_uuid, RepUuid};
use gatt_pipe::{flags, Acked, Header, Received, Receiver, Send, Sender, MAX_ATTEMPTS};
use micropb::MessageEncode;

/// The MTU a phone negotiates when it asks for the most it can carry, which is
/// where the pipe's own buffer is the narrower of the two ceilings.
const MTU: usize = 247;

/// A small one, negotiated: 41 bytes of ATT payload after the ATT header, five or six
/// of them the frame. Not [`ATT_MTU_DEFAULT`], which is a link that never said.
const SMALL_MTU: usize = 65;

/// The heap the session's buffers sit in, as the firmware leaks them.
struct Buffers {
    outgoing: Vec<u8>,
    incoming: Vec<u8>,
    points: Vec<TracePoint>,
}

impl Buffers {
    fn new(points: usize) -> Self {
        Buffers {
            outgoing: vec![0u8; MESSAGE_MAX],
            incoming: vec![0u8; MESSAGE_MAX],
            points: vec![TracePoint::ZERO; points],
        }
    }

    fn session(&mut self) -> Session<'_> {
        Session::new(&mut self.outgoing, &mut self.incoming, &mut self.points)
    }
}

fn point(i: usize) -> TracePoint {
    TracePoint {
        t_ms: 1000 + (i as u32) * 10,
        pos: [0.01 * i as f32, 0.2, -0.3],
        accel: [0, -981, 12],
        quat: [16384, 0, 1024, -2048],
    }
}

fn rep_summary(index: u32) -> msg::RepSummary {
    msg::RepSummary {
        rep_uuid: pb::uuid_to_field(&fixture_uuid(index)),
        trace_len: 96,
        chunk_count: 2,
        ..Default::default()
    }
}

fn summary(index: u32) -> msg::SyncMessage {
    sync::summary_message(rep_summary(index))
}

/// A whole trace chunk: the largest message the device sends, and the one whose frame
/// count says what a link's MTU is worth.
fn chunk() -> msg::SyncMessage {
    let points: Vec<TracePoint> = (0..TRACE_CHUNK_SAMPLES).map(point).collect();
    msg::SyncMessage {
        body: Some(msg::SyncMessage_::Body::Trace(pb::chunk_message(
            fixture_uuid(1),
            0,
            0,
            TRACE_CHUNK_SAMPLES as u32,
            &points,
        ))),
    }
}

/// A chunk whose frames do not fit the window in one go — the shape that stalls a send
/// path which only flushes a message when it *starts* one. Every field is as wide as
/// the schema allows, so the message is ~3 KB and its frames are more than `window`.
fn wide_chunk() -> msg::SyncMessage {
    let points: Vec<TracePoint> = (0..TRACE_CHUNK_SAMPLES)
        .map(|i| TracePoint {
            t_ms: u32::MAX - i as u32,
            pos: [12345.678, -98765.43, 3.4028235e38],
            accel: [-30000, 27000, -12345],
            quat: [i16::MAX, i16::MIN, 12345, -12345],
        })
        .collect();
    msg::SyncMessage {
        body: Some(msg::SyncMessage_::Body::Trace(pb::chunk_message(
            fixture_uuid(2),
            0,
            0,
            TRACE_CHUNK_SAMPLES as u32,
            &points,
        ))),
    }
}

/// Everything waiting on the wire, taken to the app. Returns the reps whose
/// summaries arrived, which is what a caller advancing a cursor would count.
fn receive(session: &mut Session<'_>, phone: &mut Phone, now: &mut u32) -> Vec<RepUuid> {
    let mut arrived = Vec::new();
    for frame in drain(session, MTU) {
        *now += 10;
        if let Some(bytes) = phone.take(session, &frame, *now) {
            let message = pb::decode_sync(&bytes).expect("what arrived is a message");
            if let Some(msg::SyncMessage_::Body::Summary(summary)) = message.body {
                arrived.push(pb::summary_uuid(&summary).expect("a summary has an identity"));
            }
        }
    }
    arrived
}

/// Everything the device is ready to notify, as the frames an app would receive.
fn drain(session: &mut Session<'_>, mtu: usize) -> Vec<Vec<u8>> {
    let mut frames = Vec::new();
    let mut frame = [0u8; FRAME_MAX];
    while let Some(len) = session.next_chunk(mtu, &mut frame) {
        assert!(len <= FRAME_MAX, "a frame bigger than the buffer");
        frames.push(frame[..len].to_vec());
    }
    frames
}

fn reply_frame(seq: u8, flag: u8) -> Vec<u8> {
    let mut out = [0u8; 8];
    let len = gatt_pipe::encode(
        Header {
            seq,
            flags: flag,
            total_len: None,
        },
        &[],
        &mut out,
    )
    .expect("a reply fits");
    out[..len].to_vec()
}

/// The app's half of one direction: it reassembles what the device notifies, and
/// hands back the acknowledgements the device's sender waits for.
struct Phone {
    receiver: Receiver,
    buffer: Vec<u8>,
}

impl Phone {
    fn new() -> Self {
        Phone {
            // The app's receiver is generated with the schema's window, so its
            // acknowledgement cadence is `window / 2` — the same number the device sends by.
            receiver: Receiver::new(pipe_sync::WINDOW),
            buffer: vec![0u8; MESSAGE_MAX],
        }
    }

    /// Take one frame, feeding any acknowledgement straight back to the device.
    /// Returns the message if that frame completed one.
    fn take(&mut self, session: &mut Session<'_>, frame: &[u8], now_ms: u32) -> Option<Vec<u8>> {
        match self.receiver.on_chunk(frame, &mut self.buffer, MESSAGE_MAX) {
            Received::Delivered { len } => {
                // The end of a message is acknowledged too, and the device's
                // sender is waiting on exactly that to start anything else.
                session.written(
                    &reply_frame(self.receiver.next_expected(), flags::ACK),
                    now_ms,
                );
                Some(self.buffer[..len].to_vec())
            }
            Received::Ack { next_expected } => {
                session.written(&reply_frame(next_expected, flags::ACK), now_ms);
                None
            }
            Received::Nak => {
                session.written(
                    &reply_frame(self.receiver.next_expected(), flags::NAK),
                    now_ms,
                );
                None
            }
            Received::Continue | Received::Dropped(_) => None,
        }
    }
}

/// Send one message from the device and reassemble it at the app, as a link that
/// loses nothing does.
fn across(session: &mut Session<'_>, phone: &mut Phone, message: &msg::SyncMessage) -> Vec<u8> {
    assert!(session.send(Kind::Summary, message), "the pipe was free");
    let mut now = 0u32;
    let mut got = None;
    for frame in drain(session, MTU) {
        now += 10;
        if let Some(bytes) = phone.take(session, &frame, now) {
            got = Some(bytes);
        }
    }
    let bytes = got.expect("the message arrived");
    assert!(session.is_idle(), "the transfer was acknowledged");
    bytes
}

#[test]
fn a_summary_crosses_and_is_the_message_that_was_sent() {
    let mut buffers = Buffers::new(4);
    let mut session = buffers.session();
    let mut phone = Phone::new();

    let message = summary(3);
    let bytes = across(&mut session, &mut phone, &message);

    let decoded = pb::decode_sync(&bytes).expect("it decodes");
    assert_eq!(decoded, message);
}

#[test]
fn a_message_larger_than_a_chunk_travels_in_several() {
    let mut buffers = Buffers::new(4);
    let mut session = buffers.session();
    let mut phone = Phone::new();

    // A whole chunk of points, which is what a trace carries: 48 of them is more
    // than one notification can hold, which is the whole reason for the pipe.
    let points: Vec<TracePoint> = (0..TRACE_CHUNK_SAMPLES).map(point).collect();
    let message = msg::SyncMessage {
        body: Some(msg::SyncMessage_::Body::Trace(pb::chunk_message(
            fixture_uuid(1),
            0,
            0,
            TRACE_CHUNK_SAMPLES as u32,
            &points,
        ))),
    };
    assert!(session.send(Kind::Trace, &message));

    let frames = drain(&mut session, MTU);
    assert!(
        frames.len() > 1,
        "a whole trace chunk has to be several notifications: {} frame(s)",
        frames.len()
    );

    let mut now = 0u32;
    let mut got = None;
    for frame in &frames {
        now += 10;
        if let Some(bytes) = phone.take(&mut session, frame, now) {
            got = Some(bytes);
        }
    }
    assert_eq!(
        pb::decode_sync(&got.expect("it arrived")).unwrap(),
        message,
        "the bytes the app reassembled are the message"
    );
}

#[test]
fn a_small_mtu_is_more_chunks_of_the_same_message() {
    let mut buffers = Buffers::new(4);
    let mut session = buffers.session();
    let message = chunk();

    // One *window* of chunks either way — the window is what a drain can take without
    // an acknowledgement — so the comparison is of the bytes it carries, not of the
    // frames a whole message needs (which the next test counts).
    assert!(session.send(Kind::Trace, &message));
    let big = drain(&mut session, MTU);
    session.reset();
    assert!(session.send(Kind::Trace, &message));
    let small = drain(&mut session, SMALL_MTU);

    assert_eq!(big.len(), small.len(), "one window of chunks either way");
    let bytes = |frames: &[Vec<u8>]| frames.iter().map(|f| f.len()).sum::<usize>();
    assert!(
        bytes(&small) < bytes(&big),
        "41 bytes of ATT payload per chunk against 244: {} vs {}",
        bytes(&small),
        bytes(&big)
    );
}

#[test]
fn the_send_path_stops_at_one_transfer_until_it_is_acknowledged() {
    let mut buffers = Buffers::new(4);
    let mut session = buffers.session();

    assert!(
        session.send(Kind::Summary, &summary(1)),
        "the first one goes"
    );
    let frames = drain(&mut session, MTU);
    assert!(!frames.is_empty());

    // One transfer at a time, and it is not done until the app says so.
    assert!(!session.is_idle());
    assert!(
        !session.send(Kind::Trace, &summary(2)),
        "the second message waits"
    );

    let mut phone = Phone::new();
    let mut now = 0;
    for frame in &frames {
        now += 10;
        phone.take(&mut session, frame, now);
    }
    assert!(session.is_idle(), "and now the pipe is free");
    assert!(session.send(Kind::Trace, &summary(2)));
}

#[test]
fn a_link_that_never_negotiated_is_framed_at_the_characteristic() {
    let mut buffers = Buffers::new(TRACE_CHUNK_SAMPLES);
    let mut session = buffers.session();
    let message = chunk();

    // 23 is the ATT default, and it is what a central that never sends the exchange
    // leaves behind — macOS does not. It is not a peer that can only take 20 bytes:
    // a frame of 20 is a trace chunk in ~150 notifications, which is what starved the
    // link (docs/GOTCHAS.md).
    assert!(session.send(Kind::Trace, &message));
    let unattended = drain(&mut session, ATT_MTU_DEFAULT);
    assert_eq!(
        unattended[0].len(),
        calypso_core::pipe::frame_ceiling(ATT_MTU_DEFAULT),
        "the characteristic's ceiling, not the ATT default's"
    );
    assert!(unattended[0].len() > ATT_MTU_DEFAULT);

    // A peer that asked for a small one gets frames that fit what it asked for.
    session.reset();
    assert!(session.send(Kind::Trace, &message));
    let negotiated = drain(&mut session, SMALL_MTU);
    assert_eq!(
        negotiated[0].len(),
        calypso_core::pipe::frame_ceiling(SMALL_MTU)
    );
    assert!(negotiated[0].len() < unattended[0].len());
}

/// The acknowledgement this device sends must be the number the *app's* sender expects:
/// the next in-order sequence, not the one it just received. An off-by-one here is
/// invisible to both ends' own tests — each is self-consistent — and looks like a
/// request that is never acknowledged (`Acked.ignored` in the app's log).
#[test]
fn a_single_frame_request_is_acknowledged_where_the_app_expects() {
    let mut buffers = Buffers::new(4);
    let mut session = buffers.session();

    // The app's side of a write: a `SyncRequest` as the schema encodes it, sent as one
    // frame (START and END together, which is what a request is).
    let mut request = [0u8; MESSAGE_MAX];
    let request_len = pipe_sync::encode_request(
        &msg::SyncRequest {
            body: Some(msg::SyncRequest_::Body::Trace(msg::TraceRequest {
                rep_uuid: pb::uuid_to_field(&fixture_uuid(3)),
                chunk_index: 0,
                all_chunks: true,
            })),
        },
        &mut request,
    )
    .expect("the request encodes into a transfer");
    let capacity =
        gatt_pipe::capacity(MTU, FRAME_MAX, request_len as u32).expect("the link carries a frame");
    let mut sender = Sender::new(pipe_sync::WINDOW);
    sender.start(request_len, MESSAGE_MAX);
    let mut frame = [0u8; FRAME_MAX];
    let len = match sender.next(&request[..request_len], capacity, &mut frame) {
        Ok(Send::Chunk(len)) => len,
        other => panic!("one frame was expected: {other:?}"),
    };

    // The device takes it, and answers what it owes.
    assert!(session.written(&frame[..len], 0).reply, "a reply is owed");
    let mut reply = [0u8; FRAME_MAX];
    let reply_len = session
        .reply(&mut reply)
        .expect("the device owes an acknowledgement");
    let seq = gatt_pipe::decode(&reply[..reply_len])
        .expect("a frame")
        .0
        .seq;

    assert_eq!(
        sender.on_ack(seq),
        Acked::Complete,
        "the app sent seq 0 and reads the acknowledgement of {seq}: an acknowledgement \
         names the sequence the receiver wants *next*"
    );
}

#[test]
fn a_busy_pipe_does_not_consume_the_trace_cursor() {
    let mut buffers = Buffers::new(TRACE_CHUNK_SAMPLES * 3);
    let mut session = buffers.session();
    session.start_trace(fixture_uuid(1), 0, TRACE_CHUNK_SAMPLES * 3, None);

    assert!(session.send_next_trace_chunk(), "the first chunk starts");
    assert!(
        !session.send_next_trace_chunk(),
        "the pipe is busy: saying so must not move the cursor on"
    );

    let mut phone = Phone::new();
    let mut now = 0u32;
    let mut indices = Vec::new();
    for _ in 0..2 {
        if !session.is_idle() {
            for frame in drain(&mut session, MTU) {
                now += 10;
                if let Some(bytes) = phone.take(&mut session, &frame, now) {
                    match pb::decode_sync(&bytes).expect("a message").body {
                        Some(msg::SyncMessage_::Body::Trace(chunk)) => indices.push(chunk.index),
                        other => panic!("expected a trace chunk, got {other:?}"),
                    }
                }
            }
        }
        assert!(session.is_idle(), "the app acknowledged the transfer");
        assert!(session.send_next_trace_chunk(), "the next chunk goes");
    }
    assert_eq!(indices, [0, 1], "in order, with nothing skipped");
}

/// A message wider than the window is sent in waves, each wave moved on by the
/// acknowledgements it earns. This is the contract the firmware's `keep_sending` keeps:
/// it flushes the message in flight rather than only ever starting one.
#[test]
fn a_message_wider_than_the_window_goes_in_waves() {
    let mut buffers = Buffers::new(TRACE_CHUNK_SAMPLES);
    let mut session = buffers.session();
    assert!(session.send(Kind::Trace, &wide_chunk()));

    let mut phone = Phone::new();
    let mut now = 0u32;
    let mut frames = 0;
    let mut waves = 0;
    while !session.is_idle() {
        let wave = drain(&mut session, MTU);
        assert!(!wave.is_empty(), "a wave that carries nothing is a stall");
        waves += 1;
        frames += wave.len();
        // What the device's log line is built on: the wave stopped because the window is
        // full and there is more of the message behind it.
        if waves == 1 {
            assert!(
                session.awaiting_ack(),
                "the first wave stops at the window, with the message unfinished"
            );
        }
        for frame in &wave {
            now += 10;
            phone.take(&mut session, frame, now);
        }
    }

    assert!(waves > 1, "the message needed more than one wave");
    assert!(
        frames > 8,
        "and more frames than the window holds: {frames}"
    );
    assert!(
        !session.awaiting_ack(),
        "and a message that is out is not waiting on the window"
    );
}

#[test]
fn a_request_written_by_the_app_arrives_with_an_acknowledgement_to_send() {
    let mut buffers = Buffers::new(4);
    let mut session = buffers.session();

    // The app asks for a rep's trace, in chunks, as the app's side would.
    let request = msg::SyncRequest {
        body: Some(msg::SyncRequest_::Body::Trace(msg::TraceRequest {
            rep_uuid: pb::uuid_to_field(&fixture_uuid(9)),
            all_chunks: true,
            chunk_index: 0,
        })),
    };
    let mut bytes = [0u8; 64];
    let len = {
        let mut writer: &mut [u8] = &mut bytes;
        let before = writer.len();
        let mut encoder = micropb::PbEncoder::new(&mut writer);
        request.encode(&mut encoder).expect("a request encodes");
        before - writer.len()
    };
    let encoded = &bytes[..len];

    let mut sender = Sender::new(gatt_pipe::MAX_WINDOW);
    sender.start(encoded.len(), MESSAGE_MAX).expect("it fits");
    let mut chunk = [0u8; FRAME_MAX];
    let mut frame = [0u8; FRAME_MAX];
    let mut now = 0u32;
    let mut messages = 0;
    let mut replies = 0;
    loop {
        now += 10;
        let capacity = gatt_pipe::capacity(SMALL_MTU, FRAME_MAX, encoded.len() as u32)
            .expect("the link carries a frame");
        let len = match sender.next(&encoded, capacity, &mut chunk) {
            Ok(Send::Chunk(len)) => len,
            Ok(Send::Done) => break,
            other => panic!("the sender said {other:?}"),
        };

        let written = session.written(&chunk[..len], now);
        messages += usize::from(written.message);
        if written.message {
            assert_eq!(session.request(), Some(request.clone()));
        }
        if written.reply {
            let len = session.reply(&mut frame).expect("a frame is owed");
            let (header, _) = gatt_pipe::decode(&frame[..len]).expect("it decodes");
            assert!(header.is_ack());
            sender.on_ack(header.seq);
            replies += 1;
        }
    }

    assert_eq!(messages, 1, "the write assembled into one message");
    assert!(replies > 0, "the last chunk is acknowledged");
}

#[test]
fn a_transfer_that_stops_moving_is_sent_again_and_then_given_up_on() {
    let mut buffers = Buffers::new(4);
    let mut session = buffers.session();
    assert!(session.send(Kind::Summary, &summary(1)));
    let frames = drain(&mut session, MTU);
    assert!(!frames.is_empty());

    // Nobody acknowledges: the link has gone, or the app has.
    let mut now = 0u32;
    let mut retries = 0;
    let mut abandoned = 0;
    for _ in 0..1000 {
        now += XFER_TIMEOUT_MS / 4;
        match session.tick(now) {
            Stalled::No => {}
            Stalled::Retrying { .. } => {
                retries += 1;
                // A retry is the whole transfer again, from the first chunk. The
                // sequence number carries on (the receiver takes its numbering from
                // the START), so it is the payloads that say it is the same message.
                let again = drain(&mut session, MTU);
                assert_eq!(again.len(), frames.len(), "the whole message again");
                for (retry, first) in again.iter().zip(frames.iter()) {
                    let (retry_header, retry_body) = gatt_pipe::decode(retry).expect("a frame");
                    let (first_header, first_body) = gatt_pipe::decode(first).expect("a frame");
                    assert!(retry_header.is_start(), "starting from the beginning");
                    assert_eq!(retry_body, first_body, "the chunks the first attempt sent");
                    assert_eq!(retry_header.total_len, first_header.total_len);
                }
            }
            Stalled::Abandoned { attempt } => {
                abandoned += 1;
                assert_eq!(attempt, MAX_ATTEMPTS);
                break;
            }
        }
    }

    assert_eq!(retries, usize::from(MAX_ATTEMPTS) - 1, "tries, then stops");
    assert_eq!(abandoned, 1);
    assert!(
        session.is_idle(),
        "and the pipe is free for the next message"
    );
}

#[test]
fn a_trace_stream_survives_the_transfers_it_is_made_of() {
    let mut buffers = Buffers::new(256);
    let mut session = buffers.session();
    let mut phone = Phone::new();

    // The device reads a trace out of the log into the session's storage: this is
    // that buffer, filled as `store::read_trace` fills it.
    {
        let storage = session.trace_storage();
        for (i, slot) in storage.iter_mut().take(100).enumerate() {
            *slot = point(i);
        }
    }
    session.start_trace(fixture_uuid(7), 1000, 100, None);
    assert!(session.streaming());

    let mut chunks = 0;
    let mut indices = Vec::new();
    let mut samples = Vec::new();
    let mut now = 0u32;
    while session.send_next_trace_chunk() {
        let mut got = None;
        for frame in drain(&mut session, MTU) {
            now += 10;
            if let Some(bytes) = phone.take(&mut session, &frame, now) {
                got = Some(bytes);
            }
        }
        let bytes = got.expect("the chunk arrived");
        match pb::decode_sync(&bytes).unwrap().body {
            Some(msg::SyncMessage_::Body::Trace(chunk)) => {
                chunks += 1;
                indices.push(chunk.index);
                for s in chunk.samples.iter() {
                    samples.push(pb::trace_point_from_message(s));
                }
            }
            other => panic!("expected a trace chunk, got {other:?}"),
        }
    }

    // One transfer per stored chunk, rather than one per three points.
    let expected = pb::chunk_count_for(100);
    assert_eq!(chunks, expected as usize);
    assert_eq!(indices, (0..expected).collect::<Vec<_>>());
    assert_eq!(samples.len(), 100, "every point of the trace");
    assert!(!session.streaming(), "and the stream is done");
}

#[test]
fn a_single_chunk_request_sends_the_one_chunk() {
    let mut buffers = Buffers::new(64);
    let mut session = buffers.session();
    for (i, slot) in session.trace_storage().iter_mut().take(48).enumerate() {
        *slot = point(i);
    }
    session.start_trace(fixture_uuid(3), 1000, 48, Some(0));

    assert!(session.send_next_trace_chunk(), "one chunk");
    let frames = drain(&mut session, MTU);
    let mut phone = Phone::new();
    let mut now = 0;
    let mut got = None;
    for frame in &frames {
        now += 10;
        got = phone.take(&mut session, frame, now).or(got);
    }
    match pb::decode_sync(&got.expect("it arrived")).unwrap().body {
        Some(msg::SyncMessage_::Body::Trace(chunk)) => {
            assert_eq!(chunk.index, 0);
            assert_eq!(chunk.samples.len(), TRACE_CHUNK_SAMPLES);
        }
        other => panic!("expected a trace chunk, got {other:?}"),
    }
    assert!(!session.send_next_trace_chunk(), "and no more");
    assert!(!session.streaming());
    assert!(session.is_idle());
}

#[test]
fn an_empty_trace_streams_nothing() {
    let mut buffers = Buffers::new(16);
    let mut session = buffers.session();
    session.trace_storage();
    session.start_trace(fixture_uuid(1), 0, 0, None);
    assert!(!session.send_next_trace_chunk());
    assert!(!session.streaming());
}

#[test]
fn a_lost_chunk_leaves_the_app_waiting_for_the_rest_of_a_trace() {
    let mut buffers = Buffers::new(64);
    let mut session = buffers.session();
    for (i, slot) in session.trace_storage().iter_mut().take(48).enumerate() {
        *slot = point(i);
    }
    session.start_trace(fixture_uuid(2), 1000, 48, None);
    assert!(session.streaming());

    assert!(session.send_next_trace_chunk(), "a chunk");
    let frames = drain(&mut session, MTU);
    assert!(!frames.is_empty());

    // The app never sees the last of it: no acknowledgement, so the transfer is
    // given up on — and a trace the app cannot assemble is not worth sending the
    // rest of.
    let mut now = 0u32;
    let mut abandoned = false;
    for _ in 0..64 {
        now += XFER_TIMEOUT_MS / 4;
        if matches!(session.tick(now), Stalled::Abandoned { .. }) {
            abandoned = true;
            break;
        }
        drain(&mut session, MTU);
    }
    assert!(abandoned, "the transfer was given up on");
    assert!(!session.streaming(), "and the rest of the trace with it");
}

#[test]
fn a_bad_frame_is_dropped_without_a_word() {
    let mut buffers = Buffers::new(4);
    let mut session = buffers.session();

    // A frame whose CRC does not check out is not half-believed: it is dropped,
    // and the sender's timeout is the recovery.
    let mut frame = reply_frame(0, flags::ACK);
    let last = frame.len() - 1;
    frame[last] ^= 0xFF;
    assert_eq!(session.written(&frame, 0), Default::default());

    // And neither is a reply to a transfer this end is not in: it is somebody
    // else's, or a stale one, and the sender ignores it.
    assert_eq!(
        session.written(&reply_frame(9, flags::ACK), 0),
        Default::default()
    );
}

#[test]
fn a_disconnect_forgets_everything_in_flight() {
    let mut buffers = Buffers::new(4);
    let mut session = buffers.session();
    assert!(session.send(Kind::Summary, &summary(1)));
    assert!(!drain(&mut session, MTU).is_empty());
    assert!(!session.is_idle());

    session.trace_storage();
    session.start_trace(fixture_uuid(1), 0, 0, None);

    session.reset();

    assert!(session.is_idle(), "a new connection starts empty");
    assert!(!session.streaming());
    assert_eq!(session.message(), b"", "and with nothing received");
}

#[test]
fn the_cursor_and_the_pipe_together_stream_a_batch() {
    // The device's summary path in miniature: a batch read out of the log, the
    // cursor advancing on each summary the pipe took, and the app receiving them
    // in order. The batch is four summaries and the pipe holds one transfer at a
    // time, so this is also the case that says what the firmware's pump is for:
    // the next one goes only once the app has acknowledged the last.
    let mut buffers = Buffers::new(4);
    let mut session = buffers.session();
    let mut phone = Phone::new();
    let batch: Vec<msg::RepSummary> = (1..=4).map(rep_summary).collect();

    let mut cursor = SummaryCursor::new();
    let mut now = 0u32;
    let mut arrived = Vec::new();
    for (index, summary) in batch.iter().enumerate() {
        while !session.send(Kind::Summary, &sync::summary_message(summary.clone())) {
            assert!(index > 0, "the first message cannot be blocked");
            arrived.extend(receive(&mut session, &mut phone, &mut now));
        }
        cursor.advance(1);
    }
    arrived.extend(receive(&mut session, &mut phone, &mut now));

    assert_eq!(cursor.next(), batch.len(), "the cursor counts what it sent");
    assert_eq!(
        arrived,
        (1..=4u32).map(fixture_uuid).collect::<Vec<_>>(),
        "and every one arrived, in order"
    );
}
