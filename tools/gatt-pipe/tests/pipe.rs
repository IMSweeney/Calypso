//! The pipe, end to end: two state machines against each other over a link that
//! loses, duplicates, corrupts and stalls.
//!
//! These are the failures a BLE link actually produces, and none of them can be
//! produced on a bench on demand — which is why the state machines hold no
//! buffers and no clock, and can be driven from a test.

use gatt_pipe::{
    self, Header, Received, Receiver, Send, Sender, DEFAULT_CHUNK_MAX, MAX_ATTEMPTS, MAX_WINDOW,
};

/// The no-progress timeout the caller would apply, in milliseconds.
const TIMEOUT_MS: u32 = 500;

/// How the link between the two ends misbehaves.
#[derive(Default, Clone, Copy)]
struct Faults {
    /// Drop the nth data chunk, counted from one.
    drop_nth: Option<usize>,
    /// Corrupt a payload byte of the nth data chunk.
    corrupt_nth: Option<usize>,
    /// Deliver the nth data chunk twice.
    duplicate_nth: Option<usize>,
    /// Swallow every acknowledgement: the sender's timeout is then the only way
    /// out, which is the case that must not hang.
    swallow_acks: bool,
}

/// What a transfer did.
struct Outcome {
    message: Option<Vec<u8>>,
    attempts: u8,
    /// The sequence numbers the sender put on the wire, in order.
    sent: Vec<u8>,
    naks: usize,
}

/// Move one message through the pipe, with faults.
fn pump(message: &[u8], mtu: usize, max_message: usize, faults: Faults) -> Outcome {
    let mut sender = Sender::new(8);
    let mut receiver = Receiver::new(8);
    sender.start(message.len(), max_message).expect("it fits");

    let mut buffer = vec![0u8; max_message];
    let mut wire = [0u8; 512];
    let mut reply = [0u8; 64];
    let mut sent = Vec::new();
    let mut naks = 0;
    let mut nth = 0usize;
    let mut now = 0u32;

    // Bounded: the protocol must reach a conclusion, and a test that spins for
    // ever would not say so.
    for _ in 0..10_000 {
        now += 10;
        let capacity = gatt_pipe::capacity(mtu, DEFAULT_CHUNK_MAX, message.len() as u32)
            .expect("the link carries a frame");

        let chunk = match sender
            .next(message, capacity, &mut wire)
            .expect("it encodes")
        {
            Send::Chunk(len) => {
                nth += 1;
                sent.push(wire[0]);
                if faults.drop_nth == Some(nth) {
                    // Gone: as if the notification never went out. It *did* go
                    // out from the sender's point of view, so that is progress.
                    sender.note_progress(now);
                    continue;
                }
                let mut chunk = wire[..len].to_vec();
                if faults.corrupt_nth == Some(nth) && chunk.len() > 4 {
                    chunk[3] ^= 0x40;
                }
                chunk
            }
            // Nothing more goes out until the peer moves or time passes.
            Send::WindowFull | Send::Done => {
                if sender.stalled(now, TIMEOUT_MS) {
                    // The caller's policy, not the state machine's: a timeout
                    // means the whole transfer goes again, as a new attempt, and
                    // `attempts` is what the limit is counted against. A refusal
                    // (a NAK) is different — that one rewinds and continues.
                    if sender.attempts() >= MAX_ATTEMPTS {
                        return Outcome {
                            message: None,
                            attempts: sender.attempts(),
                            sent,
                            naks,
                        };
                    }
                    sender.abort();
                    sender
                        .start(message.len(), max_message)
                        .expect("it still fits");
                    sender.note_progress(now);
                }
                // Nothing moved, so the progress clock is deliberately *not*
                // restarted: the stall timeout is the only thing that ends this.
                continue;
            }
        };

        let mut outcome = receiver.on_chunk(&chunk, &mut buffer, max_message);
        if faults.duplicate_nth == Some(nth) {
            // The radio delivered it twice; the receiver sees it twice.
            outcome = receiver.on_chunk(&chunk, &mut buffer, max_message);
        }

        match outcome {
            Received::Delivered { len } => {
                return Outcome {
                    message: Some(buffer[..len].to_vec()),
                    attempts: sender.attempts(),
                    sent,
                    naks,
                }
            }
            Received::Ack { next_expected } => {
                if !faults.swallow_acks {
                    let len = gatt_pipe::encode(
                        Header {
                            seq: next_expected,
                            flags: gatt_pipe::flags::ACK,
                            total_len: None,
                        },
                        &[],
                        &mut reply,
                    )
                    .expect("an ack encodes");
                    let (header, _) = gatt_pipe::decode(&reply[..len]).expect("an ack decodes");
                    sender.on_ack(header.seq);
                }
                sender.note_progress(now);
            }
            Received::Nak => {
                naks += 1;
                sender.on_nak();
                sender.note_progress(now);
            }
            Received::Continue | Received::Dropped(_) => sender.note_progress(now),
        }
    }

    panic!("the pump never concluded: the protocol can spin");
}

#[test]
fn a_one_byte_message_travels_in_one_chunk() {
    let outcome = pump(&[42], 247, 256, Faults::default());
    assert_eq!(outcome.message, Some(vec![42]));
    assert_eq!(outcome.sent.len(), 1, "start and end on the same chunk");
    assert_eq!(outcome.attempts, 1);
}

#[test]
fn a_message_that_fills_one_chunk_travels_in_one() {
    let message: Vec<u8> = (0..200).map(|i| i as u8).collect();
    let outcome = pump(&message, 247, 256, Faults::default());
    assert_eq!(outcome.message, Some(message));
    assert_eq!(outcome.sent.len(), 1);
}

#[test]
fn a_many_chunk_message_arrives_whole_and_in_order() {
    let message: Vec<u8> = (0..1000).map(|i| (i % 251) as u8).collect();
    let outcome = pump(&message, 247, 2048, Faults::default());
    assert_eq!(outcome.message, Some(message));
    assert!(outcome.sent.len() > 4, "1000 bytes is several chunks");
    assert_eq!(outcome.naks, 0);
}

#[test]
fn a_small_mtu_just_means_more_chunks() {
    let message: Vec<u8> = (0..300).map(|i| (i % 97) as u8).collect();
    let outcome = pump(&message, 23, 512, Faults::default());
    assert_eq!(outcome.message, Some(message));
    assert!(
        outcome.sent.len() > 15,
        "twenty bytes of ATT payload a chunk"
    );
}

#[test]
fn an_empty_message_is_a_message() {
    let outcome = pump(&[], 247, 256, Faults::default());
    assert_eq!(outcome.message, Some(vec![]));
}

#[test]
fn a_message_larger_than_the_reassembly_buffer_is_refused_before_it_starts() {
    let mut sender = Sender::new(8);
    let error = sender.start(300, 256).unwrap_err();
    assert!(matches!(
        error,
        gatt_pipe::SendError::TooLarge { len: 300, max: 256 }
    ));
    // Exactly the limit is fine: the buffer is the promise.
    assert!(sender.start(256, 256).is_ok());
}

#[test]
fn a_dropped_chunk_costs_a_nak_and_nothing_else() {
    let message: Vec<u8> = (0..600).map(|i| (i % 13) as u8).collect();
    let outcome = pump(
        &message,
        247,
        1024,
        Faults {
            drop_nth: Some(2),
            ..Faults::default()
        },
    );
    assert_eq!(outcome.message, Some(message), "the transfer completed");
    // The receiver sees the gap, refuses, and the sender starts the transfer
    // again — inside the same attempt, since nothing timed out.
    assert!(outcome.naks > 0, "the gap was refused");
    assert_eq!(outcome.attempts, 1, "no timeout, so no new attempt");
}

#[test]
fn a_corrupted_chunk_is_dropped_and_the_transfer_recovers() {
    let message: Vec<u8> = (0..600).map(|i| (i % 17) as u8).collect();
    let outcome = pump(
        &message,
        247,
        1024,
        Faults {
            corrupt_nth: Some(3),
            ..Faults::default()
        },
    );
    assert_eq!(outcome.message, Some(message));
}

#[test]
fn a_duplicated_chunk_is_answered_and_not_appended_twice() {
    let message: Vec<u8> = (0..600).map(|i| (i % 19) as u8).collect();
    let outcome = pump(
        &message,
        247,
        1024,
        Faults {
            duplicate_nth: Some(2),
            ..Faults::default()
        },
    );
    assert_eq!(
        outcome.message,
        Some(message),
        "a repeat must not land in the message twice"
    );
}

#[test]
fn a_transfer_that_fits_the_window_needs_no_acknowledgement_at_all() {
    // Three chunks into a window of eight: the acknowledgements are for flow
    // control, not for correctness, so losing all of them costs nothing.
    let message: Vec<u8> = (0..600).map(|i| (i % 23) as u8).collect();
    let outcome = pump(
        &message,
        247,
        1024,
        Faults {
            swallow_acks: true,
            ..Faults::default()
        },
    );
    assert_eq!(outcome.message, Some(message));
    assert_eq!(outcome.attempts, 1);
}

#[test]
fn a_reply_that_never_comes_gives_up_rather_than_hangs() {
    // Long enough to need the window to move: 4000 bytes is 17 chunks at a
    // 243-byte payload, against a window of eight.
    let message: Vec<u8> = (0..4000).map(|i| (i % 241) as u8).collect();
    let outcome = pump(
        &message,
        247,
        4096,
        Faults {
            swallow_acks: true,
            ..Faults::default()
        },
    );
    assert_eq!(outcome.message, None, "nothing was delivered");
    assert_eq!(outcome.attempts, MAX_ATTEMPTS, "and it stopped trying");
}

#[test]
fn a_start_promising_more_than_the_buffer_gets_a_nak() {
    let mut receiver = Receiver::new(8);
    let mut buffer = [0u8; 64];
    let mut chunk = [0u8; 64];

    let len = gatt_pipe::encode(
        Header {
            seq: 0,
            flags: gatt_pipe::flags::START,
            total_len: Some(65),
        },
        &[],
        &mut chunk,
    )
    .unwrap();
    assert_eq!(
        receiver.on_chunk(&chunk[..len], &mut buffer, 64),
        Received::Nak
    );
    assert!(!receiver.in_transfer(), "and nothing was kept");

    // The same promise within the buffer is taken.
    let len = gatt_pipe::encode(
        Header {
            seq: 0,
            flags: gatt_pipe::flags::START,
            total_len: Some(64),
        },
        &[],
        &mut chunk,
    )
    .unwrap();
    assert_eq!(
        receiver.on_chunk(&chunk[..len], &mut buffer, 64),
        Received::Continue
    );
    assert!(receiver.in_transfer());
}

#[test]
fn a_chunk_from_the_middle_of_nowhere_gets_a_nak() {
    let mut receiver = Receiver::new(8);
    let mut buffer = [0u8; 64];
    let mut chunk = [0u8; 64];
    let len = gatt_pipe::encode(
        Header {
            seq: 3,
            flags: 0,
            total_len: None,
        },
        b"middle",
        &mut chunk,
    )
    .unwrap();
    // No START, so there is nothing to append it to.
    assert_eq!(
        receiver.on_chunk(&chunk[..len], &mut buffer, 64),
        Received::Nak
    );
}

#[test]
fn a_second_start_restarts_the_transfer() {
    let mut receiver = Receiver::new(8);
    let mut buffer = [0u8; 64];
    let mut chunk = [0u8; 64];

    // A first transfer, abandoned part-way: the peer timed out and will retry.
    let len = gatt_pipe::encode(
        Header {
            seq: 0,
            flags: gatt_pipe::flags::START,
            total_len: Some(8),
        },
        b"half",
        &mut chunk,
    )
    .unwrap();
    receiver.on_chunk(&chunk[..len], &mut buffer, 64);

    // The retry begins again, with a sequence number of its own.
    let len = gatt_pipe::encode(
        Header {
            seq: 5,
            flags: gatt_pipe::flags::START | gatt_pipe::flags::END,
            total_len: Some(5),
        },
        b"whole",
        &mut chunk,
    )
    .unwrap();
    assert_eq!(
        receiver.on_chunk(&chunk[..len], &mut buffer, 64),
        Received::Delivered { len: 5 }
    );
    assert_eq!(&buffer[..5], b"whole");
}

#[test]
fn a_disconnect_resets_both_ends() {
    let mut sender = Sender::new(4);
    let mut receiver = Receiver::new(4);
    // The buffer is the promise: a receiving end always has one as big as the
    // transfers it accepts.
    let mut buffer = [0u8; 128];
    let mut chunk = [0u8; 64];

    let message = [7u8; 100];
    sender.start(message.len(), 128).unwrap();
    let Send::Chunk(len) = sender.next(&message, 32, &mut chunk).unwrap() else {
        panic!("a chunk goes out");
    };
    receiver.on_chunk(&chunk[..len], &mut buffer, 128);
    assert!(receiver.in_transfer());

    sender.reset();
    receiver.reset();
    assert!(sender.is_idle());
    assert!(!receiver.in_transfer());
    assert_eq!(sender.attempts(), 0, "a new connection starts over");

    // And the same sequence numbers work again, because both ends forgot.
    sender.start(5, 128).unwrap();
    let Send::Chunk(len) = sender.next(b"again", 32, &mut chunk).unwrap() else {
        panic!("a chunk goes out");
    };
    assert_eq!(chunk[0], 0, "the sequence starts at zero again");
    let (header, payload) = gatt_pipe::decode(&chunk[..len]).unwrap();
    assert!(header.is_start() && header.is_end());
    assert_eq!(payload, b"again");
}

#[test]
fn the_window_is_what_stops_a_sender_running_ahead() {
    let mut sender = Sender::new(2);
    let message = [0u8; 200];
    let mut chunk = [0u8; 64];
    sender.start(message.len(), 256).unwrap();

    // Two chunks out, and then nothing goes: this is the backpressure, and it is
    // the only thing standing between a fast device and a slow phone.
    for _ in 0..2 {
        assert!(matches!(
            sender.next(&message, 32, &mut chunk).unwrap(),
            Send::Chunk(_)
        ));
    }
    assert_eq!(
        sender.next(&message, 32, &mut chunk).unwrap(),
        Send::WindowFull
    );

    // An acknowledgement of the first chunk frees exactly one slot.
    assert_eq!(
        sender.on_ack(1),
        gatt_pipe::Acked::Advanced { in_flight: 1 }
    );
    assert!(matches!(
        sender.next(&message, 32, &mut chunk).unwrap(),
        Send::Chunk(_)
    ));
    assert_eq!(
        sender.next(&message, 32, &mut chunk).unwrap(),
        Send::WindowFull
    );
}

#[test]
fn a_window_that_could_not_be_acknowledged_is_refused() {
    // A cumulative acknowledgement against a wrapping u8 is only unambiguous
    // below half the sequence space, and the constructor is where that is said.
    let too_big = MAX_WINDOW + 1;
    assert!(std::panic::catch_unwind(|| Sender::new(too_big)).is_err());
    assert!(std::panic::catch_unwind(|| Sender::new(0)).is_err());
    assert!(std::panic::catch_unwind(|| Receiver::new(too_big)).is_err());
}
