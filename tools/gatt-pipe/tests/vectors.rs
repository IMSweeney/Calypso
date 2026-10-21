//! Every case both implementations of the pipe have to agree on.
//!
//! The cases are data — `tests/vectors/pipe.txt` — because there are two
//! implementations of this protocol: this one, and the app's in Dart. Two suites
//! is how two implementations drift apart quietly, so both read this file and
//! answer the same. The numbers in it were produced by this side, which is the
//! reference.

use gatt_pipe::{Acked, Header, Received, Receiver, Send, Sender, DEFAULT_CHUNK_MAX, MAX_ATTEMPTS};

/// The window both sides were written for; the vectors do not carry it.
const WINDOW: u8 = 8;

#[derive(Debug)]
struct Case {
    name: String,
    len: usize,
    mtu: usize,
    max_message: usize,
    drop_nth: usize,
    corrupt_nth: usize,
    duplicate_nth: usize,
    swallow_acks: bool,
    timeout_ms: u32,
    delivered: bool,
    chunks: usize,
}

impl Case {
    /// The message both sides build from its length alone.
    fn message(&self) -> Vec<u8> {
        (0..self.len).map(|i| (i % 251) as u8).collect()
    }
}

fn cases() -> Vec<Case> {
    include_str!("vectors/pipe.txt")
        .lines()
        .filter(|line| !line.trim().is_empty() && !line.starts_with('#'))
        .map(|line| {
            let field: Vec<&str> = line.split('|').collect();
            assert_eq!(field.len(), 11, "a vector has eleven fields: {line}");
            let number = |i: usize| -> usize { field[i].parse().expect("a number") };
            Case {
                name: field[0].to_string(),
                len: number(1),
                mtu: number(2),
                max_message: number(3),
                drop_nth: number(4),
                corrupt_nth: number(5),
                duplicate_nth: number(6),
                swallow_acks: number(7) == 1,
                timeout_ms: number(8) as u32,
                delivered: number(9) == 1,
                chunks: number(10),
            }
        })
        .collect()
}

/// What a transfer did.
struct Outcome {
    message: Option<Vec<u8>>,
    chunks: usize,
    attempts: u8,
}

/// Move one message through the pipe under this case's faults.
///
/// The faults and the retry policy are the case's, not this harness's: it is the
/// same shape the app's test drives, deliberately, down to the order in which a
/// chunk is dropped, corrupted and duplicated.
fn pump(case: &Case) -> Outcome {
    let message = case.message();
    let mut sender = Sender::new(WINDOW);
    let mut receiver = Receiver::new(WINDOW);
    sender
        .start(message.len(), case.max_message)
        .expect("it fits");

    let mut buffer = vec![0u8; case.max_message];
    let mut wire = [0u8; DEFAULT_CHUNK_MAX];
    let mut reply = [0u8; 16];
    let mut chunks = 0;
    let mut nth = 0;
    let mut now = 0u32;

    for _ in 0..100_000 {
        now += 10;
        let capacity = gatt_pipe::capacity(case.mtu, DEFAULT_CHUNK_MAX, message.len() as u32)
            .expect("the link carries a frame");

        let chunk = match sender
            .next(&message, capacity, &mut wire)
            .expect("it encodes")
        {
            Send::Chunk(len) => {
                nth += 1;
                chunks += 1;
                if case.drop_nth == nth {
                    sender.note_progress(now);
                    continue;
                }
                let mut chunk = wire[..len].to_vec();
                if case.corrupt_nth == nth && chunk.len() > 4 {
                    chunk[3] ^= 0x40;
                }
                chunk
            }
            Send::WindowFull | Send::Done => {
                if sender.stalled(now, case.timeout_ms) {
                    if sender.attempts() >= MAX_ATTEMPTS {
                        return Outcome {
                            message: None,
                            chunks,
                            attempts: sender.attempts(),
                        };
                    }
                    sender.abort();
                    sender
                        .start(message.len(), case.max_message)
                        .expect("it fits");
                    sender.note_progress(now);
                }
                continue;
            }
        };

        let mut outcome = receiver.on_chunk(&chunk, &mut buffer, case.max_message);
        if case.duplicate_nth == nth {
            outcome = receiver.on_chunk(&chunk, &mut buffer, case.max_message);
        }

        match outcome {
            Received::Delivered { len } => {
                return Outcome {
                    message: Some(buffer[..len].to_vec()),
                    chunks,
                    attempts: sender.attempts(),
                }
            }
            Received::Ack { next_expected } => {
                if !case.swallow_acks {
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
                    assert_ne!(sender.on_ack(header.seq), Acked::Ignored);
                }
                sender.note_progress(now);
            }
            Received::Nak => {
                sender.on_nak();
                sender.note_progress(now);
            }
            Received::Continue | Received::Dropped(_) => sender.note_progress(now),
        }
    }

    panic!("the pump never concluded: {}", case.name);
}

#[test]
fn both_implementations_answer_the_vectors() {
    let cases = cases();
    assert!(cases.len() >= 8, "the vectors are the point of this test");

    for case in cases {
        let outcome = pump(&case);
        assert_eq!(
            outcome.message.is_some(),
            case.delivered,
            "{}: delivered",
            case.name
        );
        if let Some(message) = &outcome.message {
            assert_eq!(*message, case.message(), "{}: the bytes", case.name);
        }
        assert_eq!(
            outcome.chunks, case.chunks,
            "{}: chunks on the wire (attempts {})",
            case.name, outcome.attempts
        );
    }
}
