//! A pipe, end to end, on the host.
//!
//! The generated glue (from `build.rs`) and the runtime (`gatt-pipe`) together:
//! a message larger than one characteristic value is cut into chunks by one end,
//! reassembled by the other, and acknowledged along the way. Nothing here touches
//! a radio — the "link" is a loop over two buffers — which is the point: the
//! failure modes a pipe has are timing and loss, and neither needs a radio to
//! reproduce.
//!
//! `cargo run -p gatt-gen-example-pipe`

/// The generated message types, where the table looks for them.
mod pb {
    pub mod msg {
        #![allow(nonstandard_style, unused, unused_parens, clippy::all)]
        include!(concat!(env!("OUT_DIR"), "/messages.rs"));

        pub use bulk_::*;
    }
}

/// The generated GATT table, pipe glue included.
#[allow(clippy::all, nonstandard_style, unused, irrefutable_let_patterns)]
mod gatt {
    include!(concat!(env!("OUT_DIR"), "/gatt.rs"));
}

use gatt_pipe::{Header, Received, Receiver, Send, Sender};
use pb::msg;

/// The smallest MTU a link can negotiate, so the chunking is visible: twenty
/// bytes of ATT payload per chunk, less the frame.
const MTU: usize = 23;

fn main() {
    // The table, and one message through its codec: this is what the generator
    // knows and the runtime does not.
    let server = gatt::Server::new_default("example-bulk").expect("the name fits");
    println!(
        "pipe bulk: {} -> {}, {} B a chunk, {} B a message, window {}",
        gatt::uuid::BULK_DOWN_TEXT,
        gatt::uuid::BULK_UP_TEXT,
        gatt::pipe_bulk::CHUNK_MAX,
        gatt::pipe_bulk::MESSAGE_MAX,
        gatt::pipe_bulk::WINDOW,
    );
    println!(
        "  handles: down {} up {}",
        server.bulk.down.handle, server.bulk.up.handle
    );

    let payload: heapless::Vec<u8, 256> =
        heapless::Vec::from_slice(&(0..100u8).collect::<Vec<u8>>()).expect("it fits");
    let command = msg::Command { id: 7, payload };
    let mut message = [0u8; gatt::pipe_bulk::MESSAGE_MAX];
    let encoded = gatt::pipe_bulk::encode_request(&command, &mut message).expect("it fits");

    // Two ends over one link. A real pair would be a device and a phone; here
    // they are a sender and a receiver with a buffer each, which is all the
    // runtime needs to be driven.
    let capacity = gatt_pipe::capacity(MTU, gatt::pipe_bulk::CHUNK_MAX, encoded as u32)
        .expect("the link carries a frame");
    let mut sender = Sender::new(gatt::pipe_bulk::WINDOW);
    let mut receiver = Receiver::new(gatt::pipe_bulk::WINDOW);
    sender
        .start(encoded, gatt::pipe_bulk::MESSAGE_MAX)
        .expect("it fits");

    let mut chunk = [0u8; gatt::pipe_bulk::CHUNK_MAX];
    let mut ack = [0u8; 8];
    let mut reassembled = [0u8; gatt::pipe_bulk::MESSAGE_MAX];
    let mut chunks = 0usize;
    let mut acks = 0usize;

    loop {
        let sent = match sender.next(&message[..encoded], capacity, &mut chunk) {
            Ok(Send::Chunk(len)) => len,
            Ok(Send::WindowFull | Send::Done) => break,
            Err(err) => panic!("{err}"),
        };
        chunks += 1;

        match receiver.on_chunk(
            &chunk[..sent],
            &mut reassembled,
            gatt::pipe_bulk::MESSAGE_MAX,
        ) {
            Received::Continue => {}
            Received::Ack { next_expected } => {
                acks += 1;
                let len = gatt_pipe::encode(
                    Header {
                        seq: next_expected,
                        flags: gatt_pipe::flags::ACK,
                        total_len: None,
                    },
                    &[],
                    &mut ack,
                )
                .expect("an ack fits");
                let (header, _) = gatt_pipe::decode(&ack[..len]).expect("an ack decodes");
                sender.on_ack(header.seq);
            }
            Received::Delivered { .. } => break,
            other => panic!("the receiver said {other:?}"),
        }
    }

    // What arrived is the message that was sent: the same bytes, decoded by the
    // same generated codec.
    let decoded = gatt::pipe_bulk::decode_request(&reassembled[..encoded]).expect("it decodes");
    assert_eq!(decoded, command);
    println!(
        "  a {encoded}-byte Command in {chunks} chunks of at most {capacity} B, {acks} ack(s)"
    );
    println!(
        "  decoded: id {} and {} bytes of payload",
        decoded.id,
        decoded.payload.len()
    );
}
