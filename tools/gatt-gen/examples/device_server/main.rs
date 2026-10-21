//! A generated table, compiled.
//!
//! `cargo test` builds examples, and this is the only place in the crate that
//! *compiles* what the generator emits rather than comparing its text. The two
//! files beside it are what `gatt-gen` writes for
//! `tests/fixtures/valid/device.proto` — a bare characteristic, a protobuf one,
//! and a pipe — so between them they cover every kind of value the schema can
//! declare, and the pipe glue that nothing else builds.
//!
//! Regenerate them with `UPDATE_GENERATED=1 cargo test --test example_files`;
//! that test says why this is a test rather than a script.
//!
//! Run it to watch a message go through the pipe's codec:
//! `cargo run --example device_server`.

/// `micropb-gen`'s output for the fixture's messages.
///
/// Included rather than declared as a module, for two reasons: the lints it
/// trips are the generated internals' business (the schema is the interface),
/// and rustfmt does not descend into an `include!`, so a generated file that is
/// not rustfmt-shaped stays that way. Calypso does the same thing from
/// `$OUT_DIR` — see `calypso-core/src/pb.rs`.
#[allow(nonstandard_style, unused, unused_parens, clippy::all)]
mod messages {
    include!("messages.rs");
}

#[path = "gatt.rs"]
mod gatt;

/// Where the generated table looks for its messages.
mod pb {
    pub mod msg {
        pub use crate::messages::fixture_::*;
    }
}

fn main() {
    // Building the table is what compiles the macros the generator writes: the
    // services, the property sets, the value types and the pipe module.
    let server = gatt::GattServer::new_default("example").expect("the name fits");

    println!("schema {:016x}", gatt::SCHEMA_HASH);
    println!("the fixture schema declares:");
    println!("  service {}", gatt::uuid::BATTERY_SERVICE_TEXT);
    println!("  service {}", gatt::uuid::DEVICE_SERVICE_TEXT);
    println!("  characteristic {} (bare)", gatt::uuid::BATTERY_LEVEL_TEXT);
    println!("  characteristic {} (request)", gatt::uuid::DEVICE_RX_TEXT);
    println!("  characteristic {} (response)", gatt::uuid::DEVICE_TX_TEXT);
    println!(
        "pipe \"main\": {} B a chunk, {} B a message, window {}, timeout {} ms",
        gatt::pipe_main::CHUNK_MAX,
        gatt::pipe_main::MESSAGE_MAX,
        gatt::pipe_main::WINDOW,
        gatt::pipe_main::XFER_TIMEOUT_MS,
    );
    println!(
        "  handles: request {} response {} level {}",
        server.device.rx.handle, server.device.tx.handle, server.battery.level.handle,
    );

    // The half of the generated code a declaration cannot exercise: a message
    // through the pipe's codec, both ways.
    let command = pb::msg::Command {
        id: 7,
        ..Default::default()
    };
    let mut encoded = [0u8; gatt::pipe_main::MESSAGE_MAX];
    let len = gatt::pipe_main::encode_request(&command, &mut encoded).expect("it fits");
    let decoded = gatt::pipe_main::decode_request(&encoded[..len]).expect("it comes back");
    assert_eq!(decoded.id, command.id);
    println!("a {len}-byte Command through pipe_main, both ways");
}
