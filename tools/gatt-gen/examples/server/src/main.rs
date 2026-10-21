//! A minimal BLE GATT server, generated from a `.proto`.
//!
//! [`build.rs`](../build.rs) is the whole integration — a build dependency and
//! two calls — and this file is what makes its output *compile*: every kind of
//! thing this schema declares, the messages and the table and the value types,
//! is built here. `cargo test` in this crate is therefore a check on the
//! generator.
//!
//! Nothing here touches a radio, so it runs on the host:
//! `cargo run -p gatt-gen-example-server`.

/// The generated message types, where the table looks for them.
///
/// Included rather than declared as a module because `micropb`'s output is not
/// rustfmt-shaped and does not have to be: the schema is the interface, and these
/// are its generated internals.
mod pb {
    pub mod msg {
        #![allow(nonstandard_style, unused, unused_parens, clippy::all)]
        include!(concat!(env!("OUT_DIR"), "/messages.rs"));

        /// `micropb` nests its output under the proto package; the table looks
        /// for the messages one level up.
        pub use sensor_::*;
    }
}

/// The generated GATT table.
///
/// The allow is here rather than in the generated file because an `include!`
/// cannot carry an inner attribute: a file that allowed its own lints could not be
/// included at all, which would tie the output to one way of mounting it.
#[allow(clippy::all, nonstandard_style, unused, irrefutable_let_patterns)]
mod gatt {
    include!(concat!(env!("OUT_DIR"), "/gatt_generated.rs"));
}

use pb::msg;

fn main() {
    // Building the table is what compiles the macros the generator emits: the
    // service, the property sets, and the value types.
    let server = gatt::Server::new_default("example-sensor").expect("the name fits");

    println!("the schema declares:");
    println!(
        "  service               {}",
        gatt::uuid::SENSOR_SERVICE_TEXT
    );
    println!(
        "  {} (bare u8, read+notify, max 100)",
        gatt::uuid::SENSOR_LEVEL_TEXT
    );
    println!(
        "  {} (a message, read+write, max_len 16)",
        gatt::uuid::SENSOR_SETTINGS_TEXT
    );
    println!(
        "  handles: level {} settings {}",
        server.sensor.level.handle, server.sensor.settings.handle,
    );
    println!(
        "  readable {} notifiable {}",
        server.sensor.settings.handle, server.sensor.level.handle,
    );

    // The half of the generated code a declaration cannot exercise: a message
    // through the value type the characteristic stores.
    let config = msg::Config { period_ms: 1000 };
    let value = gatt::SensorSettingsValue::encode(&config).expect("it fits");
    assert_eq!(value.decode().expect("it comes back").period_ms, 1000);
    println!(
        "a Config through the value type and back: {} bytes on the wire",
        value.as_bytes().len()
    );
}
