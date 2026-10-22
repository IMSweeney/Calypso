//! A minimal BLE GATT server, generated from a `.proto`.
//!
//! [`build.rs`](../build.rs) is the whole integration — a build dependency and two calls — and this file is what makes its output *compile*: every kind of thing this schema declares, the messages and the table and the value types, is built here. `cargo test` in this crate is therefore a check on the generator.
//!
//! Nothing here touches a radio, so it runs on the host:
//! `cargo run -p gatt-gen-example-server`.

/// The generated message types, where the table looks for them.
///
/// Included rather than declared as a module because `micropb`'s output is not rustfmt-shaped and does not have to be: the schema is the interface, and these are its generated internals.
mod pb {
    pub mod msg {
        #![allow(nonstandard_style, unused, unused_parens, clippy::all)]
        include!(concat!(env!("OUT_DIR"), "/messages.rs"));

        /// `micropb` nests its output under the proto package; the table looks for the messages one level up.
        pub use sensor_::*;
    }
}

/// The generated GATT table.
///
/// The allow is here rather than in the generated file because an `include!`
/// cannot carry an inner attribute: a file that allowed its own lints could not be included at all, which would tie the output to one way of mounting it.
#[allow(clippy::all, nonstandard_style, unused, irrefutable_let_patterns)]
mod gatt {
    include!(concat!(env!("OUT_DIR"), "/gatt.rs"));
}

use pb::msg;

fn main() {
    // Building the table is what compiles the macros the generator emits: the service, the property sets, and the value types.
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

    // The half of the generated code a declaration cannot exercise: a message through the value type the characteristic stores.
    let config = msg::Config { period_ms: 1000 };
    let value = gatt::SensorSettingsValue::encode(&config).expect("it fits");
    assert_eq!(value.decode().expect("it comes back").period_ms, 1000);
    println!(
        "a Config through the value type and back: {} bytes on the wire",
        value.as_bytes().len()
    );
}

/// What the schema promises a client, checked against the table it produced.
///
/// One test with its checks as functions, because the storage behind each characteristic is a function-local `static`: a table can be built once per process.
#[cfg(test)]
mod tests {
    use super::*;
    use trouble_host::prelude::*;

    #[test]
    fn the_table_is_what_a_client_reads_and_writes() {
        let server = gatt::Server::new_default("example-sensor").expect("the name fits");

        a_uuid_is_the_schema_text_in_wire_order();
        every_characteristic_has_its_own_handle(&server);
        only_a_notifiable_characteristic_has_a_cccd(&server);
        a_value_survives_the_characteristic(&server);
    }

    /// The text is what a client prints; the bytes under it are the same UUID in wire order, which is the one thing a generated table can get wrong on its own.
    fn a_uuid_is_the_schema_text_in_wire_order() {
        assert_eq!(
            canonical(&gatt::uuid::SENSOR_SERVICE_UUID),
            gatt::uuid::SENSOR_SERVICE_TEXT
        );
        assert_eq!(
            canonical(&gatt::uuid::SENSOR_LEVEL_UUID),
            gatt::uuid::SENSOR_LEVEL_TEXT
        );
        assert_eq!(
            canonical(&gatt::uuid::SENSOR_SETTINGS_UUID),
            gatt::uuid::SENSOR_SETTINGS_TEXT
        );
    }

    fn every_characteristic_has_its_own_handle(server: &gatt::Server) {
        assert_ne!(server.sensor.level.handle, server.sensor.settings.handle);
    }

    /// A CCCD is what a client writes to subscribe, so exactly the notifiable characteristic has one.
    fn only_a_notifiable_characteristic_has_a_cccd(server: &gatt::Server) {
        assert!(server.sensor.level.cccd_handle.is_some());
        assert!(server.sensor.settings.cccd_handle.is_none());
    }

    fn a_value_survives_the_characteristic(server: &gatt::Server) {
        let config = msg::Config { period_ms: 1000 };
        let value =
            gatt::SensorSettingsValue::encode(&config).expect("it fits the declared max_len");

        server
            .table()
            .set(&server.sensor.settings, &value)
            .expect("the table takes it");
        let stored: gatt::SensorSettingsValue = server
            .table()
            .get(&server.sensor.settings)
            .expect("and hands it back");
        assert_eq!(stored.decode().expect("it decodes"), config);
    }

    /// A UUID as a client prints it: the wire bytes reversed.
    fn canonical(uuid: &Uuid) -> String {
        let hex: Vec<String> = uuid
            .as_raw()
            .iter()
            .rev()
            .map(|b| format!("{b:02x}"))
            .collect();
        format!(
            "{}-{}-{}-{}-{}",
            hex[0..4].join(""),
            hex[4..6].join(""),
            hex[6..8].join(""),
            hex[8..10].join(""),
            hex[10..16].join("")
        )
    }
}
