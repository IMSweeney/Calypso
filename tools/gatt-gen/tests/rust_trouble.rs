//! What the `rust-trouble` generator emits, checked against the example schema.

mod common;

use common::load;
use gatt_gen::generator::server::rust_trouble::{self, Config};
use gatt_gen::generator::{Config as TargetConfig, Target};

fn generated() -> String {
    let (gatt, _) = load("valid/device.proto").unwrap();
    rust_trouble::generate(&gatt, &Config::default())
        .unwrap()
        .source
}

#[test]
fn a_characteristic_gets_exactly_the_properties_the_schema_declared() {
    let source = generated();

    // READ + NOTIFY, and no write.
    assert!(
        source.contains(
            "#[characteristic(uuid = \"00002a19-0000-1000-8000-00805f9b34fb\", read, notify)]"
        ),
        "{source}"
    );
    assert!(!source.contains("read, write"), "{source}");
    // A pipe's halves are written and notified, and carry chunks rather than a
    // decoded message.
    assert!(source.contains("write_without_response)]"), "{source}");
    assert!(
        source.contains("pub rx: Vec<u8, PIPE_CHUNK_MAX>"),
        "{source}"
    );
}

#[test]
fn a_protobuf_characteristic_gets_a_value_type_the_table_can_store() {
    let source = generated();

    assert!(source.contains("pub struct DeviceInfoValue"), "{source}");
    assert!(
        source.contains("impl AsGatt for DeviceInfoValue"),
        "{source}"
    );
    assert!(
        source.contains("impl FromGatt for DeviceInfoValue"),
        "{source}"
    );
    assert!(source.contains("pub fn encode_into"), "{source}");
    assert!(source.contains("pub fn decode(&self)"), "{source}");
    // The size the schema declares, for the buffers that have to hold one.
    assert!(
        source.contains("pub const DEVICE_INFO: usize = 32;"),
        "{source}"
    );

    // A pipe's message is checked against the buffer that reassembles it, and
    // the check is a compile-time assertion rather than generator arithmetic.
    let (gatt, _) = load("valid/device.proto").unwrap();
    let with_pipes = rust_trouble::generate(
        &gatt,
        &Config {
            pipe_msg_max: 4096,
            ..Config::default()
        },
    )
    .unwrap()
    .source;
    assert!(
        with_pipes.contains("micropb::size::max_encoded_size::<crate::pb::msg::Command>() <= 4096"),
        "{with_pipes}"
    );
}

#[test]
fn the_server_carries_every_service_and_the_schema_hash() {
    let source = generated();

    assert!(source.contains("#[gatt_server]"), "{source}");
    assert!(
        source.contains("#[gatt_service(uuid = \"0000180f-"),
        "{source}"
    );
    assert!(source.contains("pub battery: BatteryService"), "{source}");
    assert!(source.contains("pub device: DeviceService"), "{source}");
    assert!(
        source.contains("pub const SCHEMA_HASH: u64 = 0x"),
        "{source}"
    );
}

#[test]
fn the_hash_follows_the_schema() {
    let (gatt, _) = load("valid/device.proto").unwrap();
    let (other, _) = load("warn/max_len_over_20.proto").unwrap();

    assert_eq!(
        gatt.schema_hash(),
        load("valid/device.proto").unwrap().0.schema_hash()
    );
    assert_ne!(gatt.schema_hash(), other.schema_hash());
}

#[test]
fn the_file_is_named_by_the_target_and_says_how_to_regenerate_itself() {
    let (gatt, _) = load("valid/device.proto").unwrap();
    let config = Config {
        regenerate_with: Some("gatt-gen rust-trouble --out src schema.proto".to_string()),
        ..Config::default()
    };

    let output = Target::RustTrouble
        .generate(&gatt, &TargetConfig::RustTrouble(config))
        .unwrap();

    assert_eq!(output.file_name, "gatt.rs");
    assert!(
        output
            .source
            .starts_with("//! The BLE GATT table the device serves."),
        "{}",
        output.source
    );
    assert!(
        output.source.contains(
            "//! Regenerate with `gatt-gen rust-trouble --out src schema.proto` after \
             changing the schema."
        ),
        "{}",
        output.source
    );
}
