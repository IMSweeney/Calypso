//! Host-run integration tests for the device's GATT table.
//!
//! These build the exact attribute table the firmware serves over BLE and
//! exercise it the way a client (e.g. a phone) would: reading the device
//! name, writing a new one, and reading/subscribing to the gravity
//! characteristic. No radio or device required — the trouble-host attribute
//! table is platform-independent.

use calypso_core::ble::{build_gatt_table, BuildError, GattTable, GATT_TABLE_MAX};
use calypso_core::{decode_device_name, encode_device_name, DEVICE_NAME_MAX};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use trouble_host::prelude::*;

/// Builds the GATT table like the firmware does. The backing stores must be
/// declared by the caller so they outlive the table that borrows them.
macro_rules! gatt_table {
    ($name:expr) => {{
        let name_store = Box::leak(Box::new([0u8; DEVICE_NAME_MAX]));
        let gravity_store = Box::leak(Box::new([0u8; 4]));
        build_gatt_table::<NoopRawMutex>($name, name_store, gravity_store).unwrap()
    }};
}

#[test]
fn device_name_defaults_to_initial_value() {
    let gatt: GattTable<'_, NoopRawMutex> = gatt_table!("Calypso");
    let name: HeaplessString<DEVICE_NAME_MAX> = gatt.table.get(&gatt.device_name).unwrap();
    assert_eq!(name.as_str(), "Calypso");
}

#[test]
fn client_can_rename_device_and_value_persists() {
    let gatt = gatt_table!("Calypso");

    // Simulate a client write to the Device Name characteristic.
    let mut new_name: HeaplessString<DEVICE_NAME_MAX> = HeaplessString::new();
    new_name.push_str("Rowing Erg").unwrap();
    gatt.table.set(&gatt.device_name, &new_name).unwrap();

    // The firmware reads the value back after accepting a write.
    let readback: HeaplessString<DEVICE_NAME_MAX> = gatt.table.get(&gatt.device_name).unwrap();
    assert_eq!(readback.as_str(), "Rowing Erg");

    // ...and persists it via the flash codec, which must round-trip.
    let blob = encode_device_name(readback.as_str()).unwrap();
    assert_eq!(decode_device_name(&blob), Some("Rowing Erg"));
}

#[test]
fn gravity_x_is_f32_and_notifiable() {
    let gatt = gatt_table!("Calypso");

    // A CCCD is present, i.e. clients can subscribe to notifications.
    assert!(gatt.gravity_x.cccd_handle.is_some());

    // Little-endian f32 round trip, as a client would read it.
    gatt.table.set(&gatt.gravity_x, &9.81f32).unwrap();
    let value: f32 = gatt.table.get(&gatt.gravity_x).unwrap();
    assert_eq!(value, 9.81);
}

#[test]
fn characteristics_have_distinct_handles() {
    let gatt = gatt_table!("Calypso");
    assert_ne!(gatt.device_name.handle, gatt.gravity_x.handle);

    // The stack resolves value handles to characteristics; make sure both are
    // findable by the handles clients will address.
    let found = gatt
        .table
        .find_characteristic_by_value_handle::<HeaplessString<DEVICE_NAME_MAX>>(gatt.device_name.handle)
        .unwrap();
    assert_eq!(found.handle, gatt.device_name.handle);
    let found = gatt
        .table
        .find_characteristic_by_value_handle::<f32>(gatt.gravity_x.handle)
        .unwrap();
    assert_eq!(found.handle, gatt.gravity_x.handle);
}

#[test]
fn rejects_invalid_names() {
    let mut name_store = [0u8; DEVICE_NAME_MAX];
    let mut gravity_store = [0u8; 4];

    {
        let too_long = "a".repeat(DEVICE_NAME_MAX + 1);
        let result = build_gatt_table::<NoopRawMutex>(&too_long, &mut name_store, &mut gravity_store);
        assert!(matches!(result, Err(BuildError::InvalidName)));
    }
    {
        let result = build_gatt_table::<NoopRawMutex>("", &mut name_store, &mut gravity_store);
        assert!(matches!(result, Err(BuildError::InvalidName)));
    }
}

#[test]
fn table_size_const_matches() {
    // Guard against the firmware and tests disagreeing on the table size.
    let gatt = gatt_table!("Calypso");
    let _server: AttributeServer<'_, NoopRawMutex, DefaultPacketPool, GATT_TABLE_MAX, 1, 1> =
        AttributeServer::new(gatt.table);
}
