//! Host-run integration tests for the device's GATT table.
//!
//! These build the exact attribute table the firmware serves over BLE and
//! exercise it the way a client (e.g. a phone) would: reading the device
//! name, writing a new one, and reading/subscribing to the gravity
//! characteristic. No radio or device required — the trouble-host attribute
//! table is platform-independent.

use calypso_core::ble::{
    build_gatt_table, BleVec, BuildError, GattTable, NOTIFIABLE_CHARACTERISTICS, GATT_TABLE_MAX,
    SYNC_CHUNK_UUID, SYNC_REQUEST_UUID, SYNC_SERVICE_UUID, SYNC_SUMMARY_UUID,
    GRAVITY_SERVICE_UUID, GRAVITY_X_UUID, SYNC_MSG_MAX, SYNC_REQUEST_MAX,
};
use calypso_core::sync::SYNC_CHUNK_MAX;

/// The text a client sees for a GATT UUID: the stored bytes, reversed.
///
/// **Reversed**, because that is what CoreBluetooth reported for a known input.
/// The firmware logs the arrays it uses at boot, and with `00 00 ca 20 …` the
/// client reported `408813df-…b020ca0000`; with the wire-order array it reports
/// `0000ca20-…`. Advertising behaves the same way — see `tests/advertising.rs`.
fn canonical(uuid: &Uuid) -> String {
    let b = uuid.as_raw();
    assert_eq!(b.len(), 16, "only the 128-bit UUIDs are checked here");
    let hex: Vec<String> = b.iter().rev().map(|x| format!("{x:02x}")).collect();
    format!(
        "{}-{}-{}-{}-{}",
        hex[0..4].join(""),
        hex[4..6].join(""),
        hex[6..8].join(""),
        hex[8..10].join(""),
        hex[10..16].join("")
    )
}

#[test]
fn uuids_are_what_a_client_matches_against() {
    assert_eq!(canonical(&SYNC_SERVICE_UUID), "0000ca20-b0cd-11ec-871f-d45ddf138840");
    assert_eq!(canonical(&SYNC_SUMMARY_UUID), "0000ca21-b0cd-11ec-871f-d45ddf138840");
    assert_eq!(canonical(&SYNC_REQUEST_UUID), "0000ca22-b0cd-11ec-871f-d45ddf138840");
    assert_eq!(canonical(&SYNC_CHUNK_UUID), "0000ca23-b0cd-11ec-871f-d45ddf138840");
    assert_eq!(canonical(&GRAVITY_SERVICE_UUID), "0000ca10-b0cd-11ec-871f-d45ddf138840");
    assert_eq!(canonical(&GRAVITY_X_UUID), "0000ca11-b0cd-11ec-871f-d45ddf138840");
}
use calypso_core::{decode_device_name, encode_device_name, DEVICE_NAME_MAX};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use trouble_host::prelude::*;

/// Builds the GATT table like the firmware does. The backing stores must be
/// declared by the caller so they outlive the table that borrows them.
macro_rules! gatt_table {
    ($name:expr) => {{
        let name_store = Box::leak(Box::new([0u8; DEVICE_NAME_MAX]));
        let gravity_store = Box::leak(Box::new([0u8; 4]));
        let summary_store = Box::leak(Box::new([0u8; SYNC_MSG_MAX]));
        let request_store = Box::leak(Box::new([0u8; SYNC_REQUEST_MAX]));
        let chunk_store = Box::leak(Box::new([0u8; SYNC_MSG_MAX]));
        build_gatt_table::<NoopRawMutex>(
            $name,
            name_store,
            gravity_store,
            summary_store,
            request_store,
            chunk_store,
        )
        .unwrap()
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
fn sync_characteristics_are_wired_up() {
    let gatt = gatt_table!("Calypso");

    // The app subscribes to summaries and trace chunks...
    assert!(gatt.summary.cccd_handle.is_some());
    assert!(gatt.trace_chunk.cccd_handle.is_some());
    // ...and writes on-demand trace requests.
    assert!(gatt.trace_request.cccd_handle.is_none());

    // Distinct handles, findable by the value handles clients address.
    let handles = [
        gatt.gravity_x.handle,
        gatt.summary.handle,
        gatt.trace_request.handle,
        gatt.trace_chunk.handle,
    ];
    for (i, a) in handles.iter().enumerate() {
        for b in &handles[i + 1..] {
            assert_ne!(a, b);
        }
    }
    assert!(gatt
        .table
        .find_characteristic_by_value_handle::<BleVec<SYNC_MSG_MAX>>(gatt.summary.handle)
        .map(|c| c.handle == gatt.summary.handle)
        .unwrap_or(false));
    // And the chunk characteristic is sized for a *chunk*, not a summary. The type is the
    // only place that size lives, so it is the thing to pin: a chunk characteristic that
    // is too small drops notifications in silence and the trace stream stops part-way
    // (see `docs/GOTCHAS.md`).
    assert!(gatt
        .table
        .find_characteristic_by_value_handle::<BleVec<SYNC_CHUNK_MAX>>(gatt.trace_chunk.handle)
        .map(|c| c.handle == gatt.trace_chunk.handle)
        .unwrap_or(false));
}

#[test]
fn sync_payloads_round_trip_through_the_table() {
    let gatt = gatt_table!("Calypso");

    // A TraceRequest written by the app arrives as raw bytes for the
    // firmware to decode.
    let mut scratch = [0u8; SYNC_REQUEST_MAX];
    let n = calypso_core::pb::encode_trace_request(
        &calypso_core::pb::msg::TraceRequest {
            rep_uuid: calypso_core::pb::uuid_to_field(&calypso_core::uuid::fixture_uuid(42)),
            all_chunks: true,
            chunk_index: 0,
        },
        &mut scratch,
    )
    .unwrap();
    let mut request: BleVec<SYNC_REQUEST_MAX> = BleVec::new();
    request.extend_from_slice(&scratch[..n]).unwrap();
    gatt.table.set(&gatt.trace_request, &request).unwrap();
    let readback: BleVec<SYNC_REQUEST_MAX> = gatt.table.get(&gatt.trace_request).unwrap();
    let decoded = calypso_core::sync::decode_trace_request(&readback).unwrap();
    assert_eq!(
        calypso_core::pb::uuid_from_field(&decoded.rep_uuid),
        Some(calypso_core::uuid::fixture_uuid(42))
    );

    // A summary notification payload round-trips byte for byte.
    let mut summary_msg: BleVec<SYNC_MSG_MAX> = BleVec::new();
    summary_msg.extend_from_slice(&[
        0x0A, 0x04, 0x08, 0x07, 0x10, 0x01, // SyncMessage { summary { .. } }
    ])
    .unwrap();
    gatt.table.set(&gatt.summary, &summary_msg).unwrap();
    let readback: BleVec<SYNC_MSG_MAX> = gatt.table.get(&gatt.summary).unwrap();
    assert_eq!(readback.as_slice(), summary_msg.as_slice());
}

#[test]
fn rejects_invalid_names() {
    let mut name_store = [0u8; DEVICE_NAME_MAX];
    let mut gravity_store = [0u8; 4];
    let mut summary_store = [0u8; SYNC_MSG_MAX];
    let mut request_store = [0u8; SYNC_REQUEST_MAX];
    let mut chunk_store = [0u8; SYNC_MSG_MAX];

    {
        let too_long = "a".repeat(DEVICE_NAME_MAX + 1);
        let result = build_gatt_table::<NoopRawMutex>(
            &too_long,
            &mut name_store,
            &mut gravity_store,
            &mut summary_store,
            &mut request_store,
            &mut chunk_store,
        );
        assert!(matches!(result, Err(BuildError::InvalidName)));
    }
    {
        let result = build_gatt_table::<NoopRawMutex>(
            "",
            &mut name_store,
            &mut gravity_store,
            &mut summary_store,
            &mut request_store,
            &mut chunk_store,
        );
        assert!(matches!(result, Err(BuildError::InvalidName)));
    }
}

#[test]
fn table_size_const_matches() {
    // Guard against the firmware and tests disagreeing on the table size — and on
    // the number of CCCD slots, which is the same kind of silently-wrong number (see
    // `NOTIFIABLE_CHARACTERISTICS`).
    let gatt = gatt_table!("Calypso");
    let _server: AttributeServer<
        '_,
        NoopRawMutex,
        DefaultPacketPool,
        GATT_TABLE_MAX,
        NOTIFIABLE_CHARACTERISTICS,
        1,
    > = AttributeServer::new(gatt.table);
}

#[test]
fn the_server_has_a_cccd_slot_for_every_subscribable_characteristic() {
    // The firmware passes `NOTIFIABLE_CHARACTERISTICS` as `AttributeServer`'s
    // `CCCD_MAX`. A client's subscription to a CCCD the server has no slot for is
    // dropped *in silence*: `should_notify` is false for that handle, so `notify`
    // returns `Ok` without sending and the device reports a delivery that never
    // happened. Getting this number wrong is how the sync service spent a while
    // being unsubscribable while gravity worked.
    let gatt = gatt_table!("Calypso");

    assert_eq!(gatt.cccd_count, NOTIFIABLE_CHARACTERISTICS);
    // ...and the ones it counts are the ones clients actually subscribe to.
    assert!(gatt.gravity_x.cccd_handle.is_some());
    assert!(gatt.summary.cccd_handle.is_some());
    assert!(gatt.trace_chunk.cccd_handle.is_some());
}
