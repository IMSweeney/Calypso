//! Host-run integration tests for the device's GATT table.
//!
//! These build the exact attribute table the firmware serves over BLE and poke it
//! the way a client would: subscribing to the sync service, writing a request, and
//! reading what comes back. No radio or device required — the trouble-host
//! attribute table is platform-independent.
//!
//! The sync service is two characteristics, one each way: everything the device
//! sends is a `SyncMessage` on `messages`, everything the app asks for is a
//! `SyncRequest` on `requests`. What is pinned here is that shape, the ceiling on
//! each side, and the CCCD count the server has to be told (see
//! [`NOTIFIABLE_CHARACTERISTICS`]) — every one of which fails *silently* when it is
//! wrong.

use calypso_core::ble::{
    build_gatt_table, BleVec, BuildError, GattTable, NOTIFIABLE_CHARACTERISTICS, GATT_TABLE_MAX,
    SYNC_MESSAGES_UUID, SYNC_REQUESTS_UUID, SYNC_REQUEST_MAX, SYNC_SERVICE_UUID,
};
use calypso_core::sync::SYNC_CHUNK_MAX;
use calypso_core::DEVICE_NAME_MAX;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use trouble_host::prelude::*;

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
    assert_eq!(canonical(&SYNC_MESSAGES_UUID), "0000ca21-b0cd-11ec-871f-d45ddf138840");
    assert_eq!(canonical(&SYNC_REQUESTS_UUID), "0000ca22-b0cd-11ec-871f-d45ddf138840");
}

/// Builds the GATT table like the firmware does. The backing stores must be
/// declared by the caller so they outlive the table that borrows them.
macro_rules! gatt_table {
    ($name:expr) => {{
        let name_store = Box::leak(Box::new([0u8; DEVICE_NAME_MAX]));
        let messages_store = Box::leak(Box::new([0u8; SYNC_CHUNK_MAX]));
        let requests_store = Box::leak(Box::new([0u8; SYNC_REQUEST_MAX]));
        build_gatt_table::<NoopRawMutex>($name, name_store, messages_store, requests_store).unwrap()
    }};
}

#[test]
fn the_sync_service_is_one_characteristic_each_way() {
    let gatt = gatt_table!("Calypso");

    // The app subscribes to everything the device sends, which needs a CCCD ...
    assert!(gatt.messages.cccd_handle.is_some());
    // ... and writes requests, which nobody subscribes to.
    assert!(gatt.requests.cccd_handle.is_none());

    assert_ne!(gatt.messages.handle, gatt.requests.handle);

    // The type is the only place a characteristic's size lives, so it is the thing
    // to pin: the notify characteristic has to hold the largest `SyncMessage` (a
    // trace chunk), and a payload that does not fit fails to notify *silently*
    // (see `docs/GOTCHAS.md`).
    assert!(gatt
        .table
        .find_characteristic_by_value_handle::<BleVec<SYNC_CHUNK_MAX>>(gatt.messages.handle)
        .map(|c| c.handle == gatt.messages.handle)
        .unwrap_or(false));
    assert!(gatt
        .table
        .find_characteristic_by_value_handle::<BleVec<SYNC_REQUEST_MAX>>(gatt.requests.handle)
        .map(|c| c.handle == gatt.requests.handle)
        .unwrap_or(false));
}

#[test]
fn a_request_round_trips_through_the_table() {
    let gatt = gatt_table!("Calypso");

    // A `SyncRequest` written by the app arrives as raw bytes for the firmware to
    // unwrap: the envelope, then the trace request inside it.
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
    gatt.table.set(&gatt.requests, &request).unwrap();

    let readback: BleVec<SYNC_REQUEST_MAX> = gatt.table.get(&gatt.requests).unwrap();
    let decoded = calypso_core::sync::decode_trace_request(&readback).unwrap();
    assert_eq!(
        calypso_core::pb::uuid_from_field(&decoded.rep_uuid),
        Some(calypso_core::uuid::fixture_uuid(42))
    );

    // The envelope costs two bytes over the bare request, which the characteristic
    // has room for — the other half of sizing these things is that the *write* has to
    // fit too, or the client's request is rejected before the device ever sees it.
    assert!(n <= SYNC_REQUEST_MAX, "a request encodes to {n} bytes");
}

#[test]
fn a_message_round_trips_through_the_table() {
    let gatt = gatt_table!("Calypso");

    // A summary notification payload round-trips byte for byte.
    let mut summary_msg: BleVec<SYNC_CHUNK_MAX> = BleVec::new();
    summary_msg
        .extend_from_slice(&[
            0x0A, 0x04, 0x08, 0x07, 0x10, 0x01, // SyncMessage { summary { .. } }
        ])
        .unwrap();
    gatt.table.set(&gatt.messages, &summary_msg).unwrap();
    let readback: BleVec<SYNC_CHUNK_MAX> = gatt.table.get(&gatt.messages).unwrap();
    assert_eq!(readback.as_slice(), summary_msg.as_slice());
}

#[test]
fn the_device_name_is_readable_and_fixed() {
    let gatt: GattTable<'_, NoopRawMutex> = gatt_table!("Calypso");
    let name: HeaplessString<DEVICE_NAME_MAX> = gatt.table.get(&gatt.device_name).unwrap();
    assert_eq!(name.as_str(), "Calypso");

    // The characteristic is read-only, which a client reads from its declaration:
    // the property is set in `build_gatt_table`, and the firmware now handles no
    // write for it at all. What a host table can check is the value — trouble-host
    // does not expose a characteristic's properties for reading (`table.set` does
    // not check them either), so the property itself is pinned by the code, not
    // here.
}

#[test]
fn rejects_invalid_names() {
    // A name that is empty, or too long to advertise, is refused at build time
    // rather than becoming a device that is hard to find on air.
    let mut name_store = [0u8; DEVICE_NAME_MAX];
    let mut messages_store = [0u8; SYNC_CHUNK_MAX];
    let mut requests_store = [0u8; SYNC_REQUEST_MAX];

    {
        let too_long = "a".repeat(DEVICE_NAME_MAX + 1);
        let result = build_gatt_table::<NoopRawMutex>(
            &too_long,
            &mut name_store,
            &mut messages_store,
            &mut requests_store,
        );
        assert!(matches!(result, Err(BuildError::InvalidName)));
    }
    {
        let result = build_gatt_table::<NoopRawMutex>(
            "",
            &mut name_store,
            &mut messages_store,
            &mut requests_store,
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
    // being unsubscribable.
    let gatt = gatt_table!("Calypso");

    assert_eq!(gatt.cccd_count, NOTIFIABLE_CHARACTERISTICS);
    // ...and the one it counts is the one clients subscribe to.
    assert!(gatt.messages.cccd_handle.is_some());
}
