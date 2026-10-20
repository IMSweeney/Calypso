use calypso_core::ble::{
    build_gatt_table, BleVec, BuildError, GattTable, NOTIFIABLE_CHARACTERISTICS, GATT_TABLE_MAX,
    SYNC_MESSAGES_UUID, SYNC_REQUESTS_UUID, SYNC_REQUEST_MAX, SYNC_SERVICE_UUID,
};
use calypso_core::sync::SYNC_CHUNK_MAX;
use calypso_core::DEVICE_NAME_MAX;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use trouble_host::prelude::*;

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

    assert!(gatt.messages.cccd_handle.is_some());
    assert!(gatt.requests.cccd_handle.is_none());

    assert_ne!(gatt.messages.handle, gatt.requests.handle);

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

    assert!(n <= SYNC_REQUEST_MAX, "a request encodes to {n} bytes");
}

#[test]
fn a_message_round_trips_through_the_table() {
    let gatt = gatt_table!("Calypso");

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
}

#[test]
fn rejects_invalid_names() {
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
    let gatt = gatt_table!("Calypso");

    assert_eq!(gatt.cccd_count, NOTIFIABLE_CHARACTERISTICS);
    assert!(gatt.messages.cccd_handle.is_some());
}
