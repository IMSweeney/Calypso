//! The generated GATT table, exercised the way a client would.
//!
//! One test, not several: `#[gatt_service]` backs each characteristic with a
//! function-local `static StaticCell`, so a service can be built once per
//! process. The checks below are separate functions called from that one test —
//! which is the price of letting the macro own the storage (see
//! `tools/gatt-gen/PLAN.md`, errata E1).

use calypso_core::calibration::Calibration;
use calypso_core::gatt::uuid;
use calypso_core::gatt::{
    max_len, CalypsoGatt, SyncMessagesValue, SyncRequestsValue, PIPE_CHUNK_MAX, SCHEMA_HASH,
};
use calypso_core::pb::{self, msg};
use calypso_core::rep::TracePoint;
use calypso_core::sync::{self, SYNC_CHUNK_MAX};
use calypso_core::uuid::{fixture_uuid, UUID_LEN};
use static_cell::StaticCell;
use trouble_host::prelude::*;

#[test]
fn the_generated_table_is_the_interface_the_proto_declares() {
    let gatt = table();

    checks_uuids_and_properties(gatt);
    a_request_round_trips_through_the_table(gatt);
    a_message_round_trips_through_the_table(gatt);
    the_sizes_come_from_the_schema();
}

/// The table the firmware builds: GAP (name and appearance) plus the sync
/// service.
///
/// Built once, because the storage behind these characteristics is static.
fn table() -> &'static CalypsoGatt<'static> {
    static GATT: StaticCell<CalypsoGatt<'static>> = StaticCell::new();
    GATT.init(
        CalypsoGatt::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name: "Calypso",
            appearance: &appearance::sensor::GENERIC_SENSOR,
        }))
        .expect("the device name fits"),
    )
}

fn canonical(uuid: &Uuid) -> String {
    let bytes = uuid.as_raw();
    assert_eq!(bytes.len(), UUID_LEN, "only 128-bit UUIDs are checked here");
    let hex: Vec<String> = bytes.iter().rev().map(|x| format!("{x:02x}")).collect();
    format!(
        "{}-{}-{}-{}-{}",
        hex[0..4].join(""),
        hex[4..6].join(""),
        hex[6..8].join(""),
        hex[8..10].join(""),
        hex[10..16].join("")
    )
}

/// The UUIDs the app scans for and writes to, and the properties it relies on.
///
/// The schema is where those are declared now; this pins the generated table
/// against the constants the rest of the firmware (and the advertisement) uses.
fn checks_uuids_and_properties(gatt: &CalypsoGatt<'_>) {
    // The schema declares the UUIDs and this is the contract: the app matches
    // these three strings, and only these. A change here is a change to the
    // interface, so it is a literal in the test on purpose.
    assert_eq!(
        uuid::SYNC_SERVICE_TEXT,
        "0000ca20-b0cd-11ec-871f-d45ddf138840"
    );
    assert_eq!(
        uuid::SYNC_MESSAGES_TEXT,
        "0000ca21-b0cd-11ec-871f-d45ddf138840"
    );
    assert_eq!(
        uuid::SYNC_REQUESTS_TEXT,
        "0000ca22-b0cd-11ec-871f-d45ddf138840"
    );

    // The generated constants agree with the table they describe.
    assert_eq!(canonical(&uuid::SYNC_SERVICE_UUID), uuid::SYNC_SERVICE_TEXT);
    assert_eq!(
        canonical(&uuid::SYNC_MESSAGES_UUID),
        uuid::SYNC_MESSAGES_TEXT
    );
    assert_eq!(
        canonical(&uuid::SYNC_REQUESTS_UUID),
        uuid::SYNC_REQUESTS_TEXT
    );
    assert_ne!(gatt.sync.messages.handle, gatt.sync.requests.handle);

    // NOTIFY is what gives a characteristic a CCCD: a subscriber needs one, and
    // the written half must not have one.
    assert!(gatt.sync.messages.cccd_handle.is_some());
    assert!(gatt.sync.requests.cccd_handle.is_none());
}

/// What the app writes: a `SyncRequest`, decoded out of the stored value.
fn a_request_round_trips_through_the_table(gatt: &CalypsoGatt<'_>) {
    let request = msg::SyncRequest {
        body: Some(msg::SyncRequest_::Body::Trace(msg::TraceRequest {
            rep_uuid: pb::uuid_to_field(&fixture_uuid(42)),
            all_chunks: true,
            chunk_index: 0,
        })),
    };

    let value = SyncRequestsValue::encode(&request).expect("a request fits its characteristic");
    gatt.table().set(&gatt.sync.requests, &value).unwrap();

    let stored: SyncRequestsValue = gatt.table().get(&gatt.sync.requests).unwrap();
    let decoded = stored.decode().expect("what was stored decodes");
    let trace = match decoded.body {
        Some(msg::SyncRequest_::Body::Trace(trace)) => trace,
        None => panic!("the body survived"),
    };
    assert_eq!(pb::uuid_from_field(&trace.rep_uuid), Some(fixture_uuid(42)));
}

/// What the device sends: a `SyncMessage`, through the same storage.
fn a_message_round_trips_through_the_table(gatt: &CalypsoGatt<'_>) {
    let message = msg::SyncMessage {
        body: Some(msg::SyncMessage_::Body::Calibration(
            pb::calibration_message(&Calibration {
                sys: 0,
                accel: 3,
                gyro: 3,
                mag: 0,
            }),
        )),
    };

    let value = SyncMessagesValue::encode(&message).expect("a message fits its characteristic");
    gatt.table().set(&gatt.sync.messages, &value).unwrap();

    let stored: SyncMessagesValue = gatt.table().get(&gatt.sync.messages).unwrap();
    assert_eq!(stored.decode().expect("what was stored decodes"), message);
}

/// The numbers the schema declares, and the buffers that depend on them.
fn the_sizes_come_from_the_schema() {
    assert_eq!(
        SYNC_CHUNK_MAX,
        max_len::SYNC_MESSAGES,
        "the sync chunk size is the max_len the schema declares"
    );
    assert_eq!(
        max_len::SYNC_REQUESTS,
        <SyncRequestsValue as AsGatt>::MAX_SIZE
    );
    assert_eq!(
        max_len::SYNC_MESSAGES,
        <SyncMessagesValue as AsGatt>::MAX_SIZE
    );
    assert_eq!(PIPE_CHUNK_MAX, 244);
    assert_ne!(SCHEMA_HASH, 0);

    // The largest request, and the largest message the device can build, have
    // to fit the characteristics those numbers describe: a payload that does
    // not fit fails to notify *silently*, which is the failure this whole
    // exercise exists to make impossible.
    let mut scratch = [0u8; 32];
    let n = pb::encode_trace_request(
        &msg::TraceRequest {
            rep_uuid: pb::uuid_to_field(&fixture_uuid(7)),
            all_chunks: false,
            chunk_index: 1,
        },
        &mut scratch,
    )
    .expect("a trace request encodes");
    assert!(
        n <= max_len::SYNC_REQUESTS,
        "a request encodes to {n} bytes"
    );

    let chunk = pb::chunk_message(
        fixture_uuid(7),
        0,
        0,
        1,
        &[TracePoint::ZERO; sync::BLE_CHUNK_SAMPLES],
    );
    let mut scratch = [0u8; SYNC_CHUNK_MAX];
    let n = pb::encode_sync(msg::SyncMessage_::Body::Trace(chunk), &mut scratch)
        .expect("a full trace chunk encodes");
    assert!(
        n <= max_len::SYNC_MESSAGES,
        "a chunk of {n} bytes does not fit a characteristic declared as {}",
        max_len::SYNC_MESSAGES
    );
}
