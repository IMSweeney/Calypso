//! The generated GATT table, exercised the way a client would.
//!
//! One test, not several: `#[gatt_service]` backs each characteristic with a
//! function-local `static StaticCell`, so a service can be built once per
//! process. The checks below are separate functions called from that one test —
//! which is the price of letting the macro own the storage (see
//! `tools/gatt-gen/PLAN.md`, errata E1).
//!
//! Two of the three characteristics are halves of one pipe, so what a client
//! *writes* is a frame rather than a message: the table stores chunks, and the decode
//! that turns them into a `SyncRequest` belongs to `crate::pipe` (see `tests/pipe.rs`).
//! The third is a value the client reads, and the table stores it whole. What is left
//! to check here is the table itself — the UUIDs, the properties, and the size its
//! storage will and will not take.

use calypso_core::calibration::Calibration;
use calypso_core::gatt::{
    max_len, uuid, CalypsoGatt, SyncCalibrationValue, PIPE_CHUNK_MAX, PIPE_STORAGE_MAX, SCHEMA_HASH,
};
use calypso_core::pb::{self, msg};
use calypso_core::pipe::{CHUNK_MAX, FRAME_MAX, MESSAGE_MAX, XFER_TIMEOUT_MS};
use calypso_core::rep::TracePoint;
use calypso_core::uuid::{fixture_uuid, UUID_LEN};
use static_cell::StaticCell;
use trouble_host::prelude::*;

#[test]
fn the_generated_table_is_the_interface_the_proto_declares() {
    let gatt = table();

    checks_uuids_and_properties(gatt);
    a_frame_round_trips_through_the_table(gatt);
    the_calibration_value_holds_the_widest_status_the_device_writes(gatt);
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
    assert_eq!(
        uuid::SYNC_CALIBRATION_TEXT,
        "0000ca24-b0cd-11ec-871f-d45ddf138840"
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
    assert_eq!(
        canonical(&uuid::SYNC_CALIBRATION_UUID),
        uuid::SYNC_CALIBRATION_TEXT
    );
    assert_ne!(gatt.sync.messages.handle, gatt.sync.requests.handle);
    assert_ne!(gatt.sync.calibration.handle, gatt.sync.messages.handle);

    // NOTIFY is what gives a characteristic a CCCD: a subscriber needs one, and
    // neither the written half nor the readable one has one. `WRITE_NO_RSP` only,
    // since the pipe's own acknowledgement is what reports a request arriving.
    assert!(gatt.sync.messages.cccd_handle.is_some());
    assert!(gatt.sync.requests.cccd_handle.is_none());
    assert!(gatt.sync.calibration.cccd_handle.is_none());
}

/// The readable value holds the largest status the device can build.
///
/// The generated table asserts `micropb`'s *bound* for a pipe's messages, because
/// one that could not arrive is a transfer that stalls. A single-shot value has no
/// such assertion (`tools/gatt-gen/src/generator/server/rust_trouble.rs` explains
/// why), so the message the device actually writes is checked against `max_len`
/// here — the levels are `u8`s out of CALIB_STAT, so this is as wide as it gets.
fn the_calibration_value_holds_the_widest_status_the_device_writes(gatt: &CalypsoGatt<'_>) {
    let widest = pb::calibration_message(&Calibration::new(u8::MAX, u8::MAX, u8::MAX, u8::MAX));
    let value = SyncCalibrationValue::encode(&widest).expect("the widest status fits max_len");
    assert!(value.as_bytes().len() <= max_len::SYNC_CALIBRATION);

    gatt.set(&gatt.sync.calibration, &value)
        .expect("the table takes it");
    let stored: SyncCalibrationValue = gatt.get(&gatt.sync.calibration).expect("and hands it back");
    assert_eq!(stored.decode().expect("it decodes"), widest);
}

/// What the app writes is a chunk, and the table stores it as one.
///
/// **This is the test that found `PIPE_STORAGE_MAX`.** A buffer the frame can fill
/// exactly is a buffer whose write reads back empty (see `docs/GOTCHAS.md`), so the
/// storage is one byte wider than [`FRAME_MAX`] and this is where that is checked.
fn a_frame_round_trips_through_the_table(gatt: &CalypsoGatt<'_>) {
    let frame = [0x2Au8; FRAME_MAX];
    let value: heapless::Vec<u8, PIPE_STORAGE_MAX> =
        heapless::Vec::from_slice(&frame).expect("a full frame fits the characteristic");
    gatt.table().set(&gatt.sync.requests, &value).unwrap();

    let stored: heapless::Vec<u8, PIPE_STORAGE_MAX> =
        gatt.table().get(&gatt.sync.requests).unwrap();
    assert_eq!(
        stored.as_slice(),
        &frame[..],
        "a full frame survives the table"
    );

    assert_eq!(
        <heapless::Vec<u8, PIPE_STORAGE_MAX> as AsGatt>::MAX_SIZE,
        PIPE_STORAGE_MAX
    );
    assert!(
        FRAME_MAX < PIPE_STORAGE_MAX,
        "and a frame can never be the buffer's size"
    );
}

/// The numbers the schema declares, and the buffers that depend on them.
fn the_sizes_come_from_the_schema() {
    assert_eq!(PIPE_CHUNK_MAX, 244, "the schema's chunk buffer");
    assert_eq!(
        CHUNK_MAX,
        calypso_core::gatt::pipe_sync::CHUNK_MAX,
        "the pipe's ceiling and the characteristic's buffer are one number"
    );
    assert_eq!(
        FRAME_MAX, PIPE_CHUNK_MAX,
        "a frame fits the buffer it is built in"
    );
    assert_ne!(SCHEMA_HASH, 0, "the schema's hash is what the app checks");
    assert!(XFER_TIMEOUT_MS > 0);

    // A whole trace chunk — the largest message the device sends — has to encode
    // into the pipe's reassembly buffer. The generated table asserts the same
    // thing about `micropb`'s *bound* for the type at compile time; this is the
    // bytes the wire really carries.
    // Not `TracePoint::ZERO`: protobuf omits zero-valued fields, and a chunk of an
    // all-zero rep encodes to a third of what a real one does. This is a
    // rep-shaped point, which is what the ceiling has to hold.
    let point = TracePoint {
        t_ms: 12_345,
        pos: [0.25, -0.5, 0.125],
        accel: [12, -981, 3],
        quat: [16384, -2048, 1024, -512],
    };
    let chunk = pb::chunk_message(fixture_uuid(7), 0, 0, 1, &[point; pb::TRACE_CHUNK_SAMPLES]);
    let mut scratch = [0u8; MESSAGE_MAX];
    let n = pb::encode_sync(msg::SyncMessage_::Body::Trace(chunk), &mut scratch)
        .expect("a full trace chunk encodes");
    assert!(
        n <= MESSAGE_MAX,
        "a chunk of {n} bytes does not fit a {MESSAGE_MAX}-byte transfer"
    );
    assert!(
        n > PIPE_CHUNK_MAX,
        "and it is more than one notification holds ({n} bytes), which is why the pipe exists"
    );
}
