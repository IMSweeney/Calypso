//! BLE GATT table construction.
//!
//! This lives in `calypso-core` (rather than the firmware crate) because the
//! trouble-host attribute table is platform-independent: building it on the
//! host lets integration tests read and write characteristics exactly as the
//! BLE stack would when a phone connects — no radio required. See
//! `tests/gatt_table.rs`.

use embassy_sync::blocking_mutex::raw::RawMutex;
use trouble_host::prelude::*;

use crate::DEVICE_NAME_MAX;

/// Number of attributes in the table:
/// GAP service (1) + device name (2) + appearance (2) + GATT service (1)
/// + gravity service (1) + gravity-x characteristic (2) + its CCCD (1)
/// + sync service (1) + summary characteristic (2) + CCCD (1)
/// + trace-request characteristic (2) + trace-chunk characteristic (2)
/// + its CCCD (1),
///   plus 2 spare.
pub const GATT_TABLE_MAX: usize = 23;

/// Byte payload type for the sync characteristics. `heapless::Vec` is what
/// `trouble-host` implements `AsGatt`/`FromGatt` for, so values can be any
/// length up to `N` (unlike a fixed-size array, which would pad).
pub type BleVec<const N: usize> = heapless::Vec<u8, N>;

/// Largest sync message sent device -> app. A `RepSummary` message encodes to
/// ~120 bytes.
pub const SYNC_MSG_MAX: usize = 128;
/// Largest accepted `TraceRequest` from the app.
pub const SYNC_REQUEST_MAX: usize = 32;

// Every UUID below is written in **wire order**, which is the reverse of the text
// it is quoted as — `40 88 13 df 5d d4 1f 87 ec 11 cd b0 20 ca 00 00` is
// `0000ca20-b0cd-11ec-871f-d45ddf138840` to any client, in the GATT tree and in
// advertising data alike.
//
// Both halves of that were measured from one firmware build rather than reasoned
// from the spec, because reasoning got it wrong twice:
//
// - this build logs the arrays it uses at boot, and a client reported the GATT
//   service as `408813df-…b020ca0000` for a canonical array and `0000ca20-…` for
//   the wire-order one, and
// - the app's service-filtered scan matched with the wire-order array in the
//   advertisement, and had not matched before it.
//
// So: **store wire order, see the canonical text**, in both places. The tests pin
// each side of that separately (`tests/gatt_table.rs`, `tests/advertising.rs`); the
// trap is that a *client* may have cached your previous bytes under the same
// Bluetooth address, which makes a corrected firmware look unchanged (the address
// is bumped in `main.rs` for exactly that reason).

/// Byte-reverse a 128-bit UUID: wire order to text order, or back.
pub const fn reverse16(bytes: [u8; 16]) -> [u8; 16] {
    let mut out = [0u8; 16];
    let mut i = 0;
    while i < 16 {
        out[i] = bytes[15 - i];
        i += 1;
    }
    out
}

/// [`SYNC_SERVICE_UUID`]'s bytes, in wire order. Advertising data uses the same
/// array: both ends of the link reverse it the same way.
pub const SYNC_SERVICE_UUID_BYTES: [u8; 16] = [
    0x40, 0x88, 0x13, 0xdf, 0x5d, 0xd4, 0x1f, 0x87, 0xec, 0x11, 0xcd, 0xb0, 0x20, 0xca, 0x00, 0x00,
];

/// Custom GATT service for rep sync (summaries + on-demand traces).
/// Text: `0000ca20-b0cd-11ec-871f-d45ddf138840`.
pub const SYNC_SERVICE_UUID: Uuid = Uuid::new_long(SYNC_SERVICE_UUID_BYTES);
/// Rep summaries, notify (auto-streamed to a subscriber).
/// Text: `0000ca21-b0cd-11ec-871f-d45ddf138840`.
pub const SYNC_SUMMARY_UUID: Uuid = Uuid::new_long([
    0x40, 0x88, 0x13, 0xdf, 0x5d, 0xd4, 0x1f, 0x87, 0xec, 0x11, 0xcd, 0xb0, 0x21, 0xca, 0x00, 0x00,
]);
/// Trace requests, write (app -> device).
/// Text: `0000ca22-b0cd-11ec-871f-d45ddf138840`.
pub const SYNC_REQUEST_UUID: Uuid = Uuid::new_long([
    0x40, 0x88, 0x13, 0xdf, 0x5d, 0xd4, 0x1f, 0x87, 0xec, 0x11, 0xcd, 0xb0, 0x22, 0xca, 0x00, 0x00,
]);
/// Trace chunks, notify (on demand).
/// Text: `0000ca23-b0cd-11ec-871f-d45ddf138840`.
pub const SYNC_CHUNK_UUID: Uuid = Uuid::new_long([
    0x40, 0x88, 0x13, 0xdf, 0x5d, 0xd4, 0x1f, 0x87, 0xec, 0x11, 0xcd, 0xb0, 0x23, 0xca, 0x00, 0x00,
]);

/// Custom GATT service exposing the IMU's gravity vector.
/// Text: `0000ca10-b0cd-11ec-871f-d45ddf138840`.
pub const GRAVITY_SERVICE_UUID: Uuid = Uuid::new_long([
    0x40, 0x88, 0x13, 0xdf, 0x5d, 0xd4, 0x1f, 0x87, 0xec, 0x11, 0xcd, 0xb0, 0x10, 0xca, 0x00, 0x00,
]);
/// X-axis gravity characteristic (m/s^2, f32, read + notify).
/// Text: `0000ca11-b0cd-11ec-871f-d45ddf138840`.
pub const GRAVITY_X_UUID: Uuid = Uuid::new_long([
    0x40, 0x88, 0x13, 0xdf, 0x5d, 0xd4, 0x1f, 0x87, 0xec, 0x11, 0xcd, 0xb0, 0x11, 0xca, 0x00, 0x00,
]);

/// Longest an advertising packet or scan response can be (legacy BLE).
pub const ADV_MAX: usize = 31;

/// The advertisement and scan response to put on air, and how much of each is
/// used. A scan response with `scan_len == 0` is not configured at all.
pub struct Advertising {
    pub adv_data: [u8; ADV_MAX],
    pub adv_len: usize,
    pub scan_data: [u8; ADV_MAX],
    pub scan_len: usize,
}

/// Device flags: general discoverable, and no classic Bluetooth.
const ADV_FLAGS: u8 = LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED;
/// The single service a client can find this device by. The same wire-order array
/// the GATT table uses: both are reversed by the client, in the same direction.
const ADV_SERVICES: [[u8; 16]; 1] = [SYNC_SERVICE_UUID_BYTES];

/// Build the advertising and scan-response payloads for a device called `name`.
///
/// Built here rather than in the firmware for the same reason the GATT table is:
/// this is the interface a client actually sees, so a mistake should be a failing
/// host test rather than a device that is invisible on air with nothing to say
/// for itself. Both mistakes this has already had were of that kind.
///
/// Two rules, both learned the hard way:
///
/// - **The service UUID goes in the advertisement**, not the scan response: a
///   client that scans *filtered by service* — which is how the app looks for
///   this device — only ever sees advertisements.
/// - **The name goes in after it when it fits**, because a scanner searching by
///   name also only looks at the advertisement. For the default `Calypso` all of
///   it fits in 30 of the 31 bytes. A longer name goes in the scan response
///   instead, where a client that asks for one will find it.
///
/// `name` is assumed to be at most [`DEVICE_NAME_MAX`].
pub fn build_advertising(name: &str) -> Advertising {
    let mut out = Advertising {
        adv_data: [0; ADV_MAX],
        adv_len: 0,
        scan_data: [0; ADV_MAX],
        scan_len: 0,
    };

    let everything = [
        AdStructure::Flags(ADV_FLAGS),
        AdStructure::ServiceUuids128(&ADV_SERVICES),
        AdStructure::CompleteLocalName(name.as_bytes()),
    ];
    if let Ok(n) = AdStructure::encode_slice(&everything, &mut out.adv_data[..]) {
        out.adv_len = n;
        return out;
    }

    // The name did not fit. The service still must (it is what the app matches),
    // and the name moves to the scan response.
    let service = [
        AdStructure::Flags(ADV_FLAGS),
        AdStructure::ServiceUuids128(&ADV_SERVICES),
    ];
    out.adv_len = AdStructure::encode_slice(&service, &mut out.adv_data[..]).unwrap_or(0);
    out.scan_len = AdStructure::encode_slice(
        &[AdStructure::CompleteLocalName(name.as_bytes())],
        &mut out.scan_data[..],
    )
    .unwrap_or(0);
    out
}

/// The built GATT attribute table plus handles to the characteristics the
/// application cares about.
pub struct GattTable<'d, M: RawMutex> {
    pub table: AttributeTable<'d, M, GATT_TABLE_MAX>,
    /// Device Name (0x2A00), read + write.
    pub device_name: Characteristic<HeaplessString<DEVICE_NAME_MAX>>,
    /// X-axis gravity (m/s^2), read + notify.
    pub gravity_x: Characteristic<f32>,
    /// Sync messages carrying `RepSummary`, notify.
    pub summary: Characteristic<BleVec<SYNC_MSG_MAX>>,
    /// `TraceRequest` from the app, write.
    pub trace_request: Characteristic<BleVec<SYNC_REQUEST_MAX>>,
    /// Sync messages carrying `TraceChunk`, notify.
    pub trace_chunk: Characteristic<BleVec<SYNC_MSG_MAX>>,
}

#[derive(Debug, PartialEq, Eq)]
pub enum BuildError {
    /// The device name is empty or longer than [`DEVICE_NAME_MAX`].
    InvalidName,
}

/// Build the GAP service (writable device name + appearance), the mandatory
/// GATT service declaration, and the custom gravity service.
///
/// `device_name_store` and `gravity_x_store` are the backing buffers the
/// server reads/writes; they must outlive the table.
///
/// Note: `GapConfig` is intentionally not used here — it only supports a
/// read-only device name.
pub fn build_gatt_table<'d, M: RawMutex>(
    device_name: &str,
    device_name_store: &'d mut [u8],
    gravity_x_store: &'d mut [u8],
    summary_store: &'d mut [u8],
    trace_request_store: &'d mut [u8],
    trace_chunk_store: &'d mut [u8],
) -> Result<GattTable<'d, M>, BuildError> {
    let mut initial: HeaplessString<DEVICE_NAME_MAX> = HeaplessString::new();
    if device_name.is_empty() || initial.push_str(device_name).is_err() {
        return Err(BuildError::InvalidName);
    }

    let mut table: AttributeTable<'d, M, GATT_TABLE_MAX> = AttributeTable::new();

    let mut gap = table.add_service(Service::new(service::GAP));
    let device_name = gap
        .add_characteristic(
            characteristic::DEVICE_NAME,
            &[CharacteristicProp::Read, CharacteristicProp::Write],
            initial,
            device_name_store,
        )
        .build();
    gap.add_characteristic_ro(characteristic::APPEARANCE, &appearance::sensor::GENERIC_SENSOR);
    gap.build();

    table.add_service(Service::new(service::GATT)).build();

    let gravity_x = table
        .add_service(Service::new(GRAVITY_SERVICE_UUID.clone()))
        .add_characteristic(
            GRAVITY_X_UUID.clone(),
            &[CharacteristicProp::Read, CharacteristicProp::Notify],
            0.0f32,
            gravity_x_store,
        )
        .build();

    // Log sync: summaries notify out, trace requests write in, trace chunks
    // notify out on demand. See `proto/calypso.proto`.
    let mut sync = table.add_service(Service::new(SYNC_SERVICE_UUID.clone()));
    let summary = sync
        .add_characteristic(
            SYNC_SUMMARY_UUID.clone(),
            &[CharacteristicProp::Read, CharacteristicProp::Notify],
            BleVec::<SYNC_MSG_MAX>::new(),
            summary_store,
        )
        .build();
    let trace_request = sync
        .add_characteristic(
            SYNC_REQUEST_UUID.clone(),
            &[CharacteristicProp::Write, CharacteristicProp::WriteWithoutResponse],
            BleVec::<SYNC_REQUEST_MAX>::new(),
            trace_request_store,
        )
        .build();
    let trace_chunk = sync
        .add_characteristic(
            SYNC_CHUNK_UUID.clone(),
            &[CharacteristicProp::Notify],
            BleVec::<SYNC_MSG_MAX>::new(),
            trace_chunk_store,
        )
        .build();
    sync.build();

    Ok(GattTable {
        table,
        device_name,
        gravity_x,
        summary,
        trace_request,
        trace_chunk,
    })
}
