//! BLE GATT table construction.
//!
//! This lives in `calypso-core` (rather than the firmware crate) because the
//! trouble-host attribute table is platform-independent: building it on the
//! host lets integration tests read and write characteristics exactly as the
//! BLE stack would when a phone connects — no radio required. See
//! `tests/gatt_table.rs`.

use embassy_sync::blocking_mutex::raw::RawMutex;
use trouble_host::prelude::*;

use crate::sync::SYNC_CHUNK_MAX;
use crate::DEVICE_NAME_MAX;

/// Attributes GAP (device name + appearance) and the mandatory GATT service take,
/// as trouble-host counts them.
const GAP_ATTRIBUTES: usize = trouble_host::gap::GAP_SERVICE_ATTRIBUTE_COUNT;

/// Attributes the sync service adds: its declaration (1), the messages
/// characteristic (2) and its CCCD (1), the requests characteristic (2).
///
/// The CCCD is counted here and checked against the server's own count in
/// [`NOTIFIABLE_CHARACTERISTICS`]; both are numbers that fail *silently* when they
/// are wrong (see that constant).
const SYNC_SERVICE_ATTRIBUTES: usize = 6;

/// Room to grow without a boot failure.
const GATT_TABLE_HEADROOM: usize = 4;

/// Number of attributes in the table.
///
/// Running past this **panics** — the table pushes into a fixed-capacity
/// `heapless::Vec` and unwraps — so it is a boot failure rather than a silent
/// truncation, but it is still a number to keep ahead of: a characteristic added
/// below needs this raised too, and the host tests build the whole table, so they
/// are where it would show up.
pub const GATT_TABLE_MAX: usize = GAP_ATTRIBUTES + SYNC_SERVICE_ATTRIBUTES + GATT_TABLE_HEADROOM;

// The pieces have to add up to what the table actually pushes; the host tests
// build it, and this keeps the arithmetic honest where a reader can see it.
const _: () =
    assert!(GATT_TABLE_MAX > GAP_ATTRIBUTES + SYNC_SERVICE_ATTRIBUTES);

/// How many characteristics in this table a client can subscribe to: every one
/// built with `Notify`, i.e. every one with a CCCD. The sync service's two.
///
/// **This is `AttributeServer`'s `CCCD_MAX`, and getting it wrong is silent.** The
/// server keeps a per-client CCCD table of exactly this many entries, and its
/// `add_handle` drops every CCCD that does not fit without a word. A handle that is
/// not in the table means `should_notify` is false for it — and
/// `Characteristic::notify` then returns `Ok(())` *without sending anything*, so
/// the device reports success while the client sees nothing at all.
///
/// It was 1 for a while, and that one slot went to the first CCCD in the table —
/// another service's, back when there were three notifiable characteristics.
/// Subscribing to that one worked, which is what made it look like notifications
/// were fine, while subscribing to the sync service's two was accepted by the ATT
/// layer, forgotten by the server, and answered with a cheerful `calibration status
/// delivered to the app` once a second. Two clients — the app and an off-the-shelf
/// one — reported the same silence (`docs/GOTCHAS.md`).
///
/// [`build_gatt_table`] counts the CCCDs it actually created (see
/// [`GattTable::cccd_count`]) and a host test asserts the two agree, so this cannot
/// drift from the table again.
pub const NOTIFIABLE_CHARACTERISTICS: usize = 1;

/// Byte payload type for the sync characteristics. `heapless::Vec` is what
/// `trouble-host` implements `AsGatt`/`FromGatt` for, so values can be any
/// length up to `N` (unlike a fixed-size array, which would pad).
pub type BleVec<const N: usize> = heapless::Vec<u8, N>;

/// Largest `SyncRequest` accepted from the app: a `TraceRequest` and the envelope
/// around it.
///
/// There is one request characteristic and one message characteristic, so this is
/// the only other ceiling to keep ahead of. A payload that does not fit the
/// characteristic it is notified on **fails to notify silently** — the payload
/// cannot be built, so `Characteristic::notify` is never reached and the stream
/// simply stops (see `docs/GOTCHAS.md`, and [`SYNC_CHUNK_MAX`] for the other
/// direction).
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

/// Custom GATT service for rep sync. Everything the device has to say travels one
/// way over it and everything the app has to ask travels the other.
///
/// Text: `0000ca20-b0cd-11ec-871f-d45ddf138840`.
pub const SYNC_SERVICE_UUID: Uuid = Uuid::new_long(SYNC_SERVICE_UUID_BYTES);
/// Everything the device sends: a `SyncMessage`, notify. Summaries and
/// calibration stream to a subscriber; trace chunks answer a request. Sized for
/// the largest of them ([`SYNC_CHUNK_MAX`]).
///
/// Text: `0000ca21-b0cd-11ec-871f-d45ddf138840`.
pub const SYNC_MESSAGES_UUID: Uuid = Uuid::new_long([
    0x40, 0x88, 0x13, 0xdf, 0x5d, 0xd4, 0x1f, 0x87, 0xec, 0x11, 0xcd, 0xb0, 0x21, 0xca, 0x00, 0x00,
]);
/// Everything the app sends: a `SyncRequest`, write (app -> device).
///
/// Text: `0000ca22-b0cd-11ec-871f-d45ddf138840`.
pub const SYNC_REQUESTS_UUID: Uuid = Uuid::new_long([
    0x40, 0x88, 0x13, 0xdf, 0x5d, 0xd4, 0x1f, 0x87, 0xec, 0x11, 0xcd, 0xb0, 0x22, 0xca, 0x00, 0x00,
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
/// `name` is assumed to be at most [`crate::DEVICE_NAME_MAX`].
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
    /// How many characteristics here can be subscribed to — what the server's
    /// `CCCD_MAX` has to be. See [`NOTIFIABLE_CHARACTERISTICS`].
    pub cccd_count: usize,
    /// Device Name (0x2A00), read only.
    pub device_name: Characteristic<HeaplessString<DEVICE_NAME_MAX>>,
    /// Everything the device sends (`sync.messages`), notify: one `SyncMessage`
    /// per notification, whatever its body.
    ///
    /// One characteristic for all of them, sized for the largest — a trace chunk of
    /// [`crate::sync::BLE_CHUNK_SAMPLES`] points ([`SYNC_CHUNK_MAX`]). A payload that
    /// does not fit fails to notify *silently*: the payload cannot be built, so
    /// `Characteristic::notify` is never reached and the stream just stops
    /// (`docs/GOTCHAS.md`).
    pub messages: Characteristic<BleVec<SYNC_CHUNK_MAX>>,
    /// Everything the app sends (`sync.requests`), write: one `SyncRequest` per
    /// write, whatever its body.
    pub requests: Characteristic<BleVec<SYNC_REQUEST_MAX>>,
}

#[derive(Debug, PartialEq, Eq)]
pub enum BuildError {
    /// The device name is empty or longer than [`crate::DEVICE_NAME_MAX`].
    InvalidName,
}

/// Build the table the device serves: GAP with a read-only device name, the
/// mandatory GATT service declaration, and the sync service.
///
/// `device_name_store` and the two sync stores are the backing buffers the server
/// reads and writes; they must outlive the table.
///
/// The name is **read-only** now, which is the case trouble-host's own [`GapConfig`]
/// exists for — but that cannot be used here: it keeps the name in a
/// function-local `static`, and a host process that builds a second table (every
/// test in `tests/gatt_table.rs` builds one) panics with `StaticCell is already
/// full`. So GAP stays hand-built, with one property instead of two.
pub fn build_gatt_table<'d, M: RawMutex>(
    device_name: &str,
    device_name_store: &'d mut [u8],
    messages_store: &'d mut [u8],
    requests_store: &'d mut [u8],
) -> Result<GattTable<'d, M>, BuildError> {
    let mut initial: HeaplessString<DEVICE_NAME_MAX> = HeaplessString::new();
    if device_name.is_empty() || initial.push_str(device_name).is_err() {
        return Err(BuildError::InvalidName);
    }

    let mut table: AttributeTable<'d, M, GATT_TABLE_MAX> = AttributeTable::new();

    // GAP: the device name — read-only, which is what a client gets — and the
    // appearance. The appearance is the sensor one rather than `GapConfig`'s
    // `UNKNOWN`, so a scanner shows what this is.
    let mut gap = table.add_service(Service::new(service::GAP));
    let device_name = gap
        .add_characteristic(
            characteristic::DEVICE_NAME,
            &[CharacteristicProp::Read],
            initial,
            device_name_store,
        )
        .build();
    gap.add_characteristic_ro(characteristic::APPEARANCE, &appearance::sensor::GENERIC_SENSOR);
    gap.build();

    table.add_service(Service::new(service::GATT)).build();

    // Log sync: everything the device says goes out on `messages`, everything the
    // app asks for comes in on `requests`. See `proto/calypso.proto` for the two
    // messages (and for why one characteristic each is the point).
    let mut sync = table.add_service(Service::new(SYNC_SERVICE_UUID.clone()));
    let messages = sync
        .add_characteristic(
            SYNC_MESSAGES_UUID.clone(),
            &[CharacteristicProp::Notify],
            BleVec::<SYNC_CHUNK_MAX>::new(),
            messages_store,
        )
        .build();
    let requests = sync
        .add_characteristic(
            SYNC_REQUESTS_UUID.clone(),
            &[CharacteristicProp::Write, CharacteristicProp::WriteWithoutResponse],
            BleVec::<SYNC_REQUEST_MAX>::new(),
            requests_store,
        )
        .build();
    sync.build();

    // Counted from the characteristics that were actually built rather than from the
    // number of `Notify` props written above, so the two cannot drift: this is the
    // number of CCCD slots a client needs, and a slot too few is silent (see
    // [`NOTIFIABLE_CHARACTERISTICS`]).
    let cccd_count = [messages.cccd_handle, requests.cccd_handle]
        .iter()
        .filter(|handle| handle.is_some())
        .count();

    Ok(GattTable {
        table,
        cccd_count,
        device_name,
        messages,
        requests,
    })
}
