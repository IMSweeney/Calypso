//! The advertisement a client finds the device by.
//!
//! The table and the UUIDs are generated from `proto/calypso.proto` -- see
//! [`crate::gatt`]. What is left here is the interface a client *sees* before a
//! connection exists, which is why the advertising payloads are built here and
//! host-tested ([`build_advertising`], `tests/advertising.rs`): a mistake in
//! them is a device that is invisible on air with nothing to say for itself, and
//! a *client* may have cached the old bytes under the same address, which makes a
//! corrected firmware look unchanged (the address is the key such a cache lives
//! under: `main.rs` puts the schema's hash in it).

use trouble_host::prelude::*;

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
const ADV_SERVICES: [[u8; 16]; 1] = [crate::gatt::uuid::SYNC_SERVICE_BYTES];

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
