//! The advertisement a client finds the device by.

use trouble_host::prelude::*;

pub const ADV_MAX: usize = 31;

/// The advertisement and scan response to put on air, and how much of each is used.
/// A scan response with `scan_len == 0` is not configured at all.
pub struct Advertising {
    pub adv_data: [u8; ADV_MAX],
    pub adv_len: usize,
    pub scan_data: [u8; ADV_MAX],
    pub scan_len: usize,
}

const ADV_FLAGS: u8 = LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED;
const ADV_SERVICES: [[u8; 16]; 1] = [crate::gatt::uuid::SYNC_SERVICE_BYTES];

/// Build the advertising and scan-response payloads for a device called `name`.
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

    // The name did not fit; the service is what the app matches, so it stays.
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
