//! The advertisement a client finds the device by.

use trouble_host::prelude::*;

pub const ADV_MAX: usize = 31;

/// The advertisement to put on air, and how much of it is used.
/// Nothing in it (`adv_len == 0`) is a name too long for one advertisement, which `main` warns about rather than advertising in silence.
pub struct Advertising {
    pub adv_data: [u8; ADV_MAX],
    pub adv_len: usize,
}

const ADV_FLAGS: u8 = LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED;
const ADV_SERVICES: [[u8; 16]; 1] = [crate::gatt::uuid::SYNC_SERVICE_BYTES];

/// Build the advertisement for a device called `name`.
pub fn build_advertising(name: &str) -> Advertising {
    let mut out = Advertising {
        adv_data: [0; ADV_MAX],
        adv_len: 0,
    };

    let structures = [
        AdStructure::Flags(ADV_FLAGS),
        AdStructure::ServiceUuids128(&ADV_SERVICES),
        AdStructure::CompleteLocalName(name.as_bytes()),
    ];
    if let Ok(len) = AdStructure::encode_slice(&structures, &mut out.adv_data[..]) {
        out.adv_len = len;
    }
    out
}
