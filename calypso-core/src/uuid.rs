//! Rep identity: one v4 UUID per rep, minted on the device when the rep closes.
//!
//! Random rather than a counter or a timestamp, because the device has no RTC and its uptime resets on every boot.
//! Ids therefore do not sort: order comes from the archive's append order.

pub const UUID_LEN: usize = 16;

pub type RepUuid = [u8; UUID_LEN];

/// Mints a v4 UUID from a source of 32-bit randomness.
pub fn new_uuid(rand: &mut impl FnMut() -> u32) -> RepUuid {
    let mut bytes = [0u8; UUID_LEN];
    for chunk in bytes.chunks_mut(4) {
        chunk.copy_from_slice(&rand().to_be_bytes());
    }
    bytes[6] = (bytes[6] & 0x0f) | 0x40; // version 4
    bytes[8] = (bytes[8] & 0x3f) | 0x80; // variant 10xx
    bytes
}

/// A deterministic UUID for fixture rep `index`.
pub fn fixture_uuid(index: u32) -> RepUuid {
    let mut state = (index as u64) + 0x9e37_79b9_7f4a_7c15;
    let mut rand = move || {
        // SplitMix64, enough to spread indices across all 16 bytes.
        state = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        (z ^ (z >> 31)) as u32
    };
    new_uuid(&mut rand)
}

/// The first four bytes as ASCII hex, for logs and UI: `"3f2a9c81"`.
pub fn short(uuid: &RepUuid) -> [u8; 8] {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = [0u8; 8];
    for (i, byte) in uuid[..4].iter().enumerate() {
        out[i * 2] = HEX[(byte >> 4) as usize];
        out[i * 2 + 1] = HEX[(byte & 0x0f) as usize];
    }
    out
}
