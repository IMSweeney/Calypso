//! Rep identity.
//!
//! One **v4 UUID per rep**, minted on the device when the rep closes. Random
//! rather than a counter or a timestamp, because the device has no RTC and its
//! uptime resets on every boot — so neither a counter nor a clock can hand out
//! identities that are unique across boots, and a counter needs somewhere durable
//! to live. Randomness needs neither coordination nor persistence, and it also
//! makes archives from more than one device mergeable later.
//!
//! The price is that ids do not sort. Ordering comes from the archive's append
//! order (it is append-only), and the device's own log is written in order.

/// Length of a rep UUID in bytes.
pub const UUID_LEN: usize = 16;

/// A rep's identity: 16 bytes, wire-compatible with `UUID`/`GUID` tooling.
pub type RepUuid = [u8; UUID_LEN];

/// Mints a v4 UUID from a source of 32-bit randomness.
///
/// The device feeds this its hardware RNG; tests and the fixture generator feed
/// it something deterministic. RFC 4122 version and variant bits are set, so the
/// result is a real UUID rather than 16 arbitrary bytes.
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
///
/// Fixtures need stable identities, or every regenerated fixture would differ
/// and golden files and assertions would be meaningless. The values are not meant
/// to be unpredictable — only reproducible.
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
///
/// A full UUID is unreadable at a glance and too long for a log line or a list
/// row, but four bytes are plenty to recognise a rep while debugging.
pub fn short(uuid: &RepUuid) -> [u8; 8] {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = [0u8; 8];
    for (i, byte) in uuid[..4].iter().enumerate() {
        out[i * 2] = HEX[(byte >> 4) as usize];
        out[i * 2 + 1] = HEX[(byte & 0x0f) as usize];
    }
    out
}
