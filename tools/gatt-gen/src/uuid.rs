//! Canonical 128-bit UUID text, and the bytes a client actually sees.

/// Parse `0000xxxx-0000-1000-8000-00805f9b34fb` form into its 16 bytes.
///
/// Only the canonical 128-bit form is accepted: a 16-bit alias (`180f`) means
/// the same thing to a client but not the same thing to a human reading two
/// proto files in a diff, and the schema is the interface.
pub fn parse_canonical(text: &str) -> Option<[u8; 16]> {
    let stripped: String = text.chars().filter(|c| *c != '-').collect();
    if stripped.len() != 32 || !text.chars().all(|c| c.is_ascii_hexdigit() || c == '-') {
        return None;
    }
    let mut bytes = [0u8; 16];
    for (i, byte) in bytes.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&stripped[i * 2..i * 2 + 2], 16).ok()?;
    }
    Some(bytes)
}

/// The 16 bytes in **wire order**: the reverse of the canonical text's byte
/// order.
///
/// That is the order `trouble_host`'s `Uuid::new_long` stores, and the order
/// advertising data carries a 128-bit UUID in, so a client reverses it back into
/// the text the schema quotes. See `docs/GOTCHAS.md` for why this is written down
/// rather than reasoned about.
pub fn to_wire_order(canonical: &[u8; 16]) -> [u8; 16] {
    let mut out = [0u8; 16];
    for (i, byte) in canonical.iter().enumerate() {
        out[15 - i] = *byte;
    }
    out
}

/// The canonical text of 16 bytes.
pub fn to_canonical(bytes: &[u8; 16]) -> String {
    let hex: Vec<String> = bytes.iter().map(|b| format!("{b:02x}")).collect();
    let joined = hex.join("");
    format!(
        "{}-{}-{}-{}-{}",
        &joined[0..8],
        &joined[8..12],
        &joined[12..16],
        &joined[16..20],
        &joined[20..32]
    )
}
