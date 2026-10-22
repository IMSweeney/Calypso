use calypso_core::ble::{build_advertising, ADV_MAX};
use calypso_core::gatt::uuid::SYNC_SERVICE_TEXT;

fn parse_structures(data: &[u8], len: usize) -> Vec<(u8, Vec<u8>)> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < len {
        let structure_len = data[i] as usize;
        assert!(structure_len > 0, "zero-length AD structure at {i}");
        assert!(
            i + 1 + structure_len <= len,
            "AD structure runs past the end"
        );
        out.push((data[i + 1], data[i + 2..i + 1 + structure_len].to_vec()));
        i += 1 + structure_len;
    }
    out
}

fn canonical(wire: &[u8]) -> String {
    assert_eq!(wire.len(), 16);
    let hex: Vec<String> = wire.iter().rev().map(|b| format!("{b:02x}")).collect();
    format!(
        "{}-{}-{}-{}-{}",
        hex[0..4].join(""),
        hex[4..6].join(""),
        hex[6..8].join(""),
        hex[8..10].join(""),
        hex[10..16].join("")
    )
}

#[test]
fn advertises_the_service_and_the_name_together() {
    let ad = build_advertising("Calypso");

    assert!(ad.adv_len <= ADV_MAX);

    let fields = parse_structures(&ad.adv_data, ad.adv_len);
    assert_eq!(fields.len(), 3, "flags, service, name");

    let (flags_type, flags) = &fields[0];
    assert_eq!(*flags_type, 0x01, "flags");
    assert_eq!(flags.len(), 1);
    assert_ne!(flags[0] & 0x02, 0, "general discoverable");
    assert_ne!(flags[0] & 0x04, 0, "no classic Bluetooth");

    let (service_type, service) = &fields[1];
    assert_eq!(
        *service_type, 0x07,
        "complete list of 128-bit service UUIDs"
    );
    // The text, not the array: the array comes from the schema, and what a client prints is what the app matches against.
    assert_eq!(canonical(service), SYNC_SERVICE_TEXT);

    let (name_type, name) = &fields[2];
    assert_eq!(*name_type, 0x09, "complete local name");
    assert_eq!(name, b"Calypso");
}
