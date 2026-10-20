use calypso_core::ble::{build_advertising, ADV_MAX, SYNC_SERVICE_UUID_BYTES};
use calypso_core::DEVICE_NAME_MAX;

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
    assert_eq!(ad.scan_len, 0, "nothing needs the scan response");

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
    assert_eq!(service, &SYNC_SERVICE_UUID_BYTES, "the array itself");

    let (name_type, name) = &fields[2];
    assert_eq!(*name_type, 0x09, "complete local name");
    assert_eq!(name, b"Calypso");
}

#[test]
fn the_advertised_service_is_the_one_a_client_filters_on() {
    let ad = build_advertising("Calypso");
    let fields = parse_structures(&ad.adv_data, ad.adv_len);

    assert_eq!(
        canonical(&fields[1].1),
        "0000ca20-b0cd-11ec-871f-d45ddf138840"
    );
}

#[test]
fn a_name_too_long_for_the_advertisement_moves_to_the_scan_response() {
    let long = "a".repeat(DEVICE_NAME_MAX);
    let ad = build_advertising(&long);

    let fields = parse_structures(&ad.adv_data, ad.adv_len);
    assert_eq!(fields.len(), 2, "flags and service, no name");
    assert_eq!(
        canonical(&fields[1].1),
        "0000ca20-b0cd-11ec-871f-d45ddf138840"
    );

    assert!(ad.scan_len <= ADV_MAX);
    let scan = parse_structures(&ad.scan_data, ad.scan_len);
    assert_eq!(scan.len(), 1);
    assert_eq!(scan[0].0, 0x09, "complete local name");
    assert_eq!(scan[0].1, long.as_bytes());
}

#[test]
fn every_payload_fits_a_legacy_packet() {
    for len in 1..=DEVICE_NAME_MAX {
        let name = "n".repeat(len);
        let ad = build_advertising(&name);
        assert!(
            ad.adv_len > 0 && ad.adv_len <= ADV_MAX,
            "adv for {len}-byte name"
        );
        assert!(ad.scan_len <= ADV_MAX, "scan response for {len}-byte name");

        let carries = |data: &[u8], data_len: usize| {
            parse_structures(data, data_len)
                .iter()
                .any(|(kind, value)| *kind == 0x09 && value == name.as_bytes())
        };
        assert!(
            carries(&ad.adv_data, ad.adv_len) || carries(&ad.scan_data, ad.scan_len),
            "{len}-byte name advertised nowhere"
        );
    }
}
