//! Platform-independent logic shared by the Calypso firmware.
//!
//! This crate deliberately has no dependencies on esp-hal or other embedded
//! crates, so it can be unit-tested on the host with the standard Rust test
//! harness (`cargo test` in this directory). The firmware is `#![no_std]`, so
//! everything here must be too when built for the device.

#![cfg_attr(not(test), no_std)]

pub mod ble;
pub mod flashlog;
pub mod pb;
pub mod rep;
pub mod sampler;
pub mod store;
pub mod sync;

/// Longest device name supported; a complete local name must fit in the
/// 31-byte advertising packet alongside the flags field.
pub const DEVICE_NAME_MAX: usize = 22;

/// Size of the serialized device-name blob stored in flash:
/// 1 length byte + up to `DEVICE_NAME_MAX` name bytes, padded to a 4-byte
/// boundary.
pub const DEVICE_NAME_BLOB_SIZE: usize = 24;

/// Serialize a device name into its flash blob.
///
/// Layout: byte 0 is the name length, bytes 1..=len are the name, the rest is
/// padded with 0xFF (the erased state of NOR flash, so padding writes are
/// no-ops).
///
/// Returns `None` if the name is empty or longer than [`DEVICE_NAME_MAX`].
pub fn encode_device_name(name: &str) -> Option<[u8; DEVICE_NAME_BLOB_SIZE]> {
    if name.is_empty() || name.len() > DEVICE_NAME_MAX {
        return None;
    }
    let mut buf = [0xFFu8; DEVICE_NAME_BLOB_SIZE];
    buf[0] = name.len() as u8;
    buf[1..1 + name.len()].copy_from_slice(name.as_bytes());
    Some(buf)
}

/// Decode a device-name blob read back from flash.
///
/// Returns `None` if the blob is uninitialized/erased, has an out-of-range
/// length, or does not contain valid UTF-8.
pub fn decode_device_name(blob: &[u8; DEVICE_NAME_BLOB_SIZE]) -> Option<&str> {
    let len = blob[0] as usize;
    if len == 0 || len > DEVICE_NAME_MAX {
        return None;
    }
    core::str::from_utf8(&blob[1..1 + len]).ok()
}

/// Threshold (m/s²) under which a gravity component counts as "level".
pub const LEVEL_THRESHOLD: f32 = 0.1;

/// Whether the device is level along an axis, given that axis' gravity
/// component (m/s²). The device is level along an axis when gravity is
/// (almost) entirely orthogonal to it.
pub fn is_level_along_axis(gravity_component: f32) -> bool {
    gravity_component.abs() < LEVEL_THRESHOLD
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn level_when_gravity_is_orthogonal() {
        assert!(is_level_along_axis(0.0));
        assert!(is_level_along_axis(0.05));
        assert!(is_level_along_axis(-0.05));
    }

    #[test]
    fn not_level_when_gravity_along_axis() {
        assert!(!is_level_along_axis(9.8));
        assert!(!is_level_along_axis(0.5));
        assert!(!is_level_along_axis(-0.5));
    }

    #[test]
    fn threshold_is_exclusive() {
        assert!(!is_level_along_axis(LEVEL_THRESHOLD));
        assert!(!is_level_along_axis(-LEVEL_THRESHOLD));
    }

    #[test]
    fn round_trip() {
        let blob = encode_device_name("Calypso").unwrap();
        assert_eq!(decode_device_name(&blob), Some("Calypso"));
    }

    #[test]
    fn round_trip_max_length() {
        let name = "a".repeat(DEVICE_NAME_MAX);
        let blob = encode_device_name(&name).unwrap();
        assert_eq!(blob[0] as usize, DEVICE_NAME_MAX);
        assert_eq!(decode_device_name(&blob), Some(name.as_str()));
    }

    #[test]
    fn rejects_empty_name() {
        assert_eq!(encode_device_name(""), None);
    }

    #[test]
    fn rejects_too_long_name() {
        let name = "a".repeat(DEVICE_NAME_MAX + 1);
        assert_eq!(encode_device_name(&name), None);
    }

    #[test]
    fn erased_flash_decodes_to_none() {
        // Erased NOR flash reads back as 0xFF, which is an invalid length.
        let blob = [0xFFu8; DEVICE_NAME_BLOB_SIZE];
        assert_eq!(decode_device_name(&blob), None);
    }

    #[test]
    fn zero_length_decodes_to_none() {
        let mut blob = [0u8; DEVICE_NAME_BLOB_SIZE];
        blob[0] = 0;
        assert_eq!(decode_device_name(&blob), None);
    }

    #[test]
    fn invalid_utf8_decodes_to_none() {
        let mut blob = [0u8; DEVICE_NAME_BLOB_SIZE];
        blob[0] = 2;
        blob[1] = 0xFF;
        blob[2] = 0xFE;
        assert_eq!(decode_device_name(&blob), None);
    }

    #[test]
    fn trailing_bytes_are_ignored() {
        let mut blob = encode_device_name("Calypso").unwrap();
        // Simulate garbage left behind by a previous, longer name.
        blob[8] = b'X';
        assert_eq!(decode_device_name(&blob), Some("Calypso"));
    }

    #[test]
    fn padding_is_erased_state() {
        let blob = encode_device_name("hi").unwrap();
        assert!(blob[3..].iter().all(|&b| b == 0xFF));
    }
}
