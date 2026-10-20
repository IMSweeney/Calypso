//! Platform-independent logic shared by the Calypso firmware.
//!
//! This crate deliberately has no dependencies on esp-hal or other embedded
//! crates, so it can be unit-tested on the host with the standard Rust test
//! harness (`cargo test` in this directory). The firmware is `#![no_std]`, so
//! everything here must be too when built for the device.

#![cfg_attr(not(test), no_std)]

extern crate alloc;

pub mod ble;
pub mod calibration;
pub mod fixtures;
pub mod flashlog;
pub mod pb;
pub mod rep;
pub mod sampler;
pub mod store;
pub mod sync;
pub mod uuid;

/// Longest device name supported; a complete local name must fit in the
/// 31-byte advertising packet alongside the flags field.
///
/// The name is a compile-time constant in the firmware, and read-only over GATT:
/// it was writable once, which is why this is a `MAX` and not a `LEN` in the
/// advertisement's terms — a longer name moves to the scan response.
pub const DEVICE_NAME_MAX: usize = 22;

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
}
