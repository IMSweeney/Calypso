//! Platform-independent logic shared by the Calypso firmware.
//!
//! This crate deliberately has no dependencies on esp-hal or other embedded crates, so it can be unit-tested on the host with the standard Rust test harness (`cargo test` in this directory).
//! The firmware is `#![no_std]`, so everything here must be too when built for the device.

#![cfg_attr(not(test), no_std)]

extern crate alloc;

pub mod ble;
pub mod clock;
pub mod flashlog;
pub mod fusion;
/// The BLE GATT table the device serves, generated from `proto/calypso_gatt.proto` by `gatt-gen` (see `tool/gen_gatt.sh`) and committed.
///
/// The generated file is items and nothing else, so the allow lives here.
#[allow(clippy::all, nonstandard_style, unused, irrefutable_let_patterns)]
pub mod gatt;
pub mod pb;
pub mod pipe;
pub mod rep;
pub mod report;
pub mod sampler;
pub mod store;
pub mod sync;
pub mod uuid;
