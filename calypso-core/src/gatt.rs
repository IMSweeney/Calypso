//! The BLE GATT table the device serves, generated from
//! `proto/calypso_gatt.proto` by `gatt-gen`.
//!
//! This module is the generated file and nothing else: the allow is here because
//! the generated file has no inner attributes of its own, so that it can be
//! mounted either as a module file (as here) or `include!`d into one (as a build
//! script does).
//!
//! What is *not* generated: the hand-written parts of serving the table (the
//! event loop, the subscription check, the sync protocol above it), which live in
//! [`crate::ble`] and the firmware.
#![allow(clippy::all, nonstandard_style, unused, irrefutable_let_patterns)]

include!("gatt_generated.rs");
