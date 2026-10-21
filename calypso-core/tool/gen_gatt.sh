#!/bin/sh
# Regenerates the BLE GATT table (src/gatt.rs) from proto/calypso_gatt.proto.
#
# The output is checked in on purpose: the firmware build then needs no codegen
# toolchain, no protoc and nothing from the network, and a change to the table
# shows up in review as a diff. Re-run this when the .proto changes, and commit
# the result. `cargo test` fails if the committed file and the schema disagree.
#
# `gatt-gen` brings its own protoc (protoc-bin-vendored), so there is nothing to
# install beyond a Rust toolchain.
#
# Usage: ./tool/gen_gatt.sh   (from calypso-core/, or anywhere)
set -e
cd "$(dirname "$0")/../.."

cargo run --quiet --manifest-path tools/gatt-gen/Cargo.toml -- \
  rust-trouble \
  --out calypso-core/src \
  --messages crate::pb::msg \
  --server CalypsoGatt \
  --regenerate-with calypso-core/tool/gen_gatt.sh \
  proto/calypso_gatt.proto

echo "wrote calypso-core/src/gatt.rs (commit it)"
