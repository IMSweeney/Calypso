#!/bin/sh
# Regenerates the typed BLE client (lib/gatt/gatt_client.dart) from
# proto/calypso_gatt.proto.
#
# The output is checked in on purpose: flutter test and a fresh checkout then
# need no Rust toolchain and no generator, and a change to the interface shows up
# in review as a diff. Re-run this when the .proto changes, and commit the result.
#
# `gatt-gen` brings its own protoc (protoc-bin-vendored), so nothing has to be
# installed beyond a Rust toolchain.
#
# Usage: ./tool/gen_gatt.sh   (from calypso-app/, or anywhere)
set -e
cd "$(dirname "$0")/../.."

# The pipe's numbers, which have to be the ones the device was generated with
# (calypso-core/tool/gen_gatt.sh): both ends frame chunks and count a transfer's
# window the same way.
cargo run --quiet --manifest-path tools/gatt-gen/Cargo.toml -- \
  dart-universal-ble \
  --out calypso-app/lib/gatt \
  --dart-messages ../protocol/calypso.pb.dart \
  --dart-prefix pb \
  --chunk-max 244 \
  --pipe-msg-max 4096 \
  --window 127 \
  --xfer-timeout 5000 \
  --regenerate-with calypso-app/tool/gen_gatt.sh \
  proto/calypso_gatt.proto

echo "wrote calypso-app/lib/gatt/gatt_client.dart (commit it)"
