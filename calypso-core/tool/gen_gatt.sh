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

# The pipe's numbers, which both ends have to agree on.
#
#   --window 8           chunks in flight before the sender waits, and the
#                        receiver acknowledges every window / 2. **This is the
#                        flow control.** 127 was not one: a whole trace chunk
#                        left as a single blast of ~150 notifications at an
#                        18-byte frame, and the phone's own writes -- including
#                        the acknowledgements the device was waiting for --
#                        queued behind it for seconds (docs/GOTCHAS.md, "A
#                        window of 127 is not flow control").
#   --chunk-max 244      the pipe characteristic's backing buffer: one
#                        notification's worth at a 247-byte MTU.
#   --pipe-msg-max 4096  a whole `TraceChunk` (48 samples, 2459 bytes worst
#                        case), which is what makes a trace one transfer per
#                        stored chunk instead of three points a notification.
#   --xfer-timeout 5000  no progress for this long abandons a transfer.
cargo run --quiet --manifest-path tools/gatt-gen/Cargo.toml -- \
  rust-trouble \
  --out calypso-core/src \
  --messages crate::pb::msg \
  --server CalypsoGatt \
  --chunk-max 244 \
  --pipe-msg-max 4096 \
  --window 8 \
  --xfer-timeout 5000 \
  --regenerate-with calypso-core/tool/gen_gatt.sh \
  proto/calypso_gatt.proto

echo "wrote calypso-core/src/gatt.rs (commit it)"
