#!/bin/sh
# Regenerates the Dart message classes from proto/calypso.proto.
#
# The output is checked in on purpose, so `flutter test` and a fresh checkout
# work without protoc or the Dart protobuf plugin. Re-run this only when the
# .proto changes, and commit the result.
#
# Prerequisites (one-off):
#   dart pub global activate protoc_plugin
#   protoc on PATH, or PROTOC set to a binary (the Rust build already pulls one
#   in via protoc-bin-vendored if you want to point at that copy)
#
# Note: this writes files, it does not prune them. If the output ever shrinks
# (a service removed from the proto, say), delete the leftover by hand -- a stale
# calypso.pbserver.dart is an analyzer error, not a silent one.
#
# Usage: ./tool/gen_proto.sh   (from calypso-app/)
set -e
cd "$(dirname "$0")/.."

PROTOC="${PROTOC:-protoc}"
OUT="lib/protocol"

mkdir -p "$OUT"
"$PROTOC" \
  --proto_path=../proto \
  --dart_out="$OUT" \
  ../proto/calypso.proto

echo "wrote $OUT/calypso.pb.dart (commit it)"
