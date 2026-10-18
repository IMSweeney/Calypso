/// Stable rep UUIDs for tests.
///
/// Tests say `u(1)` for "rep index 1" rather than repeating UUID literals or
/// depending on the device's randomness. The values are the same bytes
/// `calypso_core::uuid::fixture_uuid` mints for that index, so a test can name a
/// checked-in fixture's rep by index and mean the same rep the generator wrote.
library;

import 'dart:typed_data';

import 'package:calypso/model/uuid.dart';

const int _golden = 0x9e3779b97f4a7c15;

/// A deterministic canonical UUID for test rep [index].
///
/// A byte-for-byte port of the Rust fixture generator: SplitMix64 consumed four
/// 32-bit words at a time, most significant byte first, then RFC 4122 version
/// and variant bits. Dart's `int` is 64-bit two's complement, so `+` and `*` wrap
/// the way Rust's `wrapping_add`/`wrapping_mul` do and `>>>` is a logical shift
/// like `>>` on a `u64`.
String testUuid(int index) {
  final bytes = Uint8List(uuidLength);
  var state = index + _golden;
  for (var word = 0; word < uuidLength ~/ 4; word++) {
    state += _golden;
    var z = state;
    z = (z ^ (z >>> 30)) * 0xbf58476d1ce4e5b9;
    z = (z ^ (z >>> 27)) * 0x94d049bb133111eb;
    final value = (z ^ (z >>> 31)) & 0xFFFFFFFF;
    bytes[word * 4] = (value >> 24) & 0xFF;
    bytes[word * 4 + 1] = (value >> 16) & 0xFF;
    bytes[word * 4 + 2] = (value >> 8) & 0xFF;
    bytes[word * 4 + 3] = value & 0xFF;
  }
  bytes[6] = (bytes[6] & 0x0F) | 0x40; // version 4
  bytes[8] = (bytes[8] & 0x3F) | 0x80; // variant 10xx
  return uuidFromBytes(bytes)!;
}

/// The UUID for test rep [index], for call sites that take one.
String u(int index) => testUuid(index);
