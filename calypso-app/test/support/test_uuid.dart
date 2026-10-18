/// Stable rep UUIDs for tests.
///
/// Tests say `u(1)` for "the first rep" rather than repeating UUID literals or
/// depending on the device's randomness — the same idea as the Rust fixture
/// generator's deterministic ids.
library;

import 'dart:typed_data';

import 'package:calypso/model/uuid.dart';

/// A deterministic canonical UUID for test rep [index].
String testUuid(int index) {
  final bytes = Uint8List(uuidLength);
  var state = index + 0x9e3779b97f4a7c15;
  for (var i = 0; i < uuidLength; i++) {
    // SplitMix64-ish: spread the index across all 16 bytes.
    state = (state + 0x9e3779b97f4a7c15) & 0xFFFFFFFFFFFFFFFF;
    var z = state;
    z = ((z ^ (z >> 30)) * 0xbf58476d1ce4e5b9) & 0xFFFFFFFFFFFFFFFF;
    z = ((z ^ (z >> 27)) * 0x94d049bb133111eb) & 0xFFFFFFFFFFFFFFFF;
    bytes[i] = ((z ^ (z >> 31)) >> 32) & 0xFF;
  }
  bytes[6] = (bytes[6] & 0x0F) | 0x40; // version 4
  bytes[8] = (bytes[8] & 0x3F) | 0x80; // variant 10xx
  return uuidFromBytes(bytes)!;
}

/// The UUID for test rep [index], for call sites that take one.
String u(int index) => testUuid(index);
