library;

import 'dart:typed_data';

import 'package:calypso/model/uuid.dart';

const int _golden = 0x9e3779b97f4a7c15;

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

String u(int index) => testUuid(index);
