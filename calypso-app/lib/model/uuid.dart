library;

import 'dart:typed_data';

const int uuidLength = 16;

String? uuidFromBytes(List<int> bytes) {
  if (bytes.length != uuidLength) return null;
  final hex = StringBuffer();
  for (var i = 0; i < uuidLength; i++) {
    if (i == 4 || i == 6 || i == 8 || i == 10) hex.write('-');
    hex.write(bytes[i].toRadixString(16).padLeft(2, '0'));
  }
  return hex.toString();
}

Uint8List? uuidToBytes(String uuid) {
  final clean = uuid.replaceAll('-', '');
  if (clean.length != uuidLength * 2) return null;
  final bytes = Uint8List(uuidLength);
  for (var i = 0; i < uuidLength; i++) {
    final byte = int.tryParse(clean.substring(i * 2, i * 2 + 2), radix: 16);
    if (byte == null) return null;
    bytes[i] = byte;
  }
  return bytes;
}

String uuidShort(String? uuid) {
  if (uuid == null) return '????????';
  final clean = uuid.replaceAll('-', '');
  return clean.length < 8 ? clean : clean.substring(0, 8);
}
