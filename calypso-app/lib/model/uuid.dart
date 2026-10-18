/// Rep identity on the client.
///
/// The device mints a **v4 UUID per rep**; the client treats it as an opaque
/// canonical string (`xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx`) and keys its archive
/// on it. Randomness on the device is what makes this unique across reboots — it
/// has no RTC, and its uptime resets every boot, so a counter or a timestamp
/// could collide.
///
/// UUIDs do not sort, so ordering in the UI comes from the archive's append
/// order rather than from the id.
library;

import 'dart:typed_data';

/// Length of a rep UUID in bytes.
const int uuidLength = 16;

/// Canonical text for a 16-byte UUID, or null if the bytes aren't 16 long.
///
/// Canonical form matters: it is the map key the archive uses, so the same rep
/// must always produce the same string.
String? uuidFromBytes(List<int> bytes) {
  if (bytes.length != uuidLength) return null;
  final hex = StringBuffer();
  for (var i = 0; i < uuidLength; i++) {
    if (i == 4 || i == 6 || i == 8 || i == 10) hex.write('-');
    hex.write(bytes[i].toRadixString(16).padLeft(2, '0'));
  }
  return hex.toString();
}

/// The bytes of a canonical UUID string, for the wire. Null if malformed.
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

/// The first eight characters, for a list row or a log line: `3f2a9c81`.
///
/// A full UUID is unreadable at a glance and far too long for a list; eight hex
/// characters are plenty to recognise a rep while keeping rows scannable.
String uuidShort(String? uuid) {
  if (uuid == null) return '????????';
  final clean = uuid.replaceAll('-', '');
  return clean.length < 8 ? clean : clean.substring(0, 8);
}
