library;

import 'package:logging/logging.dart';

import '../gatt/gatt_client.dart' show SyncClient;

final Logger bleLog = Logger('ble');
final Logger appLog = Logger('app');

/// One BLE value, as a log line: the direction, the characteristic, its length, and the bytes.
///
/// The live IMU stream is the one exception. It is a dozen notifications a second of a payload that is being recorded somewhere else, so its line carries the length and not the bytes: the hex was four fifths of the log during a recording, and building it was work done on every notification whether or not the line was kept.
String describeMessage(
  String direction,
  String characteristic,
  List<int> bytes,
) {
  final head = '$direction $characteristic ${bytes.length}B';
  if (characteristic == SyncClient.imuUuid) return head;
  final hex = bytes.map((b) => b.toRadixString(16).padLeft(2, '0'));
  return '$head ${hex.join(' ')}';
}
