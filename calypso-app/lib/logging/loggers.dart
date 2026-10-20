library;

import 'package:logging/logging.dart';

final Logger bleLog = Logger('ble');
final Logger appLog = Logger('app');

String describeMessage(String direction, String characteristic, List<int> bytes) {
  final hex = bytes.map((b) => b.toRadixString(16).padLeft(2, '0'));
  return [direction, characteristic, '${bytes.length}B', ...hex].join(' ');
}
