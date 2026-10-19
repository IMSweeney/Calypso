/// The app's named loggers.
///
/// The name is what a reader greps for, and it is the only thing shared between the
/// two places that watch the BLE conversation: the plugin seam, which sees every
/// value the radio delivered, and the transport, which sees what its
/// per-characteristic streams made of them. Both log under `ble`, so the file is one
/// story in order.
library;

import 'package:logging/logging.dart';

/// Device <-> app BLE traffic: lifecycle, and every message in either direction.
final Logger bleLog = Logger('ble');

/// The app's own business, as opposed to the device's: logging itself, and what it
/// is writing to.
final Logger appLog = Logger('app');

/// One BLE message as the log writes it: `<direction> <characteristic> <n>B <hex…>`.
///
/// Here rather than at either call site because both write it: the transport for the
/// messages it handles (`rx`, `tx`), and the plugin seam for every value the radio
/// delivered before the transport's streams filtered it (`raw`). Two spellings of the
/// same line would make the log ambiguous exactly when it is being read to settle an
/// argument.
String describeMessage(String direction, String characteristic, List<int> bytes) {
  final hex = bytes.map((b) => b.toRadixString(16).padLeft(2, '0'));
  // Joined rather than interpolated so an empty payload ends at its `0B` rather
  // than trailing a space.
  return [direction, characteristic, '${bytes.length}B', ...hex].join(' ');
}
