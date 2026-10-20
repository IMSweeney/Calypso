library;

import 'package:flutter_test/flutter_test.dart';
import 'package:logging/logging.dart';

List<LogRecord> captureLogs() {
  Logger.root.level = Level.ALL;

  final records = <LogRecord>[];
  Logger.root.onRecord.listen(records.add);
  addTearDown(Logger.root.clearListeners);
  return records;
}
