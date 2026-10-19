/// Capturing what the app logs, for the tests that want to assert on it.
///
/// The pipeline is global (`Logger.root`), which is what makes "no log file in a
/// test" free — attach nothing, write nothing — and it is also the thing to be
/// careful with: a listener outlives the test that added it unless it is unhooked.
library;

import 'package:flutter_test/flutter_test.dart';
import 'package:logging/logging.dart';

/// Collects every record logged from here until the test ends.
List<LogRecord> captureLogs() {
  // `Logger.root.level` defaults to `Level.INFO` and is checked *before* any
  // listener runs, so a test that wants the debug-level records this is usually
  // for has to open it up — exactly what the app's `startFileLogging` does.
  Logger.root.level = Level.ALL;

  final records = <LogRecord>[];
  Logger.root.onRecord.listen(records.add);
  // Listeners are global, so one test's collector must not see the next test's
  // records: `clearListeners` closes the root controller and drops everything
  // hooked to it (including this listener).
  addTearDown(Logger.root.clearListeners);
  return records;
}
