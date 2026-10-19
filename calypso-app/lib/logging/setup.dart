/// Turning the app's logging on, in one place.
///
/// The pipeline is `package:logging`, which is global: appenders hang off
/// `Logger.root.onRecord` and every `Logger('name')` in the app reports into it.
/// That makes "don't log to a file in a test" the absence of a call rather than
/// something threaded through constructors — a test that never calls
/// [startFileLogging] writes nothing, because nothing is attached.
library;

import 'dart:io';

import 'package:logging/logging.dart';

import 'rotating_file_appender.dart';

/// Starts writing everything the app logs to a rotating file, and returns the
/// appender doing it so the caller can [RotatingFileAppender.close] it.
///
/// Called from the real app's startup — never from a test, which is the whole
/// mechanism.
RotatingFileAppender startFileLogging(
  Future<Directory> Function() directory, {
  int maxBytes = 10 * 1024 * 1024,
  int keep = 3,
}) {
  // **This line is load-bearing.** `Logger.root.level` defaults to `Level.INFO`
  // and is checked *before* any listener runs, so without it every debug-level
  // record — the entire BLE conversation — is dropped, and the failure is the
  // worst kind: the file is created, opened, and stays empty. `logging_test`
  // pins it, because a silent empty log is exactly the thing this log exists to
  // rule out.
  Logger.root.level = Level.ALL;

  final appender = RotatingFileAppender(directory, maxBytes: maxBytes, keep: keep)
    ..attachToLogger(Logger.root);
  return appender;
}
