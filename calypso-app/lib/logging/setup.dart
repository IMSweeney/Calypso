library;

import 'dart:io';

import 'package:logging/logging.dart';

import 'loggers.dart';
import 'rotating_file_appender.dart';

RotatingFileAppender startFileLogging(
  Future<Directory> Function() directory, {
  int maxBytes = 10 * 1024 * 1024,
  int keep = 3,
}) {
  Logger.root.level = Level.ALL;

  final appender = RotatingFileAppender(
    directory,
    maxBytes: maxBytes,
    keep: keep,
  )..attachToLogger(Logger.root);

  appLog.info('file logging started ($maxBytes bytes per file, $keep kept)');
  return appender;
}
