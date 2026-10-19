/// A rotating file appender for `package:logging`.
///
/// Attaches to a [Logger] — in practice `Logger.root`, which is every logger in the
/// app — and appends one line per record, rotating before the file gets unwieldy.
/// The pipeline itself (levels, named loggers, the `onRecord` stream) is
/// `package:logging`; this is only the part it does not ship.
library;

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:flutter/foundation.dart' show debugPrint;
import 'package:logging/logging.dart';

/// Appends records to `name` under `directory`, rotating at [maxBytes].
///
/// **Rotation keeps [keep] files:** the current one plus the [keep] - 1 most
/// recent rotations beside it (`calypso.log`, `calypso.log.1`, `calypso.log.2`, …).
/// When the current file would grow past [maxBytes] it is renamed to `.1`, each
/// older file shifting up one and the oldest dropped, and a fresh file is started.
/// A byte budget rather than a time budget, because it is a burst of trace chunks
/// — not the clock — that makes this file grow.
///
/// **Writing never throws and never blocks.** [handle] only appends to an in-memory
/// queue; a drain runs behind the scenes, serialised so the file can never be
/// written by two hands at once. If the directory cannot be created, the file
/// cannot be opened, or a write fails, the appender quietly turns itself off: a
/// debug log that can take down the connection it is watching is worse than no log.
///
/// [directory] is a callback rather than a `Directory` because the real one comes
/// from `getApplicationSupportDirectory()`, an async platform lookup, while the
/// appender is constructed from `initState`.
class RotatingFileAppender {
  RotatingFileAppender(
    this._directory, {
    this.name = 'calypso.log',
    this.maxBytes = 10 * 1024 * 1024,
    this.keep = 3,
  }) {
    _opening = _open();
  }

  final Future<Directory> Function() _directory;

  /// Base file name; rotations get `.$n` appended.
  final String name;

  /// Rotate once the current file would exceed this.
  final int maxBytes;

  /// How many files to keep, the current one included. At least 1.
  final int keep;

  StreamSubscription<LogRecord>? _subscription;
  Directory? _dir;
  File? _file;
  IOSink? _sink;

  /// Bytes written to the current file, tracked rather than asked for, so the
  /// rotation check costs nothing and cannot race a `stat`.
  int _size = 0;

  /// Lines waiting to be written. [handle] is called from the logging path and
  /// must not await, so it queues and returns.
  final List<String> _queue = [];

  late final Future<void> _opening;

  /// The write chain: each drain is appended after the last, so only one runs at a
  /// time whatever the call pattern.
  Future<void> _tail = Future<void>.value();
  bool _drainQueued = false;
  bool _closed = false;

  /// Starts reporting [logger]'s records here, replacing any previous attachment.
  ///
  /// `Logger.root` is the usual argument, and the one that covers the app. A named
  /// logger works too, but with `hierarchicalLoggingEnabled` false (the default) a
  /// named logger's listeners only see records logged to that exact name.
  ///
  /// Whether a record gets this far is the logger's decision, not ours:
  /// `Logger.root.level` filters before any listener is called.
  void attachToLogger(Logger logger) {
    _subscription?.cancel();
    _subscription = logger.onRecord.listen(handle);
  }

  /// Writes one record. Public because it is also what a test can call directly.
  void handle(LogRecord record) {
    if (_closed) return;
    _queue.add(_line(record));
    _schedule();
  }

  /// Detaches, flushes and closes. Idempotent.
  Future<void> close() async {
    await _subscription?.cancel();
    _subscription = null;
    if (_closed) return;
    _closed = true;
    // Let the queued lines out, then close behind them.
    _schedule();
    await _tail;
    try {
      await _sink?.flush();
      await _sink?.close();
    } catch (_) {
      // Best effort, as everywhere in here.
    }
    _sink = null;
  }

  // ── Writing ─────────────────────────────────────────────────────────────

  /// One line: `<timestamp> <LEVEL> <loggerName> <message>`.
  ///
  /// Local time, because the log is read against a person's memory of a session
  /// rather than diffed against the device, whose uptime has no wall clock in it
  /// at all. The level comes from `package:logging` (`FINE`, `WARNING`, …) and is
  /// padded so it holds a column. A record's error rides along on the same line;
  /// its stack trace does not, because one line per record is what keeps rotation
  /// and grepping simple.
  String _line(LogRecord record) {
    final buffer = StringBuffer()
      ..write(record.time.toIso8601String())
      ..write(' ')
      ..write(record.level.name.padRight(7))
      ..write(' ')
      ..write(record.loggerName)
      ..write(' ')
      ..write(record.message);
    if (record.error != null) buffer.write(' (error: ${record.error})');
    return buffer.toString();
  }

  /// Queue a drain, unless one is already waiting. Coalescing matters: a trace
  /// arrives as dozens of notifications in a burst, and they should become one
  /// write, not dozens.
  void _schedule() {
    if (_drainQueued) return;
    _drainQueued = true;
    _tail = _tail.then((_) {
      _drainQueued = false;
      return _drain();
    });
  }

  Future<void> _drain() async {
    try {
      await _opening;
      while (_queue.isNotEmpty) {
        final line = _queue.removeAt(0);
        if (_sink == null) continue; // Logging is off; drop the rest.
        final chunk = utf8.encode('$line\n');
        if (_size + chunk.length > maxBytes) {
          await _rotate();
          if (_sink == null) continue;
        }
        _sink!.add(chunk);
        _size += chunk.length;
      }
      await _sink?.flush();
    } catch (_) {
      // Swallowed on purpose: see the class doc.
    }
  }

  // ── Files ───────────────────────────────────────────────────────────────

  Future<void> _open() async {
    try {
      final directory = await _directory();
      await directory.create(recursive: true);
      _dir = directory;
      _file = _path(0);
      // Pick up where a previous run left off, unless that file is already full.
      if (await _file!.exists() && await _file!.length() >= maxBytes) {
        await _shift();
      }
      await _openSink();
      debugPrint('calypso: log at ${_file!.path}');
    } catch (e) {
      _sink = null;
      debugPrint('calypso: log unavailable ($e)');
    }
  }

  Future<void> _openSink() async {
    _sink = _file!.openWrite(mode: FileMode.append);
    // Writes are fire-and-forget; an error here must not surface as an unhandled
    // async error somewhere unrelated.
    _sink!.done.ignore();
  }

  /// Shift the files along one: the oldest is dropped, the current becomes `.1`,
  /// and a fresh current file is left to be opened.
  Future<void> _shift() async {
    final oldest = _path(keep - 1);
    if (await oldest.exists()) await oldest.delete();
    for (var i = keep - 2; i >= 1; i--) {
      final from = _path(i);
      if (await from.exists()) await from.rename(_path(i + 1).path);
    }
    if (await _path(0).exists()) await _path(0).rename(_path(1).path);
    _size = 0;
  }

  /// Close the current file, shift, and start a new one. Called mid-drain, so the
  /// sink is reopened here rather than in [_open].
  Future<void> _rotate() async {
    try {
      await _sink?.flush();
      await _sink?.close();
      _sink = null;
      await _shift();
      await _openSink();
    } catch (_) {
      _sink = null;
    }
  }

  File _path(int index) {
    final fileName = index == 0 ? name : '$name.$index';
    return File('${_dir!.path}${Platform.pathSeparator}$fileName');
  }
}
