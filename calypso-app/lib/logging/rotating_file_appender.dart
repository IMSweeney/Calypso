library;

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:flutter/foundation.dart' show debugPrint;
import 'package:logging/logging.dart';

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
  final String name;
  final int maxBytes;
  final int keep;

  StreamSubscription<LogRecord>? _subscription;
  Directory? _dir;
  File? _file;
  IOSink? _sink;

  int _size = 0;
  final List<String> _queue = [];
  late final Future<void> _opening;

  Future<void> _tail = Future<void>.value();
  bool _drainQueued = false;
  bool _closed = false;

  void attachToLogger(Logger logger) {
    _subscription?.cancel();
    _subscription = logger.onRecord.listen(handle);
  }

  void handle(LogRecord record) {
    if (_closed) return;
    _queue.add(_line(record));
    _schedule();
  }

  Future<void> close() async {
    await _subscription?.cancel();
    _subscription = null;
    if (_closed) return;
    _closed = true;
    _schedule();
    await _tail;
    // Not caught here: a close that could not flush is the caller's to decide about, and it is the only one who knows whether that matters.
    await _sink?.flush();
    await _sink?.close();
    _sink = null;
  }

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

  void _schedule() {
    if (_drainQueued) return;
    _drainQueued = true;
    _tail = _tail
        .then((_) {
          _drainQueued = false;
          return _drain();
        })
        .catchError((Object e) {
          // Handled, not swallowed: the lines are still queued, so the next record retries them.
          // A logger that stops on the first transient failure is worse than one that says so and carries on.
          // `_drainQueued` has to be cleared here too, or nothing would ever drain again.
          _drainQueued = false;
          debugPrint(
            'calypso: a log write failed, and the lines stay queued ($e)',
          );
        });
  }

  Future<void> _drain() async {
    await _opening;
    var wrote = 0;
    while (wrote < _queue.length) {
      if (_sink == null) {
        wrote++;
        continue;
      }
      final chunk = utf8.encode('${_queue[wrote]}\n');
      if (_size + chunk.length > maxBytes) {
        await _rotate();
        if (_sink == null) {
          wrote++;
          continue;
        }
      }
      _sink!.add(chunk);
      _size += chunk.length;
      wrote++;
    }
    // The lines are dropped only once the flush has gone through, so a write that fails keeps them for the next drain to retry.
    await _sink?.flush();
    _queue.removeRange(0, wrote);
  }

  Future<void> _open() async {
    try {
      final directory = await _directory();
      await directory.create(recursive: true);
      _dir = directory;
      _file = _path(0);
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
    _sink!.done.ignore();
  }

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

  Future<void> _rotate() async {
    // A rotation that fails is reported and retried like any other write: the lines are still queued, and the sink is left closed rather than half-rotated.
    await _sink?.flush();
    await _sink?.close();
    _sink = null;
    await _shift();
    await _openSink();
  }

  File _path(int index) {
    final fileName = index == 0 ? name : '$name.$index';
    return File('${_dir!.path}${Platform.pathSeparator}$fileName');
  }
}
