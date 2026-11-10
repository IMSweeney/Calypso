library;

import 'dart:io';

import '../protocol/calypso.pb.dart' as pb;
import 'record_blocks.dart' show frameRecord;

/// A live IMU stream being appended to a `.pblog`.
///
/// The file is the same framed `RepRecord` stream the device's log uses, with each `ImuChunk` as a `RepRecord.imu`. The first chunk writes the `BootClock` that dates the boot: the device's uptime is not a wall clock, and deriving its epoch from the phone's clock at the moment the chunk arrives is the one anchor available to the app.
///
/// The writes are synchronous because the stream is a dozen small appends a second, on the isolate the chunks arrive on, and because a chunk is in the file as soon as it is added: nothing is held in a buffer for a kill to take.
class ImuRecording {
  ImuRecording._(this.file, this._handle, this._now);

  final File file;
  final RandomAccessFile _handle;
  final DateTime Function() _now;

  int _samples = 0;
  bool _clockWritten = false;
  bool _closed = false;

  /// Samples written so far, across every chunk.
  int get samples => _samples;

  /// Open `directory/imu-<timestamp>.pblog`. The file stays empty until the first chunk, which writes the clock and the chunk together.
  static ImuRecording start(Directory directory, {DateTime Function()? now}) {
    directory.createSync(recursive: true);
    final at = now ?? DateTime.now;
    final file = File('${directory.path}/imu-${_stamp(at())}.pblog');
    return ImuRecording._(file, file.openSync(mode: FileMode.write), at);
  }

  void add(pb.ImuChunk chunk) {
    if (_closed) return;
    if (!_clockWritten) {
      _clockWritten = true;
      final epochS = (_now().millisecondsSinceEpoch - chunk.firstTMs) ~/ 1000;
      _write(pb.RepRecord(clock: pb.BootClock(epochS: epochS)));
    }
    _samples += chunk.samples.length;
    _write(pb.RepRecord(imu: chunk));
  }

  /// Flush and close the file. Safe to call more than once; the second call does nothing.
  void stop() {
    if (_closed) return;
    _closed = true;
    _handle.flushSync();
    _handle.closeSync();
  }

  void _write(pb.RepRecord record) =>
      _handle.writeFromSync(frameRecord(record.writeToBuffer()));
}

/// A filename-safe local timestamp, with milliseconds so two recordings started in the same second do not collide.
String _stamp(DateTime at) {
  final local = at.toLocal();
  String two(int value) => value.toString().padLeft(2, '0');
  String three(int value) => value.toString().padLeft(3, '0');
  return '${local.year}${two(local.month)}${two(local.day)}'
      '-${two(local.hour)}${two(local.minute)}${two(local.second)}'
      '-${three(local.millisecond)}';
}
