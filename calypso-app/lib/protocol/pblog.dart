library;

import 'dart:typed_data';

import 'package:fixnum/fixnum.dart';

import '../model/uuid.dart';
import 'calypso.pb.dart';

class DumpReadResult {
  const DumpReadResult(this.records, {this.truncated = false});

  final List<RepRecord> records;
  final bool truncated;
}

DumpReadResult readDump(Uint8List bytes) {
  final records = <RepRecord>[];
  var pos = 0;
  while (pos + 4 <= bytes.length) {
    final length = ByteData.sublistView(
      bytes,
      pos,
      pos + 4,
    ).getUint32(0, Endian.little);
    final start = pos + 4;
    if (start + length > bytes.length) {
      return DumpReadResult(records, truncated: true);
    }
    try {
      records.add(RepRecord.fromBuffer(bytes.sublist(start, start + length)));
    } catch (_) {
      return DumpReadResult(records, truncated: true);
    }
    pos = start + length;
  }
  return DumpReadResult(records, truncated: pos != bytes.length);
}

/// What a `SyncMessage` carries, for a log line: its body and what that body is about.
/// The bytes themselves are logged at the plugin seam (`raw`), where "the value arrived at all" is answered; this is what the transport handled, without re-reading hex.
String describeSyncMessage(SyncMessage message) {
  switch (message.whichBody()) {
    case SyncMessage_Body.summary:
      return 'summary rep=${uuidFromBytes(message.summary.repUuid) ?? '?'}';
    case SyncMessage_Body.computed:
      final chunk = message.computed;
      final rep = uuidFromBytes(chunk.repUuid) ?? '?';
      return 'trace rep=$rep chunk=${chunk.index + 1}/${chunk.totalChunks}';
    case SyncMessage_Body.raw:
      final chunk = message.raw;
      final rep = uuidFromBytes(chunk.repUuid) ?? '?';
      return 'raw rep=$rep chunk=${chunk.index + 1}/${chunk.totalChunks}';
    case SyncMessage_Body.status:
      return 'status reps=${message.status.repCount}';
    case SyncMessage_Body.notSet:
      return 'empty';
  }
}

/// What the IMU's own values say, for a log line: the raw sample the filter is fed, and the attitude it makes
/// of it. Acceleration is specific force in m/s² (0.01 per LSB, gravity included), the rate is degrees per second
/// (1/16 per LSB), and the quaternion is unit (1/2^14 per LSB).
String describeImuState(ImuState state) {
  const a = 100.0;
  const g = 16.0;
  const q = 16384.0;
  return 'imu t=${state.tUptimeMs} ms '
      'f=(${(state.ax / a).toStringAsFixed(2)}, '
      '${(state.ay / a).toStringAsFixed(2)}, '
      '${(state.az / a).toStringAsFixed(2)}) m/s^2 '
      'w=(${(state.gx / g).toStringAsFixed(1)}, '
      '${(state.gy / g).toStringAsFixed(1)}, '
      '${(state.gz / g).toStringAsFixed(1)}) dps '
      'q=(${(state.qw / q).toStringAsFixed(3)}, '
      '${(state.qx / q).toStringAsFixed(3)}, '
      '${(state.qy / q).toStringAsFixed(3)}, '
      '${(state.qz / q).toStringAsFixed(3)})';
}

/// The summaries a dump holds, each dated by the boot that recorded it.
///
/// A stored summary carries only its uptime: the time is the boot's, and it travels as a `BootClock` record —
/// so a summary's time is `epoch + t_start_ms`, where the epoch is the newest clock record at or before it.
/// A dump carries no record ids to say which boot a record belongs to, so order is the only thing to go by.
///
/// The device goes one better for the boot it is *running*: it holds that boot's epoch in RAM and dates the
/// reps stored before the client wrote it, which a dump cannot know. A rep before its boot's clock record is
/// therefore left at 0 here — "no time" rather than a guess — and the archive has it from the sync itself.
///
/// The summary is dated in place: the records are this reader's own copy of the dump.
List<RepSummary> summariesOf(List<RepRecord> records) {
  var epochS = 0;
  final summaries = <RepSummary>[];
  for (final record in records) {
    if (record.hasClock()) {
      epochS = record.clock.epochS;
      continue;
    }
    if (!record.hasSummary()) continue;
    final summary = record.summary;
    if (epochS != 0) {
      summary.tUnixMs = Int64(epochS * 1000 + summary.tStartMs);
    }
    summaries.add(summary);
  }
  return summaries;
}

String? chunkUuid(TraceChunk chunk) => uuidFromBytes(chunk.repUuid);

/// The plotted chunks a dump holds for one rep, in order.
List<TraceChunk> traceChunksFor(List<RepRecord> records, String uuid) {
  final chunks =
      records
          .where((r) => r.hasComputed() && chunkUuid(r.computed) == uuid)
          .map((r) => r.computed)
          .toList()
        ..sort((a, b) => a.index.compareTo(b.index));
  return chunks;
}
