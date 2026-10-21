library;

import 'dart:typed_data';

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

/// What a `SyncMessage` carries, for a log line: its body and what that body is
/// about.
///
/// The bytes are logged at the plugin seam (`raw`), which is where "the value
/// arrived at all" is answered. Above the seam this is the more useful half of
/// the pair: what the transport actually handled, without re-reading hex.
String describeSyncMessage(SyncMessage message) {
  switch (message.whichBody()) {
    case SyncMessage_Body.summary:
      return 'summary rep=${uuidFromBytes(message.summary.repUuid) ?? '?'}';
    case SyncMessage_Body.trace:
      final chunk = message.trace;
      final rep = uuidFromBytes(chunk.repUuid) ?? '?';
      return 'trace rep=$rep chunk=${chunk.index + 1}/${chunk.totalChunks}';
    case SyncMessage_Body.status:
      return 'status reps=${message.status.repCount}';
    case SyncMessage_Body.calibration:
      final status = message.calibration;
      return 'calibration sys=${status.sys} accel=${status.accel} '
          'gyro=${status.gyro} mag=${status.mag} ready=${status.ready}';
    case SyncMessage_Body.notSet:
      return 'empty';
  }
}

List<RepSummary> summariesOf(List<RepRecord> records) =>
    records.where((r) => r.hasSummary()).map((r) => r.summary).toList();

String? chunkUuid(TraceChunk chunk) => uuidFromBytes(chunk.repUuid);

List<TraceChunk> traceChunksFor(List<RepRecord> records, String uuid) {
  final chunks =
      records
          .where((r) => r.hasTrace() && chunkUuid(r.trace) == uuid)
          .map((r) => r.trace)
          .toList()
        ..sort((a, b) => a.index.compareTo(b.index));
  return chunks;
}
