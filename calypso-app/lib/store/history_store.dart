/// The client's archive of record.
///
/// The device's log is a ring that erases its oldest sectors, so the client
/// keeps everything it has synced. The file is a `.pblog` — the same
/// `[u32 LE length][RepRecord]` framing as fixtures and captures — so history,
/// a captured fixture and a dump from the device are all one format, and a
/// fixture is just a copy of somebody's history.
///
/// Deliberately no separate index file: the framing scan *is* the index. A rep's
/// records are written contiguously (its summary followed by its chunks), so the
/// scan only has to decode summaries — a few hundred bytes each — and can skip
/// chunk payloads by their length. That keeps opening a large history cheap
/// without a second thing to keep in sync.
library;

import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import '../model/rep.dart';
import '../protocol/calypso.pb.dart' as pb;
import '../protocol/pblog.dart';
import '../sync/trace_reassembly.dart';

/// A rep in the archive: its summary, where its records live, and when the
/// client took receipt of it.
class StoredRep {
  StoredRep({
    required this.summary,
    required this.offset,
    required this.length,
    this.receivedAt,
  });

  final RepSummary summary;

  /// Byte range of this rep's whole record block (summary + chunks).
  final int offset;
  final int length;

  /// Wall-clock time the client received the summary, if known.
  ///
  /// The device has no RTC, so this is the only absolute time available. It is
  /// the time of receipt, not of the lift: if the client is closed mid-set, the
  /// reps it finds later are stamped when it comes back (see the plan's open
  /// question on wall-clock time).
  final DateTime? receivedAt;
}

/// Where [HistoryStore] records live, so tests can use a temporary directory.
class HistoryFiles {
  HistoryFiles(this.directory);

  final Directory directory;

  File get records => File('${directory.path}/history.pblog');
  File get receiptTimes => File('${directory.path}/received.json');
}

class HistoryStore {
  HistoryStore(this.files);

  final HistoryFiles files;

  final List<StoredRep> _reps = [];
  final Map<int, DateTime> _received = {};
  int _bytes = 0;

  /// Reps in the archive, oldest first.
  List<StoredRep> get reps => List.unmodifiable(_reps);

  /// Size of the records file.
  int get sizeBytes => _bytes;

  /// True if the records file ends mid-record: the tail of a write that did not
  /// finish. The records before it are still good, and the next append repairs
  /// the file (see [appendNew]).
  bool truncated = false;

  /// Size of the prefix that ended on a record boundary. A torn tail beyond it
  /// is discarded before the next append, so the framing cannot stay desynced.
  int _goodBytes = 0;

  /// Reads (or creates) the archive.
  Future<void> load() async {
    _reps.clear();
    _received.clear();
    truncated = false;

    await files.directory.create(recursive: true);
    await _loadReceiptTimes();

    if (!await files.records.exists()) {
      _bytes = 0;
      _goodBytes = 0;
      return;
    }
    final bytes = await files.records.readAsBytes();
    _bytes = bytes.length;
    _goodBytes = 0;

    var pos = 0;
    int? blockStart;
    var blockLength = 0;
    StoredRep? pending;
    while (pos + 4 <= bytes.length) {
      final length = ByteData.sublistView(bytes, pos, pos + 4).getUint32(0, Endian.little);
      final start = pos + 4;
      if (length <= 0 || start + length > bytes.length) {
        truncated = true;
        break;
      }
      // A summary starts a new block; chunks just extend the current one.
      pb.RepRecord record;
      try {
        record = pb.RepRecord.fromBuffer(bytes.sublist(start, start + length));
      } catch (_) {
        truncated = true;
        break;
      }
      if (record.hasSummary()) {
        // Everything up to this record parsed on a boundary.
        _goodBytes = pos;
        if (pending != null) {
          _reps.add(StoredRep(
            summary: pending.summary,
            offset: blockStart!,
            length: blockLength,
            receivedAt: _received[pending.summary.repId],
          ));
        }
        pending = StoredRep(
          summary: RepSummary.fromProto(record.summary),
          offset: 0,
          length: 0,
          receivedAt: _received[record.summary.repId],
        );
        blockStart = pos;
        blockLength = 0;
      }
      blockLength += 4 + length;
      pos = start + length;
    }
    if (pending == null) return;
    if (truncated) {
      // The tail rep's records were cut short. Drop it: a summary with a partial
      // trace would be worse than re-syncing, and because it is no longer
      // recorded as received, the next sync fetches it again.
      _received.remove(pending.summary.repId);
      return;
    }
    _reps.add(StoredRep(
      summary: pending.summary,
      offset: blockStart!,
      length: blockLength,
      receivedAt: _received[pending.summary.repId],
    ));
    _goodBytes = bytes.length;
  }

  /// Adds records for reps the archive does not already have.
  ///
  /// [recordBlocks] is one framed record block per rep (summary + chunks), as
  /// produced by the transport. Reps already in the archive are skipped, which is
  /// what makes re-syncing — every connection, and every fixture reload — cheap
  /// and idempotent.
  Future<int> appendNew(Map<int, Uint8List> recordBlocks, {DateTime? receivedAt}) async {
    // Repair a torn tail first: appending after a partial record would desync
    // the framing for every future read.
    if (truncated) {
      final handle = await files.records.open(mode: FileMode.append);
      await handle.truncate(_goodBytes);
      await handle.close();
      truncated = false;
    }
    var added = 0;
    final sink = files.records.openWrite(mode: FileMode.append);
    try {
      for (final entry in recordBlocks.entries) {
        if (_received.containsKey(entry.key)) continue;
        sink.add(entry.value);
        added++;
        _received[entry.key] = receivedAt ?? DateTime.now();
      }
    } finally {
      await sink.flush();
      await sink.close();
    }
    if (added > 0) await _saveReceiptTimes();
    if (added > 0 || _reps.isEmpty) await load();
    return added;
  }

  /// Whether the archive already holds this rep.
  bool contains(int repId) => _received.containsKey(repId);

  /// Reads one rep's trace out of the archive.
  Future<List<TracePoint>> trace(int repId) async {
    final stored = _reps.where((r) => r.summary.repId == repId).firstOrNull;
    if (stored == null) return const [];

    final handle = await files.records.open();
    try {
      await handle.setPosition(stored.offset);
      final block = await handle.read(stored.length);
      final dump = readDump(Uint8List.fromList(block));
      final chunks = traceChunksFor(dump.records, repId);
      if (chunks.isEmpty) return const [];
      final assembler = TraceAssembler(repId);
      for (final chunk in chunks) {
        assembler.add(chunk);
      }
      return assembler.assemble();
    } finally {
      await handle.close();
    }
  }

  Future<void> _loadReceiptTimes() async {
    if (!await files.receiptTimes.exists()) return;
    try {
      final json = jsonDecode(await files.receiptTimes.readAsString()) as Map<String, dynamic>;
      json.forEach((key, value) {
        final id = int.tryParse(key);
        final when = DateTime.tryParse('$value');
        if (id != null && when != null) _received[id] = when;
      });
    } catch (_) {
      // A corrupt receipt-times file costs timestamps, not data.
      _received.clear();
    }
  }

  Future<void> _saveReceiptTimes() async {
    final json = {for (final e in _received.entries) '${e.key}': e.value.toIso8601String()};
    await files.receiptTimes.writeAsString(jsonEncode(json));
  }
}
