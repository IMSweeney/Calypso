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
import 'rep_archive.dart';

export 'rep_archive.dart' show StoredRep;

/// Where [HistoryStore] records live, so tests can use a temporary directory.
class HistoryFiles {
  HistoryFiles(this.directory);

  final Directory directory;

  File get records => File('${directory.path}/history.pblog');
  File get receiptTimes => File('${directory.path}/received.json');
}

class HistoryStore implements RepArchive {
  HistoryStore(this.files);

  final HistoryFiles files;

  final List<StoredRep> _reps = [];
  final Map<int, DateTime> _received = {};
  int _bytes = 0;

  /// Reps in the archive, oldest first.
  @override
  List<StoredRep> get reps => List.unmodifiable(_reps);

  /// Size of the records file.
  @override
  int get sizeBytes => _bytes;

  /// True if the records file ends mid-record: the tail of a write that did not
  /// finish. The records before it are still good, and the next append repairs
  /// the file (see [appendNew]).
  @override
  bool truncated = false;

  /// Size of the prefix that ended on a record boundary. A torn tail beyond it
  /// is discarded before the next append, so the framing cannot stay desynced.
  int _goodBytes = 0;

  /// Reads (or creates) the archive.
  @override
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

    final scan = scanBlocks(bytes, _received);
    _reps.addAll(scan.reps);
    truncated = scan.truncated;
    _goodBytes = scan.goodBytes;
  }

  /// Adds records for reps the archive does not already have.
  ///
  /// [recordBlocks] is one framed record block per rep (summary + chunks), as
  /// produced by the transport. Reps already in the archive are skipped, which is
  /// what makes re-syncing — every connection, and every fixture reload — cheap
  /// and idempotent.
  @override
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
  @override
  bool contains(int repId) => _received.containsKey(repId);

  /// Reads one rep's trace out of the archive.
  @override
  Future<List<TracePoint>> trace(int repId) async {
    StoredRep? stored;
    for (final r in _reps) {
      if (r.summary.repId == repId) stored = r;
    }
    if (stored == null) return const [];

    final handle = await files.records.open();
    try {
      await handle.setPosition(stored.offset);
      final block = await handle.read(stored.length);
      return assembleTraceFromBlock(Uint8List.fromList(block), repId);
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
