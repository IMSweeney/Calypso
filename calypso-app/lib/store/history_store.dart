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

  /// The rep uuids the records file holds, which is what decides whether a rep is
  /// held — **not** [_received]. Both are rebuilt from the log by [load]; the one
  /// that answers "do we have this rep?" is the log.
  final Set<String> _uuids = {};

  /// Client receipt times by uuid, loaded from and saved to
  /// [HistoryFiles.receiptTimes].
  ///
  /// **Decoration, never truth.** The device has no RTC, so its own timestamps are
  /// uptime milliseconds and mean nothing across a reboot; this is the only absolute
  /// time available, and the only reason the side-car exists at all. Losing it costs
  /// the list its timestamps and nothing else — in particular it must never make the
  /// app treat a rep it already has as new, because appending it again would put two
  /// copies of it in the log ([appendNew]).
  final Map<String, DateTime> _received = {};
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
    _uuids.clear();
    _received.clear();
    truncated = false;

    await files.directory.create(recursive: true);
    await _loadReceiptTimes();

    if (await files.records.exists()) {
      final bytes = await files.records.readAsBytes();
      _bytes = bytes.length;
      _goodBytes = 0;

      final scan = scanBlocks(bytes, _received);
      _reps.addAll(scan.reps);
      for (final rep in _reps) {
        _uuids.add(rep.uuid);
      }
      truncated = scan.truncated;
      _goodBytes = scan.goodBytes;
    } else {
      _bytes = 0;
      _goodBytes = 0;
    }

    // Times for reps the log does not hold are not worth keeping: the rep would be
    // re-synced and re-stamped, and until then the entry is a uuid the log cannot
    // account for. Pruned for the missing-log case too, which is why this sits
    // outside the branch — and persisted there and then, so the file on disk never
    // disagrees with the log. (An unreadable records file throws above rather than
    // landing here, so a bad read cannot be mistaken for an empty log.)
    final before = _received.length;
    _received.removeWhere((uuid, _) => !_uuids.contains(uuid));
    if (_received.length != before) await _saveReceiptTimes();
  }

  /// Adds records for reps the archive does not already have.
  ///
  /// [recordBlocks] is one framed record block per rep (summary + chunks), as
  /// produced by the transport. Reps already in the archive are skipped, which is
  /// what makes re-syncing — every connection, and every fixture reload — cheap
  /// and idempotent.
  @override
  Future<int> appendNew(Map<String, Uint8List> recordBlocks, {DateTime? receivedAt}) async {
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
        // The log decides, and only the log: a missing or stale `received.json` must
        // not make this append a second copy of a rep whose records are already here.
        if (_uuids.contains(entry.key)) continue;
        sink.add(entry.value);
        added++;
        _uuids.add(entry.key);
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

  /// Whether the archive already holds this rep — asked of the log, not of the
  /// receipt times beside it.
  @override
  bool contains(String uuid) => _uuids.contains(uuid);

  /// Reads one rep's trace out of the archive.
  @override
  Future<List<TracePoint>> trace(String uuid) async {
    StoredRep? stored;
    for (final r in _reps) {
      if (r.uuid == uuid) stored = r;
    }
    if (stored == null) return const [];

    final handle = await files.records.open();
    try {
      await handle.setPosition(stored.offset);
      final block = await handle.read(stored.length);
      return assembleTraceFromBlock(Uint8List.fromList(block), uuid);
    } finally {
      await handle.close();
    }
  }

  Future<void> _loadReceiptTimes() async {
    if (!await files.receiptTimes.exists()) return;
    try {
      // Keys are rep uuids.
      final json = jsonDecode(await files.receiptTimes.readAsString()) as Map<String, dynamic>;
      json.forEach((key, value) {
        final when = DateTime.tryParse('$value');
        if (when != null) _received[key] = when;
      });
    } catch (_) {
      // A corrupt receipt-times file costs timestamps, not data.
      _received.clear();
    }
  }

  /// Best-effort, deliberately: this file is decoration (see [_received]), so a disk
  /// that will not take it costs timestamps and nothing else. Failing the append
  /// that triggered it would be the tail wagging the dog.
  Future<void> _saveReceiptTimes() async {
    final json = {for (final e in _received.entries) e.key: e.value.toIso8601String()};
    try {
      await files.receiptTimes.writeAsString(jsonEncode(json));
    } catch (_) {}
  }
}
