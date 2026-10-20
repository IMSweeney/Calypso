library;

import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import '../protocol/calypso.pb.dart' as pb;
import 'rep_archive.dart';

export 'rep_archive.dart' show StoredRep;

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
  final Set<String> _uuids = {};
  final Map<String, DateTime> _received = {};
  int _bytes = 0;

  @override
  List<StoredRep> get reps => List.unmodifiable(_reps);

  @override
  int get sizeBytes => _bytes;

  @override
  bool truncated = false;

  int _goodBytes = 0;

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

    final before = _received.length;
    _received.removeWhere((uuid, _) => !_uuids.contains(uuid));
    if (_received.length != before) await _saveReceiptTimes();
  }

  @override
  Future<int> appendNew(
    Map<String, Uint8List> recordBlocks, {
    DateTime? receivedAt,
  }) async {
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

  @override
  bool contains(String uuid) => _uuids.contains(uuid);

  @override
  Future<List<pb.TraceSample>> trace(String uuid) async {
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
      final json =
          jsonDecode(await files.receiptTimes.readAsString())
              as Map<String, dynamic>;
      json.forEach((key, value) {
        final when = DateTime.tryParse('$value');
        if (when != null) _received[key] = when;
      });
    } catch (_) {
      _received.clear();
    }
  }

  Future<void> _saveReceiptTimes() async {
    final json = {
      for (final e in _received.entries) e.key: e.value.toIso8601String(),
    };
    try {
      await files.receiptTimes.writeAsString(jsonEncode(json));
    } catch (_) {}
  }
}
