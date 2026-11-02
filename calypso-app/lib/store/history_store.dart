library;

import 'dart:io';
import 'dart:typed_data';

import '../protocol/calypso.pb.dart' as pb;
import 'rep_archive.dart';

export 'rep_archive.dart' show StoredRep;

class HistoryFiles {
  HistoryFiles(this.directory);

  final Directory directory;

  File get records => File('${directory.path}/history.pblog');
}

class HistoryStore implements RepArchive {
  HistoryStore(this.files);

  final HistoryFiles files;

  final List<StoredRep> _reps = [];
  final Set<String> _uuids = {};
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
    truncated = false;

    await files.directory.create(recursive: true);

    if (await files.records.exists()) {
      final bytes = await files.records.readAsBytes();
      _bytes = bytes.length;
      _goodBytes = 0;

      final scan = scanBlocks(bytes);
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
  }

  @override
  Future<int> appendNew(Map<String, Uint8List> recordBlocks) async {
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
      }
    } finally {
      await sink.flush();
      await sink.close();
    }
    if (added > 0 || _reps.isEmpty) await load();
    return added;
  }

  @override
  bool contains(String uuid) => _uuids.contains(uuid);

  @override
  Future<List<pb.TraceComputedPoint>> trace(String uuid) async {
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
}
