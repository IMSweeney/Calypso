library;

import 'dart:typed_data';

import '../protocol/calypso.pb.dart' as pb;
import 'rep_archive.dart';

class MemoryArchive implements RepArchive {
  MemoryArchive({Map<String, Uint8List>? blocks}) {
    if (blocks != null) _bytes.addAll(_concat(blocks.values));
  }

  final List<int> _bytes = [];
  final Map<String, DateTime> _received = {};
  final Set<String> _uuids = {};
  List<StoredRep> _reps = [];
  bool _truncated = false;

  @override
  List<StoredRep> get reps => List.unmodifiable(_reps);

  @override
  bool get truncated => _truncated;

  @override
  int get sizeBytes => _bytes.length;

  @override
  bool contains(String uuid) => _uuids.contains(uuid);

  @override
  Future<void> load() async {
    final scan = scanBlocks(Uint8List.fromList(_bytes), _received);
    _reps = scan.reps;
    _truncated = scan.truncated;
    _uuids
      ..clear()
      ..addAll([for (final rep in _reps) rep.uuid]);
    _received.removeWhere((uuid, _) => !_uuids.contains(uuid));
  }

  @override
  Future<int> appendNew(Map<String, Uint8List> recordBlocks, {DateTime? receivedAt}) async {
    var added = 0;
    for (final entry in recordBlocks.entries) {
      if (_uuids.contains(entry.key)) continue;
      _bytes.addAll(entry.value);
      _received[entry.key] = receivedAt ?? DateTime.now();
      added++;
    }
    if (added > 0) await load();
    return added;
  }

  @override
  Future<List<pb.TraceSample>> trace(String uuid) async {
    for (final stored in _reps) {
      if (stored.uuid != uuid) continue;
      final block = Uint8List.fromList(_bytes.sublist(stored.offset, stored.offset + stored.length));
      return assembleTraceFromBlock(block, uuid);
    }
    return const [];
  }

  static List<int> _concat(Iterable<Uint8List> blocks) => [
        for (final block in blocks) ...block,
      ];
}
