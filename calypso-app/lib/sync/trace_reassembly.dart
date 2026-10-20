library;

import '../model/uuid.dart';
import '../protocol/calypso.pb.dart' as pb;

class TraceAssembler {
  TraceAssembler(this.uuid);

  final String uuid;

  final Map<int, List<pb.TraceSample>> _chunks = {};

  bool add(pb.TraceChunk chunk) {
    if (uuidFromBytes(chunk.repUuid) != uuid) return false;
    _chunks[chunk.index] = chunk.samples.toList();
    return true;
  }

  int get chunkCount => _chunks.length;

  bool get isEmpty => _chunks.isEmpty;

  List<int> get indices => _chunks.keys.toList()..sort();

  List<int> missingIndices() {
    if (_chunks.isEmpty) return const [];
    final highest = indices.last;
    return [
      for (var i = 0; i <= highest; i++)
        if (!_chunks.containsKey(i)) i,
    ];
  }

  List<pb.TraceSample> assemble() => [
    for (final index in indices) ...?_chunks[index],
  ];

  void clear() => _chunks.clear();
}
