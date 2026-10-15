/// Reassembling a rep's trace from its chunks.
///
/// Traces arrive in chunks because a BLE notification is MTU-bound (~4 points
/// per notification at a 247-byte MTU), and a rep is up to 512 points. Chunks
/// can arrive out of order and a trace can be requested again, so this is
/// deliberately tolerant: chunks are keyed by index, a repeated index replaces
/// the earlier one, and chunks for other reps are ignored.
library;

import '../model/rep.dart';
import '../protocol/calypso.pb.dart' as pb;

class TraceAssembler {
  TraceAssembler(this.repId);

  final int repId;

  /// Chunk index -> points, so a duplicate or re-sent chunk is idempotent.
  final Map<int, List<TracePoint>> _chunks = {};

  /// Adds a chunk if it belongs to this rep, returning whether it did.
  bool add(pb.TraceChunk chunk) {
    if (chunk.repId != repId) return false;
    _chunks[chunk.index] = chunk.samples.map(TracePoint.fromProto).toList();
    return true;
  }

  int get chunkCount => _chunks.length;

  bool get isEmpty => _chunks.isEmpty;

  /// The indices received so far, ascending.
  List<int> get indices => _chunks.keys.toList()..sort();

  /// Gaps in the chunk indices: if a chunk went missing, its index appears
  /// here so the caller can ask for that one chunk again.
  List<int> missingIndices() {
    if (_chunks.isEmpty) return const [];
    final highest = indices.last;
    return [
      for (var i = 0; i <= highest; i++)
        if (!_chunks.containsKey(i)) i,
    ];
  }

  /// The trace so far, in chunk order. Points within a chunk are already in
  /// time order (`TraceSample.t_offset_ms` is monotonically increasing on the
  /// device), and chunks are ordered by index.
  List<TracePoint> assemble() => [
        for (final index in indices) ...?_chunks[index],
      ];

  void clear() => _chunks.clear();
}
