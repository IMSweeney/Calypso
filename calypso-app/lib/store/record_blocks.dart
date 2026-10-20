library;

import 'dart:typed_data';

import '../model/rep.dart';
import '../protocol/calypso.pb.dart' as pb;
import '../transport/transport.dart';

const int pbTraceChunkSamples = 48;

Uint8List frameRecord(List<int> bytes) {
  final out = BytesBuilder();
  final length = ByteData(4)..setUint32(0, bytes.length, Endian.little);
  out.add(length.buffer.asUint8List());
  out.add(bytes);
  return out.toBytes();
}

Uint8List buildRepBlock(pb.RepSummary summary, List<pb.TraceSample> trace) {
  final records = BytesBuilder();
  records.add(frameRecord(pb.RepRecord(summary: summary).writeToBuffer()));

  for (var start = 0; start < trace.length; start += pbTraceChunkSamples) {
    final slice = trace.sublist(
      start,
      (start + pbTraceChunkSamples).clamp(0, trace.length),
    );
    records.add(
      frameRecord(
        pb.RepRecord(
          trace: pb.TraceChunk(
            repUuid: summary.repUuid,
            index: start ~/ pbTraceChunkSamples,
            firstTOffsetMs: slice.first.tOffsetMs,
            totalChunks: 0,
            samples: slice,
          ),
        ).writeToBuffer(),
      ),
    );
  }
  return records.toBytes();
}

Future<Map<String, Uint8List>> recordBlocksFrom(RepTransport transport) async {
  final blocks = <String, Uint8List>{};
  for (final summary in await transport.summaries()) {
    final uuid = summary.uuid!;
    List<pb.TraceSample> trace;
    try {
      trace = await transport.trace(uuid);
    } on TraceUnavailableException {
      trace = const [];
    }
    blocks[uuid] = buildRepBlock(summary, trace);
  }
  return blocks;
}
