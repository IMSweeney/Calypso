library;

import 'dart:typed_data';

import '../model/rep.dart';
import '../model/uuid.dart';
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

Uint8List buildRepBlock(RepSummary summary, List<TracePoint> trace) {
  final records = BytesBuilder();
  records.add(frameRecord(
    pb.RepRecord(
      summary: pb.RepSummary(
        repUuid: uuidToBytes(summary.uuid) ?? Uint8List(0),
        tStartMs: summary.tStartMs,
        tEndMs: summary.tEndMs,
        durationMs: summary.durationMs,
        rom: summary.rom,
        peakVelocity: summary.peakVelocity,
        meanVelocity: summary.meanVelocity,
        peakAccel: summary.peakAccel,
        rollMinDeg: summary.rollMinDeg,
        rollMaxDeg: summary.rollMaxDeg,
        pitchMinDeg: summary.pitchMinDeg,
        pitchMaxDeg: summary.pitchMaxDeg,
        traceLen: summary.traceLen,
        chunkCount: summary.chunkCount,
        decimated: summary.decimated,
      ),
    ).writeToBuffer(),
  ));

  for (var start = 0; start < trace.length; start += pbTraceChunkSamples) {
    final slice = trace.sublist(start, (start + pbTraceChunkSamples).clamp(0, trace.length));
    records.add(frameRecord(
      pb.RepRecord(
        trace: pb.TraceChunk(
          repUuid: uuidToBytes(summary.uuid) ?? Uint8List(0),
          index: start ~/ pbTraceChunkSamples,
          firstTOffsetMs: slice.first.tOffsetMs,
          totalChunks: 0,
          samples: [
            for (final p in slice)
              pb.TraceSample(
                tOffsetMs: p.tOffsetMs,
                px: p.x,
                py: p.y,
                pz: p.z,
                ax: p.ax,
                ay: p.ay,
                az: p.az,
                qw: p.qw,
                qx: p.qx,
                qy: p.qy,
                qz: p.qz,
              ),
          ],
        ),
      ).writeToBuffer(),
    ));
  }
  return records.toBytes();
}

Future<Map<String, Uint8List>> recordBlocksFrom(RepTransport transport) async {
  final blocks = <String, Uint8List>{};
  for (final summary in await transport.summaries()) {
    List<TracePoint> trace;
    try {
      trace = await transport.trace(summary.uuid);
    } on TraceUnavailableException {
      trace = const [];
    }
    blocks[summary.uuid] = buildRepBlock(summary, trace);
  }
  return blocks;
}
