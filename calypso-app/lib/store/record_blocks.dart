/// Turning reps into the framed records the archive stores.
///
/// This is the bridge between "the device told me about a rep" and "the archive
/// keeps it": one `[u32 LE length][RepRecord]` block per rep, summary first, then
/// its trace chunks. It is the same framing the firmware writes and fixtures use,
/// so archiving from BLE, from a fixture and writing a fixture are one code path.
library;

import 'dart:typed_data';

import '../model/rep.dart';
import '../protocol/calypso.pb.dart' as pb;
import '../transport/transport.dart';

/// Trace points per stored chunk: the device's own 48, so an archived trace is
/// byte-compatible with what the device would have written.
const int pbTraceChunkSamples = 48;

/// Frames one record: `[u32 LE length][bytes]`.
Uint8List frameRecord(List<int> bytes) {
  final out = BytesBuilder();
  final length = ByteData(4)..setUint32(0, bytes.length, Endian.little);
  out.add(length.buffer.asUint8List());
  out.add(bytes);
  return out.toBytes();
}

/// Builds the record block for one rep: its summary followed by its trace
/// chunks, framed.
///
/// A rep whose trace is gone (the device's ring overwrote it) is still a valid
/// block: the summary is real data, and its `chunkCount` records what the client
/// was told at the time.
Uint8List buildRepBlock(RepSummary summary, List<TracePoint> trace) {
  final records = BytesBuilder();
  records.add(frameRecord(
    pb.RepRecord(
      summary: pb.RepSummary(
        repId: summary.repId,
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
          repId: summary.repId,
          index: start ~/ pbTraceChunkSamples,
          firstTOffsetMs: slice.first.tOffsetMs,
          // Stored records leave this 0: the summary's chunkCount describes the
          // trace, and this is not a stream.
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

/// Builds one record block per rep the transport can supply.
///
/// Convenient for small sets (fixtures, tests). A real device sync archives rep
/// by rep instead — see `archiveFromTransport` — so a long first sync makes
/// steady progress and can be resumed.
Future<Map<int, Uint8List>> recordBlocksFrom(RepTransport transport) async {
  final blocks = <int, Uint8List>{};
  for (final summary in await transport.summaries()) {
    List<TracePoint> trace;
    try {
      trace = await transport.trace(summary.repId);
    } on TraceUnavailableException {
      trace = const [];
    }
    blocks[summary.repId] = buildRepBlock(summary, trace);
  }
  return blocks;
}
