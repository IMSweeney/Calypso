library;

import 'dart:typed_data';

import 'package:fixnum/fixnum.dart';

import '../model/rep.dart';
import '../protocol/calypso.pb.dart' as pb;
import '../protocol/pblog.dart';
import '../sync/trace_reassembly.dart';

class StoredRep {
  StoredRep({
    required this.summary,
    required this.offset,
    required this.length,
    this.hasTrace = false,
  });

  final pb.RepSummary summary;

  final int offset;
  final int length;

  /// Whether the app stored the trace with the rep.
  ///
  /// A rep that has none was archived anyway — the summary is the part the device can always give — so this is what tells a fetched trace from a fetch that failed (see `archiveRep`).
  final bool hasTrace;

  /// Set: `scanBlocks` refuses a summary whose uuid does not parse.
  String get uuid => summary.uuid!;

  /// When the rep happened: the device's clock, which every summary it sends carries.
  ///
  /// Null only for records that predate the device having a clock at all — the app's `received.json` is gone, so there is nothing to fall back to, and a rep with no time is on no day for the list to show.
  DateTime? get recordedAt => summary.recordedAt;
}

abstract class RepArchive {
  Future<void> load();
  List<StoredRep> get reps;
  bool get truncated;
  int get sizeBytes;
  bool contains(String uuid);

  Future<int> appendNew(Map<String, Uint8List> recordBlocks);

  Future<List<pb.TraceSample>> trace(String uuid);
}

class BlockScan {
  BlockScan(this.reps, this.truncated, this.goodBytes);

  final List<StoredRep> reps;
  final bool truncated;

  final int goodBytes;
}

BlockScan scanBlocks(Uint8List bytes) {
  final reps = <StoredRep>[];
  var goodBytes = 0;
  var truncated = false;

  var pos = 0;
  int? blockStart;
  var blockLength = 0;
  var blockHasTrace = false;
  (String, pb.RepSummary)? pending;
  // The newest clock record so far: a capture read as history dates its reps from it, the way a fixture is dated
  // by [`summariesOf`]. An archive the app itself wrote has the times on the summaries already, and no clocks.
  var epochS = 0;

  while (pos + 4 <= bytes.length) {
    final length = ByteData.sublistView(
      bytes,
      pos,
      pos + 4,
    ).getUint32(0, Endian.little);
    final start = pos + 4;
    if (length == 0 || start + length > bytes.length) {
      truncated = true;
      break;
    }
    pb.RepRecord record;
    try {
      record = pb.RepRecord.fromBuffer(bytes.sublist(start, start + length));
    } catch (_) {
      truncated = true;
      break;
    }
    // A clock is not part of a rep: it sets the epoch and belongs to no block, or a capture's boot record would
    // be counted into whatever rep precedes it.
    if (record.hasClock()) {
      epochS = record.clock.epochS;
      pos = start + length;
      continue;
    }
    if (record.hasSummary()) {
      goodBytes = pos;
      final uuid = record.summary.uuid;
      if (uuid == null) {
        truncated = true;
        break;
      }
      if (epochS != 0) {
        record.summary.tUnixMs = Int64(epochS * 1000 + record.summary.tStartMs);
      }
      if (pending != null) {
        reps.add(
          StoredRep(
            summary: pending.$2,
            offset: blockStart!,
            length: blockLength,
            hasTrace: blockHasTrace,
          ),
        );
      }
      pending = (uuid, record.summary);
      blockStart = pos;
      blockLength = 0;
      blockHasTrace = false;
    }
    if (record.hasTrace()) blockHasTrace = true;
    blockLength += 4 + length;
    pos = start + length;
  }

  if (pending == null) return BlockScan(reps, truncated, goodBytes);
  if (truncated) return BlockScan(reps, truncated, goodBytes);
  reps.add(
    StoredRep(
      summary: pending.$2,
      offset: blockStart!,
      length: blockLength,
      hasTrace: blockHasTrace,
    ),
  );
  return BlockScan(reps, truncated, bytes.length);
}

List<pb.TraceSample> assembleTraceFromBlock(Uint8List block, String uuid) {
  final dump = readDump(block);
  final chunks = traceChunksFor(dump.records, uuid);
  if (chunks.isEmpty) return const [];
  final assembler = TraceAssembler(uuid);
  for (final chunk in chunks) {
    assembler.add(chunk);
  }
  return assembler.assemble();
}
