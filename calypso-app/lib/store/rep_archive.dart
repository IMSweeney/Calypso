library;

import 'dart:typed_data';

import '../model/rep.dart';
import '../protocol/calypso.pb.dart' as pb;
import '../protocol/pblog.dart';
import '../sync/trace_reassembly.dart';

class StoredRep {
  StoredRep({
    required this.summary,
    required this.offset,
    required this.length,
    this.receivedAt,
  });

  final pb.RepSummary summary;

  final int offset;
  final int length;
  final DateTime? receivedAt;

  /// Set: `scanBlocks` refuses a summary whose uuid does not parse.
  String get uuid => summary.uuid!;
}

abstract class RepArchive {
  Future<void> load();
  List<StoredRep> get reps;
  bool get truncated;
  int get sizeBytes;
  bool contains(String uuid);

  Future<int> appendNew(
    Map<String, Uint8List> recordBlocks, {
    DateTime? receivedAt,
  });

  Future<List<pb.TraceSample>> trace(String uuid);
}

class BlockScan {
  BlockScan(this.reps, this.truncated, this.goodBytes);

  final List<StoredRep> reps;
  final bool truncated;

  final int goodBytes;
}

BlockScan scanBlocks(Uint8List bytes, Map<String, DateTime> receivedAt) {
  final reps = <StoredRep>[];
  var goodBytes = 0;
  var truncated = false;

  var pos = 0;
  int? blockStart;
  var blockLength = 0;
  (String, pb.RepSummary)? pending;

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
    if (record.hasSummary()) {
      goodBytes = pos;
      final uuid = record.summary.uuid;
      if (uuid == null) {
        truncated = true;
        break;
      }
      if (pending != null) {
        reps.add(
          StoredRep(
            summary: pending.$2,
            offset: blockStart!,
            length: blockLength,
            receivedAt: receivedAt[pending.$1],
          ),
        );
      }
      pending = (uuid, record.summary);
      blockStart = pos;
      blockLength = 0;
    }
    blockLength += 4 + length;
    pos = start + length;
  }

  if (pending == null) return BlockScan(reps, truncated, goodBytes);
  if (truncated) {
    receivedAt.remove(pending.$1);
    return BlockScan(reps, truncated, goodBytes);
  }
  reps.add(
    StoredRep(
      summary: pending.$2,
      offset: blockStart!,
      length: blockLength,
      receivedAt: receivedAt[pending.$1],
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
