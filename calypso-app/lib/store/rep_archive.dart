/// What the UI needs from the archive, and the record-format logic every
/// implementation shares.
///
/// The interface exists for the same reason [RepTransport] does: the UI depends
/// on the abstraction, so a widget test can drive it with no file I/O — which
/// matters because widget tests run under a fake clock where real asynchronous
/// I/O never completes.
///
/// Reps are keyed by UUID. The device mints a random UUID per rep, so identity
/// survives reboots, re-flashes and (later) archives from more than one device.
/// UUIDs do not sort, so the order the UI shows comes from the archive's append
/// order.
library;

import 'dart:typed_data';

import '../model/rep.dart';
import '../protocol/calypso.pb.dart' as pb;
import '../protocol/pblog.dart';
import '../sync/trace_reassembly.dart';

/// A rep in an archive: its summary, where its records live, and when the client
/// took receipt of it.
class StoredRep {
  StoredRep({
    required this.summary,
    required this.offset,
    required this.length,
    this.receivedAt,
  });

  final RepSummary summary;

  /// Byte range of this rep's whole record block (summary + chunks).
  final int offset;
  final int length;

  /// Wall-clock time the client received the summary, if known.
  ///
  /// The device has no RTC, so this is the only absolute time available — and it
  /// is the time of *receipt*, not of the lift.
  final DateTime? receivedAt;

  /// The rep's UUID, as text.
  String get uuid => summary.uuid;
}

/// The archive of record: everything the client has ever synced.
abstract class RepArchive {
  /// Reads the archive into memory (cheap: it scans framing and decodes
  /// summaries, not chunk payloads).
  Future<void> load();

  /// Reps held, in the order their records appear in the file (oldest first).
  List<StoredRep> get reps;

  /// True if the records end mid-record (a write that did not finish).
  bool get truncated;

  /// Size of the records file/backing store in bytes.
  int get sizeBytes;

  /// Whether this rep is already archived.
  bool contains(String uuid);

  /// Adds rep record blocks (summary + chunks, framed) for any rep not already
  /// held, returning how many were added. Idempotent by UUID, which is what makes
  /// re-syncing cheap.
  Future<int> appendNew(Map<String, Uint8List> recordBlocks, {DateTime? receivedAt});

  /// One rep's trace, empty if it isn't archived.
  Future<List<TracePoint>> trace(String uuid);
}

/// The result of scanning a records stream.
class BlockScan {
  BlockScan(this.reps, this.truncated, this.goodBytes);

  final List<StoredRep> reps;
  final bool truncated;

  /// Length of the prefix that ended on a record boundary.
  final int goodBytes;
}

/// Scans `[u32 LE length][RepRecord]` records into rep blocks.
///
/// A rep's records are contiguous (its summary followed by its chunks), so this
/// only has to decode summaries and can step over chunk payloads by length. A rep
/// whose block was cut short is dropped: a summary with a partial trace would be
/// worse than re-syncing it later.
BlockScan scanBlocks(Uint8List bytes, Map<String, DateTime> receivedAt) {
  final reps = <StoredRep>[];
  var goodBytes = 0;
  var truncated = false;

  var pos = 0;
  int? blockStart;
  var blockLength = 0;
  // The rep being accumulated. Its offset and length are only known once the
  // block ends, so the StoredRep is built then — getting this wrong silently
  // yields reps whose traces read back empty.
  (String, RepSummary)? pending;

  while (pos + 4 <= bytes.length) {
    final length = ByteData.sublistView(bytes, pos, pos + 4).getUint32(0, Endian.little);
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
      final uuid = summaryUuid(record.summary);
      if (uuid == null) {
        // A summary with no usable identity is not a rep.
        truncated = true;
        break;
      }
      if (pending != null) {
        reps.add(StoredRep(
          summary: pending.$2,
          offset: blockStart!,
          length: blockLength,
          receivedAt: receivedAt[pending.$1],
        ));
      }
      pending = (uuid, RepSummary.fromProto(record.summary));
      blockStart = pos;
      blockLength = 0;
    }
    blockLength += 4 + length;
    pos = start + length;
  }

  if (pending == null) return BlockScan(reps, truncated, goodBytes);
  if (truncated) {
    // The tail rep's records were cut short. Drop it, and forget its receipt
    // time so the next sync fetches it again.
    receivedAt.remove(pending.$1);
    return BlockScan(reps, truncated, goodBytes);
  }
  reps.add(StoredRep(
    summary: pending.$2,
    offset: blockStart!,
    length: blockLength,
    receivedAt: receivedAt[pending.$1],
  ));
  return BlockScan(reps, truncated, bytes.length);
}

/// Reassembles one rep's trace out of an archived record block.
List<TracePoint> assembleTraceFromBlock(Uint8List block, String uuid) {
  final dump = readDump(block);
  final chunks = traceChunksFor(dump.records, uuid);
  if (chunks.isEmpty) return const [];
  final assembler = TraceAssembler(uuid);
  for (final chunk in chunks) {
    assembler.add(chunk);
  }
  return assembler.assemble();
}
