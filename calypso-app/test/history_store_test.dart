// The archive of record: append-only .pblog records, dedupe on re-sync, traces
// read back by rep, and tolerance of a torn tail.

import 'dart:io';
import 'dart:typed_data';

import 'package:calypso/model/rep.dart';
import 'package:calypso/model/uuid.dart';
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:calypso/store/history_store.dart';
import 'package:calypso/transport/fixture_transport.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:protobuf/protobuf.dart' show GeneratedMessage;

import 'support/fake_transport.dart';
import 'support/test_uuid.dart';

/// `[u32 LE length][record]`, the .pblog framing.
List<int> _frameFor(GeneratedMessage record) {
  final bytes = record.writeToBuffer();
  return [
    bytes.length & 0xFF,
    (bytes.length >> 8) & 0xFF,
    (bytes.length >> 16) & 0xFF,
    (bytes.length >> 24) & 0xFF,
    ...bytes,
  ];
}

/// A summary record for the rep with [uuid], with a trace length consistent
/// with the chunks the helpers write.
pb.RepRecord summaryRecord(String uuid, {int pointCount = 10}) => pb.RepRecord(
      summary: pb.RepSummary(
        repUuid: uuidToBytes(uuid)!,
        traceLen: pointCount,
        chunkCount: (pointCount / 4).ceil(),
      ),
    );

/// Encodes a rep the way the device does — a summary record followed by one
/// record per trace chunk, each a `RepRecord` whose body is set — then framed.
Uint8List block(String uuid, {int pointCount = 10}) {
  final points = tracePoints(pointCount);
  return Uint8List.fromList([
    ..._frameFor(summaryRecord(uuid, pointCount: pointCount)),
    for (final chunk in chunksFor(uuid, points)) ..._frameFor(pb.RepRecord(trace: chunk)),
  ]);
}

/// [block] for the test rep named by [index], matching `u(index)`.
Uint8List blockAt(int index, {int pointCount = 10}) =>
    block(u(index), pointCount: pointCount);

void main() {
  late Directory dir;
  late HistoryStore store;

  setUp(() async {
    dir = await Directory.systemTemp.createTemp('calypso_history');
    store = HistoryStore(HistoryFiles(dir));
    await store.load();
  });

  tearDown(() async {
    if (dir.existsSync()) await dir.delete(recursive: true);
  });

  test('an empty archive loads cleanly', () async {
    expect(store.reps, isEmpty);
    expect(store.sizeBytes, 0);
    expect(store.truncated, isFalse);
  });

  test('records append and read back', () async {
    final added = await store.appendNew({u(1): blockAt(1), u(2): blockAt(2)});
    expect(added, 2);
    expect(store.reps.map((r) => r.uuid), [u(1), u(2)]);
    expect(store.sizeBytes, greaterThan(0));

    // The trace comes back out of the file, in order.
    final trace = await store.trace(u(1));
    expect(trace.length, 10);
    expect(trace.first.tOffsetMs, 0);
    expect(trace.last.tOffsetMs, 90);
    expect(trace.last.pz, closeTo(-0.4, 1e-6), reason: 'positions are f32 on the wire');
  });

  test('re-syncing the same reps is idempotent', () async {
    await store.appendNew({u(1): blockAt(1), u(2): blockAt(2)});
    final sizeAfterFirst = store.sizeBytes;

    // The device re-streams everything on each connection.
    expect(await store.appendNew({u(1): blockAt(1), u(2): blockAt(2)}), 0);
    expect(store.sizeBytes, sizeAfterFirst, reason: 'nothing was appended');
    expect(store.reps.length, 2);
  });

  test('the log decides what is held, not the receipt times beside it', () async {
    // The rule this archive is built on: `history.pblog` is the source of truth and
    // `received.json` is decoration. Losing the side-car costs timestamps — and must
    // never make a rep the log already holds look new, because appending it again
    // would put two copies of it in the log.
    await store.appendNew({u(1): blockAt(1)}, receivedAt: DateTime.utc(2026, 10, 18));
    expect(store.contains(u(1)), isTrue);
    expect(store.reps.single.receivedAt, isNotNull);

    await HistoryFiles(dir).receiptTimes.delete();

    final reopened = HistoryStore(HistoryFiles(dir));
    await reopened.load();
    expect(reopened.reps, hasLength(1), reason: 'the rep is in the log');
    expect(reopened.contains(u(1)), isTrue, reason: 'so it is still held');
    expect(reopened.reps.single.receivedAt, isNull, reason: 'only its time is gone');

    // Which is what makes a re-sync a no-op rather than a duplicate.
    expect(await reopened.appendNew({u(1): blockAt(1)}), 0);
    expect(reopened.reps, hasLength(1));
    expect(File('${dir.path}/history.pblog').lengthSync(),
        HistoryFiles(dir).records.lengthSync(), reason: 'one file, one copy');
  });

  test('a receipt time for a rep the log does not hold does not make it exist', () async {
    // The other direction, and the reason it is worth pruning: the side-car cannot
    // conjure a rep into the list, and cannot accumulate uuids the log has no record
    // of either.
    final times = HistoryFiles(dir).receiptTimes;
    await times.writeAsString('{"${u(7)}": "2026-10-18T00:00:00.000Z"}');

    await store.load();
    expect(store.contains(u(7)), isFalse);
    expect(store.reps, isEmpty);
    expect(await times.readAsString(), isNot(contains(u(7))),
        reason: 'pruned on load, and the file follows the log');
  });

  test('the archive survives a reload and keeps growing', () async {
    await store.appendNew({u(1): blockAt(1)});

    final reopened = HistoryStore(HistoryFiles(dir));
    await reopened.load();
    expect(reopened.reps.map((r) => r.uuid), [u(1)]);
    expect((await reopened.trace(u(1))).length, 10);

    // A new rep appends to the same file.
    expect(await reopened.appendNew({u(2): blockAt(2)}), 1);
    final third = HistoryStore(HistoryFiles(dir));
    await third.load();
    expect(third.reps.map((r) => r.uuid), [u(1), u(2)]);
  });

  test('receipt times are recorded and survive a reload', () async {
    final when = DateTime(2026, 3, 14, 9, 30);
    await store.appendNew({u(1): blockAt(1)}, receivedAt: when);
    expect(store.reps.single.receivedAt, when);

    final reopened = HistoryStore(HistoryFiles(dir));
    await reopened.load();
    expect(reopened.reps.single.receivedAt, when,
        reason: 'the device has no RTC, so receipt time is the only absolute time');
  });

  test('a torn tail is tolerated and does not lose earlier reps', () async {
    await store.appendNew({u(1): blockAt(1), u(2): blockAt(2)});

    // Simulate a write that did not finish: chop bytes off the end.
    final file = HistoryFiles(dir).records;
    final bytes = await file.readAsBytes();
    await file.writeAsBytes(bytes.sublist(0, bytes.length - 30));

    final reopened = HistoryStore(HistoryFiles(dir));
    await reopened.load();
    expect(reopened.truncated, isTrue);
    expect(reopened.reps.map((r) => r.uuid), [u(1)],
        reason: 'rep 1 is intact; rep 2 was the one being written');
    expect((await reopened.trace(u(1))).length, 10);

    // The archive must stay usable: the torn tail is discarded rather than left
    // to desync the framing, and the rep that was cut short is re-fetched by the
    // next sync because it was never recorded as received.
    expect(await reopened.appendNew({u(2): blockAt(2), u(3): blockAt(3)}), 2);
    expect(reopened.reps.map((r) => r.uuid), [u(1), u(2), u(3)]);
    expect((await reopened.trace(u(3))).length, 10);
  });

  test('a fixture can be archived and read back like device data', () async {
    // The whole point of one format: a captured fixture and the archive are the
    // same thing.
    final fixture = File('assets/fixtures/synthetic-squat.pblog').readAsBytesSync();
    final transport = FixtureTransport.fromBytes(Uint8List.fromList(fixture));
    final summaries = await transport.summaries();

    for (final summary in summaries) {
      final points = await transport.trace(summary.uuid!);
      final records = <int>[
        ..._frameFor(summaryRecord(summary.uuid!, pointCount: points.length)),
        for (final chunk in chunksFor(summary.uuid!, points))
          ..._frameFor(pb.RepRecord(trace: chunk)),
      ];
      await store.appendNew({summary.uuid!: Uint8List.fromList(records)});
    }

    expect(store.reps.length, 3);
    // The second summary in the fixture, keyed by the uuid it carries — the
    // generator's, not a test-minted one.
    final trace = await store.trace(summaries[1].uuid!);
    expect(trace.length, summaries[1].traceLen);
  });

  test('a missing rep returns an empty trace, not an exception', () async {
    await store.appendNew({u(1): blockAt(1)});
    expect(await store.trace(u(99)), isEmpty);
  });
}
