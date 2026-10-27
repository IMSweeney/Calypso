import 'dart:io';
import 'dart:typed_data';

import 'package:calypso/model/rep.dart';
import 'package:fixnum/fixnum.dart';
import 'package:calypso/model/uuid.dart';
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:calypso/store/history_store.dart';
import 'package:calypso/transport/fixture_transport.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:protobuf/protobuf.dart' show GeneratedMessage;

import 'support/fake_transport.dart';
import 'support/test_uuid.dart';

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

pb.RepRecord summaryRecord(
  String uuid, {
  int pointCount = 10,
  int unixMs = 0,
}) => pb.RepRecord(
  summary: pb.RepSummary(
    repUuid: uuidToBytes(uuid)!,
    traceLen: pointCount,
    chunkCount: (pointCount / 4).ceil(),
    tUnixMs: Int64(unixMs),
  ),
);

Uint8List block(String uuid, {int pointCount = 10, int unixMs = 0}) {
  final points = tracePoints(pointCount);
  return Uint8List.fromList([
    ..._frameFor(summaryRecord(uuid, pointCount: pointCount, unixMs: unixMs)),
    for (final chunk in chunksFor(uuid, points))
      ..._frameFor(pb.RepRecord(trace: chunk)),
  ]);
}

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

    final trace = await store.trace(u(1));
    expect(trace.length, 10);
    expect(trace.first.tOffsetMs, 0);
    expect(trace.last.tOffsetMs, 90);
    expect(
      trace.last.pz,
      closeTo(-0.4, 1e-6),
      reason: 'positions are f32 on the wire',
    );
  });

  test('re-syncing the same reps is idempotent', () async {
    await store.appendNew({u(1): blockAt(1), u(2): blockAt(2)});
    final sizeAfterFirst = store.sizeBytes;

    expect(await store.appendNew({u(1): blockAt(1), u(2): blockAt(2)}), 0);
    expect(store.sizeBytes, sizeAfterFirst, reason: 'nothing was appended');
    expect(store.reps.length, 2);
  });

  test('the log is the only thing that decides what is held', () async {
    await store.appendNew({u(1): blockAt(1)});
    expect(store.contains(u(1)), isTrue);
    expect(store.contains(u(7)), isFalse);

    expect(await store.appendNew({u(1): blockAt(1)}), 0);
    expect(store.reps, hasLength(1));
    expect(
      File('${dir.path}/history.pblog').lengthSync(),
      HistoryFiles(dir).records.lengthSync(),
      reason: 'one file, one copy',
    );
  });

  test('the archive survives a reload and keeps growing', () async {
    await store.appendNew({u(1): blockAt(1)});

    final reopened = HistoryStore(HistoryFiles(dir));
    await reopened.load();
    expect(reopened.reps.map((r) => r.uuid), [u(1)]);
    expect((await reopened.trace(u(1))).length, 10);

    expect(await reopened.appendNew({u(2): blockAt(2)}), 1);
    final third = HistoryStore(HistoryFiles(dir));
    await third.load();
    expect(third.reps.map((r) => r.uuid), [u(1), u(2)]);
  });

  test('a rep keeps the time the device gave it, across a reload', () async {
    // 2026-03-14T09:30:00Z, to the millisecond: what the device's clock record and the rep's uptime add up to.
    const unixMs = 1_773_480_600_000;
    await store.appendNew({u(1): block(u(1), unixMs: unixMs)});
    expect(store.reps.single.recordedAt, DateTime.utc(2026, 3, 14, 9, 30));

    final reopened = HistoryStore(HistoryFiles(dir));
    await reopened.load();
    expect(
      reopened.reps.single.recordedAt,
      DateTime.utc(2026, 3, 14, 9, 30),
      reason: 'the time is in the record, so nothing has to remember it',
    );
  });

  test('a rep stored before the device had a clock has no time', () async {
    await store.appendNew({u(1): blockAt(1)});
    expect(store.reps.single.recordedAt, isNull);
  });

  test('a torn tail is tolerated and does not lose earlier reps', () async {
    await store.appendNew({u(1): blockAt(1), u(2): blockAt(2)});

    final file = HistoryFiles(dir).records;
    final bytes = await file.readAsBytes();
    await file.writeAsBytes(bytes.sublist(0, bytes.length - 30));

    final reopened = HistoryStore(HistoryFiles(dir));
    await reopened.load();
    expect(reopened.truncated, isTrue);
    expect(reopened.reps.map((r) => r.uuid), [
      u(1),
    ], reason: 'rep 1 is intact; rep 2 was the one being written');
    expect((await reopened.trace(u(1))).length, 10);

    expect(await reopened.appendNew({u(2): blockAt(2), u(3): blockAt(3)}), 2);
    expect(reopened.reps.map((r) => r.uuid), [u(1), u(2), u(3)]);
    expect((await reopened.trace(u(3))).length, 10);
  });

  test('a fixture can be archived and read back like device data', () async {
    final fixture = File(
      'assets/fixtures/synthetic-squat.pblog',
    ).readAsBytesSync();
    final transport = FixtureTransport.fromBytes(Uint8List.fromList(fixture));
    final summaries = await transport.summaries();

    for (final summary in summaries) {
      final points = await transport.trace(summary.uuid!);
      // The summary goes in as it arrived: its time is part of the record the device sent.
      final records = <int>[
        ..._frameFor(pb.RepRecord(summary: summary)),
        for (final chunk in chunksFor(summary.uuid!, points))
          ..._frameFor(pb.RepRecord(trace: chunk)),
      ];
      await store.appendNew({summary.uuid!: Uint8List.fromList(records)});
    }

    expect(store.reps.length, 3);
    final trace = await store.trace(summaries[1].uuid!);
    expect(trace.length, summaries[1].traceLen);
    expect(
      store.reps.first.recordedAt,
      isNotNull,
      reason: "the fixture carries the clock its reps are dated by",
    );
    expect(
      store.reps.first.recordedAt!.toUtc().year,
      2026,
      reason: "dated from the fixture's fixed epoch, not from when it was read",
    );
  });

  test('a missing rep returns an empty trace, not an exception', () async {
    await store.appendNew({u(1): blockAt(1)});
    expect(await store.trace(u(99)), isEmpty);
  });
}
