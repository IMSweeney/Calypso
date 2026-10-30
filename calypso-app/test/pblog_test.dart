import 'dart:io';
import 'dart:typed_data';

import 'package:calypso/model/rep.dart';
import 'package:calypso/model/uuid.dart';
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:calypso/protocol/pblog.dart';
import 'package:flutter_test/flutter_test.dart';

import 'support/test_uuid.dart';

Uint8List fixture(String name) =>
    File('assets/fixtures/$name.pblog').readAsBytesSync();

/// One record in the dump framing: `[u32 LE length][record bytes]`.
List<int> _framed(pb.RepRecord record) {
  final bytes = record.writeToBuffer();
  return [
    bytes.length & 0xFF,
    (bytes.length >> 8) & 0xFF,
    (bytes.length >> 16) & 0xFF,
    (bytes.length >> 24) & 0xFF,
    ...bytes,
  ];
}

void main() {
  test('a normal set decodes into three complete reps', () {
    final result = readDump(fixture('synthetic-squat'));
    expect(result.truncated, isFalse);

    final summaries = summariesOf(result.records);
    expect(summaries.length, 3);

    for (var i = 0; i < summaries.length; i++) {
      final summary = summaries[i];
      expect(summary.uuid, u(i), reason: 'rep $i carries its generator uuid');
      expect(summary.decimated, isFalse);
      expect(
        summary.calibrated,
        isTrue,
        reason: 'fixtures are recorded calibrated',
      );

      final chunks = traceChunksFor(result.records, u(i));
      expect(chunks.length, summary.chunkCount);
      var points = 0;
      for (var c = 0; c < chunks.length; c++) {
        expect(chunks[c].index, c);
        expect(chunkUuid(chunks[c]), u(i));
        points += chunks[c].samples.length;
      }
      expect(points, summary.traceLen);
    }

    for (final s in summaries) {
      expect(s.rom, closeTo(0.4, 0.15));
      expect(s.durationMs, greaterThan(0));
      expect(s.peakVelocity, greaterThan(0));
    }

    final first = traceChunksFor(result.records, u(0)).first;
    expect(first.samples.first.tOffsetMs, 0);
    expect(first.samples.last.tOffsetMs, greaterThan(0));

    final depth = first.samples
        .map((s) => s.pz.abs())
        .reduce((a, b) => a > b ? a : b);
    expect(depth, greaterThan(0.1), reason: 'the path should show real travel');
    for (final sample in first.samples) {
      expect(sample.qw, isNot(0));
    }
  });

  test('a one-point trace decodes', () {
    final result = readDump(fixture('single-point'));
    expect(result.truncated, isFalse);

    final summary = summariesOf(result.records).single;
    expect(summary.uuid, u(1));
    expect(summary.traceLen, 1);
    expect(summary.chunkCount, 1);

    final chunks = traceChunksFor(result.records, u(1));
    expect(chunks.single.samples.length, 1);
  });

  test('a decimated long rep keeps all 512 stored points', () {
    final result = readDump(fixture('long-decimated'));
    final summary = summariesOf(result.records).single;

    expect(summary.uuid, u(1));
    expect(summary.decimated, isTrue);
    expect(summary.traceLen, 512);

    final chunks = traceChunksFor(result.records, u(1));
    final points = chunks.fold<int>(0, (n, c) => n + c.samples.length);
    expect(points, 512);

    final all = chunks.expand((c) => c.samples).toList();
    expect(all.last.tOffsetMs, greaterThan(all[all.length - 2].tOffsetMs));
  });

  test('a rep whose trace fell off the device ring has no chunks', () {
    final result = readDump(fixture('ring-dropped'));
    final summary = summariesOf(result.records).single;

    expect(summary.uuid, u(7));
    expect(summary.chunkCount, 3, reason: 'the device still claims chunks');
    expect(
      traceChunksFor(result.records, u(7)),
      isEmpty,
      reason: 'the app must show "trace unavailable", not crash',
    );
  });

  test('extreme values decode without loss', () {
    final result = readDump(fixture('extreme-values'));
    expect(result.truncated, isFalse);

    final summary = summariesOf(result.records).single;
    expect(summary.uuid, u(99));
    expect(summary.traceLen, 4);
    expect(summary.decimated, isTrue);

    final samples = traceChunksFor(result.records, u(99)).single.samples;
    expect(samples.length, 4);
    expect(samples[0].ax, -32768);
    expect(samples[0].qw, -32768);
  });

  test('a torn dump reports truncation but keeps what it read', () {
    final result = readDump(fixture('truncated'));

    expect(result.truncated, isTrue);
    expect(
      result.records.where((r) => r.hasClock()).length,
      1,
      reason: "a dump opens with the boot's clock, which is whole",
    );
    final summary = summariesOf(result.records).single;
    expect(summary.uuid, u(0));
    expect(
      result.records.where((r) => r.hasTrace()).length,
      lessThan(summary.chunkCount),
      reason: 'the chunk being written is the one that was cut short',
    );
  });

  test('a dump dates a rep from the newest clock record before it', () {
    // The rule the device applies by record id, which a dump can only apply by order: the epoch in force when the
    // summary was read dates it, and a boot that never had a clock cannot borrow the one before it.
    const epochS = 1_767_225_600;
    Uint8List dump(List<pb.RepRecord> records) =>
        Uint8List.fromList([for (final record in records) ..._framed(record)]);
    // A stored summary carries its uptime, not a time: the boot's epoch is what dates it.
    pb.RepRecord rep(int tStartMs) => pb.RepRecord(
      summary: pb.RepSummary(repUuid: uuidToBytes(u(0))!, tStartMs: tStartMs),
    );

    final beforeAnyClock = readDump(
      dump([rep(0), pb.RepRecord(clock: pb.BootClock(epochS: 0))]),
    );
    expect(
      summariesOf(beforeAnyClock.records).single.recordedAt,
      isNull,
      reason: 'no clock had been set when it was stored',
    );

    final dated = readDump(
      dump([rep(0), pb.RepRecord(clock: pb.BootClock(epochS: epochS)), rep(0)]),
    );
    final times = summariesOf(dated.records).map((s) => s.recordedAt).toList();
    expect(times.first, isNull, reason: 'stored before the clock was set');
    expect(times.last, DateTime.utc(2026, 1, 1));

    final laterBoot = readDump(
      dump([
        pb.RepRecord(clock: pb.BootClock(epochS: epochS)),
        rep(4_000),
        pb.RepRecord(clock: pb.BootClock(epochS: 0)),
        rep(4_000),
      ]),
    );
    final across = summariesOf(laterBoot.records).toList();
    expect(
      across.first.tUnixMs.toInt(),
      epochS * 1000 + 4_000,
      reason: 'the epoch plus the rep uptime, to the millisecond',
    );
    expect(
      across.last.tUnixMs.toInt(),
      0,
      reason: 'the next boot had no clock of its own, and does not inherit one',
    );
  });

  group('a real capture', () {
    test('three reps the device recorded read back whole', () {
      final result = readDump(fixture('real-reps'));
      expect(result.truncated, isFalse);
      final summaries = summariesOf(result.records).toList();
      expect(summaries.length, 3);

      for (final s in summaries) {
        expect(s.uuid, isNotNull);
        expect(s.recordedAt, isNotNull, reason: 'a real rep carries its time');
        expect(s.timeOfDay, matches(RegExp(r'^\d{2}:\d{2}:\d{2}$')));
        expect(s.traceLen, greaterThan(0));

        final chunks = traceChunksFor(result.records, s.uuid!);
        expect(chunks.length, s.chunkCount);
        expect(
          chunks.fold<int>(0, (n, c) => n + c.samples.length),
          s.traceLen,
          reason: 'every point the summary counts is in the chunks',
        );
      }

      // The three were picked to be different shapes, so a regression in decimation or in the trace
      // reassembly cannot pass by looking like the others.
      expect(summaries.map((s) => s.durationMs).toList()..sort(), [
        1369,
        5112,
        8220,
      ]);
      expect(summaries.where((s) => s.decimated), hasLength(2));
      expect(
        summaries.every((s) => s.rom > 0.2 && s.rom < 0.7),
        isTrue,
        reason: 'a rep moves the bar a little, not a wiggle and not a fall',
      );
    });

    test('a wiggle reads back, and is smaller than any rep', () {
      // What `MIN_ROM_M` keeps out of the log: the device closed a span on motion and rest, and the sensor
      // travelled 3 cm. The app is never sent one — this fixture pins the shape of a non-rep.
      final result = readDump(fixture('real-wobble'));
      final summaries = summariesOf(result.records).toList();
      expect(summaries, hasLength(1));
      final wobble = summaries.single;
      expect(wobble.recordedAt, isNotNull);
      expect(wobble.rom, lessThan(0.1));
      expect(wobble.durationMs, greaterThan(3000));

      final real = summariesOf(readDump(fixture('real-reps')).records).toList();
      final smallest = real.map((s) => s.rom).reduce((a, b) => a < b ? a : b);
      expect(wobble.rom, lessThan(smallest));
    });
  });

  test('an empty dump is valid and empty', () {
    final result = readDump(fixture('empty'));
    expect(result.records, isEmpty);
    expect(result.truncated, isFalse);
  });

  test('trailing garbage shorter than a frame is reported', () {
    final bytes = Uint8List.fromList([1, 2, 3]);
    final result = readDump(bytes);
    expect(result.records, isEmpty);
    expect(result.truncated, isTrue);
  });
}
