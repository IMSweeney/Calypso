// Decode round-trip over every generated fixture.
//
// The fixtures come from the firmware's own protobuf encoders
// (`cargo run -p calypso-core --example gen_fixtures`), so this test is a real
// check of the wire format rather than of a hand-written sample.

import 'dart:io';
import 'dart:typed_data';

import 'package:calypso/protocol/pblog.dart';
import 'package:flutter_test/flutter_test.dart';

Uint8List fixture(String name) =>
    File('assets/fixtures/$name.pblog').readAsBytesSync();

void main() {
  test('a normal set decodes into three complete reps', () {
    final result = readDump(fixture('synthetic-squat'));
    expect(result.truncated, isFalse);

    final summaries = summariesOf(result.records);
    expect(summaries.length, 3);

    for (var i = 0; i < summaries.length; i++) {
      final summary = summaries[i];
      expect(summary.repId, i + 1, reason: 'rep ids are numbered from 1');
      expect(summary.decimated, isFalse);

      // Each summary is followed by the chunks it promises, in order, holding
      // exactly the trace length it reports.
      final chunks = traceChunksFor(result.records, summary.repId);
      expect(chunks.length, summary.chunkCount);
      var points = 0;
      for (var c = 0; c < chunks.length; c++) {
        expect(chunks[c].index, c);
        expect(chunks[c].repId, summary.repId);
        points += chunks[c].samples.length;
      }
      expect(points, summary.traceLen);
    }

    // A 0.4 m squat: the summaries should show roughly that much travel.
    for (final s in summaries) {
      expect(s.rom, closeTo(0.4, 0.15));
      expect(s.durationMs, greaterThan(0));
      expect(s.peakVelocity, greaterThan(0));
    }

    // Chunk timestamps are relative to the rep's first trace point.
    final first = traceChunksFor(result.records, 1).first;
    expect(first.samples.first.tOffsetMs, 0);
    expect(first.samples.last.tOffsetMs, greaterThan(0));

    // Position and orientation survive the fixed-point round trip. The first
    // sample is at rest, so the travel shows up later in the trace.
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
    expect(summary.traceLen, 1);
    expect(summary.chunkCount, 1);

    final chunks = traceChunksFor(result.records, summary.repId);
    expect(chunks.single.samples.length, 1);
  });

  test('a decimated long rep keeps all 512 stored points', () {
    final result = readDump(fixture('long-decimated'));
    final summary = summariesOf(result.records).single;

    expect(summary.decimated, isTrue);
    expect(summary.traceLen, 512);

    final chunks = traceChunksFor(result.records, summary.repId);
    final points = chunks.fold<int>(0, (n, c) => n + c.samples.length);
    expect(points, 512);

    // Decimation anchors on the last sample, so the rep's end is stored.
    final all = chunks.expand((c) => c.samples).toList();
    expect(all.last.tOffsetMs, greaterThan(all[all.length - 2].tOffsetMs));
  });

  test('a rep whose trace fell off the device ring has no chunks', () {
    final result = readDump(fixture('ring-dropped'));
    final summary = summariesOf(result.records).single;

    expect(summary.repId, 7);
    expect(summary.chunkCount, 3, reason: 'the device still claims chunks');
    expect(traceChunksFor(result.records, 7), isEmpty,
        reason: 'the app must show "trace unavailable", not crash');
  });

  test('extreme values decode without loss', () {
    final result = readDump(fixture('extreme-values'));
    expect(result.truncated, isFalse);

    final summary = summariesOf(result.records).single;
    expect(summary.repId, 99);
    expect(summary.traceLen, 4);
    expect(summary.decimated, isTrue);

    final samples = traceChunksFor(result.records, 99).single.samples;
    expect(samples.length, 4);
    expect(samples[0].ax, -32768); // i16::MIN survives the sint32 encoding
    expect(samples[0].qw, -32768);
  });

  test('a torn dump reports truncation but keeps what it read', () {
    final result = readDump(fixture('truncated'));

    expect(result.truncated, isTrue);
    // The cut fixture is one rep: a summary plus four chunks, with the last
    // frame clipped, so exactly four records survive.
    expect(result.records.length, 4);
    expect(summariesOf(result.records).single.repId, 1);
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
