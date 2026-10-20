import 'package:calypso/sync/trace_reassembly.dart';
import 'package:flutter_test/flutter_test.dart';

import 'support/fake_transport.dart';
import 'support/test_uuid.dart';

void main() {
  final points = tracePoints(10);
  final chunks = chunksFor(u(1), points);

  test('chunks assemble in order', () {
    final assembler = TraceAssembler(u(1));
    for (final chunk in chunks) {
      expect(assembler.add(chunk), isTrue);
    }
    expect(assembler.chunkCount, 3);
    expect(assembler.indices, [0, 1, 2]);
    expect(assembler.missingIndices(), isEmpty);
    expect(assembler.assemble().length, 10);
    expect(assembler.assemble().first.tOffsetMs, 0);
    expect(assembler.assemble().last.tOffsetMs, 90);
  });

  test('chunks assemble out of order', () {
    final assembler = TraceAssembler(u(1));
    for (final i in [2, 0, 1]) {
      assembler.add(chunks[i]);
    }
    final assembled = assembler.assemble();
    expect(assembled.length, 10);
    expect(
      assembled.map((p) => p.tOffsetMs).toList(),
      [for (var i = 0; i < 10; i++) i * 10],
    );
  });

  test('a repeated chunk is idempotent', () {
    final assembler = TraceAssembler(u(1));
    assembler.add(chunks[0]);
    assembler.add(chunks[0]);
    assembler.add(chunks[2]);
    expect(assembler.chunkCount, 2);
    expect(assembler.missingIndices(), [1]);
    expect(assembler.assemble().length, 6, reason: 'the repeat added nothing');
  });

  test('chunks for another rep are ignored', () {
    final assembler = TraceAssembler(u(1));
    expect(assembler.add(chunksFor(u(2), tracePoints(4)).single), isFalse);
    expect(assembler.isEmpty, isTrue);
  });

  test('missing chunk indices are reported for re-request', () {
    final assembler = TraceAssembler(u(1));
    assembler.add(chunks[0]);
    assembler.add(chunks[2]);
    expect(assembler.missingIndices(), [1]);
  });

  test('an assembled trace matches the input', () {
    final assembled = FakeTransport.reassemble(u(1), chunks);
    expect(assembled.length, points.length);
    for (var i = 0; i < points.length; i++) {
      expect(assembled[i].tOffsetMs, points[i].tOffsetMs);
      expect(assembled[i].pz, points[i].pz);
      expect(assembled[i].az, points[i].az);
    }
  });
}
