library;

import 'dart:async';
import 'dart:typed_data';

import '../model/rep.dart';
import '../protocol/calypso.pb.dart' as pb;
import '../protocol/pblog.dart';
import '../sync/trace_reassembly.dart';
import 'transport.dart';

class FixtureTransport implements RepTransport {
  FixtureTransport.fromBytes(
    Uint8List bytes, {
    this.replayInterval = const Duration(milliseconds: 30),
  }) : _result = readDump(bytes);

  FixtureTransport(this._result, {this.replayInterval = const Duration(milliseconds: 30)});

  final DumpReadResult _result;
  final Duration replayInterval;

  bool get truncated => _result.truncated;

  @override
  Future<List<pb.RepSummary>> summaries() async => summariesOf(_result.records);

  @override
  Stream<pb.RepSummary> watchSummaries() async* {
    for (final summary in summariesOf(_result.records)) {
      if (replayInterval > Duration.zero) await Future<void>.delayed(replayInterval);
      yield summary;
    }
  }

  @override
  Future<List<pb.TraceSample>> trace(String uuid) async {
    final chunks = traceChunksFor(_result.records, uuid);
    if (chunks.isEmpty) {
      throw TraceUnavailableException(
        uuid,
        _hasSummary(uuid) ? 'trace overwritten on the device ring' : 'no such rep',
      );
    }
    final assembler = TraceAssembler(uuid);
    for (final chunk in chunks) {
      assembler.add(chunk);
    }
    return assembler.assemble();
  }

  bool _hasSummary(String uuid) =>
      summariesOf(_result.records).any((pb.RepSummary s) => s.uuid == uuid);

  @override
  Future<void> close() async {}
}
