/// A transport backed by a `.pblog` log dump.
///
/// This is what makes the app runnable without hardware: the Simulator has no
/// CoreBluetooth, so this is the only way to drive the UI there, and it is what
/// the widget and golden tests use. `watchSummaries()` replays the dump with
/// configurable pacing, so the streaming UI is exercised for real rather than
/// being handed everything at once.
library;

import 'dart:async';
import 'dart:typed_data';

import '../model/rep.dart';
import '../model/uuid.dart';
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

  /// Delay between replayed summaries; `Duration.zero` delivers them as fast as
  /// the event loop allows.
  final Duration replayInterval;

  /// True if the dump ended mid-record — worth surfacing in a debug banner, so
  /// a torn capture isn't mistaken for the device losing data.
  bool get truncated => _result.truncated;

  @override
  Future<List<RepSummary>> summaries() async => summariesOf(_result.records)
      .map(RepSummary.fromProto)
      .toList();

  @override
  Stream<RepSummary> watchSummaries() async* {
    for (final summary in summariesOf(_result.records)) {
      if (replayInterval > Duration.zero) await Future<void>.delayed(replayInterval);
      yield RepSummary.fromProto(summary);
    }
  }

  @override
  Future<List<TracePoint>> trace(String uuid) async {
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
      summariesOf(_result.records).any((pb.RepSummary s) => summaryUuid(s) == uuid);

  @override
  Future<void> close() async {}
}
