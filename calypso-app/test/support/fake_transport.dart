/// A scriptable [RepTransport] for tests: no device, no fixtures, no timing.
///
/// Everything the sync client has to survive can be staged here: summaries
/// arriving live, a trace that the device has forgotten, a trace that never
/// answers, and chunks that arrive out of order or twice.
library;

import 'dart:async';

import 'package:calypso/model/rep.dart';
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:calypso/sync/trace_reassembly.dart';
import 'package:calypso/transport/transport.dart';

/// Builds a summary with sensible defaults, so tests only state what matters.
RepSummary summary(
  int repId, {
  int durationMs = 1400,
  double rom = 0.4,
  double peakVelocity = 0.7,
  int traceLen = 140,
  int chunkCount = 3,
  bool decimated = false,
}) =>
    RepSummary(
      repId: repId,
      tStartMs: repId * 10_000,
      tEndMs: repId * 10_000 + durationMs,
      durationMs: durationMs,
      rom: rom,
      peakVelocity: peakVelocity,
      meanVelocity: peakVelocity / 3,
      peakAccel: 2.5,
      rollMinDeg: -1,
      rollMaxDeg: 1,
      pitchMinDeg: -2,
      pitchMaxDeg: 2,
      traceLen: traceLen,
      chunkCount: chunkCount,
      decimated: decimated,
    );

/// Builds a trace of [count] points, 10 ms apart, sinking to -0.4 m.
List<TracePoint> tracePoints(int count, {int startMs = 0}) => [
      for (var i = 0; i < count; i++)
        TracePoint(
          tOffsetMs: startMs + i * 10,
          x: 0,
          y: 0,
          z: -0.4 * i / (count == 1 ? 1 : count - 1),
          ax: 0,
          ay: 0,
          az: -100,
          qw: 16384,
          qx: 0,
          qy: 0,
          qz: 0,
        ),
    ];

/// Splits points into protocol chunks of [perChunk], the way the device does.
List<pb.TraceChunk> chunksFor(int repId, List<TracePoint> points, {int perChunk = 4}) {
  final chunks = <pb.TraceChunk>[];
  for (var start = 0; start < points.length; start += perChunk) {
    final slice = points.sublist(start, (start + perChunk).clamp(0, points.length));
    chunks.add(pb.TraceChunk(
      repId: repId,
      index: chunks.length,
      firstTOffsetMs: slice.first.tOffsetMs,
    )..samples.addAll([
        for (final p in slice)
          pb.TraceSample(
            tOffsetMs: p.tOffsetMs,
            px: p.x,
            py: p.y,
            pz: p.z,
            ax: p.ax,
            ay: p.ay,
            az: p.az,
            qw: p.qw,
            qx: p.qx,
            qy: p.qy,
            qz: p.qz,
          ),
      ]));
  }
  return chunks;
}

class FakeTransport implements RepTransport {
  FakeTransport({
    List<RepSummary>? stored,
    this.tracePoints = const {},
    this.tracesMissing = const {},
    this.traceDelay = Duration.zero,
    this.failSummaries = false,
  }) : _stored = stored ?? [];

  List<RepSummary> _stored;
  final Map<int, List<TracePoint>> tracePoints;

  /// Rep ids whose trace the device no longer has (the ring overwrote them).
  final Set<int> tracesMissing;

  /// Simulates a device that accepts a request and never answers.
  final Duration traceDelay;
  final bool failSummaries;

  final _live = StreamController<RepSummary>.broadcast();
  int summariesCalls = 0;
  int traceCalls = 0;

  /// What the transport is holding now — mutate to simulate a device-side wrap.
  void setStored(List<RepSummary> stored) => _stored = stored;

  /// Pushes a summary as if the device had just recorded it.
  void emit(RepSummary s) {
    _stored = [..._stored, s];
    _live.add(s);
  }

  /// Drops the live stream, as a disconnect would.
  void dropConnection() => _live.close();

  @override
  Future<List<RepSummary>> summaries() async {
    summariesCalls++;
    if (failSummaries) throw StateError('device unreachable');
    return List.of(_stored);
  }

  @override
  Stream<RepSummary> watchSummaries() => _live.stream;

  @override
  Future<List<TracePoint>> trace(int repId) async {
    traceCalls++;
    if (traceDelay > Duration.zero) await Future<void>.delayed(traceDelay);
    if (tracesMissing.contains(repId)) {
      throw TraceUnavailableException(repId, 'trace overwritten on the device ring');
    }
    final points = tracePoints[repId];
    if (points == null) throw TraceUnavailableException(repId, 'no such rep');
    return points;
  }

  @override
  Future<void> close() async => _live.close();

  /// Reassembles a trace from protocol chunks, mirroring what the BLE transport
  /// will do with notifications.
  static List<TracePoint> reassemble(int repId, Iterable<pb.TraceChunk> chunks) {
    final assembler = TraceAssembler(repId);
    for (final chunk in chunks) {
      assembler.add(chunk);
    }
    return assembler.assemble();
  }
}
