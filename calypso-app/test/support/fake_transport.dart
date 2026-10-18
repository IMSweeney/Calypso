/// A scriptable [RepTransport] for tests: no device, no fixtures, no timing.
///
/// Everything the sync client has to survive can be staged here: summaries
/// arriving live, a trace that the device has forgotten, a trace that never
/// answers, and chunks that arrive out of order or twice.
library;

import 'dart:async';

import 'package:calypso/model/rep.dart';
import 'package:calypso/model/uuid.dart';

import 'test_uuid.dart';
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:calypso/sync/trace_reassembly.dart';
import 'package:calypso/transport/transport.dart';

/// Builds a summary with sensible defaults, so tests only state what matters.
RepSummary summary(
  int index, {
  int durationMs = 1400,
  double rom = 0.4,
  double peakVelocity = 0.7,
  int traceLen = 140,
  int chunkCount = 3,
  bool decimated = false,
}) =>
    RepSummary(
      uuid: testUuid(index),
      tStartMs: index * 10_000,
      tEndMs: index * 10_000 + durationMs,
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

/// Splits points into protocol chunks of [perChunk], the way the device streams
/// them over BLE: every chunk carries the total, so a client knows which is last.
///
/// The chunks carry [uuid], so they belong to the rep a test named.
List<pb.TraceChunk> chunksFor(String uuid, List<TracePoint> points, {int perChunk = 4}) {
  final chunks = <pb.TraceChunk>[];
  for (var start = 0; start < points.length; start += perChunk) {
    final slice = points.sublist(start, (start + perChunk).clamp(0, points.length));
    chunks.add(pb.TraceChunk(
      repUuid: uuidToBytes(uuid)!,
      index: chunks.length,
      firstTOffsetMs: slice.first.tOffsetMs,
      totalChunks: (points.length / perChunk).ceil(),
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
    Map<int, List<TracePoint>>? tracePoints,
    Set<int>? tracesMissing,
    this.traceDelay = Duration.zero,
    this.failSummaries = false,
  })  : _stored = stored ?? [],
        _points = {
          for (final entry in (tracePoints ?? <int, List<TracePoint>>{}).entries)
            testUuid(entry.key): entry.value,
        },
        _missing = {
          for (final index in tracesMissing ?? <int>{}) testUuid(index),
        };

  List<RepSummary> _stored;

  /// Traces keyed by the uuid the transport API uses, built from test indices.
  final Map<String, List<TracePoint>> _points;

  /// Reps whose trace the device no longer has (the ring overwrote them).
  final Set<String> _missing;

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
  Future<List<TracePoint>> trace(String uuid) async {
    traceCalls++;
    if (traceDelay > Duration.zero) await Future<void>.delayed(traceDelay);
    if (_missing.contains(uuid)) {
      throw TraceUnavailableException(uuid, 'trace overwritten on the device ring');
    }
    final points = _points[uuid];
    if (points == null) throw TraceUnavailableException(uuid, 'no such rep');
    return points;
  }

  @override
  Future<void> close() async => _live.close();

  /// Reassembles a trace from protocol chunks, mirroring what the BLE transport
  /// will do with notifications.
  static List<TracePoint> reassemble(String uuid, Iterable<pb.TraceChunk> chunks) {
    final assembler = TraceAssembler(uuid);
    for (final chunk in chunks) {
      assembler.add(chunk);
    }
    return assembler.assemble();
  }
}
