library;

import 'dart:math' as math;

import 'uuid.dart';

import '../protocol/calypso.pb.dart' as pb;

const int accelLsbPerMs2 = 100;
const int quatLsbPerUnit = 16384;

/// Device timestamps are uptime milliseconds: the device has no RTC, so
/// [tStartMs]/[tEndMs] are only meaningful relative to each other.
class RepSummary {
  const RepSummary({
    required this.uuid,
    required this.tStartMs,
    required this.tEndMs,
    required this.durationMs,
    required this.rom,
    required this.peakVelocity,
    required this.meanVelocity,
    required this.peakAccel,
    required this.rollMinDeg,
    required this.rollMaxDeg,
    required this.pitchMinDeg,
    required this.pitchMaxDeg,
    required this.traceLen,
    required this.chunkCount,
    required this.decimated,
  });

  final String uuid;
  final int tStartMs;
  final int tEndMs;
  final int durationMs;

  /// Range of motion in metres (peak-to-peak on the dominant axis).
  final double rom;

  /// m/s.
  final double peakVelocity;
  final double meanVelocity;

  /// m/s^2.
  final double peakAccel;

  /// The bar-tilt numbers: roll is side-to-side dip, pitch is forward lean.
  final double rollMinDeg;
  final double rollMaxDeg;
  final double pitchMinDeg;
  final double pitchMaxDeg;

  /// Trace points the device stored for this rep, across all chunks.
  final int traceLen;

  /// Chunks the device stored, at `TRACE_CHUNK_SAMPLES` (48) points each.
  final int chunkCount;

  /// True if the trace was decimated to fit the device's trace buffer.
  final bool decimated;

  /// Total tilt range, i.e. how much the bar moved off level during the rep.
  double get rollRangeDeg => rollMaxDeg - rollMinDeg;
  double get pitchRangeDeg => pitchMaxDeg - pitchMinDeg;

  factory RepSummary.fromProto(pb.RepSummary p) => RepSummary(
        uuid: uuidFromBytes(p.repUuid) ?? '',
        tStartMs: p.tStartMs,
        tEndMs: p.tEndMs,
        durationMs: p.durationMs,
        rom: p.rom,
        peakVelocity: p.peakVelocity,
        meanVelocity: p.meanVelocity,
        peakAccel: p.peakAccel,
        rollMinDeg: p.rollMinDeg,
        rollMaxDeg: p.rollMaxDeg,
        pitchMinDeg: p.pitchMinDeg,
        pitchMaxDeg: p.pitchMaxDeg,
        traceLen: p.traceLen,
        chunkCount: p.chunkCount,
        decimated: p.decimated,
      );
}

class TracePoint {
  const TracePoint({
    required this.tOffsetMs,
    required this.x,
    required this.y,
    required this.z,
    required this.ax,
    required this.ay,
    required this.az,
    required this.qw,
    required this.qx,
    required this.qy,
    required this.qz,
  });

  final int tOffsetMs;

  final double x;
  final double y;
  final double z;

  /// Raw linear acceleration (0.01 m/s^2 per LSB).
  final int ax;
  final int ay;
  final int az;

  /// Raw orientation quaternion, [w, x, y, z] (1/16384 per LSB).
  final int qw;
  final int qx;
  final int qy;
  final int qz;

  double get accelX => ax / accelLsbPerMs2;
  double get accelY => ay / accelLsbPerMs2;
  double get accelZ => az / accelLsbPerMs2;

  /// Linear acceleration magnitude in m/s^2.
  double get accelMagnitude =>
      math.sqrt((ax * ax + ay * ay + az * az).toDouble()) / accelLsbPerMs2;

  /// Orientation as unit-quaternion components [w, x, y, z].
  List<double> get quaternion => [
        qw / quatLsbPerUnit,
        qx / quatLsbPerUnit,
        qy / quatLsbPerUnit,
        qz / quatLsbPerUnit,
      ];

  factory TracePoint.fromProto(pb.TraceSample p) => TracePoint(
        tOffsetMs: p.tOffsetMs,
        x: p.px,
        y: p.py,
        z: p.pz,
        ax: p.ax,
        ay: p.ay,
        az: p.az,
        qw: p.qw,
        qx: p.qx,
        qy: p.qy,
        qz: p.qz,
      );
}
