/// Derived views on the wire types: units, tilt ranges, a canonical uuid.
library;

import 'dart:math' as math;

import '../protocol/calypso.pb.dart' as pb;
import 'uuid.dart';

const int accelLsbPerMs2 = 100;
const int quatLsbPerUnit = 16384;

extension RepSummaryX on pb.RepSummary {
  String? get uuid => uuidFromBytes(repUuid);

  double get rollRangeDeg => rollMaxDeg - rollMinDeg;
  double get pitchRangeDeg => pitchMaxDeg - pitchMinDeg;
}

extension TraceSampleX on pb.TraceSample {
  double get accelX => ax / accelLsbPerMs2;
  double get accelY => ay / accelLsbPerMs2;
  double get accelZ => az / accelLsbPerMs2;

  /// m/s².
  double get accelMagnitude =>
      math.sqrt((ax * ax + ay * ay + az * az).toDouble()) / accelLsbPerMs2;

  List<double> get quaternion => [
    qw / quatLsbPerUnit,
    qx / quatLsbPerUnit,
    qy / quatLsbPerUnit,
    qz / quatLsbPerUnit,
  ];
}
