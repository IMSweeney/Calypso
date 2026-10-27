/// Derived views on the wire types: units, tilt ranges, a canonical uuid.
library;

import 'dart:math' as math;

import 'package:fixnum/fixnum.dart';

import '../protocol/calypso.pb.dart' as pb;
import 'uuid.dart';

const int accelLsbPerMs2 = 100;
const int quatLsbPerUnit = 16384;

extension RepSummaryX on pb.RepSummary {
  String? get uuid => uuidFromBytes(repUuid);

  double get rollRangeDeg => rollMaxDeg - rollMinDeg;
  double get pitchRangeDeg => pitchMaxDeg - pitchMinDeg;

  /// When the rep happened, from the device's clock — the only time there is, since the device sets it and dates
  /// every summary it sends with it.
  ///
  /// Null when a record carries no time at all: a device that had no clock for that boot never sent the rep, so
  /// this is a dump of one written before the clock arrived (see `summariesOf`), and it is not on the list at all
  /// rather than dated from when the app happened to see it.
  DateTime? get recordedAt => tUnixMs == Int64.ZERO
      ? null
      : DateTime.fromMillisecondsSinceEpoch(tUnixMs.toInt(), isUtc: true);

  /// When the rep happened as the UI shows it: local time of day, to the second.
  ///
  /// The date is left off because the list shows one day and would repeat it on every row; the seconds are not,
  /// because a set is reps a minute or less apart and two rows reading the same minute are two rows the user cannot
  /// tell apart.
  String? get timeOfDay {
    final when = recordedAt?.toLocal();
    if (when == null) return null;
    String two(int v) => v.toString().padLeft(2, '0');
    return '${two(when.hour)}:${two(when.minute)}:${two(when.second)}';
  }
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
