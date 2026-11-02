/// Derived views on the wire types: units, tilt ranges, a canonical uuid.
library;

import 'dart:math' as math;

import 'package:fixnum/fixnum.dart';

import '../protocol/calypso.pb.dart' as pb;
import 'uuid.dart';

/// How many points a rep's plotted trace keeps, whatever its length (`calypso_core::rep::TRACE_POINTS`).
///
/// Not enforced here, just documented: a summary says how many it actually has, and a rep long enough
/// to be reduced carries fewer samples of its motion than it has milliseconds to draw.
const int tracePointsBudget = 128;

/// Angular-rate fixed point: LSB per degree per second (`sampler::GYRO_SCALE`).
const int gyroLsbPerDps = 16;

extension RepSummaryX on pb.RepSummary {
  String? get uuid => uuidFromBytes(repUuid);

  /// Whether this rep can be replayed from its own samples: a device records the raw trace, at full rate,
  /// only while a client has asked it to, and `rawChunks` is the answer.
  bool get hasRawTrace => rawChunks > 0;

  /// The filter state the rep's window begins from, or null on a record that carries none.
  pb.RepState? get origin => hasState() ? state : null;

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

/// A point of a rep's plotted path, in the units the UI speaks: metres and a unit quaternion.
extension TraceComputedPointX on pb.TraceComputedPoint {
  List<double> get position => [px, py, pz];

  /// The attitude the device's filter computed, `[w, x, y, z]`. Carried as floats on the wire, because a point
  /// of the plotted trace is drawn and not integrated.
  List<double> get quaternion => [qw, qx, qy, qz];
}

/// One sample of a rep's raw trace: what the sensor read, at the fixed point the device stored it in.
///
/// Nothing in the app replays these yet — the raw trace is for taking a session away and running the
/// device's own filter over it — so this is only the units a reader of one needs.
extension TraceRawPointX on pb.TraceRawPoint {
  /// m/s² of **specific force**: gravity included, so a still sensor reads about 9.8 up its up axis.
  List<double> get specificForce => [ax / 100.0, ay / 100.0, az / 100.0];

  /// Degrees per second, in the sensor's own frame.
  List<double> get angularRate => [
    gx / gyroLsbPerDps,
    gy / gyroLsbPerDps,
    gz / gyroLsbPerDps,
  ];

  double get specificForceMagnitude =>
      math.sqrt(specificForce.fold<double>(0, (acc, a) => acc + a * a));
}
