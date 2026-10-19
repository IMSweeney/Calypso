/// The IMU's calibration state, and the wording that turns it into something a
/// person can act on.
///
/// The device has no display, so this is where "the gyroscope needs the sensor to
/// be still" becomes a sentence on a phone. The raw levels come from the BNO055's
/// `CALIB_STAT` register (`0..3` per sensor, 3 fully calibrated), and whether reps
/// are being recorded comes from the device itself — see `proto/calypso.proto` and
/// `docs/CALIBRATION.md`.
///
/// The magnetometer is read and shown but **never** acted on: the BNO055 runs in
/// IMU mode here, which does not use it and where it never calibrates, so a hint
/// telling the user to move the bar in figure eights would be telling them to do
/// something that cannot work.
library;

import '../protocol/calypso.pb.dart' as pb;

/// A level of 3 means that sensor is fully calibrated.
const int maxCalibrationLevel = 3;

class CalibrationStatus {
  const CalibrationStatus({
    required this.sys,
    required this.accel,
    required this.gyro,
    required this.mag,
    required this.ready,
  });

  /// The aggregate the fusion uses.
  final int sys;
  final int accel;
  final int gyro;
  final int mag;

  /// Whether the device is recording reps.
  ///
  /// Computed on the device rather than re-derived here, so the app cannot
  /// disagree with the device about whether a rep would be stored.
  final bool ready;

  factory CalibrationStatus.fromProto(pb.CalibrationStatus p) => CalibrationStatus(
        sys: p.sys,
        accel: p.accel,
        gyro: p.gyro,
        mag: p.mag,
        ready: p.ready,
      );

  /// The gyroscope is not calibrated yet: it needs the sensor held still.
  bool get gyroShort => gyro < maxCalibrationLevel;

  /// The accelerometer is not calibrated yet: it needs a few still orientations.
  bool get accelShort => accel < maxCalibrationLevel;

  /// Headline for the UI.
  String get summary => 'IMU not calibrated — rep recording is paused';

  /// What to do about it, phrased as something to do with the bar.
  ///
  /// Only ever mentions the sensors that are actually short, and never the
  /// magnetometer (see the library doc).
  String get action {
    if (!gyroShort && !accelShort) {
      // Both are there and `ready` is still false, so the aggregate is catching
      // up. Nothing useful to ask for.
      return 'Almost there — the sensor is finishing up.';
    }
    if (gyroShort && accelShort) {
      return 'Leave the bar still on the rack for a few seconds, then rest it '
          'level and stand it upright, holding each still.';
    }
    if (gyroShort) {
      return 'Leave the bar still on the rack for a few seconds.';
    }
    return 'Rest the bar level, then stand it upright, holding each still.';
  }

  /// Every sensor's level, one line, including the ones not worth acting on.
  String get levels =>
      'accel $accel/$maxCalibrationLevel · gyro $gyro/$maxCalibrationLevel · '
      'sys $sys/$maxCalibrationLevel · mag $mag/$maxCalibrationLevel';

  @override
  bool operator ==(Object other) =>
      other is CalibrationStatus &&
      other.sys == sys &&
      other.accel == accel &&
      other.gyro == gyro &&
      other.mag == mag &&
      other.ready == ready;

  @override
  int get hashCode => Object.hash(sys, accel, gyro, mag, ready);

  @override
  String toString() => 'CalibrationStatus(accel: $accel, gyro: $gyro, '
      'mag: $mag, sys: $sys, ready: $ready)';
}
