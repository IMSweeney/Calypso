library;

import '../protocol/calypso.pb.dart' as pb;

const int maxCalibrationLevel = 3;

extension CalibrationStatusX on pb.CalibrationStatus {
  bool get gyroShort => gyro < maxCalibrationLevel;

  bool get accelShort => accel < maxCalibrationLevel;

  /// Whether the device is recording reps: its gate is the gyroscope alone, so this is `gyro` and not `ready`.
  ///
  /// `ready` is the device's own `can_record()` (`calibration.rs`), and it stays `true` while the accelerometer is
  /// still short — deliberately, because that records a real rep whose numbers are worth less, and gating on it cost
  /// reps at random. So `ready` is the wrong question for a hint: an app that showed one only when `!ready` went
  /// quiet for exactly the case a lifter would want telling about.
  bool get records => !gyroShort;

  /// Whether anything is worth saying: the gyroscope (reps are not recorded) or the accelerometer (they are, but
  /// their travel and velocity are worth less).
  bool get needsAttention => gyroShort || accelShort;

  String get summary => gyroShort
      ? 'IMU not calibrated — rep recording is paused'
      : 'Accelerometer not calibrated — reps record, but their travel is worth less';

  String get action {
    if (!gyroShort && !accelShort) {
      return 'Almost there — the sensor is finishing up.';
    }
    if (gyroShort && accelShort) {
      return 'Leave the bar still on the rack for a few seconds, then rest it '
          'level and stand it upright, holding each still.';
    }
    if (gyroShort) {
      return 'Leave the bar still on the rack for a few seconds.';
    }
    return 'Rest the bar level, then stand it upright, holding each still. '
        'Reps still record until it does.';
  }

  String get levels =>
      'accel $accel/$maxCalibrationLevel · gyro $gyro/$maxCalibrationLevel · '
      'sys $sys/$maxCalibrationLevel · mag $mag/$maxCalibrationLevel';
}
