library;

import '../protocol/calypso.pb.dart' as pb;

const int maxCalibrationLevel = 3;

extension CalibrationStatusX on pb.CalibrationStatus {
  bool get gyroShort => gyro < maxCalibrationLevel;

  bool get accelShort => accel < maxCalibrationLevel;

  String get summary => 'IMU not calibrated — rep recording is paused';

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
    return 'Rest the bar level, then stand it upright, holding each still.';
  }

  String get levels =>
      'accel $accel/$maxCalibrationLevel · gyro $gyro/$maxCalibrationLevel · '
      'sys $sys/$maxCalibrationLevel · mag $mag/$maxCalibrationLevel';
}
