library;

import '../protocol/calypso.pb.dart' as pb;

const int maxCalibrationLevel = 3;

class CalibrationStatus {
  const CalibrationStatus({
    required this.sys,
    required this.accel,
    required this.gyro,
    required this.mag,
    required this.ready,
  });

  final int sys;
  final int accel;
  final int gyro;
  final int mag;

  final bool ready;

  factory CalibrationStatus.fromProto(pb.CalibrationStatus p) => CalibrationStatus(
        sys: p.sys,
        accel: p.accel,
        gyro: p.gyro,
        mag: p.mag,
        ready: p.ready,
      );

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
