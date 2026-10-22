library;

import 'package:calypso/protocol/calypso.pb.dart' as pb;

/// The IMU's calibration state, as the device's characteristic holds it.
///
/// The device writes this from its own 1 Hz poll of the chip, and a client reads it whenever it likes — so a test serves one (`FakeDevice.servesCalibration`) rather than sending it.
pb.CalibrationStatus calibrationStatus({
  int sys = 3,
  int accel = 3,
  int gyro = 3,
  int mag = 0,
  bool? ready,
}) => pb.CalibrationStatus(
  sys: sys,
  accel: accel,
  gyro: gyro,
  mag: mag,
  ready: ready ?? (sys == 3 && accel == 3 && gyro == 3),
);
