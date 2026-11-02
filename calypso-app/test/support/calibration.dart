library;

import 'package:calypso/protocol/calypso.pb.dart' as pb;

/// The IMU's calibration state, as the device's characteristic holds it.
///
/// The device writes this from its own 1 Hz poll of the chip, and a client reads it whenever it likes — so a test serves one (`FakeDevice.servesCalibration`) rather than sending it.
///
/// `ready` defaults to the device's own rule and not to "everything is at 3": it is `Calibration::can_record`, which
/// is the gyroscope alone, so a device whose accelerometer is still short serves `ready: true` and records reps whose
/// numbers are worth less. A fake that called that "not ready" would hide exactly the case the app's hint has to
/// cover.
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
  ready: ready ?? (gyro == 3),
);
