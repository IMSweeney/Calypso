/// A calibration status notification, as the device sends one.
///
/// Shared by the transport tests and the widget tests, so the wire form of the
/// message is defined in one place rather than re-spelled per test file.
library;

import 'dart:typed_data';

import 'package:calypso/protocol/calypso.pb.dart' as pb;

/// A `SyncMessage` carrying a `CalibrationStatus`, encoded as the device sends it.
///
/// [ready] defaults to the device's own rule — accel, gyro and sys at 3, with the
/// magnetometer unused in IMU mode — and can be forced, so a test can pin what
/// happens if the two ever disagree.
Uint8List calibrationMessage({
  int sys = 3,
  int accel = 3,
  int gyro = 3,
  int mag = 0,
  bool? ready,
}) =>
    pb.SyncMessage(
      calibration: pb.CalibrationStatus(
        sys: sys,
        accel: accel,
        gyro: gyro,
        mag: mag,
        ready: ready ?? (sys == 3 && accel == 3 && gyro == 3),
      ),
    ).writeToBuffer();
