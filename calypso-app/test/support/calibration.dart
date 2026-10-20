library;

import 'dart:typed_data';

import 'package:calypso/protocol/calypso.pb.dart' as pb;

Uint8List calibrationMessage({
  int sys = 3,
  int accel = 3,
  int gyro = 3,
  int mag = 0,
  bool? ready,
}) => pb.SyncMessage(
  calibration: pb.CalibrationStatus(
    sys: sys,
    accel: accel,
    gyro: gyro,
    mag: mag,
    ready: ready ?? (sys == 3 && accel == 3 && gyro == 3),
  ),
).writeToBuffer();
