/// The latest IMU calibration reading the device has sent, for the UI to watch.
///
/// The device pushes it once a second, so this deliberately only notifies when the
/// reading actually changes: a rebuild a second for identical levels is exactly
/// the kind of thing that makes a list feel bad.
library;

import 'package:flutter/foundation.dart';

import '../model/calibration.dart';

class CalibrationNotifier extends ChangeNotifier {
  CalibrationStatus? _latest;

  /// The last reading from the device, or null if there isn't one: never
  /// connected, or disconnected (see [clear]).
  CalibrationStatus? get latest => _latest;

  /// Whether the device has told us it is calibrated and recording reps.
  ///
  /// False when there is no reading at all — the safe thing to believe, since
  /// nothing is being recorded by a device we are not talking to.
  bool get isReady => _latest?.ready ?? false;

  /// Record a reading from the device.
  void set(CalibrationStatus status) {
    if (_latest == status) return;
    _latest = status;
    notifyListeners();
  }

  /// Forget the reading, on disconnect.
  ///
  /// The levels are only meaningful for the connection they arrived on: a hint
  /// about a device we stopped talking to ten minutes ago would be worse than no
  /// hint at all.
  void clear() {
    if (_latest == null) return;
    _latest = null;
    notifyListeners();
  }
}
