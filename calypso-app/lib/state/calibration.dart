library;

import 'package:flutter/foundation.dart';

import '../protocol/calypso.pb.dart' as pb;

class CalibrationNotifier extends ChangeNotifier {
  pb.CalibrationStatus? _latest;

  pb.CalibrationStatus? get latest => _latest;

  /// Whether the device says it is recording reps: its gate, which is the gyroscope alone.
  ///
  /// Not "calibrated": a rep recorded with the accelerometer short is still a rep, and carries a flag saying so. A
  /// hint built on this goes quiet in exactly that case — the hint reads `needsAttention` on the levels instead.
  bool get isReady => _latest?.ready ?? false;

  void set(pb.CalibrationStatus status) {
    if (_latest == status) return;
    _latest = status;
    notifyListeners();
  }

  void clear() {
    if (_latest == null) return;
    _latest = null;
    notifyListeners();
  }
}
