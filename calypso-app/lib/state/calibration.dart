library;

import 'package:flutter/foundation.dart';

import '../model/calibration.dart';

class CalibrationNotifier extends ChangeNotifier {
  CalibrationStatus? _latest;

  CalibrationStatus? get latest => _latest;

  bool get isReady => _latest?.ready ?? false;

  void set(CalibrationStatus status) {
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
