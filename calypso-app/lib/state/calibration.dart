library;

import 'package:flutter/foundation.dart';

import '../protocol/calypso.pb.dart' as pb;

class CalibrationNotifier extends ChangeNotifier {
  pb.CalibrationStatus? _latest;

  pb.CalibrationStatus? get latest => _latest;

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
