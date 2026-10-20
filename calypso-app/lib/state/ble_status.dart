library;

import 'package:flutter/foundation.dart';

enum BleStatus {
  idle,
  unsupported,
  scanning,
  connecting,
  connected,
  disconnected,
  failed,
}

class BleStatusNotifier extends ChangeNotifier {
  BleStatus _status = BleStatus.idle;
  String? _deviceId;
  String? _error;

  BleStatus get status => _status;
  String? get deviceId => _deviceId;
  String? get error => _error;

  bool get isConnected => _status == BleStatus.connected;
  bool get isBusy => _status == BleStatus.scanning || _status == BleStatus.connecting;
  bool get canConnect =>
      _status == BleStatus.idle ||
      _status == BleStatus.disconnected ||
      _status == BleStatus.failed;

  String get label => switch (_status) {
        BleStatus.idle => 'No device',
        BleStatus.unsupported => 'Bluetooth unavailable',
        BleStatus.scanning => 'Scanning…',
        BleStatus.connecting => 'Connecting…',
        BleStatus.connected => 'Connected',
        BleStatus.disconnected => 'Disconnected',
        BleStatus.failed => 'Connection failed',
      };

  void set(BleStatus status, {String? deviceId, String? error}) {
    if (_status == status && _deviceId == deviceId && _error == error) return;
    _status = status;
    _deviceId = deviceId ?? (_status == BleStatus.idle ? null : _deviceId);
    _error = error;
    notifyListeners();
  }
}
