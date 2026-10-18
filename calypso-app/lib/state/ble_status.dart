/// What the BLE connection is doing, as the UI needs to know it.
///
/// Named `BleStatus` rather than `ConnectionState` on purpose: Flutter already
/// has a `ConnectionState` (in `AsyncSnapshot`), and importing both was the
/// cause of the `connection.` prefixes in the previous sketch.
library;

import 'package:flutter/foundation.dart';

enum BleStatus {
  /// Nothing attempted yet.
  idle,

  /// Bluetooth is off, not permitted, or absent (a missing macOS entitlement
  /// shows up here too).
  unsupported,

  /// Looking for a Calypso device.
  scanning,

  /// Found one; connecting and subscribing.
  connecting,

  /// Connected and subscribed: summaries arrive live.
  connected,

  /// Was connected, or tried to be, and is not now.
  disconnected,

  /// The last attempt failed; [BleStatusNotifier.error] says why.
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

  /// Short label for a status badge.
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
