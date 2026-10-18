/// A scriptable [BleLink]: no radio, no plugin, no device.
///
/// Everything the transport has to get right can be staged here — a device that
/// is absent, one that answers a trace request, one that stays silent, chunks
/// out of order or repeated, a stream with or without `totalChunks`, and a
/// connection that drops mid-flight.
library;

import 'dart:async';
import 'dart:typed_data';

import 'package:calypso/transport/ble_link.dart';

class FakeBleLink implements BleLink {
  FakeBleLink({this.available = true});

  bool available;

  final List<List<String>> scans = [];
  bool scanning = false;
  bool connected = false;
  String? connectedDeviceId;
  final List<String> subscribed = [];

  /// Writes the app made: (characteristicUuid, bytes).
  final List<(String, Uint8List)> writes = [];

  void Function(String deviceId, String? name)? _onScanResult;
  void Function(String deviceId, bool connected, String? error)? _onConnectionChange;
  final Map<String, StreamController<Uint8List>> _values = {};

  @override
  Future<bool> isAvailable() async => available;

  @override
  Future<void> startScan(List<String> serviceUuids) async {
    scans.add(serviceUuids);
    scanning = true;
  }

  @override
  Future<void> stopScan() async => scanning = false;

  @override
  void onScanResult(void Function(String deviceId, String? name) callback) =>
      _onScanResult = callback;

  @override
  Future<void> connect(String deviceId, {bool autoConnect = false}) async {
    connected = true;
    connectedDeviceId = deviceId;
    _onConnectionChange?.call(deviceId, true, null);
  }

  @override
  Future<void> disconnect(String deviceId) async {
    connected = false;
    _onConnectionChange?.call(deviceId, false, null);
  }

  @override
  void onConnectionChange(
          void Function(String deviceId, bool connected, String? error) callback) =>
      _onConnectionChange = callback;

  @override
  Future<void> subscribe(String deviceId, String serviceUuid, String characteristicUuid) async {
    subscribed.add(characteristicUuid);
    _values.putIfAbsent(characteristicUuid, () => StreamController<Uint8List>.broadcast());
  }

  @override
  Stream<Uint8List> valueStream(String deviceId, String characteristicUuid) =>
      _values.putIfAbsent(characteristicUuid, () => StreamController<Uint8List>.broadcast()).stream;

  @override
  Future<void> write(
    String deviceId,
    String serviceUuid,
    String characteristicUuid,
    Uint8List value,
  ) async {
    writes.add((characteristicUuid, value));
  }

  // ── Scripting ───────────────────────────────────────────────────────────

  /// A device advertising the service is seen.
  void seeDevice(String deviceId, [String? name]) =>
      _onScanResult?.call(deviceId, name);

  /// The device pushes a notification on a characteristic.
  void notify(String characteristicUuid, Uint8List value) =>
      _values.putIfAbsent(characteristicUuid, () => StreamController<Uint8List>.broadcast()).add(value);

  /// The connection drops, as it would walking out of range.
  void dropConnection([String? error]) {
    connected = false;
    _onConnectionChange?.call(connectedDeviceId ?? '', false, error);
  }

  /// A write the app made, as the raw bytes of a specific characteristic.
  Uint8List lastWriteTo(String characteristicUuid) => writes
      .lastWhere((w) => w.$1 == characteristicUuid,
          orElse: () => throw StateError('no write to $characteristicUuid'))
      .$2;

  bool get hasWriteTo =>
      writes.isNotEmpty;
}
