/// The thin seam over the BLE plugin.
///
/// Everything the transport does that is *ours* — scanning policy, the sync
/// protocol, chunk reassembly, connection state — lives in [BleTransport] on top
/// of this interface, so it can be tested with a fake and no hardware. This file
/// is the only place that imports `universal_ble`, so a plugin API change has
/// exactly one file to touch.
///
/// `universal_ble` (BSD-3) is used rather than an Android/iOS-only plugin
/// because it also covers macOS: CoreBluetooth exists on the Mac and not in the
/// iOS Simulator, so this is what makes the BLE path testable without a phone.
library;

import 'dart:typed_data';

import 'package:universal_ble/universal_ble.dart' as plugin;

/// The BLE operations this app needs. Deliberately small: if we ever need more,
/// it should be because the protocol needs it.
abstract class BleLink {
  /// Whether Bluetooth is on and usable (permission included).
  Future<bool> isAvailable();

  /// Scans for devices advertising any of [serviceUuids]. Results arrive via
  /// [onScanResult]; stop with [stopScan].
  Future<void> startScan(List<String> serviceUuids);

  Future<void> stopScan();

  /// Called for each advertisement seen during a scan.
  void onScanResult(void Function(String deviceId, String? name) callback);

  Future<void> connect(String deviceId, {bool autoConnect = false});

  Future<void> disconnect(String deviceId);

  /// Called on connection changes, including unexpected drops.
  void onConnectionChange(void Function(String deviceId, bool connected, String? error) callback);

  Future<void> subscribe(String deviceId, String serviceUuid, String characteristicUuid);

  /// Values pushed by the device on a subscribed characteristic.
  Stream<Uint8List> valueStream(String deviceId, String characteristicUuid);

  Future<void> write(
    String deviceId,
    String serviceUuid,
    String characteristicUuid,
    Uint8List value,
  );
}

/// The real implementation, forwarding to `universal_ble`.
class UniversalBleLink implements BleLink {
  @override
  Future<bool> isAvailable() async {
    final state = await plugin.UniversalBle.getBluetoothAvailabilityState();
    if (state != plugin.AvailabilityState.poweredOn) return false;
    if (!await plugin.UniversalBle.hasPermissions()) {
      await plugin.UniversalBle.requestPermissions();
    }
    return true;
  }

  @override
  Future<void> startScan(List<String> serviceUuids) => plugin.UniversalBle.startScan(
        scanFilter: plugin.ScanFilter(withServices: serviceUuids),
      );

  @override
  Future<void> stopScan() => plugin.UniversalBle.stopScan();

  @override
  void onScanResult(void Function(String deviceId, String? name) callback) {
    plugin.UniversalBle.onScanResult = (device) => callback(device.deviceId, device.name);
  }

  @override
  Future<void> connect(String deviceId, {bool autoConnect = false}) =>
      plugin.UniversalBle.connect(deviceId, autoConnect: autoConnect);

  @override
  Future<void> disconnect(String deviceId) => plugin.UniversalBle.disconnect(deviceId);

  @override
  void onConnectionChange(
      void Function(String deviceId, bool connected, String? error) callback) {
    plugin.UniversalBle.onConnectionChange =
        (deviceId, isConnected, error) => callback(deviceId, isConnected, error);
  }

  @override
  Future<void> subscribe(String deviceId, String serviceUuid, String characteristicUuid) =>
      plugin.UniversalBle.subscribeNotifications(deviceId, serviceUuid, characteristicUuid);

  @override
  Stream<Uint8List> valueStream(String deviceId, String characteristicUuid) =>
      plugin.UniversalBle.characteristicValueStream(deviceId, characteristicUuid);

  @override
  Future<void> write(
    String deviceId,
    String serviceUuid,
    String characteristicUuid,
    Uint8List value,
  ) =>
      // With response: a trace request is small and rare, and knowing it
      // arrived is worth more than the round trip it costs.
      plugin.UniversalBle.write(deviceId, serviceUuid, characteristicUuid, value);
}
