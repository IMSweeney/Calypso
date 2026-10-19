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

import '../logging/loggers.dart';

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

  /// Discover the device's services and characteristics.
  ///
  /// Required between connecting and subscribing on iOS and macOS: CoreBluetooth
  /// only populates the peripheral's service list after discovery has run, and the
  /// plugin resolves characteristics against that list. Without it the lookup
  /// fails as "characteristic not found" for a UUID that is correct — see
  /// [BleTransport.connect].
  ///
  /// Returns what the device actually offers, so a caller can say *what* is
  /// missing rather than passing on the plugin's message.
  Future<List<DiscoveredService>> discoverServices(String deviceId);

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

/// A service a device offers, and the characteristics under it.
///
/// A plain string list rather than the plugin's own types: the seam exists so that
/// nothing outside this file depends on `universal_ble`.
class DiscoveredService {
  const DiscoveredService(this.uuid, this.characteristics);

  final String uuid;
  final List<String> characteristics;

  @override
  String toString() => '$uuid[${characteristics.join(', ')}]';
}

/// The real implementation, forwarding to `universal_ble`.
///
/// It also logs what the plugin reports, at the lowest level the app can see it:
/// every value the radio delivered, every scan result, every connection change. That
/// is deliberately *below* [BleTransport]'s per-characteristic streams, because
/// those filter by UUID — so "the device never sent anything" and "the plugin
/// received it and dropped it" are indistinguishable from inside the transport, and
/// telling those two apart is the whole point of the log.
class UniversalBleLink implements BleLink {
  UniversalBleLink() {
    // `raw` rather than `rx`: these are the values the plugin received, before the
    // transport's per-characteristic streams had a look at them. The device is not
    // named — this app talks to one at a time (`CONNECTIONS_MAX` is 1 at the other
    // end too), and the scan and connected lines say which it is.
    plugin.UniversalBle.onValueChange = (deviceId, characteristicId, value, timestamp) {
      bleLog.fine(describeMessage('raw', characteristicId, value));
    };
  }

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
    plugin.UniversalBle.onScanResult = (device) {
      // The first question when nothing arrives is whether the app saw the device
      // at all; this is the answer, and it is above `connect` in the file.
      bleLog.fine('raw scan ${device.deviceId} (${device.name ?? 'no name'})');
      callback(device.deviceId, device.name);
    };
  }

  @override
  Future<void> connect(String deviceId, {bool autoConnect = false}) =>
      plugin.UniversalBle.connect(deviceId, autoConnect: autoConnect);

  @override
  Future<List<DiscoveredService>> discoverServices(String deviceId) async {
    final services = await plugin.UniversalBle.discoverServices(deviceId);
    return [
      for (final service in services)
        DiscoveredService(
          service.uuid,
          [for (final characteristic in service.characteristics) characteristic.uuid],
        ),
    ];
  }

  @override
  Future<void> disconnect(String deviceId) => plugin.UniversalBle.disconnect(deviceId);

  @override
  void onConnectionChange(
      void Function(String deviceId, bool connected, String? error) callback) {
    plugin.UniversalBle.onConnectionChange = (deviceId, isConnected, error) {
      // The plugin's view, next to the transport's own: when the two disagree is
      // exactly when this is worth having.
      bleLog.fine(
        'raw connection $deviceId connected=$isConnected'
        '${error == null ? '' : ' error=$error'}',
      );
      callback(deviceId, isConnected, error);
    };
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
