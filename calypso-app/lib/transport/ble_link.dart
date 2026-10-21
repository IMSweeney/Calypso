library;

import 'dart:typed_data';

import 'package:universal_ble/universal_ble.dart' as plugin;

import '../logging/loggers.dart';

abstract class BleLink {
  Future<bool> isAvailable();

  Future<void> startScan(List<String> serviceUuids);

  Future<void> stopScan();

  void onScanResult(void Function(String deviceId, String? name) callback);

  Future<void> connect(String deviceId, {bool autoConnect = false});

  Future<List<DiscoveredService>> discoverServices(String deviceId);

  Future<void> disconnect(String deviceId);

  void onConnectionChange(
    void Function(String deviceId, bool connected, String? error) callback,
  );

  Future<void> subscribe(
    String deviceId,
    String serviceUuid,
    String characteristicUuid,
  );

  Stream<Uint8List> valueStream(String deviceId, String characteristicUuid);

  /// Write a characteristic.
  ///
  /// `withResponse` is the write that reports failure, so it is the default; a
  /// characteristic that offers only write-without-response asks for the other.
  Future<void> write(
    String deviceId,
    String serviceUuid,
    String characteristicUuid,
    Uint8List value, {
    bool withResponse = true,
  });
}

class DiscoveredService {
  const DiscoveredService(this.uuid, this.characteristics);

  final String uuid;
  final List<String> characteristics;

  @override
  String toString() => '$uuid[${characteristics.join(', ')}]';
}

class UniversalBleLink implements BleLink {
  UniversalBleLink() {
    plugin.UniversalBle.onValueChange =
        (deviceId, characteristicId, value, timestamp) {
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
  Future<void> startScan(List<String> serviceUuids) =>
      plugin.UniversalBle.startScan(
        scanFilter: plugin.ScanFilter(withServices: serviceUuids),
      );

  @override
  Future<void> stopScan() => plugin.UniversalBle.stopScan();

  @override
  void onScanResult(void Function(String deviceId, String? name) callback) {
    plugin.UniversalBle.onScanResult = (device) {
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
        DiscoveredService(service.uuid, [
          for (final characteristic in service.characteristics)
            characteristic.uuid,
        ]),
    ];
  }

  @override
  Future<void> disconnect(String deviceId) =>
      plugin.UniversalBle.disconnect(deviceId);

  @override
  void onConnectionChange(
    void Function(String deviceId, bool connected, String? error) callback,
  ) {
    plugin.UniversalBle.onConnectionChange = (deviceId, isConnected, error) {
      bleLog.fine(
        'raw connection $deviceId connected=$isConnected'
        '${error == null ? '' : ' error=$error'}',
      );
      callback(deviceId, isConnected, error);
    };
  }

  @override
  Future<void> subscribe(
    String deviceId,
    String serviceUuid,
    String characteristicUuid,
  ) => plugin.UniversalBle.subscribeNotifications(
    deviceId,
    serviceUuid,
    characteristicUuid,
  );

  @override
  Stream<Uint8List> valueStream(String deviceId, String characteristicUuid) =>
      plugin.UniversalBle.characteristicValueStream(
        deviceId,
        characteristicUuid,
      );

  @override
  Future<void> write(
    String deviceId,
    String serviceUuid,
    String characteristicUuid,
    Uint8List value, {
    bool withResponse = true,
  }) => plugin.UniversalBle.write(
    deviceId,
    serviceUuid,
    characteristicUuid,
    value,
    withoutResponse: !withResponse,
  );
}
