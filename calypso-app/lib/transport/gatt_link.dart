library;

import 'dart:typed_data';

import '../gatt/gatt_client.dart';
import 'ble_link.dart';

/// The generated client's [GattClientLink], over this app's [BleLink].
///
/// One link, one device: scanning, connecting and reconnecting stay with the
/// transport, which is what knows about them. This is only the operations a
/// generated client asks for, so that the client itself has no opinion about how
/// the radio is reached — which is also what lets `FakeBleLink` stand in for it
/// in tests.
class BleLinkGattClient implements GattClientLink {
  BleLinkGattClient(this._link, this._deviceId);

  final BleLink _link;
  final String _deviceId;

  @override
  Future<void> subscribe(String serviceUuid, String characteristicUuid) =>
      _link.subscribe(_deviceId, serviceUuid, characteristicUuid);

  @override
  Stream<Uint8List> values(String characteristicUuid) =>
      _link.valueStream(_deviceId, characteristicUuid);

  @override
  Future<void> write(
    String serviceUuid,
    String characteristicUuid,
    Uint8List value, {
    bool withResponse = true,
  }) => _link.write(
    _deviceId,
    serviceUuid,
    characteristicUuid,
    value,
    withResponse: withResponse,
  );
}
