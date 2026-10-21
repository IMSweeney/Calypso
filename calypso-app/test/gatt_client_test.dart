import 'dart:typed_data';

import 'package:calypso/gatt/gatt_client.dart';
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:calypso/transport/gatt_link.dart';
import 'package:flutter_test/flutter_test.dart';

import 'support/fake_ble_link.dart';

/// The generated client over a fake link: what the schema says a client can do,
/// done without a radio.
void main() {
  const deviceId = 'device-1';

  late FakeBleLink link;
  late SyncClient client;

  setUp(() async {
    link = FakeBleLink();
    await link.connect(deviceId);
    // The fake refuses a subscribe before discovery, as a stack does.
    await link.discoverServices(deviceId);
    client = SyncClient(BleLinkGattClient(link, deviceId));
  });

  test('the UUIDs are the ones the schema declares', () {
    expect(SyncClient.serviceUuid, '0000ca20-b0cd-11ec-871f-d45ddf138840');
    expect(SyncClient.messagesUuid, '0000ca21-b0cd-11ec-871f-d45ddf138840');
    expect(SyncClient.requestsUuid, '0000ca22-b0cd-11ec-871f-d45ddf138840');
  });

  test(
    'subscribing goes to the messages characteristic of the service',
    () async {
      await client.subscribeMessages();

      expect(link.subscribed, [SyncClient.messagesUuid]);
      expect(link.calls, contains('subscribe:${SyncClient.messagesUuid}'));
    },
  );

  test('a notification arrives decoded', () async {
    final updates = client.messagesUpdates;
    final next = updates.first;

    link.notify(
      SyncClient.messagesUuid,
      pb.SyncMessage(
        calibration: pb.CalibrationStatus(accel: 3, gyro: 3, ready: true),
      ).writeToBuffer(),
    );

    final message = await next;
    expect(message.whichBody(), pb.SyncMessage_Body.calibration);
    expect(message.calibration.ready, isTrue);
  });

  test('a write carries the encoding of the message it was given', () async {
    final request = pb.SyncRequest(
      trace: pb.TraceRequest(
        repUuid: Uint8List.fromList(List.filled(16, 7)),
        allChunks: true,
      ),
    );

    await client.writeRequests(request);

    expect(link.writes, hasLength(1));
    final (uuid, bytes) = link.writes.single;
    expect(uuid, SyncClient.requestsUuid);
    // The device reads this: a `SyncRequest` with the rep the app asked for.
    final decoded = pb.SyncRequest.fromBuffer(bytes);
    expect(decoded.trace.repUuid, request.trace.repUuid);
    expect(decoded.trace.allChunks, isTrue);
    // The schema declares WRITE as well as WRITE_NO_RSP, and the write that
    // reports failure is the one it asks for.
    expect(link.writeResponses, [true]);
  });
}
