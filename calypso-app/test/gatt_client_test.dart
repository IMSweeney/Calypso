import 'package:calypso/gatt/gatt_client.dart';
import 'package:calypso/gatt/pipe.dart' as pipe;
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:flutter_test/flutter_test.dart';

/// The generated client, against the schema it came from.
///
/// Both characteristics belong to one pipe, so there is nothing here to do over a
/// fake link any more: what the schema declares is the UUIDs, the pipe's numbers
/// and the two directions' codecs, and what *drives* them is
/// `lib/transport/ble_transport.dart` (its own tests) over `lib/gatt/pipe.dart`
/// (the shared vectors). What is left to check is that the constants and the
/// codecs are the schema's.
void main() {
  test('the UUIDs are the ones the schema declares', () {
    expect(SyncClient.serviceUuid, '0000ca20-b0cd-11ec-871f-d45ddf138840');
    expect(SyncClient.messagesUuid, '0000ca21-b0cd-11ec-871f-d45ddf138840');
    expect(SyncClient.requestsUuid, '0000ca22-b0cd-11ec-871f-d45ddf138840');
    expect(SyncPipe.serviceUuid, SyncClient.serviceUuid);
    expect(SyncPipe.upUuid, SyncClient.messagesUuid);
    expect(SyncPipe.downUuid, SyncClient.requestsUuid);
  });

  test('the pipe is sized from the schema, and the runtime agrees', () {
    expect(SyncPipe.chunkMax, 244);
    expect(SyncPipe.messageMax, 4096);
    expect(SyncPipe.window, 127);
    expect(SyncPipe.xferTimeoutMs, 5000);

    // One frame is what a characteristic holds, so the payload a chunk may carry
    // is the frame less its own overhead — the same sum the runtime takes.
    final capacity = pipe.capacity(247, SyncPipe.chunkMax, 1000);
    expect(capacity, SyncPipe.chunkMax - pipe.overhead(1000));
    expect(
      capacity! + pipe.overhead(1000),
      SyncPipe.chunkMax,
      reason: 'the frame fills the characteristic exactly',
    );
  });

  test('a request encodes and decodes as the schema says', () {
    final request = pb.SyncRequest(
      trace: pb.TraceRequest(repUuid: List<int>.filled(16, 7), allChunks: true),
    );
    final bytes = SyncPipe.encodeRequest(request);
    expect(SyncPipe.decodeRequest(bytes).trace.allChunks, isTrue);
    expect(
      SyncPipe.decodeRequest(bytes).trace.repUuid,
      List<int>.filled(16, 7),
    );
  });

  test('a message encodes and decodes as the schema says', () {
    final message = pb.SyncMessage(
      calibration: pb.CalibrationStatus(accel: 3, gyro: 3, ready: true),
    );
    final bytes = SyncPipe.encodeResponse(message);
    expect(
      SyncPipe.decodeResponse(bytes).whichBody(),
      pb.SyncMessage_Body.calibration,
    );
    expect(SyncPipe.decodeResponse(bytes).calibration.ready, isTrue);
  });
}
