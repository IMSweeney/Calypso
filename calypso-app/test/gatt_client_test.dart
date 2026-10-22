import 'dart:typed_data';

import 'package:calypso/gatt/gatt_client.dart';
import 'package:calypso/gatt/pipe.dart' as pipe;
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:flutter_test/flutter_test.dart';

/// The generated client, against the schema it came from.
///
/// The pipe's two characteristics are the runtime's (`lib/gatt/pipe.dart`), so for
/// those the schema's half is the UUIDs, the numbers and the two directions' codecs.
/// The third characteristic carries a value, which is what the generated client's own
/// member is for — and the one thing here that needs a link, faked below.
void main() {
  test('the UUIDs are the ones the schema declares', () {
    expect(SyncClient.serviceUuid, '0000ca20-b0cd-11ec-871f-d45ddf138840');
    expect(SyncClient.messagesUuid, '0000ca21-b0cd-11ec-871f-d45ddf138840');
    expect(SyncClient.requestsUuid, '0000ca22-b0cd-11ec-871f-d45ddf138840');
    expect(SyncClient.calibrationUuid, '0000ca24-b0cd-11ec-871f-d45ddf138840');
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
      summary: pb.RepSummary(repUuid: List<int>.filled(16, 9), traceLen: 7),
    );
    final bytes = SyncPipe.encodeResponse(message);
    expect(
      SyncPipe.decodeResponse(bytes).whichBody(),
      pb.SyncMessage_Body.summary,
    );
    expect(SyncPipe.decodeResponse(bytes).summary.traceLen, 7);
  });

  test(
    'the calibration read is the schema\'s, characteristic and all',
    () async {
      final link = _FakeClientLink(
        value: pb.CalibrationStatus(
          accel: 3,
          gyro: 2,
          ready: false,
        ).writeToBuffer(),
      );

      final status = await SyncClient(link).readCalibration();

      expect(link.reads, [
        (SyncClient.serviceUuid, SyncClient.calibrationUuid),
      ]);
      expect(status.accel, 3);
      expect(status.gyro, 2);
      expect(status.ready, isFalse);
    },
  );
}

/// A link that answers reads, and refuses to be talked to otherwise.
class _FakeClientLink implements GattClientLink {
  _FakeClientLink({required this.value});

  final Uint8List value;
  final List<(String, String)> reads = [];

  @override
  Future<Uint8List> read(String serviceUuid, String characteristicUuid) async {
    reads.add((serviceUuid, characteristicUuid));
    return value;
  }

  @override
  Future<void> subscribe(String serviceUuid, String characteristicUuid) =>
      throw UnsupportedError('this client only reads');

  @override
  Stream<Uint8List> values(String characteristicUuid) =>
      throw UnsupportedError('this client only reads');

  @override
  Future<void> write(
    String serviceUuid,
    String characteristicUuid,
    Uint8List value, {
    bool withResponse = true,
  }) => throw UnsupportedError('this client only reads');
}
