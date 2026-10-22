import 'dart:typed_data';

import 'package:calypso/gatt/gatt_client.dart';
import 'package:calypso/gatt/pipe.dart';

import 'fake_ble_link.dart';

/// The device's end of the pipe, for tests.
///
/// The transport under test is one end; this is the other one doing what
/// `calypso-esp32` does — cutting a message into frames on the notified
/// characteristic, reassembling what the app writes on the other, and acknowledging
/// both. It is `pipe.dart`'s own sender and receiver used the other way round, so a
/// test that passes here has passed the protocol rather than a mock of it.
class FakeDevice {
  FakeDevice(this.link, {this.mtu = 247}) {
    // Answer each write as it happens, like a device that is listening: the app
    // waits for the acknowledgement of its request before it will believe it
    // arrived.
    link.onWrite = _onWrite;
  }

  final FakeBleLink link;

  /// The ATT MTU the device's chunks are sized from.
  final int mtu;

  /// The messages the app has written to the device, in order.
  final List<Uint8List> requests = [];

  /// Frames the device has notified, for the tests that count them.
  final List<Uint8List> notified = [];

  /// Called with each message the app writes, for a test that wants to answer it
  /// the way the firmware would (a trace request becomes a stream of chunks).
  void Function(Uint8List message)? onRequest;

  final Receiver _receiver = Receiver(SyncPipe.window);
  final Uint8List _inbound = Uint8List(SyncPipe.messageMax);

  /// How much payload one chunk carries on this link, for a message of `length`
  /// bytes (the length is part of the frame, so it belongs to the sum).
  int? payloadCapacity(int length) => capacity(mtu, SyncPipe.chunkMax, length);

  /// A frame the device puts on the wire, as the app would see it.
  Uint8List _frame(Header header, List<int> payload) {
    final out = Uint8List(SyncPipe.chunkMax);
    final length = encode(header, payload, out);
    return Uint8List.sublistView(out, 0, length);
  }

  /// Send `message` to the app: one transfer, one frame per chunk.
  ///
  /// `drop` drops that chunk of the transfer (counting from 1), which is how a
  /// notification the link loses is reproduced.
  void send(Uint8List message, {int? drop}) {
    // A sender per transfer: the sequence numbering is the receiver's to take from
    // the START, and a test's device has nothing to carry between messages.
    final sender = Sender(SyncPipe.window);
    sender.start(message.length, SyncPipe.messageMax);
    // `capacity` the function, not a local: the payload one chunk carries.
    final chunk = payloadCapacity(message.length);
    if (chunk == null) throw StateError('a $mtu-byte link carries no frame');

    final frame = Uint8List(SyncPipe.chunkMax);
    var nth = 0;
    while (true) {
      final out = sender.next(message, chunk, frame);
      if (out.state == SendState.done) return;
      if (out.state == SendState.windowFull) {
        throw StateError('the message did not fit one window');
      }
      nth++;
      if (nth == drop) continue;
      final bytes = Uint8List.fromList(frame.sublist(0, out.length));
      notified.add(bytes);
      link.notify(SyncClient.messagesUuid, bytes);
    }
  }

  /// A notification that acknowledges nothing: a frame whose CRC does not check
  /// out, which the app is expected to drop rather than half-believe.
  void sendGarbage() => link.notify(
    SyncClient.messagesUuid,
    Uint8List.fromList([0x01, 0x02, 0x03, 0x04, 0x05]),
  );

  void _onWrite(String characteristicUuid, Uint8List value) {
    if (characteristicUuid != SyncClient.requestsUuid) return;
    final frame = decode(value);
    if (frame == null) return;
    // The app acknowledging a chunk of ours: nothing for a test's device to do
    // with it, since each `send` uses a sender of its own.
    if (frame.header.isReply) return;

    final received = _receiver.onChunk(value, _inbound, SyncPipe.messageMax);
    switch (received.kind) {
      case ReceivedKind.delivered:
        final message = Uint8List.fromList(
          Uint8List.sublistView(_inbound, 0, received.length),
        );
        requests.add(message);
        link.notify(
          SyncClient.messagesUuid,
          _frame(Header(_receiver.nextExpected, PipeFlags.ack), const <int>[]),
        );
        onRequest?.call(message);
      case ReceivedKind.ack:
      case ReceivedKind.nak:
        link.notify(
          SyncClient.messagesUuid,
          _frame(Header(_receiver.nextExpected, PipeFlags.ack), const <int>[]),
        );
      case ReceivedKind.carryOn:
      case ReceivedKind.dropped:
        break;
    }
  }
}
