import 'dart:io';
import 'dart:typed_data';

import 'package:calypso/gatt/pipe.dart';
import 'package:flutter_test/flutter_test.dart';

/// The pipe, against the cases both implementations run.
///
/// `lib/gatt/pipe.dart` is a mirror of the Rust reference in `tools/gatt-pipe`, and these vectors belong to the protocol rather than to either side: one file, two state machines, the same answers. Run from `calypso-app` (which is where `flutter test` runs).
void main() {
  final file = File('../tools/gatt-pipe/tests/vectors/pipe.txt');
  if (!file.existsSync()) {
    throw StateError(
      'the vectors are missing at ${file.absolute.path}; run from calypso-app',
    );
  }
  final cases = _parse(file.readAsLinesSync());
  if (cases.isEmpty) {
    throw StateError('the vector file parsed to nothing');
  }

  test('capacity keeps the frame inside the chunk buffer', () {
    // The same ceilings as the Rust reference: the link's, and the buffer's.
    expect(capacity(23, defaultChunkMax, 100), 15);
    expect(
      capacity(247, defaultChunkMax, 100),
      defaultChunkMax - overhead(100),
    );
    // An MTU a phone asked the most of would otherwise put a five-byte frame on top of 244 bytes of payload in a 244-byte buffer.
    expect(
      capacity(517, defaultChunkMax, 100),
      defaultChunkMax - overhead(100),
    );
    expect(capacity(6, defaultChunkMax, 0), isNull);
  });

  for (final pipeCase in cases) {
    test(pipeCase.name, () {
      final outcome = _pump(pipeCase);
      expect(outcome.message != null, pipeCase.delivered, reason: 'delivered');
      if (outcome.message != null) {
        expect(outcome.message, pipeCase.message(), reason: 'the bytes');
      }
      expect(
        outcome.chunks,
        pipeCase.chunks,
        reason: 'chunks on the wire (attempts ${outcome.attempts})',
      );
    });
  }
}

/// One line of the vector file.
class _Case {
  _Case({
    required this.name,
    required this.len,
    required this.mtu,
    required this.maxMessage,
    required this.dropNth,
    required this.corruptNth,
    required this.duplicateNth,
    required this.swallowAcks,
    required this.timeoutMs,
    required this.delivered,
    required this.chunks,
  });

  final String name;
  final int len;
  final int mtu;
  final int maxMessage;
  final int dropNth;
  final int corruptNth;
  final int duplicateNth;
  final bool swallowAcks;
  final int timeoutMs;
  final bool delivered;
  final int chunks;

  /// The message both sides build from its length alone.
  Uint8List message() =>
      Uint8List.fromList([for (var i = 0; i < len; i++) i % 251]);
}

List<_Case> _parse(List<String> lines) => [
  for (final line in lines)
    if (line.trim().isNotEmpty && !line.startsWith('#')) _parseLine(line),
];

_Case _parseLine(String line) {
  final field = line.split('|');
  if (field.length != 11) {
    throw FormatException('a vector has eleven fields: $line');
  }
  return _Case(
    name: field[0],
    len: int.parse(field[1]),
    mtu: int.parse(field[2]),
    maxMessage: int.parse(field[3]),
    dropNth: int.parse(field[4]),
    corruptNth: int.parse(field[5]),
    duplicateNth: int.parse(field[6]),
    swallowAcks: field[7] == '1',
    timeoutMs: int.parse(field[8]),
    delivered: field[9] == '1',
    chunks: int.parse(field[10]),
  );
}

class _Outcome {
  const _Outcome(this.message, this.chunks, this.attempts);
  final Uint8List? message;
  final int chunks;
  final int attempts;
}

/// The window both sides were written for; the vectors do not carry it.
const _window = 8;

/// Move one message through the pipe under this case's faults.
///
/// The faults and the retry policy are the case's, and the order in which a chunk is dropped, corrupted and duplicated is the Rust harness's, deliberately: if the two disagreed about the harness they would disagree about the answers.
_Outcome _pump(_Case c) {
  final message = c.message();
  final sender = Sender(_window);
  final receiver = Receiver(_window);
  sender.start(message.length, c.maxMessage);

  final buffer = Uint8List(c.maxMessage);
  final wire = Uint8List(defaultChunkMax);
  final reply = Uint8List(16);
  var chunks = 0;
  var nth = 0;
  var now = 0;

  for (var i = 0; i < 100000; i++) {
    now += 10;
    final cap = capacity(c.mtu, defaultChunkMax, message.length)!;

    final send = sender.next(message, cap, wire);
    if (send.state != SendState.chunk) {
      if (sender.stalled(now, c.timeoutMs)) {
        if (sender.attempts >= maxAttempts) {
          return _Outcome(null, chunks, sender.attempts);
        }
        sender.abort();
        sender.start(message.length, c.maxMessage);
        sender.noteProgress(now);
      }
      continue;
    }

    nth++;
    chunks++;
    if (c.dropNth == nth) {
      sender.noteProgress(now);
      continue;
    }
    final chunk = Uint8List.fromList(wire.sublist(0, send.length));
    if (c.corruptNth == nth && chunk.length > 4) {
      chunk[3] ^= 0x40;
    }

    var received = receiver.onChunk(chunk, buffer, c.maxMessage);
    if (c.duplicateNth == nth) {
      received = receiver.onChunk(chunk, buffer, c.maxMessage);
    }

    switch (received.kind) {
      case ReceivedKind.delivered:
        return _Outcome(
          Uint8List.fromList(buffer.sublist(0, received.length)),
          chunks,
          sender.attempts,
        );
      case ReceivedKind.ack:
        if (!c.swallowAcks) {
          final len = encode(
            Header(received.nextExpected, PipeFlags.ack),
            const [],
            reply,
          );
          final decoded = decode(Uint8List.sublistView(reply, 0, len))!;
          expect(sender.onAck(decoded.header.seq), isNot(Acked.ignored));
        }
        sender.noteProgress(now);
      case ReceivedKind.nak:
        sender.onNak();
        sender.noteProgress(now);
      case ReceivedKind.carryOn:
      case ReceivedKind.dropped:
        sender.noteProgress(now);
    }
  }

  fail('the pump never concluded: ${c.name}');
}
