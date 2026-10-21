library;

import 'dart:typed_data';

/// The chunked pipe, for the app's side of a link whose characteristics are a
/// pair.
///
/// A mirror of `tools/gatt-pipe` (Rust), which is the reference: the frame, the
/// sender and the receiver, with no buffers of their own and no clock — the caller
/// brings both. Both implementations are checked against the same cases in
/// `tools/gatt-pipe/tests/vectors/pipe.txt`, because two implementations of one
/// protocol is how two implementations of one protocol drift apart.
///
/// The frame:
///
/// ```
/// | seq u8 | flags u8 | [total_len varint, START only] | payload | crc16 LE |
/// ```
///
/// `seq` is the chunk's own number on data and *cumulative* — the next number its
/// sender expects — on an acknowledgement, which is the decision the reference
/// documents at length.
class PipeFlags {
  static const int start = 0x01;
  static const int end = 0x02;
  static const int ack = 0x04;
  static const int nak = 0x08;
  static const int known = start | end | ack | nak;
}

/// `seq` and `flags`.
const int _header = 2;

/// CRC-16, little endian.
const int _crc = 2;

/// The ATT header a notification or a write costs: opcode and a two-byte handle.
const int _attHeader = 3;

/// The largest window a cumulative acknowledgement can describe with a `u8`
/// sequence number.
const int maxWindow = 127;

/// The largest chunk any link is asked to carry: a characteristic's storage is a
/// constant on the device, so the frame has a ceiling of its own.
const int defaultChunkMax = 244;

/// How many transfers a sender makes before giving up on one.
const int maxAttempts = 3;

/// CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no reflection.
int crc16(List<int> bytes) {
  var crc = 0xFFFF;
  for (final byte in bytes) {
    crc ^= (byte & 0xFF) << 8;
    for (var i = 0; i < 8; i++) {
      crc = (crc & 0x8000) != 0
          ? ((crc << 1) ^ 0x1021) & 0xFFFF
          : (crc << 1) & 0xFFFF;
    }
  }
  return crc;
}

/// Bytes a varint needs for `value`.
int varintLen(int value) {
  if (value <= 0x7F) return 1;
  if (value <= 0x3FFF) return 2;
  if (value <= 0x1FFFFF) return 3;
  if (value <= 0x0FFFFFFF) return 4;
  return 5;
}

/// Bytes of a chunk that are not payload.
int overhead(int? totalLen) =>
    _header + _crc + (totalLen == null ? 0 : varintLen(totalLen));

/// How much payload fits in one chunk of a link whose ATT MTU is [mtu].
///
/// Two ceilings, whichever is lower: the link's ([mtu] less the ATT header and
/// the frame), and the buffer's — [chunkMax] is the characteristic's backing
/// buffer and holds the *frame*, so the header and the CRC come out of it too.
/// A peer negotiating more than `chunkMax + 3` would otherwise get a chunk too
/// big for the buffer it is built in.
int? capacity(int mtu, int chunkMax, int totalLen) {
  final frame = overhead(totalLen);
  final usable = mtu - _attHeader - frame;
  final buffer = chunkMax - frame;
  if (usable < 0 || buffer < 0) return null;
  final limit = usable < buffer ? usable : buffer;
  return limit == 0 ? null : limit;
}

/// A decoded frame header.
class Header {
  const Header(this.seq, this.flags, [this.totalLen]);

  final int seq;
  final int flags;

  /// The message's encoded length, on a START.
  final int? totalLen;

  bool get isStart => flags & PipeFlags.start != 0;
  bool get isEnd => flags & PipeFlags.end != 0;
  bool get isAck => flags & PipeFlags.ack != 0;
  bool get isNak => flags & PipeFlags.nak != 0;
  bool get isReply => flags & (PipeFlags.ack | PipeFlags.nak) != 0;
}

/// Why a chunk could not be read.
enum DecodeError { short, crc, badFrame }

/// A decoded chunk: its header and its payload, which is a view into [bytes].
class Decoded {
  const Decoded(this.header, this.payload);
  final Header header;
  final Uint8List payload;
}

/// Read a frame, or say why not.
Decoded? decode(Uint8List chunk) {
  if (chunk.length < _header + _crc) return null;
  final body = chunk.sublist(0, chunk.length - _crc);
  final expected =
      chunk[chunk.length - _crc] | (chunk[chunk.length - _crc + 1] << 8);
  if (crc16(body) != expected) return null;

  final seq = body[0];
  final flags = body[1];
  if (flags & ~PipeFlags.known != 0) return null;

  var at = _header;
  int? totalLen;
  if (flags & PipeFlags.start != 0) {
    final read = readVarint(body, at);
    if (read == null) return null;
    totalLen = read.value;
    at = read.next;
  }
  final payload = Uint8List.sublistView(body, at);
  // An acknowledgement says where the receiver is, not what it received.
  if (flags & (PipeFlags.ack | PipeFlags.nak) != 0 && payload.isNotEmpty) {
    return null;
  }
  return Decoded(Header(seq, flags, totalLen), payload);
}

/// Write a frame into [out], returning its length.
int encode(Header header, List<int> payload, Uint8List out) {
  final length = header.totalLen == null ? 0 : varintLen(header.totalLen!);
  final total = _header + length + payload.length + _crc;
  out[0] = header.seq & 0xFF;
  out[1] = header.flags;
  var at = _header;
  if (header.totalLen != null) {
    at += writeVarint(header.totalLen!, out, at);
  }
  out.setRange(at, at + payload.length, payload);
  final end = at + payload.length;
  final crc = crc16(Uint8List.sublistView(out, 0, end));
  out[end] = crc & 0xFF;
  out[end + 1] = (crc >> 8) & 0xFF;
  return total;
}

/// A varint read: its value and where it ended.
class Varint {
  const Varint(this.value, this.next);
  final int value;
  final int next;
}

Varint? readVarint(Uint8List bytes, int at) {
  var value = 0;
  var shift = 0;
  var i = at;
  while (i < bytes.length) {
    if (shift > 28) return null;
    value |= (bytes[i] & 0x7F) << shift;
    i++;
    if (bytes[i - 1] & 0x80 == 0) return Varint(value, i);
    shift += 7;
  }
  return null;
}

int writeVarint(int value, Uint8List out, int at) {
  var v = value;
  var i = at;
  while (true) {
    final byte = v & 0x7F;
    v >>= 7;
    if (v == 0) {
      out[i] = byte;
      return i - at + 1;
    }
    out[i] = byte | 0x80;
    i++;
  }
}

/// What a sender wants to do next.
enum SendState { chunk, windowFull, done }

class Send {
  const Send._(this.state, this.length);
  const Send.chunk(int length) : this._(SendState.chunk, length);
  const Send.windowFull() : this._(SendState.windowFull, 0);
  const Send.done() : this._(SendState.done, 0);

  final SendState state;
  final int length;
}

/// What an acknowledgement did.
enum Acked { ignored, advanced, complete }

/// The sending half: one message at a time, at most [window] chunks in flight.
class Sender {
  Sender(this.window) {
    if (window < 1 || window > maxWindow) {
      throw ArgumentError(
        'a window of $window cannot be acknowledged by a u8 sequence',
      );
    }
  }

  final int window;

  int _seq = 0;
  int _offset = 0;
  int _messageLen = 0;
  int _inFlight = 0;
  bool _active = false;
  bool _endSent = false;
  int _attempts = 0;
  int _progressMs = 0;

  int get attempts => _attempts;
  bool get isIdle => !_active && _inFlight == 0;

  /// Begin a transfer of [messageLen] bytes, giving up if the peer's reassembly
  /// buffer could not hold it.
  void start(int messageLen, int maxMessage) {
    if (!isIdle) throw StateError('a transfer is already in flight');
    if (messageLen > maxMessage) {
      throw ArgumentError(
        'a $messageLen-byte message does not fit a $maxMessage-byte pipe',
      );
    }
    _messageLen = messageLen;
    _offset = 0;
    _inFlight = 0;
    _active = true;
    _endSent = false;
    _attempts++;
  }

  /// The next chunk to send, written into [out].
  Send next(Uint8List message, int capacity, Uint8List out) {
    if (!_active) return const Send.done();
    if (_inFlight >= window) return const Send.windowFull();

    final remaining = _messageLen - _offset;
    final take = capacity < remaining ? capacity : remaining;
    final start = _offset == 0;
    final end = _offset + take == _messageLen;

    var flags = 0;
    if (start) flags |= PipeFlags.start;
    if (end) flags |= PipeFlags.end;

    final written = encode(
      Header(_seq, flags, start ? _messageLen : null),
      Uint8List.sublistView(message, _offset, _offset + take),
      out,
    );

    _offset += take;
    _seq = (_seq + 1) & 0xFF;
    _inFlight++;
    if (end) _endSent = true;
    return Send.chunk(written);
  }

  /// An acknowledgement naming the next in-order sequence the peer expects.
  Acked onAck(int nextExpected) {
    if (_inFlight == 0) return Acked.ignored;
    final unacked = (_seq - nextExpected) & 0xFF;
    if (unacked >= _inFlight) return Acked.ignored;
    _inFlight = unacked;
    if (_inFlight == 0 && _endSent) {
      _messageLen = 0;
      _offset = 0;
      _endSent = false;
      _active = false;
      return Acked.complete;
    }
    return Acked.advanced;
  }

  /// The peer refused the transfer: send it again from the beginning.
  void onNak() {
    _offset = 0;
    _inFlight = 0;
    _endSent = false;
  }

  void noteProgress(int nowMs) => _progressMs = nowMs;

  /// Whether the transfer has made no progress for [timeoutMs].
  bool stalled(int nowMs, int timeoutMs) =>
      !isIdle && nowMs - _progressMs >= timeoutMs;

  /// Give up on the transfer, keeping the count of attempts it took.
  void abort() {
    _messageLen = 0;
    _offset = 0;
    _inFlight = 0;
    _endSent = false;
    _active = false;
  }

  /// Forget everything, including the attempt count: a new connection.
  void reset() {
    abort();
    _attempts = 0;
    _seq = 0;
    _progressMs = 0;
  }
}

/// What a receiver made of a chunk.
enum ReceivedKind {
  /// In order, and no acknowledgement is due yet.
  carryOn,

  /// Answer with an ACK naming [Received.nextExpected].
  ack,

  /// Answer with a NAK.
  nak,

  /// A whole message: [Received.length] bytes are in the caller's buffer.
  delivered,

  /// A frame that could not be read, dropped: the sender's timeout is the way
  /// out, because a corrupted acknowledgement is no better than none.
  dropped,
}

class Received {
  const Received(this.kind, {this.nextExpected = 0, this.length = 0});
  final ReceivedKind kind;
  final int nextExpected;
  final int length;
}

/// The receiving half: reassembles one message at a time into the caller's
/// buffer.
class Receiver {
  Receiver(this.window) {
    if (window < 1 || window > maxWindow) {
      throw ArgumentError(
        'a window of $window cannot be acknowledged by a u8 sequence',
      );
    }
  }

  final int window;

  int _seq = 0;
  int? _expected;
  int _used = 0;
  int _sinceAck = 0;
  bool _repeated = false;

  bool get inTransfer => _expected != null;

  /// Take one chunk. [buffer] is the reassembly buffer and [maxMessage] its size.
  Received onChunk(Uint8List chunk, Uint8List buffer, int maxMessage) {
    final frame = decode(chunk);
    if (frame == null) {
      _discard();
      return const Received(ReceivedKind.dropped);
    }
    final header = frame.header;
    final payload = frame.payload;

    if (header.isReply) return const Received(ReceivedKind.carryOn);

    if (header.isStart) {
      final expected = header.totalLen ?? 0;
      if (expected > maxMessage || expected > buffer.length) {
        _discard();
        return const Received(ReceivedKind.nak);
      }
      _expected = expected;
      _used = 0;
      _seq = header.seq;
      _sinceAck = 0;
      _repeated = false;
    } else if (_expected == null) {
      _repeated = true;
      return const Received(ReceivedKind.nak);
    }

    if (header.seq != _seq) {
      if ((header.seq + 1) & 0xFF == _seq) {
        _repeated = true;
        return _acknowledge();
      }
      _discard();
      return const Received(ReceivedKind.nak);
    }

    final expected = _expected ?? 0;
    if (_used + payload.length > expected) {
      _discard();
      return const Received(ReceivedKind.nak);
    }
    buffer.setRange(_used, _used + payload.length, payload);
    _used += payload.length;
    _seq = (_seq + 1) & 0xFF;
    _sinceAck++;

    if (header.isEnd) {
      if (_used != expected) {
        _discard();
        return const Received(ReceivedKind.nak);
      }
      final length = _used;
      _discard();
      return Received(ReceivedKind.delivered, length: length);
    }

    return _acknowledge();
  }

  void _discard() {
    _expected = null;
    _used = 0;
    _sinceAck = 0;
    _repeated = false;
  }

  Received _acknowledge() {
    final cadence = (window ~/ 2) < 1 ? 1 : window ~/ 2;
    if (_repeated || _sinceAck >= cadence) {
      _sinceAck = 0;
      _repeated = false;
      return Received(ReceivedKind.ack, nextExpected: _seq);
    }
    return const Received(ReceivedKind.carryOn);
  }

  /// Forget everything: a new connection, or a transfer this end is abandoning.
  void reset() {
    _discard();
    _seq = 0;
  }
}
