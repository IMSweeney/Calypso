library;

import 'dart:async';
import 'dart:typed_data';

import 'package:flutter/foundation.dart' show debugPrint;

import '../gatt/gatt_client.dart';
import '../gatt/pipe.dart' as pipe;
import '../logging/loggers.dart';
import '../model/uuid.dart';
import '../protocol/calypso.pb.dart' as pb;
import '../protocol/pblog.dart';
import '../state/ble_status.dart';
import '../state/calibration.dart';
import '../sync/trace_reassembly.dart';
import 'ble_link.dart';
import 'transport.dart';

enum _Direction { rx, tx }

/// The ATT MTU every chunk is sized from until the link has been asked.
///
/// 23 is the one every link can carry (20 bytes of ATT payload), so a chunk built
/// from it is a chunk any link takes: slow, and never truncated.
const int _minimumMtu = 23;

/// How long the device has to acknowledge a request before it is sent again —
/// three attempts of this, and the request has failed.
///
/// One round trip is all it is: the device acknowledges the last chunk as soon as
/// it has the request, so seconds mean a link that is there. It is deliberately
/// *not* the response budget — how long the trace takes to arrive is
/// `traceTimeout`, and that starts when the request lands.
const Duration _defaultXferTimeout = Duration(seconds: 2);

/// Reaching the device through the generated client: [SyncClient] and its
/// `GattClientLink` come from `proto/calypso_gatt.proto` (see
/// `lib/gatt/gatt_client.dart`), so the UUIDs, the directions and the message
/// types are the schema's rather than this file's.
///
/// What stays here is what a schema cannot say: scanning, connecting, the trace
/// reassembly and its timeouts, and what the archive is told.
class BleTransport implements RepTransport {
  BleTransport(
    this._link, {
    this.traceTimeout = const Duration(seconds: 30),
    this.chunkIdleTimeout = const Duration(milliseconds: 700),
    Duration xferTimeout = _defaultXferTimeout,
  }) : _xferTimeout = xferTimeout;

  final BleLink _link;
  final Duration traceTimeout;
  final Duration chunkIdleTimeout;

  /// How long the device has to acknowledge a request, per attempt.
  final Duration _xferTimeout;

  final BleStatusNotifier status = BleStatusNotifier();
  final CalibrationNotifier calibration = CalibrationNotifier();

  String? _deviceId;
  StreamSubscription<Uint8List>? _messagesSub;

  /// The app's end of the pipe: one receiver for everything the device sends, and
  /// one sender for everything the app asks with. Both directions are the same
  /// frame on the same two characteristics, so what arrives has to be sorted by the
  /// frame rather than by where it came from (`_onNotification`).
  final pipe.Receiver _receiver = pipe.Receiver(SyncPipe.window);
  final pipe.Sender _sender = pipe.Sender(SyncPipe.window);
  final Uint8List _inbound = Uint8List(SyncPipe.messageMax);
  Completer<void>? _acknowledged;

  /// The ATT MTU the chunk size is computed from, read once per connection.
  int _mtu = _minimumMtu;
  final StreamController<pb.RepSummary> _summaryCtrl =
      StreamController<pb.RepSummary>.broadcast();

  final Map<String, pb.RepSummary> _summaries = {};

  final Map<pb.SyncMessage_Body, int> _rxCounts = {};

  final Map<String, TraceAssembler> _assemblers = {};
  final Map<String, int> _expectedChunks = {};
  final Map<String, Timer> _idleTimers = {};

  final Map<String, Timer> _requestTimers = {};
  final Map<String, Completer<List<pb.TraceSample>>> _pending = {};

  final Map<String, List<pb.TraceSample>> _completed = {};

  bool _closed = false;

  Future<void> connect({
    Duration scanTimeout = const Duration(seconds: 10),
  }) async {
    if (_closed) throw StateError('BleTransport is closed');
    await _teardownConnection();

    if (!await _link.isAvailable()) {
      bleLog.warning(
        'bluetooth unavailable: off, not permitted, or not present',
      );
      status.set(
        BleStatus.unsupported,
        error: 'Bluetooth is off, not permitted, or not available',
      );
      return;
    }

    bleLog.info('scanning for ${SyncClient.serviceUuid}');
    status.set(BleStatus.scanning);
    final found = Completer<String>();
    _link.onScanResult((deviceId, name) {
      if (!found.isCompleted) found.complete(deviceId);
    });

    await _link.startScan([SyncClient.serviceUuid]);
    String deviceId;
    try {
      deviceId = await found.future.timeout(scanTimeout);
    } on TimeoutException {
      await _link.stopScan();
      bleLog.warning(
        'no Calypso device found within ${scanTimeout.inSeconds}s',
      );
      status.set(BleStatus.failed, error: 'No Calypso device found');
      return;
    } finally {
      await _link.stopScan();
    }

    bleLog.info('connecting to $deviceId');
    status.set(BleStatus.connecting, deviceId: deviceId);
    try {
      await _link.connect(deviceId);
      var services = await _link.discoverServices(deviceId);
      debugPrint(
        'calypso: device offers ${_describe(services)}; '
        'want ${SyncClient.serviceUuid}[${SyncClient.messagesUuid}, ${SyncClient.requestsUuid}]',
      );
      if (!_offersSync(services)) {
        await Future<void>.delayed(const Duration(milliseconds: 250));
        services = await _link.discoverServices(deviceId);
      }
      if (!_offersSync(services)) {
        if (_offersReversed(services)) {
          throw StateError(
            'the device looks like it exposes its UUIDs reversed '
            '(${_reversed(SyncClient.serviceUuid)}). Either it is running firmware from '
            'before the byte order was settled (reflash it), or this device has a '
            'cached GATT database from an older build — CoreBluetooth caches one per '
            'peripheral address and the plugin prefers it. Try turning Bluetooth '
            'off and on before reflashing.',
          );
        }
        throw StateError(
          'device offers ${_describe(services)}, not both characteristics of '
          '${SyncClient.serviceUuid} (${SyncClient.messagesUuid}, ${SyncClient.requestsUuid})',
        );
      }
      // Ask for the most the pipe can use — a chunk's frame is what it is sized
      // from — and take what the link gives rather than what was asked for.
      try {
        _mtu = await _link.requestMtu(deviceId, SyncPipe.chunkMax + 3);
        bleLog.info('ATT MTU $_mtu');
      } catch (e) {
        // A link that will not say: the smallest MTU every stack guarantees is
        // slower and never wrong.
        _mtu = _minimumMtu;
        bleLog.warning(
          'could not negotiate an MTU ($e); sizing chunks for $_mtu',
        );
      }

      await _link.subscribe(
        deviceId,
        SyncClient.serviceUuid,
        SyncClient.messagesUuid,
      );
      debugPrint(
        'calypso: notifications enabled for ${SyncClient.messagesUuid}',
      );
      bleLog.info('subscribed to ${SyncClient.messagesUuid}');
    } catch (e) {
      bleLog.warning('could not connect to $deviceId: $e');
      status.set(BleStatus.failed, deviceId: deviceId, error: '$e');
      return;
    }

    _deviceId = deviceId;
    _receiver.reset();
    _sender.reset();
    _link.onConnectionChange((id, connected, error) {
      if (id != _deviceId) return;
      if (!connected) _onConnectionLost(error);
    });
    try {
      // Raw frames, not messages: a notification is a chunk of a transfer until
      // the pipe has reassembled it, and the ones that are acknowledgements of the
      // app's own transfer never become a message at all.
      _messagesSub = _link
          .valueStream(deviceId, SyncClient.messagesUuid)
          .listen(
            _onNotification,
            onError: (Object error) =>
                bleLog.warning('error on ${SyncClient.messagesUuid}: $error'),
          );
    } catch (e) {
      bleLog.warning('could not listen to $deviceId: $e');
      status.set(BleStatus.failed, deviceId: deviceId, error: '$e');
      await _teardownConnection();
      return;
    }

    bleLog.info('connected to $deviceId');
    status.set(BleStatus.connected, deviceId: deviceId);
  }

  static bool _offersSync(List<DiscoveredService> services) => services.any(
    (service) =>
        _sameUuid(service.uuid, SyncClient.serviceUuid) &&
        service.characteristics.any(
          (c) => _sameUuid(c, SyncClient.messagesUuid),
        ) &&
        service.characteristics.any(
          (c) => _sameUuid(c, SyncClient.requestsUuid),
        ),
  );

  static bool _sameUuid(String a, String b) =>
      a.toLowerCase() == b.toLowerCase();

  static String _describe(List<DiscoveredService> services) =>
      services.isEmpty ? 'no services' : services.join('; ');

  static bool _offersReversed(List<DiscoveredService> services) => services.any(
    (service) => _sameUuid(service.uuid, _reversed(SyncClient.serviceUuid)),
  );

  static String _reversed(String uuid) {
    final clean = uuid.replaceAll('-', '').toLowerCase();
    final bytes = [
      for (var i = 0; i < 16; i++) clean.substring(i * 2, i * 2 + 2),
    ];
    final out = bytes.reversed.join();
    return '${out.substring(0, 8)}-${out.substring(8, 12)}-${out.substring(12, 16)}'
        '-${out.substring(16, 20)}-${out.substring(20)}';
  }

  Future<void> disconnect() async {
    await _teardownConnection();
    status.set(BleStatus.disconnected);
  }

  Future<void> _teardownConnection() async {
    calibration.clear();
    _messagesSub?.cancel();
    _messagesSub = null;
    // Whatever was in flight went with the link: half a message is not a message.
    _receiver.reset();
    _sender.reset();
    _acknowledged = null;
    final deviceId = _deviceId;
    _deviceId = null;
    if (deviceId != null) {
      try {
        await _link.disconnect(deviceId);
      } catch (_) {}
    }
  }

  void _onConnectionLost(String? error) {
    bleLog.info('connection lost${error == null ? '' : ': $error'}');
    _deviceId = null;
    calibration.clear();
    status.set(BleStatus.disconnected, error: error);
    _failPending('connection lost');
  }

  void _failPending(String reason) {
    for (final entry in _pending.entries.toList()) {
      entry.value.completeError(TraceUnavailableException(entry.key, reason));
    }
    _pending.clear();
    for (final timer in _idleTimers.values) {
      timer.cancel();
    }
    _idleTimers.clear();
    for (final timer in _requestTimers.values) {
      timer.cancel();
    }
    _requestTimers.clear();
  }

  @override
  Future<List<pb.RepSummary>> summaries() async {
    final ids = _summaries.keys.toList()..sort();
    return [for (final id in ids) _summaries[id]!];
  }

  @override
  Stream<pb.RepSummary> watchSummaries() => _summaryCtrl.stream;

  @override
  Future<List<pb.TraceSample>> trace(String uuid) async {
    final cached = _completed[uuid];
    if (cached != null) return cached;

    final deviceId = _deviceId;
    if (deviceId == null || !status.isConnected) {
      throw TraceUnavailableException(uuid, 'not connected to a device');
    }

    _assemblers[uuid] = TraceAssembler(uuid);
    _expectedChunks.remove(uuid);
    _idleTimers.remove(uuid)?.cancel();
    final completer = Completer<List<pb.TraceSample>>();
    _pending[uuid] = completer;
    final response = completer.future;
    // The failure can arrive while the request is still going out — a dropped
    // connection fails it there — and a completer nobody is listening to yet
    // reports that as an unhandled error rather than to the caller that will
    // await it below.
    unawaited(response.then((_) {}, onError: (Object _) {}));
    _requestTimers.remove(uuid)?.cancel();

    final request = pb.SyncRequest(
      trace: pb.TraceRequest(
        repUuid: uuidToBytes(uuid) ?? Uint8List(0),
        allChunks: true,
      ),
    );
    try {
      // The request itself is a transfer now, and it is acknowledged before the
      // device answers: the response budget starts when it lands, not when it is
      // sent, or a slow link would spend it on the request.
      await _request(request);
    } catch (e) {
      _pending.remove(uuid);
      _assemblers.remove(uuid);
      bleLog.warning('trace $uuid: request failed: $e');
      throw TraceUnavailableException(uuid, 'request failed: $e');
    }
    _requestTimers[uuid] = Timer(traceTimeout, () => _timeoutRequest(uuid));

    try {
      final points = await response;
      bleLog.info('trace $uuid: ${points.length} points');
      return points;
    } finally {
      _pending.remove(uuid);
      _idleTimers.remove(uuid)?.cancel();
      _requestTimers.remove(uuid)?.cancel();
    }
  }

  void _timeoutRequest(String uuid) {
    _requestTimers.remove(uuid);
    final pending = _pending.remove(uuid);
    if (pending == null || pending.isCompleted) return;
    final partial = _assemblers.remove(uuid)?.chunkCount ?? 0;
    bleLog.warning(
      partial == 0
          ? 'trace $uuid: no answer within ${traceTimeout.inSeconds}s'
          : 'trace $uuid: stream stopped after $partial chunks',
    );
    pending.completeError(
      TraceUnavailableException(
        uuid,
        partial == 0
            ? 'the device did not answer (it may no longer hold this rep)'
            : 'the stream stopped after $partial chunks',
      ),
    );
  }

  @override
  Future<void> close() async {
    _closed = true;
    await _teardownConnection();
    _failPending('transport closed');
    _completed.clear();
    await _summaryCtrl.close();
    status.set(BleStatus.idle);
  }

  /// Send one request to the device, as a transfer, and wait for the device to
  /// acknowledge it.
  ///
  /// The wait is the point of the pipe: the written characteristic has no write
  /// response, so a write reports nothing at all — the acknowledgement of the last
  /// chunk, on the other characteristic, is the only evidence the device has the
  /// request. No acknowledgement within `xferTimeout` sends the whole thing
  /// again, from the start.
  Future<void> _request(pb.SyncRequest request) async {
    final bytes = SyncPipe.encodeRequest(request);
    _logMessage(_Direction.tx, SyncClient.requestsUuid, bytes);

    for (var attempt = 1; attempt <= pipe.maxAttempts; attempt++) {
      // Whatever the attempt before this left in flight is gone: a transfer that
      // was not acknowledged is a transfer to send again, from the start.
      _sender.abort();
      _sender.start(bytes.length, SyncPipe.messageMax);
      _acknowledged = Completer<void>();
      await _writeChunks(bytes);
      try {
        await _acknowledged!.future.timeout(_xferTimeout);
        return;
      } on TimeoutException {
        bleLog.warning(
          'sync: no acknowledgement of the request '
          '(attempt $attempt of ${pipe.maxAttempts})',
        );
      }
    }
    _sender.abort();
    throw StateError(
      'the device did not answer: no acknowledgement of the request',
    );
  }

  /// Write every chunk of one transfer, as far as the window allows.
  Future<void> _writeChunks(Uint8List bytes) async {
    final capacity = pipe.capacity(_mtu, SyncPipe.chunkMax, bytes.length);
    if (capacity == null) {
      throw StateError('a $_mtu-byte link cannot carry a chunk');
    }
    final frame = Uint8List(SyncPipe.chunkMax);
    while (true) {
      final send = _sender.next(bytes, capacity, frame);
      if (send.state == pipe.SendState.done) return;
      if (send.state == pipe.SendState.windowFull) {
        // The window is 127 chunks and a request is tens of bytes: reaching this
        // means a message that should not exist, so it is said rather than waited
        // on.
        throw StateError(
          'the request did not fit one window (${bytes.length} bytes)',
        );
      }
      await _writeFrame(Uint8List.sublistView(frame, 0, send.length));
    }
  }

  /// Write one frame to the characteristic the app talks on.
  Future<void> _writeFrame(Uint8List frame) async {
    final deviceId = _deviceId;
    if (deviceId == null) return;
    bleLog.fine(
      describeMessage(_Direction.tx.name, SyncClient.requestsUuid, frame),
    );
    try {
      await _link.write(
        deviceId,
        SyncClient.serviceUuid,
        SyncClient.requestsUuid,
        frame,
        withResponse: false,
      );
    } catch (e) {
      // The device's retry is the recovery: it will not see the chunk.
      bleLog.warning('could not write a chunk: $e');
    }
  }

  /// One value from the device: a chunk of a transfer, or the acknowledgement of
  /// the app's own.
  void _onNotification(Uint8List value) {
    bleLog.fine(
      describeMessage(_Direction.rx.name, SyncClient.messagesUuid, value),
    );
    if (_closed || _deviceId == null) return;

    final frame = pipe.decode(value);
    if (frame == null) {
      bleLog.warning(
        'a frame on ${SyncClient.messagesUuid} does not check out',
      );
      return;
    }
    if (frame.header.isReply) {
      if (frame.header.isNak) {
        // The device refused the transfer: the whole thing goes again.
        bleLog.warning('sync: the device refused a chunk (nak)');
        _sender.onNak();
        return;
      }
      final acked = _sender.onAck(frame.header.seq);
      bleLog.fine('${_Direction.rx.name} ack seq=${frame.header.seq} $acked');
      if (acked == pipe.Acked.complete) _acknowledged?.complete();
      return;
    }

    final received = _receiver.onChunk(value, _inbound, SyncPipe.messageMax);
    switch (received.kind) {
      case pipe.ReceivedKind.ack:
      case pipe.ReceivedKind.nak:
        unawaited(_writeReply(_receiver.nextExpected));
      case pipe.ReceivedKind.delivered:
        // The end of a message is where an acknowledgement is owed, whatever the
        // window's cadence says: the device's sender is waiting on it to start
        // anything else.
        unawaited(_writeReply(_receiver.nextExpected));
        _onMessage(
          SyncPipe.decodeResponse(
            Uint8List.sublistView(_inbound, 0, received.length),
          ),
        );
      case pipe.ReceivedKind.dropped:
        bleLog.warning('sync: a frame on the pipe did not check out');
      case pipe.ReceivedKind.carryOn:
        break;
    }
  }

  /// The one frame the app owes the device: an acknowledgement, on the
  /// characteristic the device waits on.
  Future<void> _writeReply(int nextExpected) async {
    final frame = Uint8List(8);
    final length = pipe.encode(
      pipe.Header(nextExpected, pipe.PipeFlags.ack),
      const <int>[],
      frame,
    );
    await _writeFrame(Uint8List.sublistView(frame, 0, length));
  }

  void _onMessage(pb.SyncMessage message) {
    // The bytes are logged at the plugin seam (`raw`); this is what the
    // transport made of them.
    bleLog.fine(
      '${_Direction.rx.name} ${SyncClient.messagesUuid} '
      '${describeSyncMessage(message)}',
    );
    if (_closed || _deviceId == null) return;
    final kind = message.whichBody();
    final seen = _rxCounts.update(kind, (n) => n + 1, ifAbsent: () => 1);
    if (kind != pb.SyncMessage_Body.trace || seen % 60 == 0) {
      debugPrint('calypso: rx #$seen $kind');
    }

    switch (kind) {
      case pb.SyncMessage_Body.summary:
        final summary = message.summary;
        final uuid = uuidFromBytes(summary.repUuid);
        if (uuid == null) return; // a summary with no identity is not a rep
        final isNew = !_summaries.containsKey(uuid);
        _summaries[uuid] = summary;
        if (isNew) _summaryCtrl.add(summary);
      case pb.SyncMessage_Body.trace:
        _onChunk(message.trace);
      case pb.SyncMessage_Body.status:
        break;
      case pb.SyncMessage_Body.calibration:
        calibration.set(message.calibration);
      case pb.SyncMessage_Body.notSet:
        break;
    }
  }

  void _logMessage(
    _Direction direction,
    String characteristic,
    List<int> bytes,
  ) {
    bleLog.fine(describeMessage(direction.name, characteristic, bytes));
  }

  void _onChunk(pb.TraceChunk chunk) {
    final uuid = uuidFromBytes(chunk.repUuid);
    if (uuid == null) return;
    final waiting = _requestTimers[uuid];
    if (waiting != null) {
      waiting.cancel();
      _requestTimers[uuid] = Timer(traceTimeout, () => _timeoutRequest(uuid));
    }
    final assembler = _assemblers.putIfAbsent(uuid, () => TraceAssembler(uuid));
    assembler.add(chunk);

    if (chunk.totalChunks > 0) {
      _expectedChunks[uuid] = chunk.totalChunks;
      if (assembler.indices.length == chunk.totalChunks &&
          assembler.missingIndices().isEmpty) {
        _complete(uuid);
      }
      return;
    }

    _idleTimers[uuid]?.cancel();
    _idleTimers[uuid] = Timer(chunkIdleTimeout, () => _complete(uuid));
  }

  void _complete(String uuid) {
    _idleTimers.remove(uuid)?.cancel();
    final points =
        _assemblers.remove(uuid)?.assemble() ?? const <pb.TraceSample>[];
    _expectedChunks.remove(uuid);
    if (points.isEmpty) return;
    _completed[uuid] = points;
    final pending = _pending.remove(uuid);
    if (pending != null && !pending.isCompleted) pending.complete(points);
  }
}
