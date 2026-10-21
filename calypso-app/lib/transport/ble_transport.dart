library;

import 'dart:async';
import 'dart:typed_data';

import 'package:flutter/foundation.dart' show debugPrint;

import '../gatt/gatt_client.dart';
import '../logging/loggers.dart';
import '../model/uuid.dart';
import '../protocol/calypso.pb.dart' as pb;
import '../protocol/pblog.dart';
import '../state/ble_status.dart';
import '../state/calibration.dart';
import '../sync/trace_reassembly.dart';
import 'ble_link.dart';
import 'gatt_link.dart';
import 'transport.dart';

enum _Direction { rx, tx }

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
  });

  final BleLink _link;
  final Duration traceTimeout;
  final Duration chunkIdleTimeout;

  final BleStatusNotifier status = BleStatusNotifier();
  final CalibrationNotifier calibration = CalibrationNotifier();

  String? _deviceId;
  SyncClient? _client;
  StreamSubscription<pb.SyncMessage>? _messagesSub;
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
      _client = SyncClient(BleLinkGattClient(_link, deviceId));
      await _client!.subscribeMessages();
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
    _link.onConnectionChange((id, connected, error) {
      if (id != _deviceId) return;
      if (!connected) _onConnectionLost(error);
    });
    try {
      // The client decodes, so a payload that will not decode arrives here as a
      // stream error rather than as bytes the transport quietly drops. Say so and
      // keep listening: the next notification may be fine, and the alternative is
      // a session with nothing in it to read.
      _messagesSub = _client!.messagesUpdates.listen(
        _onMessage,
        onError: (Object error) => bleLog.warning(
          'undecodable message on ${SyncClient.messagesUuid}: $error',
        ),
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
    _client = null;
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
    _requestTimers.remove(uuid)?.cancel();
    _requestTimers[uuid] = Timer(traceTimeout, () => _timeoutRequest(uuid));

    final request = pb.SyncRequest(
      trace: pb.TraceRequest(
        repUuid: uuidToBytes(uuid) ?? Uint8List(0),
        allChunks: true,
      ),
    );
    // The bytes the client is about to send are logged as they go out; the
    // encode is the same one, done twice on a path that runs once a trace.
    _logMessage(
      _Direction.tx,
      SyncClient.requestsUuid,
      request.writeToBuffer(),
    );
    try {
      await _client!.writeRequests(request);
    } catch (e) {
      _pending.remove(uuid);
      _assemblers.remove(uuid);
      response.ignore();
      bleLog.warning('trace $uuid: request failed: $e');
      throw TraceUnavailableException(uuid, 'request failed: $e');
    }

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
