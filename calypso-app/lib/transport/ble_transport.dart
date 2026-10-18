/// The BLE transport: the device over Bluetooth, behind the same [RepTransport]
/// interface as a fixture, so the sync client and the UI never know the
/// difference.
///
/// All the logic here is protocol and state — scanning, subscribing, decoding
/// `SyncMessage`s, requesting traces, reassembling chunks, knowing when a stream
/// has ended — and it runs on top of [BleLink], the plugin seam, so it is
/// testable with a fake link and no hardware.
library;

import 'dart:async';
import 'dart:typed_data';

import '../model/rep.dart';
import '../model/uuid.dart';
import '../protocol/calypso.pb.dart' as pb;
import '../state/ble_status.dart';
import '../sync/trace_reassembly.dart';
import 'ble_link.dart';
import 'transport.dart';

/// The device's GATT layout, from `calypso-core::ble`.
///
/// A single sync service with three characteristics: a summary notification, a
/// trace-request write, and a trace-chunk notification. Both notification
/// characteristics carry `SyncMessage` envelopes; the *request* is a bare
/// `TraceRequest` (the firmware decodes it directly), which is a small asymmetry
/// worth remembering when reading the two directions side by side.
class CalypsoGatt {
  static const String service = '0000ca20-b0cd-11ec-871f-d45ddf138840';
  static const String summary = '0000ca21-b0cd-11ec-871f-d45ddf138840';
  static const String traceRequest = '0000ca22-b0cd-11ec-871f-d45ddf138840';
  static const String traceChunk = '0000ca23-b0cd-11ec-871f-d45ddf138840';
}

class BleTransport implements RepTransport {
  BleTransport(
    this._link, {
    this.traceTimeout = const Duration(seconds: 10),
    this.chunkIdleTimeout = const Duration(milliseconds: 700),
  });

  final BleLink _link;

  /// How long to wait for a requested trace before giving up. The device answers
  /// by notifying chunks, and stays silent if it no longer holds the rep.
  final Duration traceTimeout;

  /// Completion fallback for a device that doesn't send `totalChunks` (anything
  /// flashed before that field landed): a stream is over when it goes quiet.
  final Duration chunkIdleTimeout;

  /// Connection state, for the UI to observe.
  final BleStatusNotifier status = BleStatusNotifier();

  String? _deviceId;
  StreamSubscription<Uint8List>? _summarySub;
  StreamSubscription<Uint8List>? _chunkSub;
  final StreamController<RepSummary> _summaryCtrl = StreamController<RepSummary>.broadcast();

  /// Summaries the device has told us about, by rep id.
  final Map<String, pb.RepSummary> _summaries = {};

  /// Reassembly per rep in flight.
  final Map<String, TraceAssembler> _assemblers = {};
  final Map<String, int> _expectedChunks = {};
  final Map<String, Timer> _idleTimers = {};
  final Map<String, Completer<List<TracePoint>>> _pending = {};

  /// Traces already received whole, so a second request doesn't re-fetch.
  final Map<String, List<TracePoint>> _completed = {};

  bool _closed = false;

  // ── Connection ──────────────────────────────────────────────────────────

  /// Scan for the device, connect, and subscribe to both notification
  /// characteristics. Repeated calls are safe: they re-scan and re-connect.
  Future<void> connect({Duration scanTimeout = const Duration(seconds: 10)}) async {
    if (_closed) throw StateError('BleTransport is closed');
    await _teardownConnection();

    if (!await _link.isAvailable()) {
      status.set(BleStatus.unsupported,
          error: 'Bluetooth is off, not permitted, or not available');
      return;
    }

    status.set(BleStatus.scanning);
    final found = Completer<String>();
    _link.onScanResult((deviceId, name) {
      if (!found.isCompleted) found.complete(deviceId);
    });

    await _link.startScan([CalypsoGatt.service]);
    String deviceId;
    try {
      deviceId = await found.future.timeout(scanTimeout);
    } on TimeoutException {
      await _link.stopScan();
      status.set(BleStatus.failed, error: 'No Calypso device found');
      return;
    } finally {
      await _link.stopScan();
    }

    status.set(BleStatus.connecting, deviceId: deviceId);
    try {
      await _link.connect(deviceId);
      await _link.subscribe(deviceId, CalypsoGatt.service, CalypsoGatt.summary);
      await _link.subscribe(deviceId, CalypsoGatt.service, CalypsoGatt.traceChunk);
    } catch (e) {
      status.set(BleStatus.failed, deviceId: deviceId, error: '$e');
      return;
    }

    _deviceId = deviceId;
    _link.onConnectionChange((id, connected, error) {
      if (id != _deviceId) return;
      if (!connected) _onConnectionLost(error);
    });
    _summarySub = _link.valueStream(deviceId, CalypsoGatt.summary).listen(_onMessage);
    _chunkSub = _link.valueStream(deviceId, CalypsoGatt.traceChunk).listen(_onMessage);

    status.set(BleStatus.connected, deviceId: deviceId);
  }

  Future<void> disconnect() async {
    await _teardownConnection();
    status.set(BleStatus.disconnected);
  }

  Future<void> _teardownConnection() async {
    // Fire-and-forget the subscription cancels. Awaiting them can block behind an
    // event-loop turn that may never arrive (a widget test's fake clock, or a
    // wedged platform channel), and nothing here needs confirmation that the
    // cancel finished: the listeners are nulled below and `_onMessage` drops
    // anything that still arrives.
    _summarySub?.cancel();
    _chunkSub?.cancel();
    _summarySub = null;
    _chunkSub = null;
    final deviceId = _deviceId;
    _deviceId = null;
    if (deviceId != null) {
      try {
        await _link.disconnect(deviceId);
      } catch (_) {
        // Already gone.
      }
    }
  }

  void _onConnectionLost(String? error) {
    _deviceId = null;
    status.set(BleStatus.disconnected, error: error);
    _failPending('connection lost');
  }

  /// Completes every in-flight trace request with a [TraceUnavailableException]:
  /// the caller can retry, and must not hang.
  void _failPending(String reason) {
    for (final entry in _pending.entries.toList()) {
      entry.value.completeError(TraceUnavailableException(entry.key, reason));
    }
    _pending.clear();
    for (final timer in _idleTimers.values) {
      timer.cancel();
    }
    _idleTimers.clear();
  }

  // ── RepTransport ────────────────────────────────────────────────────────

  @override
  Future<List<RepSummary>> summaries() async {
    final ids = _summaries.keys.toList()..sort();
    return [for (final id in ids) RepSummary.fromProto(_summaries[id]!)];
  }

  @override
  Stream<RepSummary> watchSummaries() => _summaryCtrl.stream;

  @override
  Future<List<TracePoint>> trace(String uuid) async {
    final cached = _completed[uuid];
    if (cached != null) return cached;

    final deviceId = _deviceId;
    if (deviceId == null || !status.isConnected) {
      throw TraceUnavailableException(uuid, 'not connected to a device');
    }

    // A fresh attempt: drop anything left from an earlier, partial stream.
    _assemblers[uuid] = TraceAssembler(uuid);
    _expectedChunks.remove(uuid);
    _idleTimers.remove(uuid)?.cancel();
    final completer = Completer<List<TracePoint>>();
    _pending[uuid] = completer;
    // Take the listener *before* sending the request. The connection can drop
    // between the write and the await, and completing a future that nobody is
    // listening to yet surfaces the error as unhandled rather than delivering it
    // to this caller.
    final response = completer.future.timeout(traceTimeout);

    final request = pb.TraceRequest(
      repUuid: uuidToBytes(uuid) ?? Uint8List(0),
      allChunks: true,
    );
    try {
      await _link.write(
        deviceId,
        CalypsoGatt.service,
        CalypsoGatt.traceRequest,
        request.writeToBuffer(),
      );
    } catch (e) {
      _pending.remove(uuid);
      _assemblers.remove(uuid);
      // The response future is abandoned; mark its errors as handled so a
      // completion racing this failure cannot become an unhandled error.
      response.ignore();
      throw TraceUnavailableException(uuid, 'request failed: $e');
    }

    try {
      return await response;
    } on TimeoutException {
      _pending.remove(uuid);
      final partial = _assemblers.remove(uuid)?.chunkCount ?? 0;
      throw TraceUnavailableException(
        uuid,
        partial == 0
            ? 'the device did not answer (it may no longer hold this rep)'
            : 'the stream stopped after $partial chunks',
      );
    } finally {
      _pending.remove(uuid);
      _idleTimers.remove(uuid)?.cancel();
    }
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

  // ── Notifications ───────────────────────────────────────────────────────

  void _onMessage(Uint8List bytes) {
    // Also dropped: a straggling notification from a connection we have already
    // torn down (the subscription cancel above is not awaited).
    if (_closed || _deviceId == null) return;
    final pb.SyncMessage message;
    try {
      message = pb.SyncMessage.fromBuffer(bytes);
    } catch (_) {
      // A malformed notification is dropped rather than allowed to break the
      // subscription: the device is a separate writer we don't control.
      return;
    }
    switch (message.whichBody()) {
      case pb.SyncMessage_Body.summary:
        final summary = message.summary;
        final uuid = uuidFromBytes(summary.repUuid);
        if (uuid == null) return; // a summary with no identity is not a rep
        final isNew = !_summaries.containsKey(uuid);
        _summaries[uuid] = summary;
        if (isNew) _summaryCtrl.add(RepSummary.fromProto(summary));
      case pb.SyncMessage_Body.trace:
        _onChunk(message.trace);
      case pb.SyncMessage_Body.status:
        break;
      case pb.SyncMessage_Body.notSet:
        break;
    }
  }

  void _onChunk(pb.TraceChunk chunk) {
    final uuid = uuidFromBytes(chunk.repUuid);
    if (uuid == null) return;
    final assembler = _assemblers.putIfAbsent(uuid, () => TraceAssembler(uuid));
    assembler.add(chunk);

    if (chunk.totalChunks > 0) {
      // The device told us how many chunks this stream has, so completion is
      // exact: every index 0..total-1 arrived.
      _expectedChunks[uuid] = chunk.totalChunks;
      if (assembler.indices.length == chunk.totalChunks && assembler.missingIndices().isEmpty) {
        _complete(uuid);
      }
      return;
    }

    // No total (firmware from before that field): a quiet stream is over.
    _idleTimers[uuid]?.cancel();
    _idleTimers[uuid] = Timer(chunkIdleTimeout, () => _complete(uuid));
  }

  void _complete(String uuid) {
    _idleTimers.remove(uuid)?.cancel();
    final points = _assemblers.remove(uuid)?.assemble() ?? const <TracePoint>[];
    _expectedChunks.remove(uuid);
    if (points.isEmpty) return;
    _completed[uuid] = points;
    final pending = _pending.remove(uuid);
    if (pending != null && !pending.isCompleted) pending.complete(points);
  }
}
