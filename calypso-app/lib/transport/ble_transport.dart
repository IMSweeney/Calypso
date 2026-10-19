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

import 'package:flutter/foundation.dart' show debugPrint;
import 'package:logging/logging.dart';

import '../model/calibration.dart';
import '../model/rep.dart';
import '../model/uuid.dart';
import '../protocol/calypso.pb.dart' as pb;
import '../state/ble_status.dart';
import '../state/calibration.dart';
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

/// Which way a logged message went: the first word of its log line.
enum _Direction { rx, tx }

/// Every message this transport sends and receives is reported under the `ble`
/// logger, so a session can be read back afterwards: a console scrolls away, and
/// the question that raises ("did the device send anything at all?") gets asked on
/// a phone with nothing attached. Nothing is attached unless the app started a
/// `RotatingFileAppender` — see `lib/logging/setup.dart`.
final Logger _log = Logger('ble');

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

  /// The IMU's calibration levels, as last reported by the device. Empty until
  /// the device sends one, and cleared when the link drops.
  final CalibrationNotifier calibration = CalibrationNotifier();

  String? _deviceId;
  StreamSubscription<Uint8List>? _summarySub;
  StreamSubscription<Uint8List>? _chunkSub;
  final StreamController<RepSummary> _summaryCtrl = StreamController<RepSummary>.broadcast();

  /// Summaries the device has told us about, by rep id.
  final Map<String, pb.RepSummary> _summaries = {};

  /// Messages received, by kind, for the instrumentation line in [_onMessage].
  final Map<pb.SyncMessage_Body, int> _rxCounts = {};

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
      // **Discover before subscribing.** On iOS and macOS the plugin looks the
      // characteristic up in the peripheral's discovered service list, and
      // CoreBluetooth leaves that empty until `discoverServices` has run. Skipping
      // this does not fail as "no services" — it fails as
      // `characteristicNotFound`, naming a UUID that is perfectly correct, which is
      // exactly how it read on the first run against real hardware.
      var services = await _link.discoverServices(deviceId);
      // Instrumentation, not a test: what the device offers and what we wanted,
      // every time, successful or not. Two rounds of this link disagreeing about UUID
      // byte order were settled by exactly this pair of lines, and the firmware logs
      // its side (`sync service uuid …`) at boot.
      debugPrint(
        'calypso: device offers ${_describe(services)}; '
        'want ${CalypsoGatt.service}[${CalypsoGatt.summary}, ${CalypsoGatt.traceChunk}]',
      );
      if (!_offersSync(services)) {
        // CoreBluetooth's tree can still be filling in when discovery returns, so
        // give it one more chance before believing the device doesn't have it.
        await Future<void>.delayed(const Duration(milliseconds: 250));
        services = await _link.discoverServices(deviceId);
      }
      if (!_offersSync(services)) {
        // A device from before the UUID byte order was settled exposes every UUID
        // reversed. That is worth saying plainly: it is a firmware version, not a
        // bug in the app, and the fix is to flash the current firmware.
        if (_offersReversed(services)) {
          // Two causes, and they need different fixes, so name both. The device's
          // Bluetooth address is its identity, so a client that cached the old GATT
          // database keeps seeing it under the *same* identity across reflashes —
          // which is how "reflash it" turned into a circle.
          throw StateError(
            'the device looks like it exposes its UUIDs reversed '
            '(${_reversed(CalypsoGatt.service)}). Either it is running firmware from '
            'before the byte order was settled (reflash it), or this device has a '
            'cached GATT database from an older build — CoreBluetooth caches one per '
            'peripheral address and the plugin prefers it. Try turning Bluetooth '
            'off and on before reflashing.',
          );
        }
        // Otherwise say what the device *does* offer. The plugin's own error for
        // this names the characteristic we asked for, which is correct — so it
        // reads as a device fault while telling us nothing about the device.
        throw StateError(
          'device offers ${_describe(services)}, not ${CalypsoGatt.summary} '
          'under ${CalypsoGatt.service}',
        );
      }
      await _link.subscribe(deviceId, CalypsoGatt.service, CalypsoGatt.summary);
      await _link.subscribe(deviceId, CalypsoGatt.service, CalypsoGatt.traceChunk);
      // Instrumentation: subscribing is the half of this link the app initiates. A
      // hint that never appears has three shapes — never sent, notifications never
      // enabled, or received and ignored — and the device logs the first, this logs
      // the second, and `calypso: rx` logs the third.
      debugPrint(
        'calypso: notifications enabled for ${CalypsoGatt.summary} and '
        '${CalypsoGatt.traceChunk}',
      );
    } catch (e) {
      status.set(BleStatus.failed, deviceId: deviceId, error: '$e');
      return;
    }

    _deviceId = deviceId;
    _link.onConnectionChange((id, connected, error) {
      if (id != _deviceId) return;
      if (!connected) _onConnectionLost(error);
    });
    // Inside a guard too: a stream that cannot be opened is a failed connection,
    // not something to leak into the widget tree as an unhandled error.
    try {
      _summarySub = _link
          .valueStream(deviceId, CalypsoGatt.summary)
          .listen((bytes) => _onMessage(bytes, CalypsoGatt.summary));
      _chunkSub = _link
          .valueStream(deviceId, CalypsoGatt.traceChunk)
          .listen((bytes) => _onMessage(bytes, CalypsoGatt.traceChunk));
    } catch (e) {
      status.set(BleStatus.failed, deviceId: deviceId, error: '$e');
      await _teardownConnection();
      return;
    }

    status.set(BleStatus.connected, deviceId: deviceId);
  }

  /// Whether a discovered tree holds the sync service and its summary
  /// characteristic, the minimum needed to talk to a device.
  static bool _offersSync(List<DiscoveredService> services) => services.any(
        (service) =>
            _sameUuid(service.uuid, CalypsoGatt.service) &&
            service.characteristics.any((c) => _sameUuid(c, CalypsoGatt.summary)),
      );

  /// UUIDs arrive in whatever case the platform prefers — CoreBluetooth gives
  /// upper case — and the protocol's identity is the value, not the spelling.
  static bool _sameUuid(String a, String b) => a.toLowerCase() == b.toLowerCase();

  static String _describe(List<DiscoveredService> services) =>
      services.isEmpty ? 'no services' : services.join('; ');

  /// Whether the device offers the sync service with its UUID bytes reversed — the
  /// signature of firmware from before the two byte orders were sorted out (see the
  /// note in `calypso-core/src/ble.rs`: the same UUID needs one order in the GATT
  /// tree and the opposite in advertising data, and getting it wrong is silent at
  /// both ends).
  static bool _offersReversed(List<DiscoveredService> services) =>
      services.any((service) => _sameUuid(service.uuid, _reversed(CalypsoGatt.service)));

  /// The same UUID with its 16 bytes in the opposite order.
  static String _reversed(String uuid) {
    final clean = uuid.replaceAll('-', '').toLowerCase();
    final bytes = [for (var i = 0; i < 16; i++) clean.substring(i * 2, i * 2 + 2)];
    final out = bytes.reversed.join();
    return '${out.substring(0, 8)}-${out.substring(8, 12)}-${out.substring(12, 16)}'
        '-${out.substring(16, 20)}-${out.substring(20)}';
  }

  Future<void> disconnect() async {
    await _teardownConnection();
    status.set(BleStatus.disconnected);
  }

  Future<void> _teardownConnection() async {
    // The calibration levels belong to a connection, so they go with it.
    calibration.clear();
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
    calibration.clear();
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
    final payload = request.writeToBuffer();
    // Logged before the write, not after: a write that fails is exactly the thing
    // the log is for, and "requested but never sent" is worth telling apart from
    // "never requested".
    _logMessage(_Direction.tx, CalypsoGatt.traceRequest, payload);
    try {
      await _link.write(
        deviceId,
        CalypsoGatt.service,
        CalypsoGatt.traceRequest,
        payload,
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

  /// [characteristic] is which of the two notification characteristics carried
  /// it: the streams are merged here, and only the caller knows which is which, so
  /// the log can say.
  void _onMessage(Uint8List bytes, String characteristic) {
    // Logged before the guards below: the point of the file is what the radio
    // actually delivered, including a straggler from a connection already torn
    // down.
    _logMessage(_Direction.rx, characteristic, bytes);
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
    // Instrumentation: what actually arrives, by kind, with a running count. A
    // screen that shows nothing has two possible causes — never sent, or sent and
    // ignored — and the device logs its side of this, so this is the other half.
    // (Traces arrive dozens of messages to a rep; everything else is rare.)
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
        if (isNew) _summaryCtrl.add(RepSummary.fromProto(summary));
      case pb.SyncMessage_Body.trace:
        _onChunk(message.trace);
      case pb.SyncMessage_Body.status:
        break;
      case pb.SyncMessage_Body.calibration:
        calibration.set(CalibrationStatus.fromProto(message.calibration));
      case pb.SyncMessage_Body.notSet:
        break;
    }
  }

  /// One log line's worth of a BLE message: `<direction> <uuid> <n>B <hex>`.
  ///
  /// The vocabulary is the transport's, not the logger's: the pipeline knows about
  /// timestamps and levels, this knows a GATT characteristic when it sees one.
  void _logMessage(_Direction direction, String characteristic, List<int> bytes) {
    final hex = bytes.map((b) => b.toRadixString(16).padLeft(2, '0'));
    // Joined rather than interpolated so an empty payload ends at its `0B`
    // rather than trailing a space.
    _log.fine([direction.name, characteristic, '${bytes.length}B', ...hex].join(' '));
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
