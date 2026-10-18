/// The sync client: merges summaries from a transport, and fetches traces.
///
/// It owns the client's copy of the log in memory and is the only thing that
/// talks to a [RepTransport]. Because the device's log is a ring that erases
/// its oldest sectors, the client is the archive of record: reps it has seen
/// are never dropped just because the device no longer has them.
library;

import 'dart:async';

import '../model/rep.dart';
import '../transport/transport.dart';

/// What the sync client is doing, for the UI to display.
enum SyncState {
  /// Nothing has happened yet.
  idle,

  /// Catch-up from the transport is in flight.
  syncing,

  /// Connected and listening for new summaries.
  live,

  /// A trace request is in flight.
  streamingTrace,

  /// The last operation failed; [SyncClient.lastError] says why.
  failed,
}

/// Why a trace request produced nothing.
enum TraceResult {
  /// A complete trace is available.
  ok,

  /// The device no longer has the trace (its ring overwrote it).
  unavailable,

  /// The request timed out or the transport failed.
  failed,
}

class SyncClient {
  SyncClient(this._transport, {this.traceTimeout = const Duration(seconds: 12)});

  final RepTransport _transport;

  /// How long to wait for a trace before giving up. The device answers a trace
  /// request by notifying chunks; if it has forgotten the rep it stays silent,
  /// so the client must not hang forever.
  final Duration traceTimeout;

  final Map<String, RepSummary> _byUuid = {};
  StreamSubscription<RepSummary>? _subscription;

  SyncState _state = SyncState.idle;
  SyncState get state => _state;

  Object? _lastError;
  Object? get lastError => _lastError;

  /// Summaries known to the client, in the order they arrived.
  ///
  /// UUIDs carry no order, so arrival order is what the list shows — and for a
  /// device that streams its log oldest-first, that is oldest-first.
  List<RepSummary> get summaries => _byUuid.values.toList();

  /// Counters, mostly for tests and a debug overlay.
  int duplicatesIgnored = 0;

  /// Merges one summary, returning true if it was new. Duplicates are ignored,
  /// which is what makes re-syncing cheap and the ring wrap harmless.
  bool merge(RepSummary summary) {
    if (_byUuid.containsKey(summary.uuid)) {
      duplicatesIgnored++;
      return false;
    }
    _byUuid[summary.uuid] = summary;
    return true;
  }

  /// Catch-up: asks the transport for everything it has and merges it.
  /// Returns how many summaries were new (zero when already up to date).
  Future<int> syncSummaries() async {
    _state = SyncState.syncing;
    try {
      final incoming = await _transport.summaries();
      final added = incoming.where(merge).length;
      _state = SyncState.live;
      _lastError = null;
      return added;
    } catch (e) {
      _state = SyncState.failed;
      _lastError = e;
      rethrow;
    }
  }

  /// Subscribes to the transport's summary stream, merging as they arrive.
  ///
  /// Completing or erroring means the connection dropped: the subscription is
  /// dropped too, and the caller can [stop] and re-sync. Summaries already held
  /// stay held — a dropped connection must not lose the archive.
  void start() {
    _subscription?.cancel();
    _subscription = _transport.watchSummaries().listen(
      (summary) {
        merge(summary);
        _state = SyncState.live;
      },
      onError: (Object e) {
        _state = SyncState.failed;
        _lastError = e;
      },
      onDone: () {
        _state = SyncState.idle;
        _subscription = null;
      },
    );
  }

  Future<void> stop() async {
    await _subscription?.cancel();
    _subscription = null;
  }

  /// Fetches a rep's trace, or explains why it couldn't.
  ///
  /// A rep whose summary is known but whose trace is gone is the expected
  /// case after a device-side wrap: `TraceResult.unavailable`, not an error.
  Future<(TraceResult, List<TracePoint>)> fetchTrace(String uuid) async {
    _state = SyncState.streamingTrace;
    try {
      final points = await _transport.trace(uuid).timeout(traceTimeout);
      _state = SyncState.live;
      _lastError = null;
      return (TraceResult.ok, points);
    } on TraceUnavailableException catch (e) {
      _state = SyncState.live;
      _lastError = e;
      return (TraceResult.unavailable, const <TracePoint>[]);
    } on TimeoutException catch (e) {
      _state = SyncState.live;
      _lastError = e;
      return (TraceResult.failed, const <TracePoint>[]);
    } catch (e) {
      _state = SyncState.failed;
      _lastError = e;
      return (TraceResult.failed, const <TracePoint>[]);
    }
  }

  Future<void> close() async {
    await stop();
    await _transport.close();
    _state = SyncState.idle;
  }
}
