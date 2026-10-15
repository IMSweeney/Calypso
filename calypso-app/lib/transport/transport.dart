/// The seam that keeps everything except BLE testable without a device.
///
/// The sync client only ever talks to this interface, so the whole data path
/// runs in a plain `flutter test` with no platform code, and the app runs in
/// the Simulator (which has no CoreBluetooth at all) on a fixture transport.
library;

import '../model/rep.dart';

/// Raised when a rep's trace is not (or no longer) available.
///
/// The device log is a ring: a rep can be summarised long before its trace is
/// requested, and by then the trace may have been erased.
class TraceUnavailableException implements Exception {
  const TraceUnavailableException(this.repId, [this.reason = 'not in the log']);
  final int repId;
  final String reason;

  @override
  String toString() => 'TraceUnavailableException($repId: $reason)';
}

/// A source of rep data: the device over BLE, a recorded fixture, or a
/// recording wrapper around either.
abstract class RepTransport {
  /// Summaries the source currently holds, oldest first.
  ///
  /// Cheap to call more than once: the sync client uses it to catch up, and
  /// duplicate rep ids are ignored there.
  Future<List<RepSummary>> summaries();

  /// Summaries as they arrive (a BLE notification stream; a paced replay for
  /// fixtures). Completing or erroring means the connection dropped.
  Stream<RepSummary> watchSummaries();

  /// One rep's complete trace, reassembled from its chunks.
  ///
  /// Throws [TraceUnavailableException] if the source doesn't have it.
  Future<List<TracePoint>> trace(int repId);

  /// Releases any resources (connection, subscriptions).
  Future<void> close();
}
