library;

import '../model/rep.dart';

class TraceUnavailableException implements Exception {
  const TraceUnavailableException(this.uuid, [this.reason = 'not in the log']);

  final String uuid;
  final String reason;

  @override
  String toString() => 'TraceUnavailableException($uuid: $reason)';
}

abstract class RepTransport {
  Future<List<RepSummary>> summaries();
  Stream<RepSummary> watchSummaries();
  Future<List<TracePoint>> trace(String uuid);
  Future<void> close();
}
