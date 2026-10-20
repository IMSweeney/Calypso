library;

import '../protocol/calypso.pb.dart' as pb;

class TraceUnavailableException implements Exception {
  const TraceUnavailableException(this.uuid, [this.reason = 'not in the log']);

  final String uuid;
  final String reason;

  @override
  String toString() => 'TraceUnavailableException($uuid: $reason)';
}

abstract class RepTransport {
  Future<List<pb.RepSummary>> summaries();
  Stream<pb.RepSummary> watchSummaries();
  Future<List<pb.TraceSample>> trace(String uuid);
  Future<void> close();
}
