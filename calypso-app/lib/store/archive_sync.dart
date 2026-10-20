library;

import '../model/rep.dart';
import '../transport/transport.dart';
import 'record_blocks.dart';
import 'rep_archive.dart';

Future<int> archiveFromTransport(
  RepArchive archive,
  RepTransport source, {
  void Function(int done, int total)? onProgress,
}) async {
  final summaries = await source.summaries();
  var added = 0;
  for (var i = 0; i < summaries.length; i++) {
    final summary = summaries[i];
    onProgress?.call(i, summaries.length);
    if (archive.contains(summary.uuid)) continue;
    added += await archiveRep(archive, source, summary);
  }
  onProgress?.call(summaries.length, summaries.length);
  return added;
}

Future<int> archiveRep(RepArchive archive, RepTransport source, RepSummary summary) async {
  if (archive.contains(summary.uuid)) return 0;
  List<TracePoint> trace;
  try {
    trace = await source.trace(summary.uuid);
  } on TraceUnavailableException {
    trace = const [];
  }
  return archive.appendNew({summary.uuid: buildRepBlock(summary, trace)});
}
