/// Bringing the archive up to date from a transport.
///
/// Deliberately per-rep rather than one bulk pass: a device with a full log takes
/// one trace round trip per rep (a 1.4 s rep is ~47 notifications at three points
/// each), so the caller needs progress and the work needs to be resumable — reps
/// the archive already holds are skipped, which is also what makes re-syncing
/// after a disconnection cheap.
library;

import '../model/rep.dart';
import '../transport/transport.dart';
import 'record_blocks.dart';
import 'rep_archive.dart';

/// Archives every rep the transport lists, one at a time.
///
/// [onProgress] is called with (done, total) as it goes, so a UI can show a
/// counter. Returns how many reps were added.
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
    if (archive.contains(summary.repId)) continue;
    added += await archiveRep(archive, source, summary);
  }
  onProgress?.call(summaries.length, summaries.length);
  return added;
}

/// Archives a single rep, fetched from [source]. Returns 1 if it was added, 0 if
/// the archive already had it.
///
/// Used for the incremental case (a summary arriving live, or a rep just
/// recorded) as well as by [archiveFromTransport].
Future<int> archiveRep(RepArchive archive, RepTransport source, RepSummary summary) async {
  if (archive.contains(summary.repId)) return 0;
  List<TracePoint> trace;
  try {
    trace = await source.trace(summary.repId);
  } on TraceUnavailableException {
    // The device has already forgotten this trace. Archive the summary anyway:
    // it is real data, and the app can say the trace is unavailable rather than
    // inventing one.
    trace = const [];
  }
  return archive.appendNew({summary.repId: buildRepBlock(summary, trace)});
}
