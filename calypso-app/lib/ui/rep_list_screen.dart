library;

import 'dart:async';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart' show rootBundle;
import 'package:path_provider/path_provider.dart';

import '../logging/rotating_file_appender.dart';
import '../logging/setup.dart';
import '../model/rep.dart';
import '../protocol/calypso.pb.dart' as pb;
import '../store/archive_sync.dart';
import '../store/history_store.dart';
import '../store/rep_archive.dart';
import '../transport/ble_link.dart';
import '../transport/ble_transport.dart';
import '../transport/fixture_transport.dart';
import '../transport/transport.dart';
import '../widgets/connection_status.dart';
import '../widgets/device_report_banner.dart';
import 'plot_screen.dart';

const String seedFixture = String.fromEnvironment(
  'FIXTURE',
  defaultValue: 'synthetic-squat',
);

/// The day heading's date, without a locale: the app carries no localisations, and three letters is all a heading needs.
const List<String> _weekdays = [
  'Mon',
  'Tue',
  'Wed',
  'Thu',
  'Fri',
  'Sat',
  'Sun',
];
const List<String> _months = [
  'Jan',
  'Feb',
  'Mar',
  'Apr',
  'May',
  'Jun',
  'Jul',
  'Aug',
  'Sep',
  'Oct',
  'Nov',
  'Dec',
];

/// How a rep's trace is getting on, which is the one thing the list cannot show from the summary alone: a rep is stored the moment its fetch is over, so "not in the archive yet" *is* "being fetched", and the archive knows whether the trace came with it.
enum TraceState {
  /// The app has the trace: the row can be plotted.
  fetched,

  /// Being fetched — or waiting its turn in the queue behind another rep.
  fetching,

  /// The fetch is over and the archive has the rep without its trace.
  missing,
}

/// One row: a rep the device has told the app about, and where its trace got to.
///
/// A row only exists for a rep the device could date and that happened today (`_rows`), which is why the uuid and the time are not nullable here.
class RepRow {
  const RepRow(this.summary, this.state);

  final pb.RepSummary summary;
  final TraceState state;

  String get uuid => summary.uuid!;

  /// What the row is called: the time it happened. The plot screen is opened from a row and named the same way.
  String get label => summary.timeOfDay!;
}

class RepListScreen extends StatefulWidget {
  const RepListScreen({
    super.key,
    this.historyDirectory,
    this.archive,
    this.source,
    this.ble,
    this.fixture = seedFixture,
    this.clock,
  });

  final Directory? historyDirectory;
  final RepArchive? archive;
  final RepTransport? source;
  final BleTransport? ble;
  final String fixture;

  /// The wall clock the list is a day of. Injected by tests, which date their reps from a fixed epoch.
  final DateTime Function()? clock;

  @override
  State<RepListScreen> createState() => _RepListScreenState();
}

class _RepListScreenState extends State<RepListScreen> {
  RepArchive? _archive;
  late final BleTransport _ble;

  RotatingFileAppender? _logging;
  String? _error;
  bool _loading = true;
  int _synced = 0;
  int _syncDone = 0;
  int _syncTotal = 0;

  StreamSubscription<pb.RepSummary>? _live;
  Future<void> _writes = Future<void>.value();

  /// The day the list is on, or null while it is on today. Null rather than today's date, so a screen left open across midnight follows the clock instead of pinning the day it was opened on.
  DateTime? _day;

  @override
  void initState() {
    super.initState();
    final injected = widget.ble;
    if (injected != null) {
      _ble = injected;
    } else {
      _logging = startFileLogging(getApplicationSupportDirectory);
      _ble = BleTransport(UniversalBleLink());
    }
    _live = _ble.watchSummaries().listen(_onSummary);
    _load();
  }

  @override
  void dispose() {
    _live?.cancel();
    if (widget.ble == null) {
      _ble.close();
      _logging?.close();
    }
    super.dispose();
  }

  void _onSummary(pb.RepSummary summary) {
    final archive = _archive;
    if (archive == null || !mounted) return;
    // It goes on screen now, with its trace still to come: a fetch is not instant, and a rep that is invisible until it is archived is a rep the user cannot tell from one the device never sent.
    setState(() {});
    unawaited(_addToLog(archive, summary));
  }

  DateTime _now() => (widget.clock ?? DateTime.now)();

  DateTime get _today {
    final local = _now().toLocal();
    return DateTime(local.year, local.month, local.day);
  }

  /// One day either side of [day], as a calendar date: built from the date rather than by subtracting 24 hours, so a page stays a calendar date across a daylight-saving change, and `DateTime` normalises a day number that walks off either end of a month.
  static DateTime _shift(DateTime day, int days) =>
      DateTime(day.year, day.month, day.day + days);

  /// What the heading over the list says: `Today` needs no date, and `Yesterday` reads faster than one. Anything further back is the date, with the year only when it is not this year's.
  String get _dayHeading {
    final day = _day;
    if (day == null) return 'Today';
    final today = _today;
    if (_onDay(today, _shift(day, 1))) return 'Yesterday';
    final date =
        '${_weekdays[day.weekday - 1]} ${day.day} ${_months[day.month - 1]}';
    return day.year == today.year ? date : '$date ${day.year}';
  }

  void _earlierDay() => setState(() => _day = _shift(_day ?? _today, -1));

  void _laterDay() {
    final day = _day;
    if (day == null) return;
    final next = _shift(day, 1);
    setState(() => _day = _onDay(next, _today) ? null : next);
  }

  /// Back to today from however many pages back, which is the one page the arrows cannot reach in a single press.
  /// Null is this screen's way of saying today, so a screen left open across midnight lands on the new day rather than returning to the one it was opened on.
  void _toToday() => setState(() => _day = null);

  /// The day a seeded fixture's reps are dated, so the app opens on the session the fixture is: a recording read back months later would otherwise sit behind a today page that no arrow on today leads away from. Null when the newest rep is today's, or when nothing in the archive is dated — the screen stays on today then, and the forward arrow belongs off.
  DateTime? _seedDay(RepArchive archive) {
    DateTime? newest;
    for (final rep in archive.reps) {
      final when = rep.recordedAt?.toLocal();
      if (when == null) continue;
      final day = DateTime(when.year, when.month, when.day);
      if (newest == null || day.isAfter(newest)) newest = day;
    }
    return newest == null || _onDay(newest, _today) ? null : newest;
  }

  /// Whether [when] falls on [day]'s date, on this device's clock: a rep is "today's" by the calendar, not by being less than 24 hours old.
  static bool _onDay(DateTime? when, DateTime day) {
    final local = when?.toLocal();
    return local != null &&
        local.year == day.year &&
        local.month == day.month &&
        local.day == day.day;
  }

  /// What the list shows: one day's reps, newest first.
  ///
  /// The archive holds the reps whose fetch is over; the transport holds the summaries that are not archived yet — which is the same thing as being fetched. Which side a rep is on, and whether its block carries a trace, is its state.
  ///
  /// **Ordered by when a rep happened, not by when the app heard about it.** A catch-up after a week apart arrives in one burst, so the order the archive was written in says nothing about the week; the time on each summary does, and it is what the row is labelled with too.
  ///
  /// **One day, and only reps the device could date.** A list that grows for ever buries today under everything before it, so the day is the page. A rep with no time at all — a capture of a boot whose clock was never set — cannot be placed on any day and so is on no page.
  List<RepRow> _rows(RepArchive? archive) {
    final day = _day ?? _today;
    final stored = archive?.reps ?? const <StoredRep>[];
    final archived = {for (final rep in stored) rep.uuid};
    final rows = <RepRow>[
      for (final summary in _ble.pushedSummaries)
        if (summary.uuid case final uuid? when !archived.contains(uuid))
          RepRow(summary, TraceState.fetching),
      for (final rep in stored)
        RepRow(
          rep.summary,
          rep.hasTrace ? TraceState.fetched : TraceState.missing,
        ),
    ];
    // `sort` is not stable, so the comparator has to be total: two reps in the same millisecond are ordered by uuid rather than by whatever order the list happened to be built in.
    rows.sort((a, b) {
      final at = a.summary.tUnixMs.toInt();
      final bt = b.summary.tUnixMs.toInt();
      if (at != bt) return bt.compareTo(at);
      return (a.summary.uuid ?? '').compareTo(b.summary.uuid ?? '');
    });
    return [
      for (final row in rows)
        if (_onDay(row.summary.recordedAt, day)) row,
    ];
  }

  /// The state of a rep's trace, as the row's trailing widget.
  ///
  /// An icon and its word, and no spinner: a spinner never stops animating, and a widget test that waits for the tree to settle would wait for ever.
  Widget _traceState(TraceState state) {
    final scheme = Theme.of(context).colorScheme;
    final (icon, label, colour) = switch (state) {
      TraceState.fetched => (
        Icons.check_circle_outline,
        'trace',
        scheme.primary,
      ),
      TraceState.fetching => (
        Icons.downloading_outlined,
        'fetching',
        scheme.secondary,
      ),
      TraceState.missing => (Icons.error_outline, 'no trace', scheme.error),
    };
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Icon(icon, size: 16, color: colour),
        const SizedBox(width: 6),
        Text(label, style: TextStyle(color: colour, fontSize: 12)),
      ],
    );
  }

  Future<void> _addToLog(RepArchive archive, pb.RepSummary summary) async {
    try {
      final added = await _queueWrite(() => archiveRep(archive, _ble, summary));
      if (mounted && added > 0) setState(() => _synced += added);
    } catch (e) {
      if (mounted) setState(() => _error = '$e');
    }
  }

  Future<T> _queueWrite<T>(Future<T> Function() write) {
    final result = _writes.then((_) => write());
    _writes = result.then((_) {}, onError: (Object _) {});
    return result;
  }

  Future<void> _load() async {
    try {
      final archive =
          widget.archive ??
          HistoryStore(
            HistoryFiles(
              widget.historyDirectory ?? await getApplicationSupportDirectory(),
            ),
          );
      await archive.load();

      var synced = 0;
      var seeded = false;
      var source = widget.source;
      if (source == null && widget.fixture.isNotEmpty) {
        final data = await rootBundle.load(
          'assets/fixtures/${widget.fixture}.pblog',
        );
        source = FixtureTransport.fromBytes(data.buffer.asUint8List());
        seeded = true;
      }
      if (source != null) {
        synced = await archiveFromTransport(archive, source);
      }

      if (!mounted) return;
      setState(() {
        _archive = archive;
        _synced = synced;
        _loading = false;
        // A fixture is a recording of a session and this build is showing it, so it opens on the day it happened.
        if (seeded) _day = _seedDay(archive);
      });
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _error = '$e';
        _loading = false;
      });
    }
  }

  Future<void> _connect() async {
    final archive = _archive;
    if (archive == null) return;
    await _ble.connect();
    if (!_ble.status.isConnected) return;

    try {
      final added = await _queueWrite(
        () => archiveFromTransport(
          archive,
          _ble,
          onProgress: (done, total) {
            if (mounted) {
              setState(() {
                _syncDone = done;
                _syncTotal = total;
              });
            }
          },
        ),
      );
      if (!mounted) return;
      setState(() {
        _synced += added;
        _syncDone = 0;
        _syncTotal = 0;
      });
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _error = '$e';
        _syncDone = 0;
        _syncTotal = 0;
      });
    }
  }

  Future<void> _disconnect() async {
    await _ble.disconnect();
  }

  String get _statusLine {
    if (_syncTotal > 0) return 'syncing $_syncDone/$_syncTotal';
    if (_synced > 0) return '+$_synced synced';
    return '';
  }

  /// The day heading with an arrow either side of it: left is backwards in time, right is forwards, and forwards stops at today because nothing has happened tomorrow.
  ///
  /// The calendar at the end is the way back from a page more than one day back — a fixture's session, or a look at last month — which the arrows would otherwise take that many presses to return from. It is disabled rather than hidden while the screen is on today, so the arrows do not move under the finger as a page turns.
  ///
  /// The gap before the arrows is the calendar button's own width (`kMinInteractiveDimension`, which is what an `IconButton` is), so the heading sits in the middle of the bar rather than in the middle of what is left of it once the extra button is on one side.
  Widget _dayBar() => Row(
    children: [
      const SizedBox(width: kMinInteractiveDimension),
      IconButton(
        onPressed: _earlierDay,
        icon: const Icon(Icons.chevron_left),
        tooltip: 'Earlier day',
      ),
      Expanded(
        child: Text(
          _dayHeading,
          textAlign: TextAlign.center,
          style: Theme.of(context).textTheme.titleMedium,
        ),
      ),
      IconButton(
        onPressed: _day == null ? null : _laterDay,
        icon: const Icon(Icons.chevron_right),
        tooltip: 'Later day',
      ),
      IconButton(
        onPressed: _day == null ? null : _toToday,
        icon: const Icon(Icons.today),
        tooltip: 'Today',
      ),
    ],
  );

  /// The list with nothing on this day, which is not the same as an empty archive: the arrows are how you find out where the reps are, so the words only have to say that this page is bare.
  Widget _empty(RepArchive? archive) {
    final error = _error;
    if (error != null) return Center(child: Text(error));
    final ever = archive?.reps.isNotEmpty ?? false;
    return Center(child: Text(ever ? 'No reps' : 'No reps yet'));
  }

  @override
  Widget build(BuildContext context) {
    final archive = _archive;
    final rows = _rows(archive);
    final statusLine = _statusLine;
    return Scaffold(
      appBar: AppBar(
        title: const Text('Calypso'),
        actions: [
          if (statusLine.isNotEmpty)
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 8),
              child: Center(child: Text(statusLine)),
            ),
          ConnectionStatusBadge(
            status: _ble.status,
            onConnect: _connect,
            onDisconnect: _disconnect,
          ),
          const SizedBox(width: 8),
        ],
      ),
      body: Column(
        children: [
          DeviceReportBanner(reports: _ble.reports, status: _ble.status),
          _dayBar(),
          Expanded(
            child: _loading
                ? const Center(child: CircularProgressIndicator())
                : rows.isEmpty
                ? _empty(archive)
                : ListView.separated(
                    itemCount: rows.length,
                    separatorBuilder: (_, _) => const Divider(height: 1),
                    itemBuilder: (context, index) {
                      final row = rows[index];
                      final s = row.summary;
                      return ListTile(
                        key: ValueKey<String>(row.uuid),
                        title: Text(row.label),
                        subtitle: Text(
                          '${(s.durationMs / 1000).toStringAsFixed(1)} s · '
                          'ROM ${s.rom.toStringAsFixed(2)} m · '
                          'peak ${s.peakVelocity.toStringAsFixed(2)} m/s · '
                          'tilt ${s.rollRangeDeg.toStringAsFixed(1)}° / '
                          '${s.pitchRangeDeg.toStringAsFixed(1)}°'
                          '${s.hasRawTrace ? ' · replayable' : ''}',
                        ),
                        trailing: _traceState(row.state),
                        // A rep still being fetched has nothing to plot yet, and the screen it opens would say so in more words.
                        onTap: row.state == TraceState.fetching
                            ? null
                            : () => Navigator.of(context).push(
                                MaterialPageRoute(
                                  builder: (_) => PlotScreen(
                                    store: archive!,
                                    repUuid: row.uuid,
                                    label: row.label,
                                  ),
                                ),
                              ),
                      );
                    },
                  ),
          ),
        ],
      ),
    );
  }
}
