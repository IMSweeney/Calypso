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
import '../model/uuid.dart';
import '../store/archive_sync.dart';
import '../store/history_store.dart';
import '../store/rep_archive.dart';
import '../transport/ble_link.dart';
import '../transport/ble_transport.dart';
import '../transport/fixture_transport.dart';
import '../transport/transport.dart';
import '../widgets/calibration_hint.dart';
import '../widgets/connection_status.dart';
import 'plot_screen.dart';

const String seedFixture = String.fromEnvironment(
  'FIXTURE',
  defaultValue: 'synthetic-squat',
);

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
class RepRow {
  const RepRow(this.summary, this.receivedAt, this.state);

  final pb.RepSummary summary;
  final DateTime? receivedAt;
  final TraceState state;
}

class RepListScreen extends StatefulWidget {
  const RepListScreen({
    super.key,
    this.historyDirectory,
    this.archive,
    this.source,
    this.ble,
    this.fixture = seedFixture,
  });

  final Directory? historyDirectory;
  final RepArchive? archive;
  final RepTransport? source;
  final BleTransport? ble;
  final String fixture;

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

  /// What the list shows: every rep the device has told the app about, newest first.
  ///
  /// The archive holds the reps whose fetch is over; the transport holds the summaries that are not archived yet — which is the same thing as being fetched. Which side a rep is on, and whether its block carries a trace, is its state.
  List<RepRow> _rows(RepArchive? archive) {
    final stored = archive?.reps ?? const <StoredRep>[];
    final rows = <RepRow>[
      for (final rep in stored.reversed)
        RepRow(
          rep.summary,
          rep.receivedAt,
          rep.hasTrace ? TraceState.fetched : TraceState.missing,
        ),
    ];
    final archived = {for (final rep in stored) rep.uuid};
    final fetching = [
      for (final summary in _ble.pushedSummaries)
        if (summary.uuid case final uuid? when !archived.contains(uuid))
          summary,
    ];
    rows.insertAll(0, [
      for (final summary in fetching.reversed)
        RepRow(summary, null, TraceState.fetching),
    ]);
    return rows;
  }

  /// The state of a rep's trace, as the row's trailing widget.
  ///
  /// An icon and its word, and no spinner: a spinner never stops animating, and a widget test that waits for the tree to settle would wait for ever (docs/GOTCHAS.md).
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
      var source = widget.source;
      if (source == null && widget.fixture.isNotEmpty) {
        final data = await rootBundle.load(
          'assets/fixtures/${widget.fixture}.pblog',
        );
        source = FixtureTransport.fromBytes(data.buffer.asUint8List());
      }
      if (source != null) {
        synced = await archiveFromTransport(archive, source);
      }

      if (!mounted) return;
      setState(() {
        _archive = archive;
        _synced = synced;
        _loading = false;
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

  String _formatTime(DateTime? when) {
    if (when == null) return '—';
    final local = when.toLocal();
    String two(int v) => v.toString().padLeft(2, '0');
    return '${local.year}-${two(local.month)}-${two(local.day)} '
        '${two(local.hour)}:${two(local.minute)}';
  }

  String get _statusLine {
    if (_syncTotal > 0) return 'syncing $_syncDone/$_syncTotal';
    if (_synced > 0) return '+$_synced synced';
    return '';
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
          CalibrationHint(calibration: _ble.calibration, status: _ble.status),
          Expanded(
            child: _loading
                ? const Center(child: CircularProgressIndicator())
                : rows.isEmpty
                ? Center(child: Text(_error ?? 'No reps yet'))
                : ListView.separated(
                    itemCount: rows.length,
                    separatorBuilder: (_, _) => const Divider(height: 1),
                    itemBuilder: (context, index) {
                      final row = rows[index];
                      final s = row.summary;
                      return ListTile(
                        title: Text(
                          'Rep ${uuidShort(s.uuid)} · ${_formatTime(row.receivedAt)}',
                        ),
                        subtitle: Text(
                          '${(s.durationMs / 1000).toStringAsFixed(1)} s · '
                          'ROM ${s.rom.toStringAsFixed(2)} m · '
                          'peak ${s.peakVelocity.toStringAsFixed(2)} m/s · '
                          'tilt ${s.rollRangeDeg.toStringAsFixed(1)}° / '
                          '${s.pitchRangeDeg.toStringAsFixed(1)}°'
                          '${s.decimated ? ' · decimated' : ''}',
                        ),
                        trailing: _traceState(row.state),
                        // A rep still being fetched has nothing to plot yet, and the screen it opens would say so in more words.
                        onTap: row.state == TraceState.fetching
                            ? null
                            : () => Navigator.of(context).push(
                                MaterialPageRoute(
                                  builder: (_) => PlotScreen(
                                    store: archive!,
                                    repUuid: s.uuid!,
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
