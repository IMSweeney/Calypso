/// The rep list: what the archive holds, newest first, with a timestamp each.
///
/// Deliberately no session grouping: the device writes an undifferentiated log,
/// and grouping into sets is a judgement about training structure better made
/// with real data on screen.
///
/// The screen takes its archive and its sources by injection. That is not just
/// for tests: a widget test runs under a fake clock where real asynchronous I/O
/// never completes, so a screen that reads a file in `initState` hangs with the
/// spinner running. The app builds the file-backed archive itself; tests pass an
/// in-memory one.
library;

import 'dart:async';
import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart' show rootBundle;
import 'package:path_provider/path_provider.dart';

import '../logging/rotating_file_appender.dart';
import '../logging/setup.dart';
import '../model/rep.dart';
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

/// Fixture to seed the archive with when there is no device, so the app has
/// something to show. Override with `--dart-define=FIXTURE=name`, or empty to
/// skip.
const String seedFixture = String.fromEnvironment('FIXTURE', defaultValue: 'synthetic-squat');

class RepListScreen extends StatefulWidget {
  const RepListScreen({
    super.key,
    this.historyDirectory,
    this.archive,
    this.source,
    this.ble,
    this.fixture = seedFixture,
  });

  /// Where history lives. Injected by tests (a temporary directory); the app
  /// uses the platform's application support directory.
  final Directory? historyDirectory;

  /// The archive to show. Injected by tests (an in-memory archive).
  final RepArchive? archive;

  /// Where to sync from when there is no device. Injected by tests; the app
  /// uses a fixture asset.
  final RepTransport? source;

  /// The BLE transport. Injected by tests (with a fake link); the app builds one
  /// over `universal_ble`, which does nothing until `connect()` is called.
  final BleTransport? ble;

  /// Fixture asset to seed from when no [source] is given, or empty.
  final String fixture;

  @override
  State<RepListScreen> createState() => _RepListScreenState();
}

class _RepListScreenState extends State<RepListScreen> {
  RepArchive? _archive;
  late final BleTransport _ble;

  /// The file appender, when this screen is the one that turned logging on (see
  /// [initState]).
  RotatingFileAppender? _logging;
  String? _error;
  bool _loading = true;
  int _synced = 0;
  int _syncDone = 0;
  int _syncTotal = 0;

  /// Summaries as the device reports them, for as long as this screen lives.
  ///
  /// Subscribed once rather than per connection: the transport's stream outlives a
  /// link, and summaries arrive on it only while one is up. Every one of them is
  /// written to the log and the list is rebuilt from the log, so a rep that closes
  /// while the app is connected appears without another Connect.
  StreamSubscription<RepSummary>? _live;

  /// Archive writes, chained so they never interleave.
  ///
  /// A summary can arrive while the catch-up is still fetching traces, and two
  /// writers racing on the same rep would each find it absent and append it twice.
  /// The log is the source of truth, so it must not be the place that collects
  /// mistakes.
  Future<void> _writes = Future<void>.value();

  @override
  void initState() {
    super.initState();
    final injected = widget.ble;
    if (injected != null) {
      _ble = injected;
    } else {
      // Logging is turned on here rather than in `main` so that it comes with the
      // transport this screen owns. A test injects its own transport and never
      // reaches this, which is the whole of "no file logging in tests": nothing is
      // attached, so nothing is written — no seam to thread through, and no real
      // I/O under the fake clock (the same reason `_load` takes its archive
      // injected).
      _logging = startFileLogging(getApplicationSupportDirectory);
      _ble = BleTransport(UniversalBleLink());
    }
    // **The log is the source of truth, including for what arrives live**: the
    // device pushes a summary as each rep closes, this writes that rep's records to
    // the archive, and the list below is built from the archive. Nothing in the UI
    // keeps rep state of its own, so nothing can disagree with what is on disk.
    _live = _ble.watchSummaries().listen(_onSummary);
    _load();
  }

  @override
  void dispose() {
    _live?.cancel();
    // Only tear down what we own; an injected transport belongs to the test.
    if (widget.ble == null) {
      _ble.close();
      _logging?.close();
    }
    super.dispose();
  }

  /// A rep the device has just closed: fetch its trace, write it to the log, and let
  /// the log re-serve the list.
  void _onSummary(RepSummary summary) {
    final archive = _archive;
    if (archive == null) return;
    unawaited(_addToLog(archive, summary));
  }

  Future<void> _addToLog(RepArchive archive, RepSummary summary) async {
    try {
      final added = await _queueWrite(() => archiveRep(archive, _ble, summary));
      if (mounted && added > 0) setState(() => _synced += added);
    } catch (e) {
      // The rep is not lost: the device still has it, and the next catch-up (or
      // the next live summary, which is idempotent) will archive it.
      if (mounted) setState(() => _error = '$e');
    }
  }

  /// Runs [write] after every write queued before it, and hands back its result —
  /// including a failure, which must not wedge the ones behind it.
  Future<T> _queueWrite<T>(Future<T> Function() write) {
    final result = _writes.then((_) => write());
    _writes = result.then((_) {}, onError: (Object _) {});
    return result;
  }

  Future<void> _load() async {
    try {
      final archive = widget.archive ??
          HistoryStore(
              HistoryFiles(widget.historyDirectory ?? await getApplicationSupportDirectory()));
      await archive.load();

      // Seed from a fixture when there is no device, so the app is usable on a
      // simulator (which has no Bluetooth at all) and in tests. Whatever the
      // source, the archive is the same, which is the point of one format.
      var synced = 0;
      var source = widget.source;
      if (source == null && widget.fixture.isNotEmpty) {
        final data = await rootBundle.load('assets/fixtures/${widget.fixture}.pblog');
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

  /// Connects to the device and brings the archive up to date from it.
  ///
  /// Progress is shown because a first sync costs one trace round trip per rep:
  /// a 1.4 s rep is ~47 notifications at three points each.
  Future<void> _connect() async {
    final archive = _archive;
    if (archive == null) return;
    await _ble.connect();
    if (!_ble.status.isConnected) return; // the badge says why not

    try {
      // Catch-up: whatever the transport has already been told, in one pass. Reps
      // that arrive from here on are written by [_onSummary], and both paths are
      // idempotent against the log.
      final added = await _queueWrite(() => archiveFromTransport(
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
          ));
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
      // The hint sits above the list, not inside it: an uncalibrated device is
      // exactly the case where the list is empty, and "No reps yet" on its own
      // looks like the app is broken rather than waiting for the IMU.
      body: Column(
        children: [
          CalibrationHint(calibration: _ble.calibration, status: _ble.status),
          Expanded(
            child: _loading
                ? const Center(child: CircularProgressIndicator())
                : archive == null || archive.reps.isEmpty
                    ? Center(child: Text(_error ?? 'No reps yet'))
                    : ListView.separated(
                        itemCount: archive.reps.length,
                        separatorBuilder: (_, _) => const Divider(height: 1),
                        itemBuilder: (context, index) {
                          // Newest first: the rep you just did is the
                          // interesting one.
                          final stored = archive.reps[archive.reps.length - 1 - index];
                          final s = stored.summary;
                          return ListTile(
                            title: Text(
                                'Rep ${uuidShort(s.uuid)} · ${_formatTime(stored.receivedAt)}'),
                            subtitle: Text(
                              '${(s.durationMs / 1000).toStringAsFixed(1)} s · '
                              'ROM ${s.rom.toStringAsFixed(2)} m · '
                              'peak ${s.peakVelocity.toStringAsFixed(2)} m/s · '
                              'tilt ${s.rollRangeDeg.toStringAsFixed(1)}° / '
                              '${s.pitchRangeDeg.toStringAsFixed(1)}°'
                              '${s.decimated ? ' · decimated' : ''}',
                            ),
                            trailing: const Icon(Icons.view_in_ar_outlined),
                            onTap: () => Navigator.of(context).push(
                              MaterialPageRoute(
                                builder: (_) =>
                                    PlotScreen(store: archive, repUuid: s.uuid),
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
