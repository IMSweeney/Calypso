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
    if (archive == null) return;
    unawaited(_addToLog(archive, summary));
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
      final archive = widget.archive ??
          HistoryStore(
              HistoryFiles(widget.historyDirectory ?? await getApplicationSupportDirectory()));
      await archive.load();

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

  Future<void> _connect() async {
    final archive = _archive;
    if (archive == null) return;
    await _ble.connect();
    if (!_ble.status.isConnected) return;

    try {
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
                                    PlotScreen(store: archive, repUuid: stored.uuid),
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
