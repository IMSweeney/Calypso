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

import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart' show rootBundle;
import 'package:path_provider/path_provider.dart';

import '../model/uuid.dart';
import '../store/archive_sync.dart';
import '../store/history_store.dart';
import '../store/rep_archive.dart';
import '../transport/ble_link.dart';
import '../transport/ble_transport.dart';
import '../transport/fixture_transport.dart';
import '../transport/transport.dart';
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
  String? _error;
  bool _loading = true;
  int _synced = 0;
  int _syncDone = 0;
  int _syncTotal = 0;

  @override
  void initState() {
    super.initState();
    _ble = widget.ble ?? BleTransport(UniversalBleLink());
    _load();
  }

  @override
  void dispose() {
    // Only tear down a transport we own; an injected one belongs to the test.
    if (widget.ble == null) _ble.close();
    super.dispose();
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

    final added = await archiveFromTransport(
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
    );
    if (!mounted) return;
    setState(() {
      _synced += added;
      _syncDone = 0;
      _syncTotal = 0;
    });
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
      body: _loading
          ? const Center(child: CircularProgressIndicator())
          : archive == null || archive.reps.isEmpty
              ? Center(child: Text(_error ?? 'No reps yet'))
              : ListView.separated(
                  itemCount: archive.reps.length,
                  separatorBuilder: (_, _) => const Divider(height: 1),
                  itemBuilder: (context, index) {
                    // Newest first: the rep you just did is the interesting one.
                    final stored = archive.reps[archive.reps.length - 1 - index];
                    final s = stored.summary;
                    return ListTile(
                      title: Text('Rep ${uuidShort(s.uuid)} · ${_formatTime(stored.receivedAt)}'),
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
                          builder: (_) => PlotScreen(store: archive, repUuid: s.uuid),
                        ),
                      ),
                    );
                  },
                ),
    );
  }
}
