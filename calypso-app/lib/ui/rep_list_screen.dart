/// The rep list: what the archive holds, newest first, with a timestamp each.
///
/// Deliberately no session grouping yet (see the plan's open questions): the
/// device writes an undifferentiated log, and grouping it into sets is a
/// judgement call about training structure that is better made once there is
/// real data on screen. For now this is a flat list of reps with their times.
library;

import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart' show rootBundle;
import 'package:path_provider/path_provider.dart';

import '../store/history_store.dart';
import '../store/record_blocks.dart';
import '../store/rep_archive.dart';
import '../transport/fixture_transport.dart';
import '../transport/transport.dart';
import 'plot_screen.dart';

/// Fixture to load on start-up, so the app has something to show with no device.
/// Override with `--dart-define=FIXTURE=name`, or empty to skip.
const String seedFixture =
    String.fromEnvironment('FIXTURE', defaultValue: 'synthetic-squat');

class RepListScreen extends StatefulWidget {
  const RepListScreen({
    super.key,
    this.historyDirectory,
    this.archive,
    this.source,
    this.fixture = seedFixture,
  });

  /// Where history lives. Injected by tests (a temporary directory); the app
  /// uses the platform's application support directory.
  final Directory? historyDirectory;

  /// The archive to show. Injected by tests (an in-memory archive), because
  /// widget tests run under a fake clock where real file I/O never completes.
  final RepArchive? archive;

  /// Where to sync from. Injected by tests; the app uses a fixture asset (and a
  /// BLE transport once phase 5 lands).
  final RepTransport? source;

  /// Fixture asset to archive on start-up when no [source] is given, or empty.
  final String fixture;

  @override
  State<RepListScreen> createState() => _RepListScreenState();
}

class _RepListScreenState extends State<RepListScreen> {
  RepArchive? _store;
  String? _error;
  bool _loading = true;
  int _synced = 0;

  @override
  void initState() {
    super.initState();
    _load();
  }

  Future<void> _load() async {
    try {
      final store = widget.archive ??
          HistoryStore(HistoryFiles(
              widget.historyDirectory ?? await getApplicationSupportDirectory()));
      await store.load();

      // Bring the archive up to date from whatever source we have. With no
      // device wired up yet, that is a fixture: the archive is the same either
      // way, which is the point of one record format.
      var synced = 0;
      var source = widget.source;
      if (source == null && widget.fixture.isNotEmpty) {
        final data = await rootBundle.load('assets/fixtures/${widget.fixture}.pblog');
        source = FixtureTransport.fromBytes(data.buffer.asUint8List());
      }
      if (source != null) {
        synced = await store.appendNew(await recordBlocksFrom(source));
      }

      if (!mounted) return;
      setState(() {
        _store = store;
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

  String _formatTime(DateTime? when) {
    if (when == null) return '—';
    final local = when.toLocal();
    String two(int v) => v.toString().padLeft(2, '0');
    return '${local.year}-${two(local.month)}-${two(local.day)} '
        '${two(local.hour)}:${two(local.minute)}';
  }

  @override
  Widget build(BuildContext context) {
    final store = _store;
    return Scaffold(
      appBar: AppBar(
        title: const Text('Calypso'),
        actions: [
          if (_synced > 0)
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16),
              child: Center(child: Text('+$_synced synced')),
            ),
        ],
      ),
      body: _loading
          ? const Center(child: CircularProgressIndicator())
          : store == null || store.reps.isEmpty
              ? Center(child: Text(_error ?? 'No reps yet'))
              : ListView.separated(
                  itemCount: store.reps.length,
                  separatorBuilder: (_, __) => const Divider(height: 1),
                  itemBuilder: (context, index) {
                    // Newest first: the rep you just did is the interesting one.
                    final stored = store.reps[store.reps.length - 1 - index];
                    final s = stored.summary;
                    return ListTile(
                      title: Text('Rep ${s.repId} · ${_formatTime(stored.receivedAt)}'),
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
                          builder: (_) => PlotScreen(store: store, repId: s.repId),
                        ),
                      ),
                    );
                  },
                ),
    );
  }
}
