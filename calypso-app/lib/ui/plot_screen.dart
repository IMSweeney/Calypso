/// A minimal screen for looking at one rep: pick a rep, see its 3D bar path,
/// scrub through it.
///
/// This is deliberately thin — the rep list and session grouping are phase 4 —
/// but it is the thing that turns the plot model into something you can look at
/// on the Mac, driven entirely by a fixture (no device, no BLE).
library;

import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart' show rootBundle;

import '../model/rep.dart';
import '../plot/rep_plot.dart';
import '../transport/fixture_transport.dart';
import '../transport/transport.dart';

/// Which fixture to open; override with `--dart-define=FIXTURE=name`.
const String fixtureName =
    String.fromEnvironment('FIXTURE', defaultValue: 'synthetic-squat');

class PlotScreen extends StatefulWidget {
  const PlotScreen({super.key});

  @override
  State<PlotScreen> createState() => _PlotScreenState();
}

class _PlotScreenState extends State<PlotScreen> {
  FixtureTransport? _transport;
  List<RepSummary> _summaries = const [];
  int? _selectedRepId;
  List<TracePoint> _trace = const [];
  int? _scrubIndex;
  String? _error;
  bool _loading = true;

  @override
  void initState() {
    super.initState();
    _load();
  }

  Future<void> _load() async {
    try {
      final data = await rootBundle.load('assets/fixtures/$fixtureName.pblog');
      final bytes = Uint8List.sublistView(data);
      final transport = FixtureTransport.fromBytes(bytes);
      final summaries = await transport.summaries();
      if (!mounted) return;
      setState(() {
        _transport = transport;
        _summaries = summaries;
        _selectedRepId = summaries.isEmpty ? null : summaries.first.repId;
        _loading = false;
      });
      final repId = _selectedRepId;
      if (repId != null) await _selectRep(repId);
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _error = '$e';
        _loading = false;
      });
    }
  }

  Future<void> _selectRep(int repId) async {
    final transport = _transport;
    if (transport == null) return;
    setState(() {
      _selectedRepId = repId;
      _scrubIndex = null;
    });
    try {
      final trace = await transport.trace(repId);
      if (!mounted) return;
      setState(() => _trace = trace);
    } on TraceUnavailableException catch (e) {
      if (!mounted) return;
      setState(() {
        _trace = const [];
        _error = 'Rep $repId: ${e.reason}';
      });
    }
  }

  RepSummary? get _selected {
    for (final s in _summaries) {
      if (s.repId == _selectedRepId) return s;
    }
    return null;
  }

  @override
  Widget build(BuildContext context) {
    if (_loading) {
      return const Scaffold(body: Center(child: CircularProgressIndicator()));
    }
    if (_summaries.isEmpty) {
      return Scaffold(
        appBar: AppBar(title: Text('Calypso — $fixtureName')),
        body: Center(child: Text(_error ?? 'No reps in this fixture')),
      );
    }

    final selected = _selected;
    return Scaffold(
      appBar: AppBar(
        title: Text('Calypso — $fixtureName'),
        actions: [
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 12),
            child: DropdownButton<int>(
              value: _selectedRepId,
              onChanged: (id) => id == null ? null : _selectRep(id),
              items: [
                for (final s in _summaries)
                  DropdownMenuItem(
                    value: s.repId,
                    child: Text('Rep ${s.repId} · ${s.rom.toStringAsFixed(2)} m'),
                  ),
              ],
            ),
          ),
        ],
      ),
      body: Column(
        children: [
          if (_error != null)
            MaterialBanner(
              content: Text(_error!),
              actions: [TextButton(onPressed: () => setState(() => _error = null), child: const Text('Dismiss'))],
            ),
          Expanded(
            child: _trace.isEmpty
                ? const Center(child: Text('No trace available for this rep'))
                : RepPlotView(
                    points: _trace,
                    scrubIndex: _scrubIndex,
                    onScrub: (index) => setState(() => _scrubIndex = index),
                  ),
          ),
          if (selected != null)
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 8, 16, 0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Text('${selected.durationMs} ms'),
                  Text('ROM ${selected.rom.toStringAsFixed(2)} m'),
                  Text('peak v ${selected.peakVelocity.toStringAsFixed(2)} m/s'),
                  Text('tilt ${selected.rollRangeDeg.toStringAsFixed(1)}° roll / '
                      '${selected.pitchRangeDeg.toStringAsFixed(1)}° pitch'),
                ],
              ),
            ),
          if (_trace.isNotEmpty)
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 0, 16, 8),
              child: Slider(
                value: (_scrubIndex ?? 0).toDouble(),
                max: (_trace.length - 1).toDouble(),
                divisions: _trace.length > 1 ? _trace.length - 1 : null,
                label: '${(_scrubIndex == null ? 0 : _trace[_scrubIndex!].tOffsetMs)} ms',
                onChanged: (v) => setState(() => _scrubIndex = v.round()),
              ),
            ),
        ],
      ),
    );
  }
}
