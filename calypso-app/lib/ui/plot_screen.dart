/// A minimal screen for looking at one rep: pick a rep, see its 3D bar path,
/// scrub through it.
///
/// This is deliberately thin — the rep list and session grouping are phase 4 —
/// but it is the thing that turns the plot model into something you can look at
/// on the Mac, driven entirely by a fixture (no device, no BLE).
library;

import 'package:flutter/material.dart';

import '../model/rep.dart';
import '../plot/rep_plot.dart';
import '../store/history_store.dart';

/// One rep from the archive: its summary, its 3D bar path, and a scrubber.
class PlotScreen extends StatefulWidget {
  const PlotScreen({super.key, required this.store, required this.repId});

  final HistoryStore store;
  final int repId;

  @override
  State<PlotScreen> createState() => _PlotScreenState();
}

class _PlotScreenState extends State<PlotScreen> {
  RepSummary? _summary;
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
      final repId = widget.repId;
      final stored = widget.store.reps.where((r) => r.summary.repId == repId).firstOrNull;
      final trace = await widget.store.trace(repId);
      if (!mounted) return;
      setState(() {
        _summary = stored?.summary;
        _trace = trace;
        _loading = false;
        if (trace.isEmpty) {
          _error = 'Rep $repId: no trace in the archive '
              '(${stored?.summary.chunkCount ?? 0} chunks were expected)';
        }
      });
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _error = '$e';
        _loading = false;
      });
    }
  }

  RepSummary? get _selected => _summary;

  @override
  Widget build(BuildContext context) {
    if (_loading) {
      return const Scaffold(body: Center(child: CircularProgressIndicator()));
    }
    final selected = _selected;
    return Scaffold(
      appBar: AppBar(
        title: Text(selected == null ? 'Rep ${widget.repId}' : 'Rep ${selected.repId}'),
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
