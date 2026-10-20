library;

import 'package:flutter/material.dart';

import '../model/rep.dart';
import '../model/uuid.dart';
import '../plot/rep_plot.dart';
import '../store/rep_archive.dart';

class PlotScreen extends StatefulWidget {
  const PlotScreen({super.key, required this.store, required this.repUuid});

  final RepArchive store;
  final String repUuid;

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
      final repUuid = widget.repUuid;
      final stored = widget.store.reps.where((r) => r.uuid == repUuid).firstOrNull;
      final trace = await widget.store.trace(repUuid);
      if (!mounted) return;
      setState(() {
        _summary = stored?.summary;
        _trace = trace;
        _loading = false;
        if (trace.isEmpty) {
          _error = 'Rep ${uuidShort(repUuid)}: no trace in the archive '
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
        title: Text('Rep ${uuidShort(widget.repUuid)}'),
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
              child: Wrap(
                alignment: WrapAlignment.spaceBetween,
                spacing: 16,
                runSpacing: 4,
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
