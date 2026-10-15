// Calypso client — entry point.
//
// Phase 3: a fixture-driven screen that plots one rep in 3D. The rep list,
// sessions and BLE live in later phases (see docs/app/plan.md).

import 'package:flutter/material.dart';

import 'ui/plot_screen.dart';

void main() => runApp(const CalypsoApp());

class CalypsoApp extends StatelessWidget {
  const CalypsoApp({super.key});

  @override
  Widget build(BuildContext context) => MaterialApp(
        title: 'Calypso',
        theme: ThemeData(colorSchemeSeed: Colors.indigo, useMaterial3: true),
        home: const PlotScreen(),
      );
}
