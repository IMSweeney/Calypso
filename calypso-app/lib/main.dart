// Calypso client — entry point.
//
// Phase 4: the rep list over the local archive, with the 3D plot behind it.
// BLE transport is phase 5 (see docs/app/plan.md).

import 'package:flutter/material.dart';

import 'ui/rep_list_screen.dart';

void main() => runApp(const CalypsoApp());

class CalypsoApp extends StatelessWidget {
  const CalypsoApp({super.key});

  @override
  Widget build(BuildContext context) => MaterialApp(
        title: 'Calypso',
        theme: ThemeData(colorSchemeSeed: Colors.indigo, useMaterial3: true),
        home: const RepListScreen(),
      );
}
