// Calypso client — entry point.
//
// The rep list over the local archive, with the 3D plot behind it. The BLE
// transport connects to the device; with none, a fixture seeds the archive so
// the app is usable (and testable) with no hardware.

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
