// Calypso client — entry point.
//
// Placeholder while the data layer is built: the plot and the rep list arrive
// with their phases (see docs/app/plan.md). Phase 1 only needs the app to
// build and the fixture decoding to be tested.

import 'package:flutter/material.dart';

void main() => runApp(const CalypsoApp());

class CalypsoApp extends StatelessWidget {
  const CalypsoApp({super.key});

  @override
  Widget build(BuildContext context) => MaterialApp(
        title: 'Calypso',
        theme: ThemeData(colorSchemeSeed: Colors.indigo, useMaterial3: true),
        home: const Scaffold(
          body: Center(
            child: Padding(
              padding: EdgeInsets.all(24),
              child: Text(
                'Calypso\n\nData layer in progress: rep summaries and the 3D bar '
                'path plot are the next phases.',
                textAlign: TextAlign.center,
              ),
            ),
          ),
        ),
      );
}
