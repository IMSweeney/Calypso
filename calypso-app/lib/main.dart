import 'dart:async';
import 'dart:io';

import 'package:flutter/material.dart';

import 'ui/rep_list_screen.dart';

void main() {
  runZonedGuarded(
    () => runApp(const CalypsoApp()),
    // The last word, and fatal on purpose.
    // An unhandled error is either a bug or a diagnostic that failed — the log appender's own write failures arrive here — and an app that carries on without either is undiagnosable.
    (error, stack) {
      debugPrint('calypso: unhandled error: $error');
      debugPrint('$stack');
      exit(1);
    },
  );
}

class CalypsoApp extends StatelessWidget {
  const CalypsoApp({super.key});

  @override
  Widget build(BuildContext context) => MaterialApp(
    title: 'Calypso',
    theme: ThemeData(colorSchemeSeed: Colors.indigo, useMaterial3: true),
    home: const RepListScreen(),
  );
}
