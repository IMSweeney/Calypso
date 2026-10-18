// The list screen, driven through its injection seams.
//
// The screen takes an archive and a source, so these tests use an in-memory
// archive and a transport built from fixture bytes read *synchronously*. That is
// deliberate: widget tests run under a fake clock, where real asynchronous I/O
// (file reads, asset loading) never completes — a screen that does I/O in
// initState hangs there with the spinner running until pumpAndSettle times out.

import 'dart:io';
import 'dart:typed_data';

import 'package:calypso/state/ble_status.dart';
import 'package:calypso/store/memory_archive.dart';
import 'package:calypso/transport/ble_transport.dart';
import 'package:calypso/store/rep_archive.dart';
import 'package:calypso/transport/fixture_transport.dart';
import 'package:calypso/transport/transport.dart';
import 'package:calypso/ui/plot_screen.dart';
import 'package:calypso/ui/rep_list_screen.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'support/fake_ble_link.dart';

/// Fixture bytes, read synchronously: no async I/O in the widget path.
FixtureTransport fixtureTransport(String name) =>
    FixtureTransport.fromBytes(
      Uint8List.fromList(File('assets/fixtures/$name.pblog').readAsBytesSync()),
    );

Future<void> pumpList(
  WidgetTester tester, {
  required String fixture,
  RepArchive? archive,
  RepTransport? source,
}) async {
  await tester.pumpWidget(MaterialApp(
    home: RepListScreen(archive: archive, source: source ?? fixtureTransport(fixture)),
  ));
  await tester.pumpAndSettle();
}

void main() {
  _bleWiring();

  testWidgets('it archives the source and lists the reps newest first',
      (tester) async {
    final archive = MemoryArchive();
    await pumpList(tester, fixture: 'synthetic-squat', archive: archive);

    expect(find.textContaining('Rep'), findsNWidgets(3));
    expect(find.text('+3 synced'), findsOneWidget);

    // Newest first: rep 3 is the top row and rep 1 the bottom.
    final titles = tester
        .widgetList<ListTile>(find.byType(ListTile))
        .map((t) => (t.title as Text).data!)
        .toList();
    expect(titles.first, startsWith('Rep 3 ·'));
    expect(titles.last, startsWith('Rep 1 ·'));

    // Each row carries the numbers, and a timestamp (receipt time: the device
    // has no clock of its own).
    expect(find.textContaining('ROM 0.4'), findsWidgets);
    expect(find.textContaining('tilt'), findsWidgets);
    expect(titles.first, matches(RegExp(r'\d{4}-\d{2}-\d{2} \d{2}:\d{2}')));

    // The archive really holds them.
    expect(archive.reps.length, 3);
    expect(archive.reps.last.receivedAt, isNotNull);
  });

  testWidgets('re-opening the app does not duplicate the archive', (tester) async {
    final archive = MemoryArchive();
    for (var i = 0; i < 2; i++) {
      // A real restart replaces the whole tree, so unmount first: pumping the
      // same widget type again would reuse the existing State and never re-run
      // the sync at all.
      await tester.pumpWidget(const SizedBox.shrink());
      await pumpList(tester, fixture: 'synthetic-squat', archive: archive);
    }
    expect(find.textContaining('Rep'), findsNWidgets(3));
    expect(find.textContaining('synced'), findsNothing,
        reason: 'the second run had nothing new');
    expect(archive.reps.length, 3);
  });

  testWidgets('an empty archive says so', (tester) async {
    await pumpList(tester, fixture: 'empty', archive: MemoryArchive());
    expect(find.text('No reps yet'), findsOneWidget);
  });

  testWidgets('tapping a rep opens its plot', (tester) async {
    await pumpList(tester, fixture: 'synthetic-squat', archive: MemoryArchive());

    await tester.tap(find.textContaining('Rep 1 ·'));
    await tester.pumpAndSettle();

    expect(find.byType(PlotScreen), findsOneWidget);
    expect(find.text('Rep 1'), findsOneWidget);
    expect(find.textContaining('peak v'), findsOneWidget);
    expect(find.textContaining('ROM'), findsWidgets);
  });

  testWidgets('a rep whose trace is gone explains itself instead of plotting nothing',
      (tester) async {
    await pumpList(tester, fixture: 'ring-dropped', archive: MemoryArchive());

    expect(find.textContaining('Rep 7'), findsOneWidget);
    await tester.tap(find.textContaining('Rep 7 ·'));
    await tester.pumpAndSettle();

    expect(find.textContaining('no trace in the archive'), findsOneWidget);
    expect(find.text('No trace available for this rep'), findsOneWidget);
  });

  testWidgets('the archive is kept when the source goes away', (tester) async {
    // Archive a rep, then re-pump with a source that has nothing to offer: the
    // list still shows what the client already had (it is the archive of record).
    final archive = MemoryArchive();
    await pumpList(tester, fixture: 'synthetic-squat', archive: archive);
    expect(find.textContaining('Rep'), findsNWidgets(3));

    await pumpList(tester, fixture: 'empty', archive: archive, source: fixtureTransport('empty'));
    expect(find.textContaining('Rep'), findsNWidgets(3));
  });
}

/// The BLE wiring, with a fake link: the Connect action drives the transport,
/// the badge follows its state, and no plugin is touched.
void _bleWiring() {
  testWidgets('connecting updates the badge and subscribes, with no hardware',
      (tester) async {
    final link = FakeBleLink();
    final ble = BleTransport(link, traceTimeout: const Duration(milliseconds: 50));

    await tester.pumpWidget(MaterialApp(
      home: RepListScreen(archive: MemoryArchive(), ble: ble, fixture: ''),
    ));
    await tester.pumpAndSettle();

    // Nothing to show yet, and no device.
    expect(find.text('No reps yet'), findsOneWidget);
    expect(find.text('No device'), findsOneWidget);

    await tester.tap(find.text('Connect'));
    await tester.pump();
    expect(find.text('Scanning…'), findsOneWidget);

    // The device answers the scan, so the transport connects and subscribes.
    link.seeDevice('device-1', 'Calypso');
    for (var i = 0; i < 8; i++) {
      await tester.pump(const Duration(milliseconds: 10));
    }

    expect(find.text('Connected'), findsOneWidget);
    expect(link.scans.single, [CalypsoGatt.service]);
    expect(link.subscribed, [CalypsoGatt.summary, CalypsoGatt.traceChunk]);
    expect(find.text('Disconnect'), findsOneWidget);

    // Disconnecting is reflected too.
    await tester.tap(find.text('Disconnect'));
    for (var i = 0; i < 8; i++) {
      await tester.pump(const Duration(milliseconds: 10));
    }
    // Diagnostic: is the transport disconnected, the badge label, or the rebuild?
    expect(ble.status.status, BleStatus.disconnected);
    expect(find.text('Disconnected'), findsOneWidget);
    expect(find.text('Disconnected'), findsOneWidget);
  });
}
