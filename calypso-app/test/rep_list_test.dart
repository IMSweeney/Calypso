import 'dart:io';
import 'dart:typed_data';

import 'package:calypso/model/uuid.dart';
import 'package:fixnum/fixnum.dart';
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:calypso/state/ble_status.dart';
import 'package:calypso/store/memory_archive.dart';
import 'package:calypso/gatt/gatt_client.dart';
import 'package:calypso/transport/ble_transport.dart';
import 'package:calypso/store/rep_archive.dart';
import 'package:calypso/transport/fixture_transport.dart';
import 'package:calypso/transport/transport.dart';
import 'package:calypso/ui/plot_screen.dart';
import 'package:calypso/ui/rep_list_screen.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'support/fake_ble_link.dart';
import 'support/fake_device.dart';
import 'support/test_uuid.dart';

/// A rep's trace as the device answers a request with it: one chunk, one point.
Uint8List traceChunk(String uuid, {int totalChunks = 1}) => pb.SyncMessage(
  computed: pb.TraceChunk(
    repUuid: uuidToBytes(uuid)!,
    index: 0,
    firstTOffsetMs: 0,
    totalChunks: totalChunks,
    samples: [
      pb.TraceComputedPoint(
        tOffsetMs: 0,
        px: 0,
        py: 0,
        pz: 0,
        qw: 16384,
        qx: 0,
        qy: 0,
        qz: 0,
      ),
    ],
  ),
).writeToBuffer();

FixtureTransport fixtureTransport(String name) => FixtureTransport.fromBytes(
  Uint8List.fromList(File('assets/fixtures/$name.pblog').readAsBytesSync()),
);

Future<void> pumpList(
  WidgetTester tester, {
  required String fixture,
  RepArchive? archive,
  RepTransport? source,
  DateTime Function() clock = fixtureNow,
}) async {
  await tester.pumpWidget(
    MaterialApp(
      home: RepListScreen(
        archive: archive,
        source: source ?? fixtureTransport(fixture),
        clock: clock,
      ),
    ),
  );
  await tester.pumpAndSettle();
}

/// The same epoch the fixtures are dated from: a device stamps every summary it sends, so a fake one does too.
const int deviceEpochMs = 1_767_225_600_000;

/// The clock the screen is "today" by: the instant the fixtures and the fake device's summaries are dated from, so
/// their reps are today's. A screen on the real clock would be a day those reps are not on.
DateTime fixtureNow() => DateTime.fromMillisecondsSinceEpoch(deviceEpochMs);

/// The label a rep at [unixMs] should carry: the local time of day, to the second.
String clockLabel(int unixMs) {
  final t = DateTime.fromMillisecondsSinceEpoch(unixMs).toLocal();
  String two(int v) => v.toString().padLeft(2, '0');
  return '${two(t.hour)}:${two(t.minute)}:${two(t.second)}';
}

/// A screen connected to a fake device that sends summaries when asked, with [clock] as the day it is showing.
Future<(FakeBleLink, FakeDevice, BleTransport)> connectedList(
  WidgetTester tester, {
  RepArchive? archive,
  DateTime Function() clock = fixtureNow,
}) async {
  final link = FakeBleLink();
  final device = FakeDevice(link);
  final ble = BleTransport(
    link,
    traceTimeout: const Duration(milliseconds: 50),
  );
  await tester.pumpWidget(
    MaterialApp(
      home: RepListScreen(
        archive: archive ?? MemoryArchive(),
        ble: ble,
        fixture: '',
        clock: clock,
      ),
    ),
  );
  await tester.pumpAndSettle();
  await tester.tap(find.text('Connect'));
  await tester.pump();
  link.seeDevice('device-1', 'Calypso');
  for (var i = 0; i < 8; i++) {
    await tester.pump(const Duration(milliseconds: 10));
  }
  return (link, device, ble);
}

/// The rows on screen, by the rep each one is, newest first.
List<String> rowUuids(WidgetTester tester) => [
  for (final tile in tester.widgetList<ListTile>(find.byType(ListTile)))
    (tile.key as ValueKey<String>).value,
];

/// The text of every row, newest first.
List<String> rowTitles(WidgetTester tester) => [
  for (final tile in tester.widgetList<ListTile>(find.byType(ListTile)))
    (tile.title as Text).data!,
];

Uint8List summaryMessage(String uuid, {int unixMs = deviceEpochMs}) =>
    pb.SyncMessage(
      summary: pb.RepSummary(
        repUuid: uuidToBytes(uuid)!,
        durationMs: 1400,
        rom: 0.41,
        peakVelocity: 0.7,
        computedLen: 3,
        computedChunks: 1,
        tUnixMs: Int64(unixMs),
      ),
    ).writeToBuffer();

void main() {
  _bleWiring();
  _calibrationHint();
  _dayBoundary();
  _liveSummaries();
  _traceStates();

  testWidgets('it archives the source and lists the reps newest first', (
    tester,
  ) async {
    final archive = MemoryArchive();
    await pumpList(tester, fixture: 'synthetic-squat', archive: archive);

    expect(find.byType(ListTile), findsNWidgets(3));
    expect(find.text('+3 synced'), findsOneWidget);

    expect(rowUuids(tester), [u(2), u(1), u(0)], reason: 'newest first');
    expect(
      rowTitles(
        tester,
      ).every((t) => RegExp(r'^\d{2}:\d{2}:\d{2}$').hasMatch(t)),
      isTrue,
      reason:
          'a row is labelled with the time it happened; the day is the list’s',
    );

    expect(find.textContaining('ROM 0.4'), findsWidgets);
    expect(find.textContaining('tilt'), findsWidgets);

    expect(archive.reps.length, 3);
    expect(
      archive.reps.last.recordedAt,
      isNotNull,
      reason: 'the fixture carries the clock its reps are dated by',
    );
  });

  testWidgets('a rep is listed where it happened, however late it arrived', (
    tester,
  ) async {
    // A catch-up after a week apart arrives in one burst, and nothing says the burst is in order. What places a
    // rep is the time the device put on it, not the order the app happened to hear about it.
    final link = FakeBleLink();
    final device = FakeDevice(link);
    final ble = BleTransport(
      link,
      traceTimeout: const Duration(milliseconds: 50),
    );
    final archive = MemoryArchive();
    await tester.pumpWidget(
      MaterialApp(
        home: RepListScreen(
          archive: archive,
          ble: ble,
          fixture: '',
          clock: fixtureNow,
        ),
      ),
    );
    await tester.pumpAndSettle();

    await tester.tap(find.text('Connect'));
    await tester.pump();
    link.seeDevice('device-1', 'Calypso');
    for (var i = 0; i < 8; i++) {
      await tester.pump(const Duration(milliseconds: 10));
    }

    device.send(summaryMessage(u(2), unixMs: deviceEpochMs + 120_000));
    await tester.pumpAndSettle();
    device.send(summaryMessage(u(1), unixMs: deviceEpochMs + 60_000));
    await tester.pumpAndSettle();

    expect(
      rowUuids(tester),
      [u(2), u(1)],
      reason:
          'the newer rep is on top; the older one stays below it, though it arrived last',
    );
    expect(rowTitles(tester), [
      clockLabel(deviceEpochMs + 120_000),
      clockLabel(deviceEpochMs + 60_000),
    ]);

    await ble.close();
  });

  testWidgets('re-opening the app does not duplicate the archive', (
    tester,
  ) async {
    final archive = MemoryArchive();
    for (var i = 0; i < 2; i++) {
      await tester.pumpWidget(const SizedBox.shrink());
      await pumpList(tester, fixture: 'synthetic-squat', archive: archive);
    }
    expect(find.byType(ListTile), findsNWidgets(3));
    expect(
      find.textContaining('synced'),
      findsNothing,
      reason: 'the second run had nothing new',
    );
    expect(archive.reps.length, 3);
  });

  testWidgets('an empty archive says so', (tester) async {
    await pumpList(tester, fixture: 'empty', archive: MemoryArchive());
    expect(find.text('No reps yet'), findsOneWidget);
  });

  testWidgets('tapping a rep opens its plot', (tester) async {
    await pumpList(
      tester,
      fixture: 'synthetic-squat',
      archive: MemoryArchive(),
    );

    final row = find.byKey(ValueKey(u(0)));
    final label = (tester.widget<ListTile>(row).title as Text).data!;
    await tester.tap(row);
    await tester.pumpAndSettle();

    expect(find.byType(PlotScreen), findsOneWidget);
    expect(
      find.widgetWithText(AppBar, label),
      findsOneWidget,
      reason: 'the plot names the rep the way the row did',
    );
    expect(find.textContaining('peak v'), findsOneWidget);
    expect(find.textContaining('ROM'), findsWidgets);
  });

  testWidgets(
    'a rep whose trace is gone explains itself instead of plotting nothing',
    (tester) async {
      await pumpList(tester, fixture: 'ring-dropped', archive: MemoryArchive());

      expect(find.byKey(ValueKey(u(7))), findsOneWidget);
      await tester.tap(find.byKey(ValueKey(u(7))));
      await tester.pumpAndSettle();

      expect(
        find.textContaining('has no trace in the archive'),
        findsOneWidget,
      );
      expect(find.text('No trace available for this rep'), findsOneWidget);
    },
  );

  testWidgets('the archive is kept when the source goes away', (tester) async {
    final archive = MemoryArchive();
    await pumpList(tester, fixture: 'synthetic-squat', archive: archive);
    expect(find.byType(ListTile), findsNWidgets(3));

    await pumpList(
      tester,
      fixture: 'empty',
      archive: archive,
      source: fixtureTransport('empty'),
    );
    expect(find.byType(ListTile), findsNWidgets(3));
  });
}

void _calibrationHint() {
  testWidgets('nothing is claimed before connecting', (tester) async {
    final link = FakeBleLink();
    final ble = BleTransport(
      link,
      traceTimeout: const Duration(milliseconds: 50),
    );

    await tester.pumpWidget(
      MaterialApp(
        home: RepListScreen(
          archive: MemoryArchive(),
          ble: ble,
          fixture: '',
          clock: fixtureNow,
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(find.textContaining('IMU not calibrated'), findsNothing);
    expect(find.text('No reps yet'), findsOneWidget);
  });
}

/// The day heading's arrows, found the way the person using the screen finds them: by their tooltip.
Finder earlierDay() => find.byTooltip('Earlier day');
Finder laterDay() => find.byTooltip('Later day');
Finder todayButton() => find.byTooltip('Today');

Future<void> pageBack(WidgetTester tester) async {
  await tester.tap(earlierDay());
  await tester.pumpAndSettle();
}

Future<void> pageForward(WidgetTester tester) async {
  await tester.tap(laterDay());
  await tester.pumpAndSettle();
}

/// Whether the calendar button can be pressed at all.
bool canGoToToday(WidgetTester tester) =>
    tester
        .widget<IconButton>(find.widgetWithIcon(IconButton, Icons.today))
        .onPressed !=
    null;

/// Whether the forward arrow can be pressed at all.
bool canPageForward(WidgetTester tester) =>
    tester
        .widget<IconButton>(
          find.widgetWithIcon(IconButton, Icons.chevron_right),
        )
        .onPressed !=
    null;

/// The list is a day at a time: it opens on today, and the arrows walk back through the archive.
void _dayBoundary() {
  testWidgets('the arrows walk a day at a time and stop at today', (
    tester,
  ) async {
    final archive = MemoryArchive();
    final (_, device, ble) = await connectedList(tester, archive: archive);

    expect(find.text('Today'), findsOneWidget);
    expect(
      canPageForward(tester),
      isFalse,
      reason: 'nothing happened tomorrow',
    );

    device.send(
      summaryMessage(u(1), unixMs: deviceEpochMs - Duration.millisecondsPerDay),
    );
    await tester.pumpAndSettle();

    expect(find.byKey(ValueKey(u(1))), findsNothing);
    expect(find.text('No reps'), findsOneWidget);
    expect(archive.contains(u(1)), isTrue, reason: 'a day away is not lost');

    await pageBack(tester);
    expect(find.text('Yesterday'), findsOneWidget);
    expect(find.byKey(ValueKey(u(1))), findsOneWidget);
    expect(canPageForward(tester), isTrue);

    // A rep that arrives now belongs to today, not to the day being read.
    device.send(summaryMessage(u(2)));
    await tester.pumpAndSettle();
    expect(find.byKey(ValueKey(u(2))), findsNothing);

    await pageForward(tester);
    expect(find.text('Today'), findsOneWidget);
    expect(find.byKey(ValueKey(u(2))), findsOneWidget);
    expect(find.byKey(ValueKey(u(1))), findsNothing);

    await ble.close();
  });

  testWidgets('the calendar button comes back to today from any page', (
    tester,
  ) async {
    // A rep three days back: the arrows can be pressed to it, but only three presses bring you back.
    final archive = MemoryArchive();
    final (_, device, ble) = await connectedList(tester, archive: archive);
    device.send(
      summaryMessage(
        u(1),
        unixMs: deviceEpochMs - 3 * Duration.millisecondsPerDay,
      ),
    );
    await tester.pumpAndSettle();

    expect(
      canGoToToday(tester),
      isFalse,
      reason: 'already on today, so there is nowhere to go',
    );

    await pageBack(tester);
    await pageBack(tester);
    await pageBack(tester);
    expect(find.text('Today'), findsNothing);
    expect(find.byKey(ValueKey(u(1))), findsOneWidget);
    expect(canGoToToday(tester), isTrue);

    await tester.tap(todayButton());
    await tester.pumpAndSettle();

    expect(find.text('Today'), findsOneWidget);
    expect(
      find.byKey(ValueKey(u(1))),
      findsNothing,
      reason: 'three days ago is not today',
    );
    expect(find.text('No reps'), findsOneWidget);
    expect(archive.contains(u(1)), isTrue, reason: 'a day away is not lost');
    expect(canGoToToday(tester), isFalse);
    expect(canPageForward(tester), isFalse);

    await ble.close();
  });

  testWidgets('further back, the heading is the date', (tester) async {
    final (_, _, ble) = await connectedList(tester);

    await pageBack(tester);
    await pageBack(tester);

    expect(find.text('Today'), findsNothing);
    expect(find.text('Yesterday'), findsNothing);
    expect(
      find.textContaining(
        RegExp(r'^[A-Z][a-z]{2} \d{1,2} [A-Z][a-z]{2}( \d{4})?$'),
      ),
      findsOneWidget,
    );

    await ble.close();
  });

  testWidgets('a seeded fixture opens on the day its session happened', (
    tester,
  ) async {
    // The build is showing a recording rather than living in the day it is run: opening on today would put the
    // session behind a page the forward arrow cannot reach.
    DateTime later() => DateTime.fromMillisecondsSinceEpoch(
      deviceEpochMs + const Duration(days: 30).inMilliseconds,
    );
    final ble = BleTransport(
      FakeBleLink(),
      traceTimeout: const Duration(milliseconds: 50),
    );
    await tester.pumpWidget(
      MaterialApp(
        home: RepListScreen(
          archive: MemoryArchive(),
          ble: ble,
          fixture: 'synthetic-squat',
          clock: later,
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(find.byType(ListTile), findsNWidgets(3));
    expect(find.text('Today'), findsNothing);
    expect(find.text('Yesterday'), findsNothing);
    expect(
      find.textContaining(RegExp(r'^[A-Z][a-z]{2} \d{1,2} [A-Z][a-z]{2}')),
      findsOneWidget,
    );
    expect(canPageForward(tester), isTrue, reason: 'today is ahead of here');

    await ble.close();
  });

  testWidgets('a rep the device could not date is on no page', (tester) async {
    final archive = MemoryArchive();
    final (_, device, ble) = await connectedList(tester, archive: archive);

    device.send(summaryMessage(u(3), unixMs: 0));
    await tester.pumpAndSettle();
    expect(
      find.byKey(ValueKey(u(3))),
      findsNothing,
      reason: 'it cannot be placed on any day',
    );

    await pageBack(tester);
    await pageBack(tester);
    expect(find.byKey(ValueKey(u(3))), findsNothing);

    await ble.close();
  });
}

void _liveSummaries() {
  testWidgets(
    'a rep that arrives while connected is written to the log and listed',
    (tester) async {
      final link = FakeBleLink();
      final device = FakeDevice(link);
      final ble = BleTransport(
        link,
        traceTimeout: const Duration(milliseconds: 50),
      );
      final archive = MemoryArchive();

      await tester.pumpWidget(
        MaterialApp(
          home: RepListScreen(
            archive: archive,
            ble: ble,
            fixture: '',
            clock: fixtureNow,
          ),
        ),
      );
      await tester.pumpAndSettle();
      expect(find.text('No reps yet'), findsOneWidget);

      await tester.tap(find.text('Connect'));
      await tester.pump();
      link.seeDevice('device-1', 'Calypso');
      for (var i = 0; i < 8; i++) {
        await tester.pump(const Duration(milliseconds: 10));
      }
      expect(find.text('Connected'), findsOneWidget);

      device.send(summaryMessage(u(0)));
      await tester.pumpAndSettle();

      expect(
        find.byKey(ValueKey(u(0))),
        findsOneWidget,
        reason: 'the list is the log, so it shows what the log just gained',
      );
      expect(archive.contains(u(0)), isTrue, reason: 'and the log has it');

      device.send(summaryMessage(u(0)));
      await tester.pumpAndSettle();
      expect(
        find.byKey(ValueKey(u(0))),
        findsOneWidget,
        reason: 'one row, one record',
      );
      expect(archive.reps, hasLength(1));

      // While connected, the levels are read on a timer: close the link so it does not outlive the tree.
      await ble.close();
    },
  );
}

/// The list says where each rep's trace is: fetched, being fetched, or gone. A rep that is invisible until it is archived is a rep the user cannot tell from one the device never sent.
void _traceStates() {
  testWidgets('a rep whose trace is in flight says so', (tester) async {
    final (_, device, ble) = await connectedList(tester);

    device.send(summaryMessage(u(0)));
    // The request has gone out and nothing has answered it: no time has passed, so this is the fetch in flight and not a fetch that failed.
    await tester.pump();
    await tester.pump();

    expect(find.byKey(ValueKey(u(0))), findsOneWidget);
    expect(find.text('fetching'), findsOneWidget);
    expect(find.text('trace'), findsNothing);

    // Nothing ever answers, so let the fetch fail and leave no timer behind.
    await tester.pumpAndSettle();
    expect(find.text('no trace'), findsOneWidget);

    // The calibration poll runs while connected: close the link so it does not outlive the tree (docs/GOTCHAS.md).
    await ble.close();
  });

  testWidgets('a rep whose trace arrived says so, and opens', (tester) async {
    final (_, device, ble) = await connectedList(tester);
    // Answer each request with the rep's trace, as the device would.
    device.onRequest = (message) {
      final requested = uuidFromBytes(
        pb.SyncRequest.fromBuffer(message).trace.repUuid,
      );
      if (requested != null) device.send(traceChunk(requested));
    };

    device.send(summaryMessage(u(1)));
    await tester.pumpAndSettle();

    expect(find.text('trace'), findsOneWidget);
    expect(find.text('no trace'), findsNothing);

    await tester.tap(find.byKey(ValueKey(u(1))));
    await tester.pumpAndSettle();
    expect(find.byType(PlotScreen), findsOneWidget);

    await ble.close();
  });
}

void _bleWiring() {
  testWidgets('a fault from the device is a banner, not a silent log line', (
    tester,
  ) async {
    final link = FakeBleLink();
    final device = FakeDevice(link);
    final ble = BleTransport(
      link,
      traceTimeout: const Duration(milliseconds: 50),
    );

    await tester.pumpWidget(
      MaterialApp(
        home: RepListScreen(
          archive: MemoryArchive(),
          ble: ble,
          fixture: '',
          clock: fixtureNow,
        ),
      ),
    );
    await tester.pumpAndSettle();

    await tester.tap(find.text('Connect'));
    await tester.pump();
    link.seeDevice('device-1', 'Calypso');
    for (var i = 0; i < 8; i++) {
      await tester.pump(const Duration(milliseconds: 10));
    }
    expect(find.text('Connected'), findsOneWidget);

    // The device says a rep could not be saved: what it sends is a kind, so the sentence
    // on screen is the app's (`state/device_reports.dart`).
    device.sendReport(pb.ReportKind.REPORT_KIND_REP_NOT_STORED);
    await tester.pump();

    expect(find.textContaining('could not save a rep'), findsOneWidget);

    await tester.tap(find.text('Disconnect'));
    for (var i = 0; i < 8; i++) {
      await tester.pump(const Duration(milliseconds: 10));
    }
    expect(
      find.textContaining('could not save a rep'),
      findsNothing,
      reason: 'a fault belongs to the connection it was heard on',
    );
  });

  testWidgets('connecting updates the badge and subscribes, with no hardware', (
    tester,
  ) async {
    final link = FakeBleLink();
    final ble = BleTransport(
      link,
      traceTimeout: const Duration(milliseconds: 50),
    );

    await tester.pumpWidget(
      MaterialApp(
        home: RepListScreen(
          archive: MemoryArchive(),
          ble: ble,
          fixture: '',
          clock: fixtureNow,
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(find.text('No reps yet'), findsOneWidget);
    expect(find.text('No device'), findsOneWidget);

    await tester.tap(find.text('Connect'));
    await tester.pump();
    expect(find.text('Scanning…'), findsOneWidget);

    link.seeDevice('device-1', 'Calypso');
    for (var i = 0; i < 8; i++) {
      await tester.pump(const Duration(milliseconds: 10));
    }

    expect(find.text('Connected'), findsOneWidget);
    expect(link.scans.single, [SyncClient.serviceUuid]);
    expect(link.subscribed, [SyncClient.messagesUuid, SyncClient.reportsUuid]);
    expect(find.text('Disconnect'), findsOneWidget);

    await tester.tap(find.text('Disconnect'));
    for (var i = 0; i < 8; i++) {
      await tester.pump(const Duration(milliseconds: 10));
    }
    expect(ble.status.status, BleStatus.disconnected);
    expect(find.text('Disconnected'), findsOneWidget);
  });
}
