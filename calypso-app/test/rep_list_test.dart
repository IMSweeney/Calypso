import 'dart:io';
import 'dart:typed_data';

import 'package:calypso/model/uuid.dart';
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

import 'support/calibration.dart';
import 'support/fake_ble_link.dart';
import 'support/test_uuid.dart';

FixtureTransport fixtureTransport(String name) => FixtureTransport.fromBytes(
  Uint8List.fromList(File('assets/fixtures/$name.pblog').readAsBytesSync()),
);

Future<void> pumpList(
  WidgetTester tester, {
  required String fixture,
  RepArchive? archive,
  RepTransport? source,
}) async {
  await tester.pumpWidget(
    MaterialApp(
      home: RepListScreen(
        archive: archive,
        source: source ?? fixtureTransport(fixture),
      ),
    ),
  );
  await tester.pumpAndSettle();
}

Uint8List summaryMessage(String uuid) => pb.SyncMessage(
  summary: pb.RepSummary(
    repUuid: uuidToBytes(uuid)!,
    durationMs: 1400,
    rom: 0.41,
    peakVelocity: 0.7,
    traceLen: 3,
    chunkCount: 1,
  ),
).writeToBuffer();

void main() {
  _bleWiring();
  _calibrationHint();
  _liveSummaries();

  testWidgets('it archives the source and lists the reps newest first', (
    tester,
  ) async {
    final archive = MemoryArchive();
    await pumpList(tester, fixture: 'synthetic-squat', archive: archive);

    expect(find.textContaining('Rep'), findsNWidgets(3));
    expect(find.text('+3 synced'), findsOneWidget);

    final titles = tester
        .widgetList<ListTile>(find.byType(ListTile))
        .map((t) => (t.title as Text).data!)
        .toList();
    expect(titles.first, startsWith('Rep ${uuidShort(u(2))} ·'));
    expect(titles.last, startsWith('Rep ${uuidShort(u(0))} ·'));

    expect(find.textContaining('ROM 0.4'), findsWidgets);
    expect(find.textContaining('tilt'), findsWidgets);
    expect(titles.first, matches(RegExp(r'\d{4}-\d{2}-\d{2} \d{2}:\d{2}')));

    expect(archive.reps.length, 3);
    expect(archive.reps.last.receivedAt, isNotNull);
  });

  testWidgets('re-opening the app does not duplicate the archive', (
    tester,
  ) async {
    final archive = MemoryArchive();
    for (var i = 0; i < 2; i++) {
      await tester.pumpWidget(const SizedBox.shrink());
      await pumpList(tester, fixture: 'synthetic-squat', archive: archive);
    }
    expect(find.textContaining('Rep'), findsNWidgets(3));
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

    await tester.tap(find.textContaining('Rep ${uuidShort(u(0))} ·'));
    await tester.pumpAndSettle();

    expect(find.byType(PlotScreen), findsOneWidget);
    expect(find.text('Rep ${uuidShort(u(0))}'), findsOneWidget);
    expect(find.textContaining('peak v'), findsOneWidget);
    expect(find.textContaining('ROM'), findsWidgets);
  });

  testWidgets(
    'a rep whose trace is gone explains itself instead of plotting nothing',
    (tester) async {
      await pumpList(tester, fixture: 'ring-dropped', archive: MemoryArchive());

      expect(find.textContaining('Rep ${uuidShort(u(7))}'), findsOneWidget);
      await tester.tap(find.textContaining('Rep ${uuidShort(u(7))} ·'));
      await tester.pumpAndSettle();

      expect(find.textContaining('no trace in the archive'), findsOneWidget);
      expect(find.text('No trace available for this rep'), findsOneWidget);
    },
  );

  testWidgets('the archive is kept when the source goes away', (tester) async {
    final archive = MemoryArchive();
    await pumpList(tester, fixture: 'synthetic-squat', archive: archive);
    expect(find.textContaining('Rep'), findsNWidgets(3));

    await pumpList(
      tester,
      fixture: 'empty',
      archive: archive,
      source: fixtureTransport('empty'),
    );
    expect(find.textContaining('Rep'), findsNWidgets(3));
  });
}

void _calibrationHint() {
  Future<FakeBleLink> pumpConnected(WidgetTester tester) async {
    final link = FakeBleLink();
    final ble = BleTransport(
      link,
      traceTimeout: const Duration(milliseconds: 50),
    );

    await tester.pumpWidget(
      MaterialApp(
        home: RepListScreen(archive: MemoryArchive(), ble: ble, fixture: ''),
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
    return link;
  }

  testWidgets('an uncalibrated device explains itself, then stops', (
    tester,
  ) async {
    final link = await pumpConnected(tester);

    expect(find.textContaining('IMU not calibrated'), findsNothing);

    link.notify(
      SyncClient.messagesUuid,
      calibrationMessage(sys: 1, accel: 1, gyro: 2),
    );
    await tester.pumpAndSettle();

    expect(find.textContaining('IMU not calibrated'), findsOneWidget);
    expect(find.textContaining('rep recording is paused'), findsOneWidget);
    expect(find.textContaining('still on the rack'), findsOneWidget);
    expect(find.textContaining('accel 1/3'), findsOneWidget);
    expect(find.textContaining('mag 0/3'), findsOneWidget);

    link.notify(SyncClient.messagesUuid, calibrationMessage());
    await tester.pumpAndSettle();
    expect(find.textContaining('IMU not calibrated'), findsNothing);
  });

  testWidgets('a dropped connection takes a stale hint with it', (
    tester,
  ) async {
    final link = await pumpConnected(tester);

    link.notify(
      SyncClient.messagesUuid,
      calibrationMessage(sys: 1, accel: 1, gyro: 1),
    );
    await tester.pumpAndSettle();
    expect(find.textContaining('IMU not calibrated'), findsOneWidget);

    link.dropConnection('out of range');
    await tester.pumpAndSettle();
    expect(find.textContaining('IMU not calibrated'), findsNothing);
  });

  testWidgets('nothing is claimed before connecting', (tester) async {
    final link = FakeBleLink();
    final ble = BleTransport(
      link,
      traceTimeout: const Duration(milliseconds: 50),
    );

    await tester.pumpWidget(
      MaterialApp(
        home: RepListScreen(archive: MemoryArchive(), ble: ble, fixture: ''),
      ),
    );
    await tester.pumpAndSettle();

    expect(find.textContaining('IMU not calibrated'), findsNothing);
    expect(find.text('No reps yet'), findsOneWidget);
  });
}

void _liveSummaries() {
  testWidgets(
    'a rep that arrives while connected is written to the log and listed',
    (tester) async {
      final link = FakeBleLink();
      final ble = BleTransport(
        link,
        traceTimeout: const Duration(milliseconds: 50),
      );
      final archive = MemoryArchive();

      await tester.pumpWidget(
        MaterialApp(
          home: RepListScreen(archive: archive, ble: ble, fixture: ''),
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

      link.notify(SyncClient.messagesUuid, summaryMessage(u(0)));
      await tester.pumpAndSettle();

      expect(
        find.textContaining('Rep ${uuidShort(u(0))}'),
        findsOneWidget,
        reason: 'the list is the log, so it shows what the log just gained',
      );
      expect(archive.contains(u(0)), isTrue, reason: 'and the log has it');

      link.notify(SyncClient.messagesUuid, summaryMessage(u(0)));
      await tester.pumpAndSettle();
      expect(
        find.textContaining('Rep ${uuidShort(u(0))}'),
        findsOneWidget,
        reason: 'one row, one record',
      );
      expect(archive.reps, hasLength(1));
    },
  );
}

void _bleWiring() {
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
        home: RepListScreen(archive: MemoryArchive(), ble: ble, fixture: ''),
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
    expect(link.subscribed, [SyncClient.messagesUuid]);
    expect(find.text('Disconnect'), findsOneWidget);

    await tester.tap(find.text('Disconnect'));
    for (var i = 0; i < 8; i++) {
      await tester.pump(const Duration(milliseconds: 10));
    }
    expect(ble.status.status, BleStatus.disconnected);
    expect(find.text('Disconnected'), findsOneWidget);
  });
}
