import 'dart:io';
import 'dart:typed_data';

import 'package:calypso/logging/setup.dart';
import 'package:calypso/transport/ble_transport.dart';
import 'package:calypso/transport/transport.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:logging/logging.dart';

import 'support/calibration.dart';
import 'support/fake_ble_link.dart';
import 'support/logging.dart';
import 'support/test_uuid.dart';

void main() {
  late Directory dir;

  setUp(() async {
    dir = await Directory.systemTemp.createTemp('calypso-log-');
  });

  tearDown(() async {
    Logger.root.clearListeners();
    if (dir.existsSync()) await dir.delete(recursive: true);
  });

  File logFile(String name) => File('${dir.path}/$name');

  Future<List<List<String>>> bleLines(String name) async {
    final file = logFile(name);
    if (!await file.exists()) return const [];
    return [
      for (final line in await file.readAsLines())
        if (line.contains(' ble ')) line.trim().split(RegExp(r'\s+')),
    ];
  }

  String message(int i) => 'rx ca21 1B ${i.toRadixString(16).padLeft(2, '0')}';

  String line(String direction, String characteristic, List<int> bytes) {
    final hex = bytes.map((b) => b.toRadixString(16).padLeft(2, '0')).join(' ');
    return hex.isEmpty
        ? '$direction $characteristic ${bytes.length}B'
        : '$direction $characteristic ${bytes.length}B $hex';
  }

  group('startFileLogging', () {
    test('says so at startup, so a quiet file is never ambiguous', () async {
      final logging = startFileLogging(() async => dir);
      await logging.close();

      final lines = await logFile('calypso.log').readAsLines();
      expect(lines, hasLength(1));
      expect(lines.single, contains('file logging started'));
      expect(lines.single, contains(' app '));
    });

    test('records below the root level still reach the file', () async {
      final logging = startFileLogging(() async => dir);
      Logger('ble').fine(message(0));
      await logging.close();

      final lines = await bleLines('calypso.log');
      expect(lines, hasLength(1));
      expect(lines.single, hasLength(7));
      expect(
        lines.single.sublist(1),
        ['FINE', 'ble', 'rx', 'ca21', '1B', '00'],
        reason: 'time, level, logger name, message',
      );
      expect(DateTime.tryParse(lines.single.first), isNotNull, reason: 'timestamped');
    });

    test('appends to what a previous session left', () async {
      final first = startFileLogging(() async => dir);
      Logger('ble').fine(message(1));
      await first.close();

      final second = startFileLogging(() async => dir);
      Logger('ble').fine(message(2));
      await second.close();

      final lines = await bleLines('calypso.log');
      expect(lines, hasLength(2), reason: 'the earlier session is still there');
      expect(lines.last.last, '02');
    });

    test('rotates at maxBytes, keeps the newest files, loses no line', () async {
      const keep = 3;
      final logging = startFileLogging(() async => dir, maxBytes: 120, keep: keep);
      final log = Logger('ble');
      for (var i = 0; i < 12; i++) {
        log.fine(message(i));
      }
      await logging.close();

      expect(await logFile('calypso.log').exists(), isTrue);
      expect(await logFile('calypso.log.$keep').exists(), isFalse);

      final payloads = <int>[];
      for (final name in ['calypso.log.2', 'calypso.log.1', 'calypso.log']) {
        for (final fields in await bleLines(name)) {
          expect(fields, hasLength(7), reason: 'a line is never half-written');
          payloads.add(int.parse(fields.last, radix: 16));
        }
      }
      expect(payloads, isNotEmpty);
      expect(
        payloads,
        orderedEquals([for (var i = payloads.first; i <= payloads.last; i++) i]),
        reason: 'no line lost at a rotation boundary',
      );
      expect(payloads.last, 11);
    });

    test('closing detaches: nothing is written afterwards', () async {
      final logging = startFileLogging(() async => dir);
      Logger('ble').fine(message(1));
      await logging.close();

      Logger('ble').fine(message(2));
      await logging.close(); // idempotent

      final lines = await bleLines('calypso.log');
      expect(lines, hasLength(1), reason: 'the appender stopped listening');
      expect(lines.single.last, '01');
    });

    test('a log it cannot open is dropped, and never thrown', () async {
      final blocker = File('${dir.path}/blocker')
        ..writeAsStringSync('not a directory');
      final logging = startFileLogging(() async => Directory('${blocker.path}/nested'));

      Logger('ble').fine(message(1));
      await logging.close();
    });
  });

  test('the transport reports the session, and every message in it', () async {
    final records = captureLogs();
    final link = FakeBleLink();
    final transport = BleTransport(
      link,
      traceTimeout: const Duration(milliseconds: 50),
      chunkIdleTimeout: const Duration(milliseconds: 20),
    );

    final connecting = transport.connect();
    await pumpEventQueue();
    link.seeDevice('device-1', 'Calypso');
    await connecting;

    final calibration = calibrationMessage(sys: 1, accel: 1, gyro: 2);
    link.notify(CalypsoGatt.messages, calibration);
    link.notify(CalypsoGatt.messages, Uint8List(0));
    await pumpEventQueue();

    await expectLater(
      transport.trace(u(1)),
      throwsA(isA<TraceUnavailableException>()),
    );

    link.dropConnection('out of range');
    await pumpEventQueue();
    await transport.close();

    final session = [
      for (final record in records)
        if (record.level >= Level.INFO) record.message,
    ];
    expect(session, contains('scanning for ${CalypsoGatt.service}'));
    expect(session, contains('connecting to device-1'));
    expect(session, contains('subscribed to ${CalypsoGatt.messages}'));
    expect(session, contains('connected to device-1'));
    expect(session, contains('connection lost: out of range'));

    expect(records.map((r) => r.loggerName), everyElement('ble'));
    final messages = [for (final record in records) record.message];
    expect(
      messages,
      contains(line('rx', CalypsoGatt.messages, calibration)),
      reason: 'inbound, on the characteristic it arrived on, bytes and all',
    );
    expect(
      messages,
      contains(line('rx', CalypsoGatt.messages, Uint8List(0))),
      reason: 'the other stream is named separately',
    );
    expect(
      messages.where((m) => m.startsWith('tx ${CalypsoGatt.requests} ')),
      hasLength(1),
      reason: 'the request the app sent',
    );
  });
}
