// Logging: the pipeline is `package:logging`, and what is ours is the rotating
// file appender and the one function that turns it on. The failure cases here
// matter as much as the happy one — a debug log that throws while you are
// debugging the thing it is watching is worse than no log, and a log that is
// silently empty is worse still.

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

  /// A file's lines, split into fields:
  /// `<timestamp> <LEVEL> <loggerName> <rx|tx> <uuid> <n>B <hex…>`.
  ///
  /// On runs of whitespace, not on single spaces: the level is padded to hold a
  /// column, so the gaps are deliberate.
  Future<List<List<String>>> fieldsOf(String name) async {
    final file = logFile(name);
    if (!await file.exists()) return const [];
    return [
      for (final line in await file.readAsLines())
        if (line.trim().isNotEmpty) line.trim().split(RegExp(r'\s+')),
    ];
  }

  /// A message as the BLE transport logs one, for the rep numbered [i].
  String message(int i) => 'rx ca21 1B ${i.toRadixString(16).padLeft(2, '0')}';

  /// The hex a payload should appear as.
  ///
  /// Computed from the bytes themselves rather than written out by hand: what the
  /// test is pinning is "the bytes that crossed the link, verbatim", and the
  /// *encoding* of the message that carried them is not its business (the device's
  /// own encoder omits zero-valued fields; Dart's protobuf does not, so the same
  /// logical message is a different number of bytes on each side).
  String hexOf(List<int> bytes) =>
      bytes.map((b) => b.toRadixString(16).padLeft(2, '0')).join(' ');

  group('startFileLogging', () {
    test('records below the root level still reach the file', () async {
      // The trap this exists for: `Logger.root.level` defaults to `Level.INFO`, so
      // a debug-level line is dropped *before* any appender sees it — and the file
      // is created anyway, leaving a log that looks fine and is empty.
      final logging = startFileLogging(() async => dir);
      Logger('ble').fine(message(0));
      await logging.close();

      final lines = await fieldsOf('calypso.log');
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

      final lines = await fieldsOf('calypso.log');
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

      // Rotations shift up, so anything past `keep - 1` is gone...
      expect(await logFile('calypso.log').exists(), isTrue);
      expect(await logFile('calypso.log.$keep').exists(), isFalse);

      // ...and what survives is whole lines, in order, ending with the last write.
      final payloads = <int>[];
      for (final name in ['calypso.log.2', 'calypso.log.1', 'calypso.log']) {
        for (final fields in await fieldsOf(name)) {
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

      final lines = await fieldsOf('calypso.log');
      expect(lines, hasLength(1), reason: 'the appender stopped listening');
      expect(lines.single.last, '01');
    });

    test('a log it cannot open is dropped, and never thrown', () async {
      // A *file* where the directory should be: `create(recursive: true)` fails.
      final blocker = File('${dir.path}/blocker')
        ..writeAsStringSync('not a directory');
      final logging = startFileLogging(() async => Directory('${blocker.path}/nested'));

      Logger('ble').fine(message(1)); // queued, then dropped when the write fails
      await logging.close();
    });
  });

  test('the transport reports every message it sends and receives', () async {
    // No logger is passed in anywhere: the transport logs under `ble` and this
    // collects it from the pipeline, which is also why a test that does not
    // capture writes no file.
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
    link.notify(CalypsoGatt.summary, calibration);
    link.notify(CalypsoGatt.traceChunk, Uint8List(0));
    await pumpEventQueue();

    // A trace request is the app's half of the conversation, logged as it goes out.
    await expectLater(
      transport.trace(u(1)),
      throwsA(isA<TraceUnavailableException>()),
    );
    await transport.close();

    expect(records, isNotEmpty);
    expect(records.map((r) => r.loggerName), everyElement('ble'));
    expect(records.map((r) => r.level), everyElement(Level.FINE));

    final messages = [for (final record in records) record.message];
    expect(
      messages,
      contains('rx ${CalypsoGatt.summary} ${calibration.length}B '
          '${hexOf(calibration)}'),
      reason: 'inbound, on the characteristic it arrived on, bytes and all',
    );
    expect(
      messages,
      contains('rx ${CalypsoGatt.traceChunk} 0B'),
      reason: 'the other stream is named separately',
    );
    expect(
      messages.where((m) => m.startsWith('tx ${CalypsoGatt.traceRequest} ')),
      hasLength(1),
      reason: 'the request the app sent',
    );
  });
}
