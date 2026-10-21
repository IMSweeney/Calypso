import 'dart:async';
import 'dart:typed_data';

import 'package:calypso/model/uuid.dart';
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:calypso/state/ble_status.dart';
import 'package:calypso/store/memory_archive.dart';
import 'package:calypso/store/archive_sync.dart';
import 'package:calypso/gatt/gatt_client.dart';
import 'package:calypso/transport/ble_link.dart';
import 'package:calypso/transport/ble_transport.dart';
import 'package:calypso/model/calibration.dart';
import 'package:calypso/model/rep.dart';
import 'package:calypso/transport/transport.dart';
import 'package:flutter_test/flutter_test.dart';

import 'support/calibration.dart';
import 'support/fake_ble_link.dart';
import 'support/test_uuid.dart';

pb.RepSummary summaryProto(String uuid, {int traceLen = 7}) => pb.RepSummary(
  repUuid: uuidToBytes(uuid)!,
  tStartMs: 10_000,
  tEndMs: 11_400,
  durationMs: 1400,
  rom: 0.41,
  peakVelocity: 0.7,
  meanVelocity: 0.3,
  peakAccel: 2.5,
  rollMinDeg: -1,
  rollMaxDeg: 1,
  pitchMinDeg: -2,
  pitchMaxDeg: 2,
  traceLen: traceLen,
  chunkCount: (traceLen / 3).ceil(),
);

Uint8List summaryMessage(String uuid, {int traceLen = 7}) => pb.SyncMessage(
  summary: summaryProto(uuid, traceLen: traceLen),
).writeToBuffer();

Uint8List chunkMessage(
  String uuid,
  int index,
  int total, {
  int points = 3,
  int from = 0,
  int? totalOverride,
}) {
  final samples = [
    for (var i = 0; i < points; i++)
      pb.TraceSample(
        tOffsetMs: (from + i) * 10,
        px: 0,
        py: 0,
        pz: -0.01 * (from + i),
        ax: 0,
        ay: 0,
        az: -100,
        qw: 16384,
        qx: 0,
        qy: 0,
        qz: 0,
      ),
  ];
  return pb.SyncMessage(
    trace: pb.TraceChunk(
      repUuid: uuidToBytes(uuid)!,
      index: index,
      firstTOffsetMs: from * 10,
      totalChunks: totalOverride ?? total,
      samples: samples,
    ),
  ).writeToBuffer();
}

Future<BleTransport> connectedTransport(
  FakeBleLink link, {
  Duration? traceTimeout,
}) async {
  final transport = BleTransport(
    link,
    traceTimeout: traceTimeout ?? const Duration(milliseconds: 200),
    chunkIdleTimeout: const Duration(milliseconds: 50),
  );
  final connecting = transport.connect();
  await pumpEventQueue();
  link.seeDevice('device-1', 'Calypso');
  await connecting;
  return transport;
}

void main() {
  test(
    'connect scans for the service, subscribes, and reports connected',
    () async {
      final link = FakeBleLink();
      final transport = await connectedTransport(link);

      expect(link.scans.single, [SyncClient.serviceUuid]);
      expect(
        link.scanning,
        isFalse,
        reason: 'the scan stops once a device is found',
      );
      expect(link.subscribed, [SyncClient.messagesUuid]);
      expect(transport.status.status, BleStatus.connected);
      expect(transport.status.deviceId, 'device-1');
    },
  );

  test('discovery happens between connecting and subscribing', () async {
    final link = FakeBleLink();
    final transport = await connectedTransport(link);

    expect(link.calls, [
      'connect',
      'discoverServices',
      'subscribe:${SyncClient.messagesUuid}',
    ]);
    expect(link.discovered, ['device-1']);
    expect(transport.status.status, BleStatus.connected);
    await transport.close();
  });

  test(
    'a device that does not offer the sync service says so, and what it has',
    () async {
      final link = FakeBleLink()
        ..services = const [
          DiscoveredService('0000fff0-0000-1000-8000-00805f9b34fb', [
            '0000fff1-0000-1000-8000-00805f9b34fb',
          ]),
        ];

      final transport = await connectedTransport(link);

      final error = transport.status.error ?? '';
      expect(transport.status.status, BleStatus.failed);
      expect(link.subscribed, isEmpty, reason: 'nothing was subscribed to');
      expect(
        error,
        contains('0000fff0-0000-1000-8000-00805f9b34fb'),
        reason: 'what the device offers is in the message',
      );
      expect(
        error,
        contains('0000ca21-b0cd-11ec-871f-d45ddf138840'),
        reason: 'and so is what we were looking for',
      );
    },
  );

  test('a reversed GATT tree names both causes', () async {
    final link = FakeBleLink()
      ..services = const [
        DiscoveredService('408813df-5dd4-1f87-ec11-cdb020ca0000', [
          '408813df-5dd4-1f87-ec11-cdb021ca0000',
        ]),
      ];

    final transport = await connectedTransport(link);

    expect(transport.status.status, BleStatus.failed);
    expect(transport.status.error, contains('reversed'));
    expect(transport.status.error, contains('reflash'));
    expect(
      transport.status.error,
      contains('cached GATT database'),
      reason: 'and the cause reflashing cannot fix',
    );
  });

  test('no device found is reported, not thrown', () async {
    final link = FakeBleLink();
    final transport = BleTransport(
      link,
      traceTimeout: const Duration(milliseconds: 50),
    );

    await transport.connect(scanTimeout: const Duration(milliseconds: 20));

    expect(transport.status.status, BleStatus.failed);
    expect(transport.status.error, contains('No Calypso device'));
    expect(link.subscribed, isEmpty);
  });

  test('Bluetooth being unavailable is its own state', () async {
    final link = FakeBleLink(available: false);
    final transport = BleTransport(link);

    await transport.connect();

    expect(transport.status.status, BleStatus.unsupported);
    expect(link.scans, isEmpty, reason: 'never scans without Bluetooth');
  });

  test(
    'summaries arrive from notifications and are emitted once each',
    () async {
      final link = FakeBleLink();
      final transport = await connectedTransport(link);
      final seen = <String>[];
      final sub = transport.watchSummaries().listen((s) => seen.add(s.uuid!));

      link.notify(SyncClient.messagesUuid, summaryMessage(u(1)));
      link.notify(SyncClient.messagesUuid, summaryMessage(u(2)));
      link.notify(SyncClient.messagesUuid, summaryMessage(u(2)));
      await pumpEventQueue();

      expect(seen, [u(1), u(2)]);
      expect(
        (await transport.summaries()).map((s) => s.uuid),
        unorderedEquals([u(1), u(2)]),
      );
      await sub.cancel();
      await transport.close();
    },
  );

  test(
    'a trace request is written to the request characteristic and reassembled',
    () async {
      final link = FakeBleLink();
      final transport = await connectedTransport(link);
      link.notify(SyncClient.messagesUuid, summaryMessage(u(1), traceLen: 7));
      await pumpEventQueue();

      final pending = transport.trace(u(1));
      await pumpEventQueue();

      final request = pb.SyncRequest.fromBuffer(
        link.lastWriteTo(SyncClient.requestsUuid),
      ).trace;
      expect(uuidFromBytes(request.repUuid), u(1));
      expect(request.allChunks, isTrue);

      link.notify(
        SyncClient.messagesUuid,
        chunkMessage(u(1), 2, 3, points: 1, from: 6),
      );
      link.notify(SyncClient.messagesUuid, chunkMessage(u(1), 0, 3, from: 0));
      link.notify(SyncClient.messagesUuid, chunkMessage(u(1), 0, 3, from: 0));
      link.notify(SyncClient.messagesUuid, chunkMessage(u(1), 1, 3, from: 3));
      await pumpEventQueue();

      final trace = await pending;
      expect(trace.length, 7, reason: '3 + 3 + 1 points, duplicates ignored');
      expect(trace.first.tOffsetMs, 0);
      expect(trace.last.tOffsetMs, 60);
      expect(trace.last.pz, closeTo(-0.06, 1e-6));
      await transport.close();
    },
  );

  test('a chunk for another rep is ignored', () async {
    final link = FakeBleLink();
    final transport = await connectedTransport(link);
    final pending = transport.trace(u(1));
    await pumpEventQueue();

    link.notify(SyncClient.messagesUuid, chunkMessage(u(99), 0, 1, from: 0));
    link.notify(SyncClient.messagesUuid, chunkMessage(u(1), 0, 1, from: 0));
    await pumpEventQueue();

    expect((await pending).length, 3);
    await transport.close();
  });

  test('without totalChunks, a quiet stream counts as complete', () async {
    final link = FakeBleLink();
    final transport = await connectedTransport(link);
    final pending = transport.trace(u(1));
    await pumpEventQueue();

    link.notify(
      SyncClient.messagesUuid,
      chunkMessage(u(1), 0, 0, totalOverride: 0),
    );
    await pumpEventQueue();

    final trace = await pending;
    expect(trace.length, 3, reason: 'the idle timeout closed the stream');
    await transport.close();
  });

  test(
    'a device that never answers produces TraceUnavailable, not a hang',
    () async {
      final link = FakeBleLink();
      final transport = await connectedTransport(
        link,
        traceTimeout: const Duration(milliseconds: 30),
      );

      await expectLater(
        transport.trace(u(7)),
        throwsA(
          isA<TraceUnavailableException>()
              .having((e) => e.uuid, 'uuid', u(7))
              .having((e) => e.reason, 'reason', contains('did not answer')),
        ),
      );
      await transport.close();
    },
  );

  test('a partial stream that stops times out with what it got', () async {
    final link = FakeBleLink();
    final transport = await connectedTransport(
      link,
      traceTimeout: const Duration(milliseconds: 40),
    );
    final pending = transport.trace(u(3));
    await pumpEventQueue();

    link.notify(SyncClient.messagesUuid, chunkMessage(u(3), 0, 5, from: 0));
    link.notify(SyncClient.messagesUuid, chunkMessage(u(3), 1, 5, from: 3));
    await pumpEventQueue();

    await expectLater(
      pending,
      throwsA(
        isA<TraceUnavailableException>().having(
          (e) => e.reason,
          'reason',
          contains('2 chunks'),
        ),
      ),
    );
    await transport.close();
  });

  test('a dropped connection fails the in-flight request', () async {
    final link = FakeBleLink();
    final transport = await connectedTransport(
      link,
      traceTimeout: const Duration(seconds: 5),
    );
    final pending = transport.trace(u(1));
    await pumpEventQueue();

    final expectation = expectLater(
      pending,
      throwsA(isA<TraceUnavailableException>()),
    );

    link.dropConnection('out of range');
    await pumpEventQueue();

    expect(transport.status.status, BleStatus.disconnected);
    expect(transport.status.error, 'out of range');
    await expectation;
  });

  test('a trace already received is not re-fetched', () async {
    final link = FakeBleLink();
    final transport = await connectedTransport(link);
    final first = transport.trace(u(1));
    await pumpEventQueue();
    link.notify(SyncClient.messagesUuid, chunkMessage(u(1), 0, 1, from: 0));
    await first;

    final writesBefore = link.writes.length;
    final second = await transport.trace(u(1));
    expect(second.length, 3);
    expect(link.writes.length, writesBefore, reason: 'served from cache');
    await transport.close();
  });

  test(
    'a chunk restarts the trace timeout, so a slow stream still lands',
    () async {
      final link = FakeBleLink();
      final transport = await connectedTransport(
        link,
        traceTimeout: const Duration(milliseconds: 120),
      );

      final pending = transport.trace(u(1));
      await pumpEventQueue();
      link.notify(SyncClient.messagesUuid, chunkMessage(u(1), 0, 2));
      await Future<void>.delayed(const Duration(milliseconds: 90));
      link.notify(SyncClient.messagesUuid, chunkMessage(u(1), 1, 2));
      await Future<void>.delayed(const Duration(milliseconds: 90));

      expect((await pending).length, 6, reason: 'two 3-point chunks');
      await transport.close();
    },
  );

  test(
    'calibration arrives from the device and leaves with the connection',
    () async {
      final link = FakeBleLink();
      final transport = await connectedTransport(link);

      expect(transport.calibration.latest, isNull);

      link.notify(
        SyncClient.messagesUuid,
        calibrationMessage(sys: 1, accel: 1, gyro: 2),
      );
      await pumpEventQueue();

      final latest = transport.calibration.latest!;
      expect(latest.accel, 1);
      expect(latest.gyro, 2);
      expect(latest.ready, isFalse);
      expect(latest.action, contains('still on the rack'));

      var notifications = 0;
      transport.calibration.addListener(() => notifications++);
      link.notify(
        SyncClient.messagesUuid,
        calibrationMessage(sys: 1, accel: 1, gyro: 2),
      );
      await pumpEventQueue();
      expect(notifications, 0);

      link.notify(SyncClient.messagesUuid, calibrationMessage());
      await pumpEventQueue();
      expect(notifications, 1);
      expect(transport.calibration.isReady, isTrue);

      link.dropConnection('out of range');
      await pumpEventQueue();
      expect(transport.calibration.latest, isNull);
      expect(transport.calibration.isReady, isFalse);
    },
  );

  test('malformed notifications are ignored, not fatal', () async {
    final link = FakeBleLink();
    final transport = await connectedTransport(link);

    link.notify(
      SyncClient.messagesUuid,
      Uint8List.fromList([0xFF, 0xFF, 0xFF, 0xFF]),
    );
    link.notify(SyncClient.messagesUuid, Uint8List.fromList([0x00]));
    await pumpEventQueue();

    expect(await transport.summaries(), isEmpty);
    expect(
      transport.status.status,
      BleStatus.connected,
      reason: 'still connected',
    );

    link.notify(SyncClient.messagesUuid, summaryMessage(u(4)));
    await pumpEventQueue();
    expect((await transport.summaries()).single.uuid, u(4));
    await transport.close();
  });

  test(
    'device to plot: a synced trace lands in the archive and reads back',
    () async {
      final link = FakeBleLink();
      final transport = await connectedTransport(link);
      link.notify(SyncClient.messagesUuid, summaryMessage(u(1), traceLen: 7));
      link.notify(SyncClient.messagesUuid, summaryMessage(u(2), traceLen: 7));
      await pumpEventQueue();

      transport.watchSummaries().listen((_) {});
      unawaited(() async {
        while (transport.status.isConnected) {
          await Future<void>.delayed(const Duration(milliseconds: 5));
          if (link.writes.isEmpty) continue;
          final request = pb.SyncRequest.fromBuffer(
            link.lastWriteTo(SyncClient.requestsUuid),
          ).trace;
          final requested = uuidFromBytes(request.repUuid);
          if (requested == null) continue;
          for (var index = 0; index < 3; index++) {
            link.notify(
              SyncClient.messagesUuid,
              chunkMessage(
                requested,
                index,
                3,
                points: index == 2 ? 1 : 3,
                from: index * 3,
              ),
            );
          }
        }
      }());

      final archive = MemoryArchive();
      final added = await archiveFromTransport(archive, transport);
      expect(added, 2);

      final trace = await archive.trace(u(1));
      expect(trace.length, 7);
      expect(trace.last.pz, closeTo(-0.06, 1e-6));
      expect(archive.reps.map((r) => r.uuid), unorderedEquals([u(1), u(2)]));
      await transport.close();
    },
  );
}
