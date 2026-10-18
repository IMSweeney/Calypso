// The sync client against a scripted transport: no device, no BLE, no timing.

import 'dart:async';

import 'package:calypso/sync/sync_client.dart';
import 'package:flutter_test/flutter_test.dart';

import 'support/fake_transport.dart';
import 'support/test_uuid.dart';

void main() {
  test('catch-up merges everything the device has, once', () async {
    final transport = FakeTransport(stored: [summary(1), summary(2), summary(3)]);
    final client = SyncClient(transport);

    expect(await client.syncSummaries(), 3);
    expect(client.summaries.map((s) => s.uuid), [u(1), u(2), u(3)]);
    expect(client.state, SyncState.live);

    // Syncing again changes nothing: summaries are keyed by rep uuid.
    expect(await client.syncSummaries(), 0);
    expect(client.duplicatesIgnored, 3);
    expect(client.summaries.length, 3);
  });

  test('live summaries arrive and merge as they are recorded', () async {
    final transport = FakeTransport(stored: [summary(1)]);
    final client = SyncClient(transport);
    // As the app will: catch up first, then follow live.
    client.start();
    await client.syncSummaries();
    await pumpEventQueue();

    transport.emit(summary(2));
    transport.emit(summary(3));
    await pumpEventQueue();

    expect(client.summaries.map((s) => s.uuid), [u(1), u(2), u(3)]);
    expect(client.state, SyncState.live);

    // A summary that arrives twice (re-subscribe, or a device re-send) is
    // merged once.
    transport.emit(summary(3));
    await pumpEventQueue();
    expect(client.summaries.length, 3);
    expect(client.duplicatesIgnored, 1);

    await client.close();
  });

  test('a device-side wrap does not cost the client its archive', () async {
    final transport = FakeTransport(stored: [summary(1), summary(2), summary(3)]);
    final client = SyncClient(transport);
    await client.syncSummaries();
    expect(client.summaries.length, 3);

    // The ring wrapped: reps 1 and 2 are gone from the device, and 4 is new.
    transport.setStored([summary(3), summary(4)]);
    expect(await client.syncSummaries(), 1);

    // The client keeps what it saw — it is the archive of record — and picks
    // up the new rep.
    expect(client.summaries.map((s) => s.uuid), [u(1), u(2), u(3), u(4)]);
  });

  test('a trace still on the device comes back whole', () async {
    final points = tracePoints(10);
    final transport = FakeTransport(stored: [summary(1)], tracePoints: {1: points});
    final client = SyncClient(transport);

    final (result, trace) = await client.fetchTrace(u(1));
    expect(result, TraceResult.ok);
    expect(trace.length, 10);
    expect(trace.last.z, closeTo(-0.4, 1e-9));
  });

  test('a trace the device has forgotten reports unavailable, not an error',
      () async {
    final transport = FakeTransport(stored: [summary(1), summary(2)], tracesMissing: {1});
    final client = SyncClient(transport);

    // A rep that has no summary at all: the app never knew about it.
    final (unknown, unknownTrace) = await client.fetchTrace(u(99));
    expect(unknown, TraceResult.unavailable);
    expect(unknownTrace, isEmpty);

    // A rep the app knows about, whose trace the ring has since erased.
    final (gone, goneTrace) = await client.fetchTrace(u(1));
    expect(gone, TraceResult.unavailable);
    expect(goneTrace, isEmpty);
    expect(client.state, isNot(SyncState.failed),
        reason: 'a missing trace is expected, not a failure');
  });

  test('a trace request that never answers times out', () async {
    final transport = FakeTransport(
      stored: [summary(1)],
      tracePoints: {1: tracePoints(4)},
      traceDelay: const Duration(milliseconds: 200),
    );
    final client = SyncClient(transport, traceTimeout: const Duration(milliseconds: 20));

    final (result, trace) = await client.fetchTrace(u(1));
    expect(result, TraceResult.failed);
    expect(trace, isEmpty);
    expect(client.lastError, isA<TimeoutException>());
  });

  test('a transport failure surfaces without losing the archive', () async {
    final transport = FakeTransport(stored: [summary(1)]);
    final client = SyncClient(transport);
    await client.syncSummaries();

    // The device becomes unreachable mid-session.
    final broken = SyncClient(FakeTransport(failSummaries: true));
    await expectLater(broken.syncSummaries(), throwsA(isA<StateError>()));
    expect(broken.state, SyncState.failed);
    expect(broken.lastError, isA<StateError>());

    // The client that already had data keeps it.
    expect(client.summaries.length, 1);
  });

  test('a dropped connection keeps what was already synced', () async {
    final transport = FakeTransport(stored: [summary(1)]);
    final client = SyncClient(transport);
    client.start();
    await client.syncSummaries();
    await pumpEventQueue();
    transport.emit(summary(2));
    await pumpEventQueue();
    expect(client.summaries.length, 2);

    transport.dropConnection();
    await pumpEventQueue();
    expect(client.summaries.length, 2, reason: 'the archive survives a disconnect');
  });
}
