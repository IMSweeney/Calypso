import 'dart:io';

import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:calypso/protocol/pblog.dart';
import 'package:calypso/store/imu_recording.dart';
import 'package:flutter_test/flutter_test.dart';

pb.ImuChunk chunk(int seq, {int firstTMs = 5000}) => pb.ImuChunk(
  firstTMs: firstTMs,
  sampleRateHz: 100,
  seq: seq,
  samples: [
    pb.TraceRawPoint(tOffsetMs: seq * 80, ax: 981, ay: -2, az: 12, gx: 16),
  ],
);

void main() {
  test('a recording is a pblog of one clock and the live chunks', () async {
    final directory = await Directory.systemTemp.createTemp('calypso-imu-');
    addTearDown(() => directory.delete(recursive: true));
    final now = DateTime.fromMillisecondsSinceEpoch(1_700_000_000_000);
    final recording = ImuRecording.start(directory, now: () => now);

    recording.add(chunk(0));
    recording.add(chunk(1));
    recording.stop();

    final dump = readDump(await recording.file.readAsBytes());
    expect(dump.truncated, isFalse);
    expect(dump.records.map((r) => r.whichBody()), [
      pb.RepRecord_Body.clock,
      pb.RepRecord_Body.imu,
      pb.RepRecord_Body.imu,
    ]);
    // The clock is the phone's wall time at the first chunk minus that chunk's device uptime.
    expect(
      dump.records.first.clock.epochS,
      (now.millisecondsSinceEpoch - 5000) ~/ 1000,
    );
    expect(dump.records[1].imu.seq, 0);
    expect(dump.records[2].imu.seq, 1);
    expect(dump.records[1].imu.samples.single.ax, 981);
    expect(recording.samples, 2);
  });

  test('a recording with no chunks writes no clock', () async {
    final directory = await Directory.systemTemp.createTemp('calypso-imu-');
    addTearDown(() => directory.delete(recursive: true));
    final recording = ImuRecording.start(directory);

    recording.stop();

    expect(await recording.file.length(), 0);
    expect(recording.samples, 0);
  });
}
