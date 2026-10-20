import 'dart:io';
import 'dart:math' as math;

import 'package:calypso/model/rep.dart';
import 'package:calypso/plot/rep_plot.dart';
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:calypso/transport/fixture_transport.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'support/test_uuid.dart';

List<pb.TraceSample> squatTrace({
  int count = 160,
  double depth = 0.4,
  double drift = 0.03,
}) => [
  for (var i = 0; i < count; i++)
    pb.TraceSample(
      tOffsetMs: i * 10,
      px: drift * (1 - math.cos(2 * math.pi * i / (count - 1))) / 2,
      py: 0.01 * math.sin(2 * math.pi * i / (count - 1)),
      pz: -depth * (1 - math.cos(2 * math.pi * i / (count - 1))) / 2,
      ax: 0,
      ay: 0,
      az: -100,
      qw: 16384,
      qx: 0,
      qy: 0,
      qz: 0,
    ),
];

Future<void> pumpPlot(
  WidgetTester tester,
  List<pb.TraceSample> points, {
  int? scrubIndex,
  Size size = const Size(400, 300),
}) async {
  await tester.binding.setSurfaceSize(size);
  addTearDown(() => tester.binding.setSurfaceSize(null));
  await tester.pumpWidget(
    MaterialApp(
      home: Scaffold(
        body: Center(
          child: SizedBox(
            width: size.width,
            height: size.height,
            child: RepPlotView(points: points, scrubIndex: scrubIndex),
          ),
        ),
      ),
    ),
  );
  await tester.pumpAndSettle();
}

void main() {
  _fixtureEndToEnd();

  testWidgets('a squat rep, from the default camera', (tester) async {
    await pumpPlot(tester, squatTrace());
    await expectLater(
      find.byType(RepPlotView),
      matchesGoldenFile('goldens/plot_squat.png'),
    );
  });

  testWidgets('the same rep mid-scrub, showing the orientation gizmo', (
    tester,
  ) async {
    await pumpPlot(tester, squatTrace(), scrubIndex: 80);
    await expectLater(
      find.byType(RepPlotView),
      matchesGoldenFile('goldens/plot_squat_scrubbed.png'),
    );
  });

  testWidgets('a one-point trace does not break the view', (tester) async {
    await pumpPlot(tester, [
      pb.TraceSample(
        tOffsetMs: 0,
        px: 0,
        py: 0,
        pz: -0.01,
        ax: 0,
        ay: 0,
        az: -100,
        qw: 16384,
        qx: 0,
        qy: 0,
        qz: 0,
      ),
    ]);
    await expectLater(
      find.byType(RepPlotView),
      matchesGoldenFile('goldens/plot_single_point.png'),
    );
  });
}

void _fixtureEndToEnd() {
  testWidgets('a dipped squat fixture, straight from the generator', (
    tester,
  ) async {
    final bytes = File('assets/fixtures/dip-left.pblog').readAsBytesSync();
    final transport = FixtureTransport.fromBytes(bytes);
    final summaries = await transport.summaries();
    expect(summaries, isNotEmpty);
    expect(summaries.first.uuid, u(1));
    final trace = await transport.trace(summaries.first.uuid!);
    expect(trace.length, greaterThan(50));

    await pumpPlot(tester, trace, scrubIndex: trace.length ~/ 2);
    await expectLater(
      find.byType(RepPlotView),
      matchesGoldenFile('goldens/plot_dip_fixture.png'),
    );
  });
}
