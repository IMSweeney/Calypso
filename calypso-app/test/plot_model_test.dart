import 'dart:math' as math;
import 'dart:ui' show Offset, Size;

import 'package:calypso/plot/plot_model.dart';
import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';

List<pb.TraceSample> squatTrace({int count = 100, double depth = 0.4}) => [
      for (var i = 0; i < count; i++)
        pb.TraceSample(
          tOffsetMs: i * 10,
          px: 0.02 * i / count,
          py: 0,
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

void main() {
  test('the axis fit frames the path and keeps a minimum size', () {
    final path = PlotPath.from(squatTrace(count: 50, depth: 0.4));
    expect(path.bounds.center.z, closeTo(-0.2, 0.01));
    expect(path.bounds.halfExtent, greaterThanOrEqualTo(0.2 - 0.01));

    final flat = PlotPath.from([
      pb.TraceSample(
        tOffsetMs: 0,
        px: 0,
        py: 0,
        pz: 0,
        ax: 0,
        ay: 0,
        az: 0,
        qw: 16384,
        qx: 0,
        qy: 0,
        qz: 0,
      ),
      pb.TraceSample(
        tOffsetMs: 10,
        px: 0.001,
        py: 0,
        pz: 0,
        ax: 0,
        ay: 0,
        az: 0,
        qw: 16384,
        qx: 0,
        qy: 0,
        qz: 0,
      ),
    ]);
    expect(flat.bounds.halfExtent, 0.12);
  });

  test('decimation respects the cap and keeps the last point', () {
    final trace = squatTrace(count: 1000);
    final path = PlotPath.from(trace, maxPoints: 200);

    expect(path.length, lessThanOrEqualTo(200));
    expect(path.length, greaterThan(150));
    expect(path.points.last.tOffsetMs, trace.last.tOffsetMs,
        reason: 'the rep\'s end is what you plot');
    expect(path.points.first.tOffsetMs, trace[0].tOffsetMs,
        reason: 'and so is its start');
  });

  test('decimation is a no-op when the trace already fits', () {
    final trace = squatTrace(count: 120);
    final path = PlotPath.from(trace, maxPoints: 512);
    expect(path.length, trace.length);
    expect(path.points, trace);
  });

  test('an empty or single-point trace does not throw', () {
    final empty = PlotPath.from(const []);
    expect(empty.length, 0);
    expect(empty.speedMax, 0);
    expect(empty.pathLength, 0);
    expect(empty.speedT(0), 0);

    final one = PlotPath.from(squatTrace(count: 1));
    expect(one.length, 1);
    expect(one.speedT(0), 0);
  });

  test('speed is derived from consecutive positions', () {
    final trace = [
      for (var i = 0; i < 5; i++)
        pb.TraceSample(
          tOffsetMs: i * 10,
          px: i * 0.01,
          py: 0,
          pz: 0,
          ax: 0,
          ay: 0,
          az: 0,
          qw: 16384,
          qx: 0,
          qy: 0,
          qz: 0,
        ),
    ];
    final path = PlotPath.from(trace);
    expect(path.speeds.first, 0, reason: 'no previous point to measure against');
    for (var i = 1; i < path.length; i++) {
      expect(path.speeds[i], closeTo(1.0, 1e-9));
    }
    expect(path.speedMax, closeTo(1.0, 1e-9));
    expect(path.speedT(1), closeTo(1.0, 1e-9));
    expect(path.speedT(0), 0);
    expect(path.pathLength, closeTo(0.04, 1e-9));
  });

  test('a squat path is fastest mid-descent and slowest at the turn', () {
    final path = PlotPath.from(squatTrace(count: 200, depth: 0.4));
    final quarter = path.length ~/ 4;
    final half = path.length ~/ 2;

    expect(path.speeds[quarter], greaterThan(path.speeds[1]));
    expect(path.speeds[quarter], greaterThan(path.speeds[half]));

    var slowest = 1, fastest = 1;
    for (var i = 1; i < path.length; i++) {
      if (path.speeds[i] < path.speeds[slowest]) slowest = i;
      if (path.speeds[i] > path.speeds[fastest]) fastest = i;
    }
    final nearQuarter = (fastest - quarter).abs() <= path.length ~/ 10;
    final nearThreeQuarters = (fastest - 3 * quarter).abs() <= path.length ~/ 10;
    expect(nearQuarter || nearThreeQuarters, isTrue,
        reason: 'fastest at $fastest, expected near $quarter or ${3 * quarter}');
    expect(path.speeds[slowest], lessThan(path.speedMax * 0.1));
    expect(path.speedMax, greaterThan(0.3));
    expect(path.speedMax, lessThan(1.5));
  });

  test('the camera fits the bounds and stays above the target', () {
    final path = PlotPath.from(squatTrace(count: 50, depth: 0.4));
    final camera = OrbitCamera()..fit(path.bounds);

    expect(camera.target.x, closeTo(path.bounds.center.x, 1e-9));
    expect(camera.target.z, closeTo(path.bounds.center.z, 1e-9));
    expect(camera.eye.z, greaterThan(camera.target.z), reason: 'looking down');

    camera.orbitBy(0, 1000);
    expect(camera.elevationDeg, OrbitCamera.maxElevationDeg);
    camera.orbitBy(0, -10000);
    expect(camera.elevationDeg, OrbitCamera.minElevationDeg);
    camera.zoomBy(0.0001);
    expect(camera.distance, OrbitCamera.minDistance);
    camera.zoomBy(10000);
    expect(camera.distance, OrbitCamera.maxDistance);
  });

  test('projection puts the target in the middle of the viewport', () {
    final camera = OrbitCamera(distance: 2)..fit(AxisBounds(Vector3.zero(), 0.4));
    const viewport = Size(400, 300);
    final projection = PlotProjection(camera, viewport);

    final centre = projection.toScreen(camera.target);
    expect(centre!.dx, closeTo(200, 1e-6));
    expect(centre.dy, closeTo(150, 1e-6));

    final behind = camera.eye + (camera.eye - camera.target);
    expect(projection.toScreen(behind), isNull);
    final forward = (camera.target - camera.eye).normalized();
    expect(projection.depth(camera.target),
        lessThan(projection.depth(camera.target + forward)));
    expect(projection.depth(camera.target),
        greaterThan(projection.depth(camera.target - forward)));
  });

  test('hit testing finds the path point under the cursor', () {
    final path = PlotPath.from(squatTrace(count: 100, depth: 0.4));
    final camera = OrbitCamera()..fit(path.bounds);
    const viewport = Size(400, 300);
    final projection = PlotProjection(camera, viewport);

    final target = path.points[path.length ~/ 2];
    final screen = projection.toScreen(Vector3(target.px, target.py, target.pz))!;
    final hit = projection.nearestPoint(path, screen);
    expect(hit, isNotNull);
    expect(path.points[hit!].tOffsetMs, target.tOffsetMs);

    expect(projection.nearestPoint(path, const Offset(1, 1)), isNull);
  });
}
