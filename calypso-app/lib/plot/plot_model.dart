library;

import 'dart:math' as math;
import 'dart:ui' show Offset, Size;

import 'package:vector_math/vector_math_64.dart';

import '../model/rep.dart';

class PlotPath {
  PlotPath._(this.points, this.speeds, this.speedMax, this.bounds);

  final List<TracePoint> points;
  final List<double> speeds;
  final double speedMax;
  final AxisBounds bounds;

  int get length => points.length;

  double speedT(int index) =>
      speedMax <= 0 ? 0 : (speeds[index] / speedMax).clamp(0.0, 1.0);

  double get pathLength {
    var total = 0.0;
    for (var i = 1; i < points.length; i++) {
      total += _distance(points[i - 1], points[i]);
    }
    return total;
  }

  static PlotPath from(List<TracePoint> all, {int maxPoints = 512}) {
    if (all.isEmpty) {
      return PlotPath._(const [], const [], 0, AxisBounds.of(const []));
    }
    final points = _decimate(all, maxPoints);

    final speeds = <double>[0];
    for (var i = 1; i < points.length; i++) {
      final dtMs = points[i].tOffsetMs - points[i - 1].tOffsetMs;
      final d = _distance(points[i - 1], points[i]);
      speeds.add(dtMs <= 0 ? 0 : d * 1000 / dtMs);
    }
    var speedMax = 0.0;
    for (final s in speeds) {
      if (s > speedMax) speedMax = s;
    }
    return PlotPath._(points, speeds, speedMax, AxisBounds.of(points));
  }

  static List<TracePoint> _decimate(List<TracePoint> all, int maxPoints) {
    final n = all.length;
    if (maxPoints < 2 || n <= maxPoints) return List.of(all);

    var stride = ((n - 1) / (maxPoints - 1)).ceil();
    while (true) {
      final indices = <int>[
        for (var i = 0; i < n; i += stride) i,
        if ((n - 1) % stride != 0) n - 1,
      ];
      if (indices.length <= maxPoints) {
        return [for (final i in indices) all[i]];
      }
      stride++;
    }
  }

  static double _distance(TracePoint a, TracePoint b) {
    final dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    return math.sqrt(dx * dx + dy * dy + dz * dz);
  }
}

class AxisBounds {
  AxisBounds(this.center, this.halfExtent);

  final Vector3 center;
  final double halfExtent;

  static AxisBounds of(List<TracePoint> points, {double minHalfExtent = 0.12}) {
    if (points.isEmpty) return AxisBounds(Vector3.zero(), minHalfExtent);
    var minX = double.infinity, minY = double.infinity, minZ = double.infinity;
    var maxX = -double.infinity, maxY = -double.infinity, maxZ = -double.infinity;
    for (final p in points) {
      minX = math.min(minX, p.x);
      maxX = math.max(maxX, p.x);
      minY = math.min(minY, p.y);
      maxY = math.max(maxY, p.y);
      minZ = math.min(minZ, p.z);
      maxZ = math.max(maxZ, p.z);
    }
    final center = Vector3((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2);
    final extent = math.max(
      math.max(maxX - minX, maxY - minY),
      math.max(maxZ - minZ, minHalfExtent * 2),
    );
    return AxisBounds(center, math.max(extent / 2, minHalfExtent));
  }
}

class OrbitCamera {
  OrbitCamera({
    this.azimuthDeg = -55,
    this.elevationDeg = 22,
    this.distance = 2,
    Vector3? target,
  }) : target = target ?? Vector3.zero();

  double azimuthDeg;
  double elevationDeg;
  double distance;
  Vector3 target;

  static const double minElevationDeg = -85;
  static const double maxElevationDeg = 85;
  static const double minDistance = 0.2;
  static const double maxDistance = 20;
  static const double fovDeg = 35;

  Vector3 get eye {
    final az = azimuthDeg * math.pi / 180;
    final el = elevationDeg * math.pi / 180;
    return target +
        Vector3(
          distance * math.cos(el) * math.cos(az),
          distance * math.cos(el) * math.sin(az),
          distance * math.sin(el),
        );
  }

  void fit(AxisBounds bounds) {
    target = bounds.center.clone();
    final fov = fovDeg * math.pi / 180;
    // A little margin so the path doesn't touch the edges.
    distance = (bounds.halfExtent / math.sin(fov / 2)) * 1.6;
    distance = distance.clamp(minDistance, maxDistance);
  }

  void orbitBy(double deltaAzimuthDeg, double deltaElevationDeg) {
    azimuthDeg = (azimuthDeg + deltaAzimuthDeg) % 360;
    elevationDeg =
        (elevationDeg + deltaElevationDeg).clamp(minElevationDeg, maxElevationDeg);
  }

  void zoomBy(double factor) {
    distance = (distance * factor).clamp(minDistance, maxDistance);
  }
}

class PlotProjection {
  PlotProjection(this.camera, this.viewport) {
    _eye = camera.eye;
    final forward = (camera.target - _eye).normalized();
    final worldUp = Vector3(0, 0, 1);
    var right = forward.cross(worldUp);
    if (right.length2 < 1e-9) right = Vector3(1, 0, 0); // looking straight down
    right.normalize();
    final up = right.cross(forward).normalized();
    _forward = forward;
    _right = right;
    _up = up;
    _focal =
        (viewport.shortestSide / 2) / math.tan(OrbitCamera.fovDeg * math.pi / 360);
  }

  final OrbitCamera camera;
  final Size viewport;

  late final Vector3 _eye;
  late final Vector3 _forward;
  late final Vector3 _right;
  late final Vector3 _up;
  late final double _focal;

  static const double nearPlane = 0.01;

  double depth(Vector3 world) => (world - _eye).dot(_forward);

  Offset? toScreen(Vector3 world) {
    final d = world - _eye;
    final z = d.dot(_forward);
    if (z <= nearPlane) return null;
    final x = d.dot(_right);
    final y = d.dot(_up);
    return Offset(
      viewport.width / 2 + x * _focal / z,
      viewport.height / 2 - y * _focal / z,
    );
  }

  int? nearestPoint(PlotPath path, Offset at, {double maxDistancePx = 24}) {
    int? best;
    var bestDistance = maxDistancePx;
    for (var i = 0; i < path.length; i++) {
      final p = path.points[i];
      final screen = toScreen(Vector3(p.x, p.y, p.z));
      if (screen == null) continue;
      final d = (screen - at).distance;
      if (d < bestDistance) {
        bestDistance = d;
        best = i;
      }
    }
    return best;
  }
}
