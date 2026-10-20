library;

import 'package:flutter/material.dart';
import 'package:vector_math/vector_math_64.dart' show Vector3;

import '../model/rep.dart';
import 'plot_model.dart';

const Color _slowColour = Color(0xFF4C6EF5);
const Color _midColour = Color(0xFF51CF66);
const Color _fastColour = Color(0xFFFF6B35);

Color colourForSpeed(double t) {
  final clamped = t.clamp(0.0, 1.0);
  if (clamped <= 0.5) {
    return Color.lerp(_slowColour, _midColour, clamped * 2)!;
  }
  return Color.lerp(_midColour, _fastColour, (clamped - 0.5) * 2)!;
}

class RepPlotPainter extends CustomPainter {
  RepPlotPainter({
    required this.path,
    required this.camera,
    this.scrubIndex,
  });

  final PlotPath path;
  final OrbitCamera camera;

  final int? scrubIndex;

  @override
  void paint(Canvas canvas, Size size) {
    if (size.isEmpty || path.length == 0) return;
    final projection = PlotProjection(camera, size);
    final bounds = path.bounds;

    _paintGrid(canvas, projection, bounds);
    _paintAxes(canvas, projection, bounds);
    _paintPath(canvas, projection);
    _paintMarkers(canvas, projection);
    _paintGizmo(canvas, projection);
  }

  void _paintGrid(Canvas canvas, PlotProjection projection, AxisBounds bounds) {
    final paint = Paint()
      ..color = const Color(0xFF2A3040)
      ..strokeWidth = 1;
    const divisions = 8;
    final step = bounds.halfExtent * 2 / divisions;
    final z = bounds.center.z - bounds.halfExtent;
    for (var i = 0; i <= divisions; i++) {
      final offset = -bounds.halfExtent + i * step;
      final x0 = bounds.center.x + offset;
      final y0 = bounds.center.y + offset;
      _line(canvas, projection, paint, Vector3(x0, bounds.center.y - bounds.halfExtent, z),
          Vector3(x0, bounds.center.y + bounds.halfExtent, z));
      _line(canvas, projection, paint, Vector3(bounds.center.x - bounds.halfExtent, y0, z),
          Vector3(bounds.center.x + bounds.halfExtent, y0, z));
    }
  }

  void _paintAxes(Canvas canvas, PlotProjection projection, AxisBounds bounds) {
    final origin = Vector3(
      bounds.center.x - bounds.halfExtent,
      bounds.center.y - bounds.halfExtent,
      bounds.center.z - bounds.halfExtent,
    );
    final length = bounds.halfExtent * 2;
    final axes = [
      (Vector3(1, 0, 0), const Color(0xFFE03131)), // x
      (Vector3(0, 1, 0), const Color(0xFF2F9E44)), // y
      (Vector3(0, 0, 1), const Color(0xFF1971C2)), // z
    ];
    for (final (direction, colour) in axes) {
      final paint = Paint()
        ..color = colour
        ..strokeWidth = 2;
      _line(canvas, projection, paint, origin, origin + direction * length);
    }
  }

  void _paintPath(Canvas canvas, PlotProjection projection) {
    for (var i = 1; i < path.length; i++) {
      final a = path.points[i - 1];
      final b = path.points[i];
      final paint = Paint()
        ..color = colourForSpeed(path.speedT(i))
        ..strokeWidth = 3
        ..strokeCap = StrokeCap.round;
      _line(canvas, projection, paint,
          Vector3(a.x, a.y, a.z), Vector3(b.x, b.y, b.z));
    }
  }

  void _paintMarkers(Canvas canvas, PlotProjection projection) {
    if (path.length == 0) return;
    final first = path.points.first;
    final last = path.points.last;
    _dot(canvas, projection, Vector3(first.x, first.y, first.z),
        const Color(0xFFFFFFFF), 4);
    _dot(canvas, projection, Vector3(last.x, last.y, last.z),
        const Color(0xFFFFD43B), 5);

    final index = scrubIndex;
    if (index != null && index >= 0 && index < path.length) {
      final p = path.points[index];
      _dot(canvas, projection, Vector3(p.x, p.y, p.z), const Color(0xFFF8F9FA), 7);
      _dot(canvas, projection, Vector3(p.x, p.y, p.z), const Color(0xFF212529), 3);
    }
  }

  void _paintGizmo(Canvas canvas, PlotProjection projection) {
    if (path.length == 0) return;
    final index = (scrubIndex ?? 0).clamp(0, path.length - 1).toInt();
    final point = path.points[index];
    final axisLength = path.bounds.halfExtent * 0.35;
    final quaternion = point.quaternion;
    final base = Vector3(point.x, point.y, point.z);

    const colours = [
      Color(0xFFE03131),
      Color(0xFF2F9E44),
      Color(0xFF1971C2),
    ];
    final axes = [Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)];
    for (var i = 0; i < 3; i++) {
      final rotated = _rotateByQuaternion(quaternion, axes[i]) * axisLength;
      final paint = Paint()
        ..color = colours[i]
        ..strokeWidth = 3;
      _line(canvas, projection, paint, base, base + rotated);
    }
  }

  static Vector3 _rotateByQuaternion(List<double> q, Vector3 v) {
    final w = q[0], x = q[1], y = q[2], z = q[3];
    final tx = 2 * (y * v.z - z * v.y);
    final ty = 2 * (z * v.x - x * v.z);
    final tz = 2 * (x * v.y - y * v.x);
    return Vector3(
      v.x + w * tx + (y * tz - z * ty),
      v.y + w * ty + (z * tx - x * tz),
      v.z + w * tz + (x * ty - y * tx),
    );
  }

  void _line(Canvas canvas, PlotProjection projection, Paint paint, Vector3 a, Vector3 b) {
    final from = projection.toScreen(a);
    final to = projection.toScreen(b);
    if (from == null || to == null) return;
    canvas.drawLine(from, to, paint);
  }

  void _dot(Canvas canvas, PlotProjection projection, Vector3 at, Color colour, double radius) {
    final screen = projection.toScreen(at);
    if (screen == null) return;
    canvas.drawCircle(screen, radius, Paint()..color = colour);
  }

  @override
  bool shouldRepaint(RepPlotPainter oldDelegate) {
    final old = oldDelegate;
    return old.path != path ||
        old.camera != camera ||
        old.scrubIndex != scrubIndex ||
        old.camera.azimuthDeg != camera.azimuthDeg ||
        old.camera.elevationDeg != camera.elevationDeg ||
        old.camera.distance != camera.distance;
  }
}

class RepPlotView extends StatefulWidget {
  const RepPlotView({
    super.key,
    required this.points,
    this.scrubIndex,
    this.onScrub,
    this.background = const Color(0xFF14161C),
  });

  final List<TracePoint> points;
  final int? scrubIndex;
  final ValueChanged<int>? onScrub;

  final Color background;

  @override
  State<RepPlotView> createState() => RepPlotViewState();
}

class RepPlotViewState extends State<RepPlotView> {
  late PlotPath _path;
  final OrbitCamera _camera = OrbitCamera();

  @override
  void initState() {
    super.initState();
    _rebuild();
  }

  @override
  void didUpdateWidget(RepPlotView oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.points != widget.points) _rebuild();
  }

  void _rebuild() {
    _path = PlotPath.from(widget.points);
    _camera.fit(_path.bounds);
  }

  void refit() => setState(() => _camera.fit(_path.bounds));

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onDoubleTap: refit,
      onScaleStart: (_) {},
      onScaleUpdate: (details) {
        setState(() {
          if (details.pointerCount == 1) {
            _camera.orbitBy(details.focalPointDelta.dx * 0.6, -details.focalPointDelta.dy * 0.6);
          } else if (details.scale != 1.0) {
            _camera.zoomBy(1 / details.scale);
          }
        });
      },
      onTapUp: widget.onScrub == null
          ? null
          : (details) {
              final size = context.size;
              if (size == null) return;
              final projection = PlotProjection(_camera, size);
              final hit = projection.nearestPoint(_path, details.localPosition);
              if (hit != null) widget.onScrub!(hit);
            },
      child: Container(
        color: widget.background,
        child: CustomPaint(
          painter: RepPlotPainter(
            path: _path,
            camera: _camera,
            scrubIndex: widget.scrubIndex,
          ),
          size: Size.infinite,
        ),
      ),
    );
  }
}
