library;

import 'package:flutter/material.dart';

import '../model/calibration.dart';
import '../state/ble_status.dart';
import '../state/calibration.dart';

class CalibrationHint extends StatelessWidget {
  const CalibrationHint({
    super.key,
    required this.calibration,
    required this.status,
  });

  final CalibrationNotifier calibration;
  final BleStatusNotifier status;

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: Listenable.merge([calibration, status]),
      builder: (context, _) {
        final latest = calibration.latest;
        // On the levels, not on `ready`: `ready` is the device's recording gate, which opens on the gyroscope alone
        // and so stays true through an accelerometer that has wandered (see `model/calibration.dart`).
        if (!status.isConnected || latest == null || !latest.needsAttention) {
          return const SizedBox.shrink();
        }
        final scheme = Theme.of(context).colorScheme;
        // Red means no reps are being recorded; the advisory colour means they are, and this is about how much to
        // trust them. The two are worth telling apart at a glance from across a gym.
        final paused = !latest.records;
        final background = paused
            ? scheme.errorContainer
            : scheme.secondaryContainer;
        final foreground = paused
            ? scheme.onErrorContainer
            : scheme.onSecondaryContainer;
        return Material(
          color: background,
          child: Padding(
            padding: const EdgeInsets.fromLTRB(16, 10, 16, 10),
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Icon(
                  paused ? Icons.tune : Icons.info_outline,
                  size: 20,
                  color: foreground,
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        latest.summary,
                        style: TextStyle(
                          color: foreground,
                          fontWeight: FontWeight.w600,
                        ),
                      ),
                      const SizedBox(height: 2),
                      Text(latest.action, style: TextStyle(color: foreground)),
                      const SizedBox(height: 4),
                      Text(
                        latest.levels,
                        style: Theme.of(context).textTheme.bodySmall,
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        );
      },
    );
  }
}
