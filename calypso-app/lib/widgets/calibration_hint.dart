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
        if (!status.isConnected || latest == null || latest.ready) {
          return const SizedBox.shrink();
        }
        final scheme = Theme.of(context).colorScheme;
        return Material(
          color: scheme.errorContainer,
          child: Padding(
            padding: const EdgeInsets.fromLTRB(16, 10, 16, 10),
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Icon(Icons.tune, size: 20, color: scheme.onErrorContainer),
                const SizedBox(width: 12),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        latest.summary,
                        style: TextStyle(
                          color: scheme.onErrorContainer,
                          fontWeight: FontWeight.w600,
                        ),
                      ),
                      const SizedBox(height: 2),
                      Text(latest.action),
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
