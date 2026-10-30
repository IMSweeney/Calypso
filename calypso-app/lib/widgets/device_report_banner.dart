library;

import 'package:flutter/material.dart';

import '../state/ble_status.dart';
import '../state/device_reports.dart';

/// The latest thing the device said went wrong.
///
/// The device sends a kind and the app owns the sentence ([`DeviceReports`]), so this is
/// one line and a count: a fault that has happened four times is a device that is still
/// failing, and one that has happened once may already be over.
///
/// Closing it lasts for the kind on screen and until the link comes up again
/// ([DeviceReports.dismiss]): the device repeats a fault that is still happening once a
/// second, so a banner a repeat could reopen is a banner that cannot be closed. Every
/// arrival is in the app's log whether or not it was on screen.
class DeviceReportBanner extends StatelessWidget {
  const DeviceReportBanner({
    super.key,
    required this.reports,
    required this.status,
  });

  final DeviceReports reports;
  final BleStatusNotifier status;

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: Listenable.merge([reports, status]),
      builder: (context, _) {
        final message = reports.message;
        if (!status.isConnected || message == null) {
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
                Icon(
                  Icons.report_problem_outlined,
                  size: 20,
                  color: scheme.onErrorContainer,
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        message,
                        style: TextStyle(
                          color: scheme.onErrorContainer,
                          fontWeight: FontWeight.w600,
                        ),
                      ),
                      if (reports.count > 1) ...[
                        const SizedBox(height: 2),
                        Text(
                          'Reported ${reports.count} times this session',
                          style: Theme.of(context).textTheme.bodySmall,
                        ),
                      ],
                    ],
                  ),
                ),
                IconButton(
                  onPressed: reports.dismiss,
                  icon: const Icon(Icons.close, size: 18),
                  color: scheme.onErrorContainer,
                  tooltip: 'Dismiss',
                  visualDensity: VisualDensity.compact,
                ),
              ],
            ),
          ),
        );
      },
    );
  }
}
