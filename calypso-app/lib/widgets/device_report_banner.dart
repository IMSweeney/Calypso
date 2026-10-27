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
/// There is nothing to dismiss. The device keeps no state to dismiss — it sends a fault
/// when it happens and never says "and it is fine now" — so a banner that could be closed
/// would come back on the next report anyway, and would be lying about a state nobody
/// knows.
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
              ],
            ),
          ),
        );
      },
    );
  }
}
