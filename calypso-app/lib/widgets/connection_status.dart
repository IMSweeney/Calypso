library;

import 'package:flutter/material.dart';

import '../state/ble_status.dart';

class ConnectionStatusBadge extends StatelessWidget {
  const ConnectionStatusBadge({
    super.key,
    required this.status,
    this.onConnect,
    this.onDisconnect,
  });

  final BleStatusNotifier status;
  final VoidCallback? onConnect;
  final VoidCallback? onDisconnect;

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: status,
      builder: (context, _) {
        final (icon, colour) = switch (status.status) {
          BleStatus.connected => (Icons.bluetooth_connected, Colors.lightGreenAccent),
          BleStatus.scanning => (Icons.bluetooth_searching, Colors.amberAccent),
          BleStatus.connecting => (Icons.bluetooth_searching, Colors.amberAccent),
          BleStatus.unsupported => (Icons.bluetooth_disabled, Colors.orangeAccent),
          BleStatus.failed => (Icons.bluetooth_disabled, Colors.redAccent),
          _ => (Icons.bluetooth, Colors.white70),
        };
        return Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(icon, size: 18, color: colour),
            const SizedBox(width: 6),
            Text(status.label),
            if (status.error != null)
              Padding(
                padding: const EdgeInsets.only(left: 6),
                child: Tooltip(
                  message: status.error!,
                  child: const Icon(Icons.info_outline, size: 16),
                ),
              ),
            const SizedBox(width: 4),
            if (status.isConnected)
              TextButton(onPressed: onDisconnect, child: const Text('Disconnect'))
            else
              TextButton(
                onPressed: status.canConnect ? onConnect : null,
                child: const Text('Connect'),
              ),
          ],
        );
      },
    );
  }
}
