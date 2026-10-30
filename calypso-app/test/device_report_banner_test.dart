import 'package:calypso/protocol/calypso.pb.dart' as pb;
import 'package:calypso/state/ble_status.dart';
import 'package:calypso/state/device_reports.dart';
import 'package:calypso/widgets/device_report_banner.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  // The device repeats a fault that is still happening once a second, so a close that the
  // repeat undid would be a button that does nothing. What it closes is the kind.
  testWidgets('a closed fault stays closed until a different one arrives', (
    tester,
  ) async {
    final reports = DeviceReports()
      ..add(pb.ReportKind.REPORT_KIND_REP_NOT_STORED);
    final status = BleStatusNotifier()
      ..set(BleStatus.connected, deviceId: 'device-1');

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DeviceReportBanner(reports: reports, status: status),
        ),
      ),
    );
    expect(find.textContaining('could not save a rep'), findsOneWidget);

    await tester.tap(find.byTooltip('Dismiss'));
    await tester.pump();
    expect(find.textContaining('could not save a rep'), findsNothing);

    reports.add(pb.ReportKind.REPORT_KIND_REP_NOT_STORED);
    await tester.pump();
    expect(
      find.textContaining('could not save a rep'),
      findsNothing,
      reason: 'the same fault, still happening, is not news',
    );

    reports.add(pb.ReportKind.REPORT_KIND_IMU_READ_FAILED);
    await tester.pump();
    expect(
      find.textContaining('cannot read its motion sensor'),
      findsOneWidget,
      reason: 'a different kind is',
    );
  });
}
