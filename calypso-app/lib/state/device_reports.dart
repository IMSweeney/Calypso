library;

import 'package:flutter/foundation.dart';

import '../protocol/calypso.pb.dart' as pb;

/// The device's own faults, as they arrive.
///
/// The device sends a **kind and nothing else** ([`pb.Report`], `proto/calypso.proto`):
/// the sentence is this end's, so [`describeReport`] is the one place a kind becomes words.
///
/// These are events, not state — the characteristic is notify-only, there is no "and it
/// is fine now", and nothing is read back. So the latest one is what the UI shows, and
/// [count] is how many have arrived since the link came up, which is what tells a single
/// glitch from a device that is still failing.
class DeviceReports extends ChangeNotifier {
  pb.ReportKind? _latest;
  int _count = 0;

  /// The most recent fault this connection, or `null` for a device that has not
  /// complained.
  pb.ReportKind? get latest => _latest;

  /// How many faults have arrived since the app connected.
  int get count => _count;

  /// The latest fault as a sentence, or `null` when there has not been one.
  String? get message {
    final latest = _latest;
    return latest == null ? null : describeReport(latest);
  }

  void add(pb.ReportKind kind) {
    _latest = kind;
    _count += 1;
    notifyListeners();
  }

  void clear() {
    if (_latest == null && _count == 0) return;
    _latest = null;
    _count = 0;
    notifyListeners();
  }
}

/// What the device's report means, in the app's words.
///
/// A kind names a **class** of failure, not an incident: the same sentence is shown
/// however many times it arrives, and the device's own detail (which rep, which flash
/// error) is in its serial log rather than here. Nothing matches on this text.
String describeReport(pb.ReportKind kind) => switch (kind) {
  pb.ReportKind.REPORT_KIND_LOG_UNUSABLE =>
    'The device cannot read its rep log, so nothing it records will sync.',
  pb.ReportKind.REPORT_KIND_REP_NOT_STORED =>
    'The device could not save a rep: one from this session is missing.',
  pb.ReportKind.REPORT_KIND_REP_WINDOW_UNAVAILABLE =>
    'The device lost a rep\'s samples before it could compute them.',
  pb.ReportKind.REPORT_KIND_IMU_READ_FAILED =>
    'The device cannot read its motion sensor: nothing is being recorded.',
  pb.ReportKind.REPORT_KIND_CALIBRATION_PROFILE =>
    'The device\'s saved calibration did not apply. Recalibrate and check it '
        'survives a restart.',
  pb.ReportKind.REPORT_KIND_CLOCK_NOT_PERSISTED =>
    'The device kept the time in memory only: after a restart its stored reps '
        'will have no date.',
  pb.ReportKind.REPORT_KIND_TRANSFER_ABANDONED =>
    'A message between the device and the app was dropped and given up on.',
  pb.ReportKind.REPORT_KIND_REQUEST_REFUSED =>
    'The device did not understand a request from this app: app and firmware '
        'may be different versions.',
  pb.ReportKind.REPORT_KIND_TRACE_UNREADABLE =>
    'The device could not read a trace back out of its log, so that rep has '
        'no path.',
  pb.ReportKind.REPORT_KIND_NOTIFY_FAILED =>
    'The device could not send a notification.',
  pb.ReportKind.REPORT_KIND_RADIO_FAILED =>
    'The device\'s radio reported a failure: it may be hard to reach.',
  _ => 'The device reported a problem (#${kind.value}).',
};
