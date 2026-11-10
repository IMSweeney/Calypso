// This is a generated file - do not edit.
//
// Generated from calypso.proto.

// @dart = 3.3

// ignore_for_file: annotate_overrides, camel_case_types, comment_references
// ignore_for_file: constant_identifier_names
// ignore_for_file: curly_braces_in_flow_control_structures
// ignore_for_file: deprecated_member_use_from_same_package, library_prefixes
// ignore_for_file: non_constant_identifier_names, prefer_relative_imports

import 'dart:core' as $core;

import 'package:protobuf/protobuf.dart' as $pb;

/// Which of a rep's two traces a request is about.
class TraceKind extends $pb.ProtobufEnum {
  static const TraceKind TRACE_KIND_COMPUTED =
      TraceKind._(0, _omitEnumNames ? '' : 'TRACE_KIND_COMPUTED');
  static const TraceKind TRACE_KIND_RAW =
      TraceKind._(1, _omitEnumNames ? '' : 'TRACE_KIND_RAW');

  static const $core.List<TraceKind> values = <TraceKind>[
    TRACE_KIND_COMPUTED,
    TRACE_KIND_RAW,
  ];

  static final $core.List<TraceKind?> _byValue =
      $pb.ProtobufEnum.$_initByValueList(values, 1);
  static TraceKind? valueOf($core.int value) =>
      value < 0 || value >= _byValue.length ? null : _byValue[value];

  const TraceKind._(super.value, super.name);
}

/// The classes of failure that reach the phone. Each is something a person holding the phone can act on, or can at least be told about in a sentence;
/// the firmware's own log has the rest.
class ReportKind extends $pb.ProtobufEnum {
  static const ReportKind REPORT_KIND_UNSPECIFIED =
      ReportKind._(0, _omitEnumNames ? '' : 'REPORT_KIND_UNSPECIFIED');

  /// The rep log did not mount, or could not be read: nothing this boot is stored, and nothing will ever be offered to sync. A bad partition table or a flash that will not answer leaves this device a sensor with no record.
  static const ReportKind REPORT_KIND_LOG_UNUSABLE =
      ReportKind._(1, _omitEnumNames ? '' : 'REPORT_KIND_LOG_UNUSABLE');

  /// A rep closed and could not be written: the trace is lost, and no summary will arrive for it either.
  static const ReportKind REPORT_KIND_REP_NOT_STORED =
      ReportKind._(2, _omitEnumNames ? '' : 'REPORT_KIND_REP_NOT_STORED');

  /// A rep was detected but its samples were no longer in the ring, so it was dropped before it could be computed. A rep the app will never see.
  static const ReportKind REPORT_KIND_REP_WINDOW_UNAVAILABLE = ReportKind._(
      3, _omitEnumNames ? '' : 'REPORT_KIND_REP_WINDOW_UNAVAILABLE');

  /// The sampler could not read the IMU: nothing is recorded while this lasts.
  /// Reported at most once a second however many reads fail.
  static const ReportKind REPORT_KIND_IMU_READ_FAILED =
      ReportKind._(4, _omitEnumNames ? '' : 'REPORT_KIND_IMU_READ_FAILED');

  /// The client set the clock and the device kept it in RAM only: this boot's reps are dated, a later boot's log has no record of the epoch.
  static const ReportKind REPORT_KIND_CLOCK_NOT_PERSISTED =
      ReportKind._(6, _omitEnumNames ? '' : 'REPORT_KIND_CLOCK_NOT_PERSISTED');

  /// The pipe gave up on a message the app is waiting for (three attempts, no acknowledgement). The app's own transfer timeout says the same thing a few seconds later.
  static const ReportKind REPORT_KIND_TRANSFER_ABANDONED =
      ReportKind._(7, _omitEnumNames ? '' : 'REPORT_KIND_TRANSFER_ABANDONED');

  /// The app asked for something this device will not serve: a body it does not know, a request it cannot decode, or a trace asked for with nothing subscribed. Always an app firmware-mismatch rather than a device fault.
  static const ReportKind REPORT_KIND_REQUEST_REFUSED =
      ReportKind._(8, _omitEnumNames ? '' : 'REPORT_KIND_REQUEST_REFUSED');

  /// A requested trace could not be read back out of the log. The app is waiting on it and will time out.
  static const ReportKind REPORT_KIND_TRACE_UNREADABLE =
      ReportKind._(9, _omitEnumNames ? '' : 'REPORT_KIND_TRACE_UNREADABLE');

  /// A notification the device tried to send failed on the link.
  static const ReportKind REPORT_KIND_NOTIFY_FAILED =
      ReportKind._(10, _omitEnumNames ? '' : 'REPORT_KIND_NOTIFY_FAILED');

  /// The radio's own machinery failed: the HCI runner errored, or advertising could not be started. The device may be invisible or unreachable.
  static const ReportKind REPORT_KIND_RADIO_FAILED =
      ReportKind._(11, _omitEnumNames ? '' : 'REPORT_KIND_RADIO_FAILED');

  /// The sampler stopped long enough to leave a gap in its timeline. A rep that spans the gap was measured across motion nobody saw, so its travel can read long, and a rep that happened inside the gap was never seen at all.
  static const ReportKind REPORT_KIND_SAMPLER_HOLE =
      ReportKind._(12, _omitEnumNames ? '' : 'REPORT_KIND_SAMPLER_HOLE');

  static const $core.List<ReportKind> values = <ReportKind>[
    REPORT_KIND_UNSPECIFIED,
    REPORT_KIND_LOG_UNUSABLE,
    REPORT_KIND_REP_NOT_STORED,
    REPORT_KIND_REP_WINDOW_UNAVAILABLE,
    REPORT_KIND_IMU_READ_FAILED,
    REPORT_KIND_CLOCK_NOT_PERSISTED,
    REPORT_KIND_TRANSFER_ABANDONED,
    REPORT_KIND_REQUEST_REFUSED,
    REPORT_KIND_TRACE_UNREADABLE,
    REPORT_KIND_NOTIFY_FAILED,
    REPORT_KIND_RADIO_FAILED,
    REPORT_KIND_SAMPLER_HOLE,
  ];

  static final $core.List<ReportKind?> _byValue =
      $pb.ProtobufEnum.$_initByValueList(values, 12);
  static ReportKind? valueOf($core.int value) =>
      value < 0 || value >= _byValue.length ? null : _byValue[value];

  const ReportKind._(super.value, super.name);
}

const $core.bool _omitEnumNames =
    $core.bool.fromEnvironment('protobuf.omit_enum_names');
