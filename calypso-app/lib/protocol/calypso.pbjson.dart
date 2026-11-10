// This is a generated file - do not edit.
//
// Generated from calypso.proto.

// @dart = 3.3

// ignore_for_file: annotate_overrides, camel_case_types, comment_references
// ignore_for_file: constant_identifier_names
// ignore_for_file: curly_braces_in_flow_control_structures
// ignore_for_file: deprecated_member_use_from_same_package, library_prefixes
// ignore_for_file: non_constant_identifier_names, prefer_relative_imports
// ignore_for_file: unused_import

import 'dart:convert' as $convert;
import 'dart:core' as $core;
import 'dart:typed_data' as $typed_data;

@$core.Deprecated('Use traceKindDescriptor instead')
const TraceKind$json = {
  '1': 'TraceKind',
  '2': [
    {'1': 'TRACE_KIND_COMPUTED', '2': 0},
    {'1': 'TRACE_KIND_RAW', '2': 1},
  ],
};

/// Descriptor for `TraceKind`. Decode as a `google.protobuf.EnumDescriptorProto`.
final $typed_data.Uint8List traceKindDescriptor = $convert.base64Decode(
    'CglUcmFjZUtpbmQSFwoTVFJBQ0VfS0lORF9DT01QVVRFRBAAEhIKDlRSQUNFX0tJTkRfUkFXEA'
    'E=');

@$core.Deprecated('Use reportKindDescriptor instead')
const ReportKind$json = {
  '1': 'ReportKind',
  '2': [
    {'1': 'REPORT_KIND_UNSPECIFIED', '2': 0},
    {'1': 'REPORT_KIND_LOG_UNUSABLE', '2': 1},
    {'1': 'REPORT_KIND_REP_NOT_STORED', '2': 2},
    {'1': 'REPORT_KIND_REP_WINDOW_UNAVAILABLE', '2': 3},
    {'1': 'REPORT_KIND_IMU_READ_FAILED', '2': 4},
    {'1': 'REPORT_KIND_CLOCK_NOT_PERSISTED', '2': 6},
    {'1': 'REPORT_KIND_TRANSFER_ABANDONED', '2': 7},
    {'1': 'REPORT_KIND_REQUEST_REFUSED', '2': 8},
    {'1': 'REPORT_KIND_TRACE_UNREADABLE', '2': 9},
    {'1': 'REPORT_KIND_NOTIFY_FAILED', '2': 10},
    {'1': 'REPORT_KIND_RADIO_FAILED', '2': 11},
    {'1': 'REPORT_KIND_SAMPLER_HOLE', '2': 12},
  ],
  '4': [
    {'1': 5, '2': 5},
  ],
};

/// Descriptor for `ReportKind`. Decode as a `google.protobuf.EnumDescriptorProto`.
final $typed_data.Uint8List reportKindDescriptor = $convert.base64Decode(
    'CgpSZXBvcnRLaW5kEhsKF1JFUE9SVF9LSU5EX1VOU1BFQ0lGSUVEEAASHAoYUkVQT1JUX0tJTk'
    'RfTE9HX1VOVVNBQkxFEAESHgoaUkVQT1JUX0tJTkRfUkVQX05PVF9TVE9SRUQQAhImCiJSRVBP'
    'UlRfS0lORF9SRVBfV0lORE9XX1VOQVZBSUxBQkxFEAMSHwobUkVQT1JUX0tJTkRfSU1VX1JFQU'
    'RfRkFJTEVEEAQSIwofUkVQT1JUX0tJTkRfQ0xPQ0tfTk9UX1BFUlNJU1RFRBAGEiIKHlJFUE9S'
    'VF9LSU5EX1RSQU5TRkVSX0FCQU5ET05FRBAHEh8KG1JFUE9SVF9LSU5EX1JFUVVFU1RfUkVGVV'
    'NFRBAIEiAKHFJFUE9SVF9LSU5EX1RSQUNFX1VOUkVBREFCTEUQCRIdChlSRVBPUlRfS0lORF9O'
    'T1RJRllfRkFJTEVEEAoSHAoYUkVQT1JUX0tJTkRfUkFESU9fRkFJTEVEEAsSHAoYUkVQT1JUX0'
    'tJTkRfU0FNUExFUl9IT0xFEAwiBAgFEAU=');

@$core.Deprecated('Use traceComputedPointDescriptor instead')
const TraceComputedPoint$json = {
  '1': 'TraceComputedPoint',
  '2': [
    {'1': 't_offset_ms', '3': 1, '4': 1, '5': 13, '10': 'tOffsetMs'},
    {'1': 'px', '3': 2, '4': 1, '5': 2, '10': 'px'},
    {'1': 'py', '3': 3, '4': 1, '5': 2, '10': 'py'},
    {'1': 'pz', '3': 4, '4': 1, '5': 2, '10': 'pz'},
    {'1': 'qw', '3': 5, '4': 1, '5': 2, '10': 'qw'},
    {'1': 'qx', '3': 6, '4': 1, '5': 2, '10': 'qx'},
    {'1': 'qy', '3': 7, '4': 1, '5': 2, '10': 'qy'},
    {'1': 'qz', '3': 8, '4': 1, '5': 2, '10': 'qz'},
  ],
};

/// Descriptor for `TraceComputedPoint`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List traceComputedPointDescriptor = $convert.base64Decode(
    'ChJUcmFjZUNvbXB1dGVkUG9pbnQSHgoLdF9vZmZzZXRfbXMYASABKA1SCXRPZmZzZXRNcxIOCg'
    'JweBgCIAEoAlICcHgSDgoCcHkYAyABKAJSAnB5Eg4KAnB6GAQgASgCUgJwehIOCgJxdxgFIAEo'
    'AlICcXcSDgoCcXgYBiABKAJSAnF4Eg4KAnF5GAcgASgCUgJxeRIOCgJxehgIIAEoAlICcXo=');

@$core.Deprecated('Use traceRawPointDescriptor instead')
const TraceRawPoint$json = {
  '1': 'TraceRawPoint',
  '2': [
    {'1': 't_offset_ms', '3': 1, '4': 1, '5': 13, '10': 'tOffsetMs'},
    {'1': 'ax', '3': 2, '4': 1, '5': 17, '10': 'ax'},
    {'1': 'ay', '3': 3, '4': 1, '5': 17, '10': 'ay'},
    {'1': 'az', '3': 4, '4': 1, '5': 17, '10': 'az'},
    {'1': 'gx', '3': 5, '4': 1, '5': 17, '10': 'gx'},
    {'1': 'gy', '3': 6, '4': 1, '5': 17, '10': 'gy'},
    {'1': 'gz', '3': 7, '4': 1, '5': 17, '10': 'gz'},
  ],
};

/// Descriptor for `TraceRawPoint`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List traceRawPointDescriptor = $convert.base64Decode(
    'Cg1UcmFjZVJhd1BvaW50Eh4KC3Rfb2Zmc2V0X21zGAEgASgNUgl0T2Zmc2V0TXMSDgoCYXgYAi'
    'ABKBFSAmF4Eg4KAmF5GAMgASgRUgJheRIOCgJhehgEIAEoEVICYXoSDgoCZ3gYBSABKBFSAmd4'
    'Eg4KAmd5GAYgASgRUgJneRIOCgJnehgHIAEoEVICZ3o=');

@$core.Deprecated('Use repSummaryDescriptor instead')
const RepSummary$json = {
  '1': 'RepSummary',
  '2': [
    {'1': 'rep_uuid', '3': 1, '4': 1, '5': 12, '10': 'repUuid'},
    {'1': 't_start_ms', '3': 2, '4': 1, '5': 13, '10': 'tStartMs'},
    {'1': 't_end_ms', '3': 3, '4': 1, '5': 13, '10': 'tEndMs'},
    {'1': 'duration_ms', '3': 4, '4': 1, '5': 13, '10': 'durationMs'},
    {'1': 'rom', '3': 5, '4': 1, '5': 2, '10': 'rom'},
    {'1': 'peak_velocity', '3': 6, '4': 1, '5': 2, '10': 'peakVelocity'},
    {'1': 'roll_min_deg', '3': 9, '4': 1, '5': 2, '10': 'rollMinDeg'},
    {'1': 'roll_max_deg', '3': 10, '4': 1, '5': 2, '10': 'rollMaxDeg'},
    {'1': 'pitch_min_deg', '3': 11, '4': 1, '5': 2, '10': 'pitchMinDeg'},
    {'1': 'pitch_max_deg', '3': 12, '4': 1, '5': 2, '10': 'pitchMaxDeg'},
    {'1': 'computed_len', '3': 13, '4': 1, '5': 13, '10': 'computedLen'},
    {'1': 'computed_chunks', '3': 14, '4': 1, '5': 13, '10': 'computedChunks'},
    {'1': 't_unix_ms', '3': 16, '4': 1, '5': 4, '10': 'tUnixMs'},
    {
      '1': 'state',
      '3': 17,
      '4': 1,
      '5': 11,
      '6': '.calypso.RepState',
      '10': 'state'
    },
    {
      '1': 'config',
      '3': 18,
      '4': 1,
      '5': 11,
      '6': '.calypso.RepConfig',
      '10': 'config'
    },
    {'1': 'raw_len', '3': 19, '4': 1, '5': 13, '10': 'rawLen'},
    {'1': 'raw_chunks', '3': 20, '4': 1, '5': 13, '10': 'rawChunks'},
  ],
  '9': [
    {'1': 7, '2': 8},
    {'1': 8, '2': 9},
    {'1': 15, '2': 16},
  ],
  '10': ['mean_velocity', 'peak_accel', 'flags'],
};

/// Descriptor for `RepSummary`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List repSummaryDescriptor = $convert.base64Decode(
    'CgpSZXBTdW1tYXJ5EhkKCHJlcF91dWlkGAEgASgMUgdyZXBVdWlkEhwKCnRfc3RhcnRfbXMYAi'
    'ABKA1SCHRTdGFydE1zEhgKCHRfZW5kX21zGAMgASgNUgZ0RW5kTXMSHwoLZHVyYXRpb25fbXMY'
    'BCABKA1SCmR1cmF0aW9uTXMSEAoDcm9tGAUgASgCUgNyb20SIwoNcGVha192ZWxvY2l0eRgGIA'
    'EoAlIMcGVha1ZlbG9jaXR5EiAKDHJvbGxfbWluX2RlZxgJIAEoAlIKcm9sbE1pbkRlZxIgCgxy'
    'b2xsX21heF9kZWcYCiABKAJSCnJvbGxNYXhEZWcSIgoNcGl0Y2hfbWluX2RlZxgLIAEoAlILcG'
    'l0Y2hNaW5EZWcSIgoNcGl0Y2hfbWF4X2RlZxgMIAEoAlILcGl0Y2hNYXhEZWcSIQoMY29tcHV0'
    'ZWRfbGVuGA0gASgNUgtjb21wdXRlZExlbhInCg9jb21wdXRlZF9jaHVua3MYDiABKA1SDmNvbX'
    'B1dGVkQ2h1bmtzEhoKCXRfdW5peF9tcxgQIAEoBFIHdFVuaXhNcxInCgVzdGF0ZRgRIAEoCzIR'
    'LmNhbHlwc28uUmVwU3RhdGVSBXN0YXRlEioKBmNvbmZpZxgSIAEoCzISLmNhbHlwc28uUmVwQ2'
    '9uZmlnUgZjb25maWcSFwoHcmF3X2xlbhgTIAEoDVIGcmF3TGVuEh0KCnJhd19jaHVua3MYFCAB'
    'KA1SCXJhd0NodW5rc0oECAcQCEoECAgQCUoECA8QEFINbWVhbl92ZWxvY2l0eVIKcGVha19hY2'
    'NlbFIFZmxhZ3M=');

@$core.Deprecated('Use repConfigDescriptor instead')
const RepConfig$json = {
  '1': 'RepConfig',
  '2': [
    {'1': 'accel_range_g', '3': 1, '4': 1, '5': 13, '10': 'accelRangeG'},
    {'1': 'gyro_range_dps', '3': 2, '4': 1, '5': 13, '10': 'gyroRangeDps'},
    {
      '1': 'accel_bandwidth_hz',
      '3': 3,
      '4': 1,
      '5': 13,
      '10': 'accelBandwidthHz'
    },
    {
      '1': 'gyro_bandwidth_hz',
      '3': 4,
      '4': 1,
      '5': 13,
      '10': 'gyroBandwidthHz'
    },
    {'1': 'sample_rate_hz', '3': 5, '4': 1, '5': 13, '10': 'sampleRateHz'},
    {'1': 'detector_version', '3': 6, '4': 1, '5': 13, '10': 'detectorVersion'},
  ],
};

/// Descriptor for `RepConfig`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List repConfigDescriptor = $convert.base64Decode(
    'CglSZXBDb25maWcSIgoNYWNjZWxfcmFuZ2VfZxgBIAEoDVILYWNjZWxSYW5nZUcSJAoOZ3lyb1'
    '9yYW5nZV9kcHMYAiABKA1SDGd5cm9SYW5nZURwcxIsChJhY2NlbF9iYW5kd2lkdGhfaHoYAyAB'
    'KA1SEGFjY2VsQmFuZHdpZHRoSHoSKgoRZ3lyb19iYW5kd2lkdGhfaHoYBCABKA1SD2d5cm9CYW'
    '5kd2lkdGhIehIkCg5zYW1wbGVfcmF0ZV9oehgFIAEoDVIMc2FtcGxlUmF0ZUh6EikKEGRldGVj'
    'dG9yX3ZlcnNpb24YBiABKA1SD2RldGVjdG9yVmVyc2lvbg==');

@$core.Deprecated('Use repStateDescriptor instead')
const RepState$json = {
  '1': 'RepState',
  '2': [
    {'1': 'vx', '3': 1, '4': 1, '5': 2, '10': 'vx'},
    {'1': 'vy', '3': 2, '4': 1, '5': 2, '10': 'vy'},
    {'1': 'vz', '3': 3, '4': 1, '5': 2, '10': 'vz'},
    {'1': 'qw', '3': 4, '4': 1, '5': 2, '10': 'qw'},
    {'1': 'qx', '3': 5, '4': 1, '5': 2, '10': 'qx'},
    {'1': 'qy', '3': 6, '4': 1, '5': 2, '10': 'qy'},
    {'1': 'qz', '3': 7, '4': 1, '5': 2, '10': 'qz'},
    {'1': 'accel_bias_x', '3': 8, '4': 1, '5': 2, '10': 'accelBiasX'},
    {'1': 'accel_bias_y', '3': 9, '4': 1, '5': 2, '10': 'accelBiasY'},
    {'1': 'accel_bias_z', '3': 10, '4': 1, '5': 2, '10': 'accelBiasZ'},
    {'1': 'gyro_bias_x', '3': 11, '4': 1, '5': 2, '10': 'gyroBiasX'},
    {'1': 'gyro_bias_y', '3': 12, '4': 1, '5': 2, '10': 'gyroBiasY'},
    {'1': 'gyro_bias_z', '3': 13, '4': 1, '5': 2, '10': 'gyroBiasZ'},
    {'1': 'settled', '3': 14, '4': 1, '5': 8, '10': 'settled'},
  ],
};

/// Descriptor for `RepState`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List repStateDescriptor = $convert.base64Decode(
    'CghSZXBTdGF0ZRIOCgJ2eBgBIAEoAlICdngSDgoCdnkYAiABKAJSAnZ5Eg4KAnZ6GAMgASgCUg'
    'J2ehIOCgJxdxgEIAEoAlICcXcSDgoCcXgYBSABKAJSAnF4Eg4KAnF5GAYgASgCUgJxeRIOCgJx'
    'ehgHIAEoAlICcXoSIAoMYWNjZWxfYmlhc194GAggASgCUgphY2NlbEJpYXNYEiAKDGFjY2VsX2'
    'JpYXNfeRgJIAEoAlIKYWNjZWxCaWFzWRIgCgxhY2NlbF9iaWFzX3oYCiABKAJSCmFjY2VsQmlh'
    'c1oSHgoLZ3lyb19iaWFzX3gYCyABKAJSCWd5cm9CaWFzWBIeCgtneXJvX2JpYXNfeRgMIAEoAl'
    'IJZ3lyb0JpYXNZEh4KC2d5cm9fYmlhc196GA0gASgCUglneXJvQmlhc1oSGAoHc2V0dGxlZBgO'
    'IAEoCFIHc2V0dGxlZA==');

@$core.Deprecated('Use traceChunkDescriptor instead')
const TraceChunk$json = {
  '1': 'TraceChunk',
  '2': [
    {'1': 'rep_uuid', '3': 1, '4': 1, '5': 12, '10': 'repUuid'},
    {'1': 'index', '3': 2, '4': 1, '5': 13, '10': 'index'},
    {'1': 'first_t_offset_ms', '3': 3, '4': 1, '5': 13, '10': 'firstTOffsetMs'},
    {
      '1': 'samples',
      '3': 4,
      '4': 3,
      '5': 11,
      '6': '.calypso.TraceComputedPoint',
      '10': 'samples'
    },
    {'1': 'total_chunks', '3': 5, '4': 1, '5': 13, '10': 'totalChunks'},
  ],
};

/// Descriptor for `TraceChunk`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List traceChunkDescriptor = $convert.base64Decode(
    'CgpUcmFjZUNodW5rEhkKCHJlcF91dWlkGAEgASgMUgdyZXBVdWlkEhQKBWluZGV4GAIgASgNUg'
    'VpbmRleBIpChFmaXJzdF90X29mZnNldF9tcxgDIAEoDVIOZmlyc3RUT2Zmc2V0TXMSNQoHc2Ft'
    'cGxlcxgEIAMoCzIbLmNhbHlwc28uVHJhY2VDb21wdXRlZFBvaW50UgdzYW1wbGVzEiEKDHRvdG'
    'FsX2NodW5rcxgFIAEoDVILdG90YWxDaHVua3M=');

@$core.Deprecated('Use rawChunkDescriptor instead')
const RawChunk$json = {
  '1': 'RawChunk',
  '2': [
    {'1': 'rep_uuid', '3': 1, '4': 1, '5': 12, '10': 'repUuid'},
    {'1': 'index', '3': 2, '4': 1, '5': 13, '10': 'index'},
    {'1': 'first_t_offset_ms', '3': 3, '4': 1, '5': 13, '10': 'firstTOffsetMs'},
    {
      '1': 'samples',
      '3': 4,
      '4': 3,
      '5': 11,
      '6': '.calypso.TraceRawPoint',
      '10': 'samples'
    },
    {'1': 'total_chunks', '3': 5, '4': 1, '5': 13, '10': 'totalChunks'},
  ],
};

/// Descriptor for `RawChunk`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List rawChunkDescriptor = $convert.base64Decode(
    'CghSYXdDaHVuaxIZCghyZXBfdXVpZBgBIAEoDFIHcmVwVXVpZBIUCgVpbmRleBgCIAEoDVIFaW'
    '5kZXgSKQoRZmlyc3RfdF9vZmZzZXRfbXMYAyABKA1SDmZpcnN0VE9mZnNldE1zEjAKB3NhbXBs'
    'ZXMYBCADKAsyFi5jYWx5cHNvLlRyYWNlUmF3UG9pbnRSB3NhbXBsZXMSIQoMdG90YWxfY2h1bm'
    'tzGAUgASgNUgt0b3RhbENodW5rcw==');

@$core.Deprecated('Use imuChunkDescriptor instead')
const ImuChunk$json = {
  '1': 'ImuChunk',
  '2': [
    {'1': 'first_t_ms', '3': 1, '4': 1, '5': 13, '10': 'firstTMs'},
    {'1': 'sample_rate_hz', '3': 2, '4': 1, '5': 13, '10': 'sampleRateHz'},
    {'1': 'seq', '3': 3, '4': 1, '5': 13, '10': 'seq'},
    {
      '1': 'samples',
      '3': 4,
      '4': 3,
      '5': 11,
      '6': '.calypso.TraceRawPoint',
      '10': 'samples'
    },
  ],
};

/// Descriptor for `ImuChunk`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List imuChunkDescriptor = $convert.base64Decode(
    'CghJbXVDaHVuaxIcCgpmaXJzdF90X21zGAEgASgNUghmaXJzdFRNcxIkCg5zYW1wbGVfcmF0ZV'
    '9oehgCIAEoDVIMc2FtcGxlUmF0ZUh6EhAKA3NlcRgDIAEoDVIDc2VxEjAKB3NhbXBsZXMYBCAD'
    'KAsyFi5jYWx5cHNvLlRyYWNlUmF3UG9pbnRSB3NhbXBsZXM=');

@$core.Deprecated('Use bootClockDescriptor instead')
const BootClock$json = {
  '1': 'BootClock',
  '2': [
    {'1': 'epoch_s', '3': 1, '4': 1, '5': 13, '10': 'epochS'},
  ],
};

/// Descriptor for `BootClock`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List bootClockDescriptor =
    $convert.base64Decode('CglCb290Q2xvY2sSFwoHZXBvY2hfcxgBIAEoDVIGZXBvY2hT');

@$core.Deprecated('Use repRecordDescriptor instead')
const RepRecord$json = {
  '1': 'RepRecord',
  '2': [
    {
      '1': 'summary',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.calypso.RepSummary',
      '9': 0,
      '10': 'summary'
    },
    {
      '1': 'computed',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.calypso.TraceChunk',
      '9': 0,
      '10': 'computed'
    },
    {
      '1': 'clock',
      '3': 3,
      '4': 1,
      '5': 11,
      '6': '.calypso.BootClock',
      '9': 0,
      '10': 'clock'
    },
    {
      '1': 'raw',
      '3': 4,
      '4': 1,
      '5': 11,
      '6': '.calypso.RawChunk',
      '9': 0,
      '10': 'raw'
    },
    {
      '1': 'imu',
      '3': 5,
      '4': 1,
      '5': 11,
      '6': '.calypso.ImuChunk',
      '9': 0,
      '10': 'imu'
    },
  ],
  '8': [
    {'1': 'body'},
  ],
};

/// Descriptor for `RepRecord`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List repRecordDescriptor = $convert.base64Decode(
    'CglSZXBSZWNvcmQSLwoHc3VtbWFyeRgBIAEoCzITLmNhbHlwc28uUmVwU3VtbWFyeUgAUgdzdW'
    '1tYXJ5EjEKCGNvbXB1dGVkGAIgASgLMhMuY2FseXBzby5UcmFjZUNodW5rSABSCGNvbXB1dGVk'
    'EioKBWNsb2NrGAMgASgLMhIuY2FseXBzby5Cb290Q2xvY2tIAFIFY2xvY2sSJQoDcmF3GAQgAS'
    'gLMhEuY2FseXBzby5SYXdDaHVua0gAUgNyYXcSJQoDaW11GAUgASgLMhEuY2FseXBzby5JbXVD'
    'aHVua0gAUgNpbXVCBgoEYm9keQ==');

@$core.Deprecated('Use traceRequestDescriptor instead')
const TraceRequest$json = {
  '1': 'TraceRequest',
  '2': [
    {'1': 'rep_uuid', '3': 1, '4': 1, '5': 12, '10': 'repUuid'},
    {
      '1': 'kind',
      '3': 4,
      '4': 1,
      '5': 14,
      '6': '.calypso.TraceKind',
      '10': 'kind'
    },
  ],
  '9': [
    {'1': 2, '2': 3},
    {'1': 3, '2': 4},
  ],
  '10': ['all_chunks', 'chunk_index'],
};

/// Descriptor for `TraceRequest`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List traceRequestDescriptor = $convert.base64Decode(
    'CgxUcmFjZVJlcXVlc3QSGQoIcmVwX3V1aWQYASABKAxSB3JlcFV1aWQSJgoEa2luZBgEIAEoDj'
    'ISLmNhbHlwc28uVHJhY2VLaW5kUgRraW5kSgQIAhADSgQIAxAEUgphbGxfY2h1bmtzUgtjaHVu'
    'a19pbmRleA==');

@$core.Deprecated('Use devModeDescriptor instead')
const DevMode$json = {
  '1': 'DevMode',
  '2': [
    {'1': 'enabled', '3': 1, '4': 1, '5': 8, '10': 'enabled'},
  ],
};

/// Descriptor for `DevMode`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List devModeDescriptor =
    $convert.base64Decode('CgdEZXZNb2RlEhgKB2VuYWJsZWQYASABKAhSB2VuYWJsZWQ=');

@$core.Deprecated('Use syncRequestDescriptor instead')
const SyncRequest$json = {
  '1': 'SyncRequest',
  '2': [
    {
      '1': 'trace',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.calypso.TraceRequest',
      '9': 0,
      '10': 'trace'
    },
    {
      '1': 'dev_mode',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.calypso.DevMode',
      '9': 0,
      '10': 'devMode'
    },
  ],
  '8': [
    {'1': 'body'},
  ],
};

/// Descriptor for `SyncRequest`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List syncRequestDescriptor = $convert.base64Decode(
    'CgtTeW5jUmVxdWVzdBItCgV0cmFjZRgBIAEoCzIVLmNhbHlwc28uVHJhY2VSZXF1ZXN0SABSBX'
    'RyYWNlEi0KCGRldl9tb2RlGAIgASgLMhAuY2FseXBzby5EZXZNb2RlSABSB2Rldk1vZGVCBgoE'
    'Ym9keQ==');

@$core.Deprecated('Use syncMessageDescriptor instead')
const SyncMessage$json = {
  '1': 'SyncMessage',
  '2': [
    {
      '1': 'summary',
      '3': 1,
      '4': 1,
      '5': 11,
      '6': '.calypso.RepSummary',
      '9': 0,
      '10': 'summary'
    },
    {
      '1': 'computed',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.calypso.TraceChunk',
      '9': 0,
      '10': 'computed'
    },
    {
      '1': 'raw',
      '3': 4,
      '4': 1,
      '5': 11,
      '6': '.calypso.RawChunk',
      '9': 0,
      '10': 'raw'
    },
  ],
  '8': [
    {'1': 'body'},
  ],
  '9': [
    {'1': 3, '2': 4},
  ],
  '10': ['status'],
};

/// Descriptor for `SyncMessage`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List syncMessageDescriptor = $convert.base64Decode(
    'CgtTeW5jTWVzc2FnZRIvCgdzdW1tYXJ5GAEgASgLMhMuY2FseXBzby5SZXBTdW1tYXJ5SABSB3'
    'N1bW1hcnkSMQoIY29tcHV0ZWQYAiABKAsyEy5jYWx5cHNvLlRyYWNlQ2h1bmtIAFIIY29tcHV0'
    'ZWQSJQoDcmF3GAQgASgLMhEuY2FseXBzby5SYXdDaHVua0gAUgNyYXdCBgoEYm9keUoECAMQBF'
    'IGc3RhdHVz');

@$core.Deprecated('Use reportDescriptor instead')
const Report$json = {
  '1': 'Report',
  '2': [
    {
      '1': 'kind',
      '3': 1,
      '4': 1,
      '5': 14,
      '6': '.calypso.ReportKind',
      '10': 'kind'
    },
  ],
};

/// Descriptor for `Report`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List reportDescriptor = $convert.base64Decode(
    'CgZSZXBvcnQSJwoEa2luZBgBIAEoDjITLmNhbHlwc28uUmVwb3J0S2luZFIEa2luZA==');
