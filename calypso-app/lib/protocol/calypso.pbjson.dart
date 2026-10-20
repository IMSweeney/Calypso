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

@$core.Deprecated('Use sampleDescriptor instead')
const Sample$json = {
  '1': 'Sample',
  '2': [
    {'1': 't_ms', '3': 1, '4': 1, '5': 13, '10': 'tMs'},
    {'1': 'ax', '3': 2, '4': 1, '5': 17, '10': 'ax'},
    {'1': 'ay', '3': 3, '4': 1, '5': 17, '10': 'ay'},
    {'1': 'az', '3': 4, '4': 1, '5': 17, '10': 'az'},
    {'1': 'qw', '3': 5, '4': 1, '5': 17, '10': 'qw'},
    {'1': 'qx', '3': 6, '4': 1, '5': 17, '10': 'qx'},
    {'1': 'qy', '3': 7, '4': 1, '5': 17, '10': 'qy'},
    {'1': 'qz', '3': 8, '4': 1, '5': 17, '10': 'qz'},
  ],
};

/// Descriptor for `Sample`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List sampleDescriptor = $convert.base64Decode(
    'CgZTYW1wbGUSEQoEdF9tcxgBIAEoDVIDdE1zEg4KAmF4GAIgASgRUgJheBIOCgJheRgDIAEoEV'
    'ICYXkSDgoCYXoYBCABKBFSAmF6Eg4KAnF3GAUgASgRUgJxdxIOCgJxeBgGIAEoEVICcXgSDgoC'
    'cXkYByABKBFSAnF5Eg4KAnF6GAggASgRUgJxeg==');

@$core.Deprecated('Use traceSampleDescriptor instead')
const TraceSample$json = {
  '1': 'TraceSample',
  '2': [
    {'1': 't_offset_ms', '3': 1, '4': 1, '5': 13, '10': 'tOffsetMs'},
    {'1': 'px', '3': 2, '4': 1, '5': 2, '10': 'px'},
    {'1': 'py', '3': 3, '4': 1, '5': 2, '10': 'py'},
    {'1': 'pz', '3': 4, '4': 1, '5': 2, '10': 'pz'},
    {'1': 'ax', '3': 5, '4': 1, '5': 17, '10': 'ax'},
    {'1': 'ay', '3': 6, '4': 1, '5': 17, '10': 'ay'},
    {'1': 'az', '3': 7, '4': 1, '5': 17, '10': 'az'},
    {'1': 'qw', '3': 8, '4': 1, '5': 17, '10': 'qw'},
    {'1': 'qx', '3': 9, '4': 1, '5': 17, '10': 'qx'},
    {'1': 'qy', '3': 10, '4': 1, '5': 17, '10': 'qy'},
    {'1': 'qz', '3': 11, '4': 1, '5': 17, '10': 'qz'},
  ],
};

/// Descriptor for `TraceSample`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List traceSampleDescriptor = $convert.base64Decode(
    'CgtUcmFjZVNhbXBsZRIeCgt0X29mZnNldF9tcxgBIAEoDVIJdE9mZnNldE1zEg4KAnB4GAIgAS'
    'gCUgJweBIOCgJweRgDIAEoAlICcHkSDgoCcHoYBCABKAJSAnB6Eg4KAmF4GAUgASgRUgJheBIO'
    'CgJheRgGIAEoEVICYXkSDgoCYXoYByABKBFSAmF6Eg4KAnF3GAggASgRUgJxdxIOCgJxeBgJIA'
    'EoEVICcXgSDgoCcXkYCiABKBFSAnF5Eg4KAnF6GAsgASgRUgJxeg==');

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
    {'1': 'mean_velocity', '3': 7, '4': 1, '5': 2, '10': 'meanVelocity'},
    {'1': 'peak_accel', '3': 8, '4': 1, '5': 2, '10': 'peakAccel'},
    {'1': 'roll_min_deg', '3': 9, '4': 1, '5': 2, '10': 'rollMinDeg'},
    {'1': 'roll_max_deg', '3': 10, '4': 1, '5': 2, '10': 'rollMaxDeg'},
    {'1': 'pitch_min_deg', '3': 11, '4': 1, '5': 2, '10': 'pitchMinDeg'},
    {'1': 'pitch_max_deg', '3': 12, '4': 1, '5': 2, '10': 'pitchMaxDeg'},
    {'1': 'trace_len', '3': 13, '4': 1, '5': 13, '10': 'traceLen'},
    {'1': 'chunk_count', '3': 14, '4': 1, '5': 13, '10': 'chunkCount'},
    {'1': 'decimated', '3': 15, '4': 1, '5': 8, '10': 'decimated'},
  ],
};

/// Descriptor for `RepSummary`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List repSummaryDescriptor = $convert.base64Decode(
    'CgpSZXBTdW1tYXJ5EhkKCHJlcF91dWlkGAEgASgMUgdyZXBVdWlkEhwKCnRfc3RhcnRfbXMYAi'
    'ABKA1SCHRTdGFydE1zEhgKCHRfZW5kX21zGAMgASgNUgZ0RW5kTXMSHwoLZHVyYXRpb25fbXMY'
    'BCABKA1SCmR1cmF0aW9uTXMSEAoDcm9tGAUgASgCUgNyb20SIwoNcGVha192ZWxvY2l0eRgGIA'
    'EoAlIMcGVha1ZlbG9jaXR5EiMKDW1lYW5fdmVsb2NpdHkYByABKAJSDG1lYW5WZWxvY2l0eRId'
    'CgpwZWFrX2FjY2VsGAggASgCUglwZWFrQWNjZWwSIAoMcm9sbF9taW5fZGVnGAkgASgCUgpyb2'
    'xsTWluRGVnEiAKDHJvbGxfbWF4X2RlZxgKIAEoAlIKcm9sbE1heERlZxIiCg1waXRjaF9taW5f'
    'ZGVnGAsgASgCUgtwaXRjaE1pbkRlZxIiCg1waXRjaF9tYXhfZGVnGAwgASgCUgtwaXRjaE1heE'
    'RlZxIbCgl0cmFjZV9sZW4YDSABKA1SCHRyYWNlTGVuEh8KC2NodW5rX2NvdW50GA4gASgNUgpj'
    'aHVua0NvdW50EhwKCWRlY2ltYXRlZBgPIAEoCFIJZGVjaW1hdGVk');

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
      '6': '.calypso.TraceSample',
      '10': 'samples'
    },
    {'1': 'total_chunks', '3': 5, '4': 1, '5': 13, '10': 'totalChunks'},
  ],
};

/// Descriptor for `TraceChunk`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List traceChunkDescriptor = $convert.base64Decode(
    'CgpUcmFjZUNodW5rEhkKCHJlcF91dWlkGAEgASgMUgdyZXBVdWlkEhQKBWluZGV4GAIgASgNUg'
    'VpbmRleBIpChFmaXJzdF90X29mZnNldF9tcxgDIAEoDVIOZmlyc3RUT2Zmc2V0TXMSLgoHc2Ft'
    'cGxlcxgEIAMoCzIULmNhbHlwc28uVHJhY2VTYW1wbGVSB3NhbXBsZXMSIQoMdG90YWxfY2h1bm'
    'tzGAUgASgNUgt0b3RhbENodW5rcw==');

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
      '1': 'trace',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.calypso.TraceChunk',
      '9': 0,
      '10': 'trace'
    },
  ],
  '8': [
    {'1': 'body'},
  ],
};

/// Descriptor for `RepRecord`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List repRecordDescriptor = $convert.base64Decode(
    'CglSZXBSZWNvcmQSLwoHc3VtbWFyeRgBIAEoCzITLmNhbHlwc28uUmVwU3VtbWFyeUgAUgdzdW'
    '1tYXJ5EisKBXRyYWNlGAIgASgLMhMuY2FseXBzby5UcmFjZUNodW5rSABSBXRyYWNlQgYKBGJv'
    'ZHk=');

@$core.Deprecated('Use traceRequestDescriptor instead')
const TraceRequest$json = {
  '1': 'TraceRequest',
  '2': [
    {'1': 'rep_uuid', '3': 1, '4': 1, '5': 12, '10': 'repUuid'},
    {'1': 'all_chunks', '3': 2, '4': 1, '5': 8, '10': 'allChunks'},
    {'1': 'chunk_index', '3': 3, '4': 1, '5': 13, '10': 'chunkIndex'},
  ],
};

/// Descriptor for `TraceRequest`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List traceRequestDescriptor = $convert.base64Decode(
    'CgxUcmFjZVJlcXVlc3QSGQoIcmVwX3V1aWQYASABKAxSB3JlcFV1aWQSHQoKYWxsX2NodW5rcx'
    'gCIAEoCFIJYWxsQ2h1bmtzEh8KC2NodW5rX2luZGV4GAMgASgNUgpjaHVua0luZGV4');

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
  ],
  '8': [
    {'1': 'body'},
  ],
};

/// Descriptor for `SyncRequest`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List syncRequestDescriptor = $convert.base64Decode(
    'CgtTeW5jUmVxdWVzdBItCgV0cmFjZRgBIAEoCzIVLmNhbHlwc28uVHJhY2VSZXF1ZXN0SABSBX'
    'RyYWNlQgYKBGJvZHk=');

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
      '1': 'trace',
      '3': 2,
      '4': 1,
      '5': 11,
      '6': '.calypso.TraceChunk',
      '9': 0,
      '10': 'trace'
    },
    {
      '1': 'status',
      '3': 3,
      '4': 1,
      '5': 11,
      '6': '.calypso.SyncStatus',
      '9': 0,
      '10': 'status'
    },
    {
      '1': 'calibration',
      '3': 4,
      '4': 1,
      '5': 11,
      '6': '.calypso.CalibrationStatus',
      '9': 0,
      '10': 'calibration'
    },
  ],
  '8': [
    {'1': 'body'},
  ],
};

/// Descriptor for `SyncMessage`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List syncMessageDescriptor = $convert.base64Decode(
    'CgtTeW5jTWVzc2FnZRIvCgdzdW1tYXJ5GAEgASgLMhMuY2FseXBzby5SZXBTdW1tYXJ5SABSB3'
    'N1bW1hcnkSKwoFdHJhY2UYAiABKAsyEy5jYWx5cHNvLlRyYWNlQ2h1bmtIAFIFdHJhY2USLQoG'
    'c3RhdHVzGAMgASgLMhMuY2FseXBzby5TeW5jU3RhdHVzSABSBnN0YXR1cxI+CgtjYWxpYnJhdG'
    'lvbhgEIAEoCzIaLmNhbHlwc28uQ2FsaWJyYXRpb25TdGF0dXNIAFILY2FsaWJyYXRpb25CBgoE'
    'Ym9keQ==');

@$core.Deprecated('Use calibrationStatusDescriptor instead')
const CalibrationStatus$json = {
  '1': 'CalibrationStatus',
  '2': [
    {'1': 'sys', '3': 1, '4': 1, '5': 13, '10': 'sys'},
    {'1': 'accel', '3': 2, '4': 1, '5': 13, '10': 'accel'},
    {'1': 'gyro', '3': 3, '4': 1, '5': 13, '10': 'gyro'},
    {'1': 'mag', '3': 4, '4': 1, '5': 13, '10': 'mag'},
    {'1': 'ready', '3': 5, '4': 1, '5': 8, '10': 'ready'},
  ],
};

/// Descriptor for `CalibrationStatus`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List calibrationStatusDescriptor = $convert.base64Decode(
    'ChFDYWxpYnJhdGlvblN0YXR1cxIQCgNzeXMYASABKA1SA3N5cxIUCgVhY2NlbBgCIAEoDVIFYW'
    'NjZWwSEgoEZ3lybxgDIAEoDVIEZ3lybxIQCgNtYWcYBCABKA1SA21hZxIUCgVyZWFkeRgFIAEo'
    'CFIFcmVhZHk=');

@$core.Deprecated('Use syncStatusDescriptor instead')
const SyncStatus$json = {
  '1': 'SyncStatus',
  '2': [
    {'1': 'rep_count', '3': 1, '4': 1, '5': 13, '10': 'repCount'},
    {'1': 'oldest_rep_uuid', '3': 2, '4': 1, '5': 12, '10': 'oldestRepUuid'},
    {'1': 'newest_rep_uuid', '3': 3, '4': 1, '5': 12, '10': 'newestRepUuid'},
    {
      '1': 'trace_unavailable',
      '3': 4,
      '4': 1,
      '5': 8,
      '10': 'traceUnavailable'
    },
  ],
};

/// Descriptor for `SyncStatus`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List syncStatusDescriptor = $convert.base64Decode(
    'CgpTeW5jU3RhdHVzEhsKCXJlcF9jb3VudBgBIAEoDVIIcmVwQ291bnQSJgoPb2xkZXN0X3JlcF'
    '91dWlkGAIgASgMUg1vbGRlc3RSZXBVdWlkEiYKD25ld2VzdF9yZXBfdXVpZBgDIAEoDFINbmV3'
    'ZXN0UmVwVXVpZBIrChF0cmFjZV91bmF2YWlsYWJsZRgEIAEoCFIQdHJhY2VVbmF2YWlsYWJsZQ'
    '==');
