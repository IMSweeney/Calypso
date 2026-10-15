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
    {'1': 'rep_id', '3': 1, '4': 1, '5': 13, '10': 'repId'},
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
    'CgpSZXBTdW1tYXJ5EhUKBnJlcF9pZBgBIAEoDVIFcmVwSWQSHAoKdF9zdGFydF9tcxgCIAEoDV'
    'IIdFN0YXJ0TXMSGAoIdF9lbmRfbXMYAyABKA1SBnRFbmRNcxIfCgtkdXJhdGlvbl9tcxgEIAEo'
    'DVIKZHVyYXRpb25NcxIQCgNyb20YBSABKAJSA3JvbRIjCg1wZWFrX3ZlbG9jaXR5GAYgASgCUg'
    'xwZWFrVmVsb2NpdHkSIwoNbWVhbl92ZWxvY2l0eRgHIAEoAlIMbWVhblZlbG9jaXR5Eh0KCnBl'
    'YWtfYWNjZWwYCCABKAJSCXBlYWtBY2NlbBIgCgxyb2xsX21pbl9kZWcYCSABKAJSCnJvbGxNaW'
    '5EZWcSIAoMcm9sbF9tYXhfZGVnGAogASgCUgpyb2xsTWF4RGVnEiIKDXBpdGNoX21pbl9kZWcY'
    'CyABKAJSC3BpdGNoTWluRGVnEiIKDXBpdGNoX21heF9kZWcYDCABKAJSC3BpdGNoTWF4RGVnEh'
    'sKCXRyYWNlX2xlbhgNIAEoDVIIdHJhY2VMZW4SHwoLY2h1bmtfY291bnQYDiABKA1SCmNodW5r'
    'Q291bnQSHAoJZGVjaW1hdGVkGA8gASgIUglkZWNpbWF0ZWQ=');

@$core.Deprecated('Use traceChunkDescriptor instead')
const TraceChunk$json = {
  '1': 'TraceChunk',
  '2': [
    {'1': 'rep_id', '3': 1, '4': 1, '5': 13, '10': 'repId'},
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
  ],
};

/// Descriptor for `TraceChunk`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List traceChunkDescriptor = $convert.base64Decode(
    'CgpUcmFjZUNodW5rEhUKBnJlcF9pZBgBIAEoDVIFcmVwSWQSFAoFaW5kZXgYAiABKA1SBWluZG'
    'V4EikKEWZpcnN0X3Rfb2Zmc2V0X21zGAMgASgNUg5maXJzdFRPZmZzZXRNcxIuCgdzYW1wbGVz'
    'GAQgAygLMhQuY2FseXBzby5UcmFjZVNhbXBsZVIHc2FtcGxlcw==');

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
    {'1': 'rep_id', '3': 1, '4': 1, '5': 13, '10': 'repId'},
    {'1': 'all_chunks', '3': 2, '4': 1, '5': 8, '10': 'allChunks'},
    {'1': 'chunk_index', '3': 3, '4': 1, '5': 13, '10': 'chunkIndex'},
  ],
};

/// Descriptor for `TraceRequest`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List traceRequestDescriptor = $convert.base64Decode(
    'CgxUcmFjZVJlcXVlc3QSFQoGcmVwX2lkGAEgASgNUgVyZXBJZBIdCgphbGxfY2h1bmtzGAIgAS'
    'gIUglhbGxDaHVua3MSHwoLY2h1bmtfaW5kZXgYAyABKA1SCmNodW5rSW5kZXg=');

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
  ],
  '8': [
    {'1': 'body'},
  ],
};

/// Descriptor for `SyncMessage`. Decode as a `google.protobuf.DescriptorProto`.
final $typed_data.Uint8List syncMessageDescriptor = $convert.base64Decode(
    'CgtTeW5jTWVzc2FnZRIvCgdzdW1tYXJ5GAEgASgLMhMuY2FseXBzby5SZXBTdW1tYXJ5SABSB3'
    'N1bW1hcnkSKwoFdHJhY2UYAiABKAsyEy5jYWx5cHNvLlRyYWNlQ2h1bmtIAFIFdHJhY2USLQoG'
    'c3RhdHVzGAMgASgLMhMuY2FseXBzby5TeW5jU3RhdHVzSABSBnN0YXR1c0IGCgRib2R5');

@$core.Deprecated('Use syncStatusDescriptor instead')
const SyncStatus$json = {
  '1': 'SyncStatus',
  '2': [
    {'1': 'rep_count', '3': 1, '4': 1, '5': 13, '10': 'repCount'},
    {'1': 'oldest_rep_id', '3': 2, '4': 1, '5': 13, '10': 'oldestRepId'},
    {'1': 'newest_rep_id', '3': 3, '4': 1, '5': 13, '10': 'newestRepId'},
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
    'CgpTeW5jU3RhdHVzEhsKCXJlcF9jb3VudBgBIAEoDVIIcmVwQ291bnQSIgoNb2xkZXN0X3JlcF'
    '9pZBgCIAEoDVILb2xkZXN0UmVwSWQSIgoNbmV3ZXN0X3JlcF9pZBgDIAEoDVILbmV3ZXN0UmVw'
    'SWQSKwoRdHJhY2VfdW5hdmFpbGFibGUYBCABKAhSEHRyYWNlVW5hdmFpbGFibGU=');
