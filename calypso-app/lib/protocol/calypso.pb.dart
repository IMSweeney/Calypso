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

export 'package:protobuf/protobuf.dart' show GeneratedMessageGenericExtensions;

/// One IMU sample as sampled at 100 Hz by the firmware.
class Sample extends $pb.GeneratedMessage {
  factory Sample({
    $core.int? tMs,
    $core.int? ax,
    $core.int? ay,
    $core.int? az,
    $core.int? qw,
    $core.int? qx,
    $core.int? qy,
    $core.int? qz,
  }) {
    final result = Sample._();
    if (tMs != null) result.tMs = tMs;
    if (ax != null) result.ax = ax;
    if (ay != null) result.ay = ay;
    if (az != null) result.az = az;
    if (qw != null) result.qw = qw;
    if (qx != null) result.qx = qx;
    if (qy != null) result.qy = qy;
    if (qz != null) result.qz = qz;
    return result;
  }

  Sample._();

  factory Sample.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      Sample()..mergeFromBuffer(data, registry);
  factory Sample.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      Sample()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Sample',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: Sample.$_createMessage)
    ..aI(1, _omitFieldNames ? '' : 'tMs', fieldType: $pb.PbFieldType.OU3)
    ..aI(2, _omitFieldNames ? '' : 'ax', fieldType: $pb.PbFieldType.OS3)
    ..aI(3, _omitFieldNames ? '' : 'ay', fieldType: $pb.PbFieldType.OS3)
    ..aI(4, _omitFieldNames ? '' : 'az', fieldType: $pb.PbFieldType.OS3)
    ..aI(5, _omitFieldNames ? '' : 'qw', fieldType: $pb.PbFieldType.OS3)
    ..aI(6, _omitFieldNames ? '' : 'qx', fieldType: $pb.PbFieldType.OS3)
    ..aI(7, _omitFieldNames ? '' : 'qy', fieldType: $pb.PbFieldType.OS3)
    ..aI(8, _omitFieldNames ? '' : 'qz', fieldType: $pb.PbFieldType.OS3)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Sample clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Sample copyWith(void Function(Sample) updates) =>
      super.copyWith((message) => updates(message as Sample)) as Sample;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use Sample() / Sample.new instead')
  static Sample create() => Sample._();
  static $pb.GeneratedMessage $_createMessage() => Sample._();
  @$core.override
  Sample createEmptyInstance() => Sample._();
  @$core.pragma('dart2js:noInline')
  static Sample getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<Sample>(Sample.$_createMessage);
  static Sample? _defaultInstance;

  @$pb.TagNumber(1)
  $core.int get tMs => $_getIZ(0);
  @$pb.TagNumber(1)
  set tMs($core.int value) => $_setUnsignedInt32(0, value);
  @$pb.TagNumber(1)
  $core.bool hasTMs() => $_has(0);
  @$pb.TagNumber(1)
  void clearTMs() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.int get ax => $_getIZ(1);
  @$pb.TagNumber(2)
  set ax($core.int value) => $_setSignedInt32(1, value);
  @$pb.TagNumber(2)
  $core.bool hasAx() => $_has(1);
  @$pb.TagNumber(2)
  void clearAx() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.int get ay => $_getIZ(2);
  @$pb.TagNumber(3)
  set ay($core.int value) => $_setSignedInt32(2, value);
  @$pb.TagNumber(3)
  $core.bool hasAy() => $_has(2);
  @$pb.TagNumber(3)
  void clearAy() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.int get az => $_getIZ(3);
  @$pb.TagNumber(4)
  set az($core.int value) => $_setSignedInt32(3, value);
  @$pb.TagNumber(4)
  $core.bool hasAz() => $_has(3);
  @$pb.TagNumber(4)
  void clearAz() => $_clearField(4);

  @$pb.TagNumber(5)
  $core.int get qw => $_getIZ(4);
  @$pb.TagNumber(5)
  set qw($core.int value) => $_setSignedInt32(4, value);
  @$pb.TagNumber(5)
  $core.bool hasQw() => $_has(4);
  @$pb.TagNumber(5)
  void clearQw() => $_clearField(5);

  @$pb.TagNumber(6)
  $core.int get qx => $_getIZ(5);
  @$pb.TagNumber(6)
  set qx($core.int value) => $_setSignedInt32(5, value);
  @$pb.TagNumber(6)
  $core.bool hasQx() => $_has(5);
  @$pb.TagNumber(6)
  void clearQx() => $_clearField(6);

  @$pb.TagNumber(7)
  $core.int get qy => $_getIZ(6);
  @$pb.TagNumber(7)
  set qy($core.int value) => $_setSignedInt32(6, value);
  @$pb.TagNumber(7)
  $core.bool hasQy() => $_has(6);
  @$pb.TagNumber(7)
  void clearQy() => $_clearField(7);

  @$pb.TagNumber(8)
  $core.int get qz => $_getIZ(7);
  @$pb.TagNumber(8)
  set qz($core.int value) => $_setSignedInt32(7, value);
  @$pb.TagNumber(8)
  $core.bool hasQz() => $_has(7);
  @$pb.TagNumber(8)
  void clearQz() => $_clearField(8);
}

/// One point of a stored rep trace.
///
/// Both the derived displacement and the raw inputs are kept, so the app can
/// re-derive traces with better algorithms later without a firmware update.
class TraceSample extends $pb.GeneratedMessage {
  factory TraceSample({
    $core.int? tOffsetMs,
    $core.double? px,
    $core.double? py,
    $core.double? pz,
    $core.int? ax,
    $core.int? ay,
    $core.int? az,
    $core.int? qw,
    $core.int? qx,
    $core.int? qy,
    $core.int? qz,
  }) {
    final result = TraceSample._();
    if (tOffsetMs != null) result.tOffsetMs = tOffsetMs;
    if (px != null) result.px = px;
    if (py != null) result.py = py;
    if (pz != null) result.pz = pz;
    if (ax != null) result.ax = ax;
    if (ay != null) result.ay = ay;
    if (az != null) result.az = az;
    if (qw != null) result.qw = qw;
    if (qx != null) result.qx = qx;
    if (qy != null) result.qy = qy;
    if (qz != null) result.qz = qz;
    return result;
  }

  TraceSample._();

  factory TraceSample.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      TraceSample()..mergeFromBuffer(data, registry);
  factory TraceSample.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      TraceSample()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'TraceSample',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: TraceSample.$_createMessage)
    ..aI(1, _omitFieldNames ? '' : 'tOffsetMs', fieldType: $pb.PbFieldType.OU3)
    ..aD(2, _omitFieldNames ? '' : 'px', fieldType: $pb.PbFieldType.OF)
    ..aD(3, _omitFieldNames ? '' : 'py', fieldType: $pb.PbFieldType.OF)
    ..aD(4, _omitFieldNames ? '' : 'pz', fieldType: $pb.PbFieldType.OF)
    ..aI(5, _omitFieldNames ? '' : 'ax', fieldType: $pb.PbFieldType.OS3)
    ..aI(6, _omitFieldNames ? '' : 'ay', fieldType: $pb.PbFieldType.OS3)
    ..aI(7, _omitFieldNames ? '' : 'az', fieldType: $pb.PbFieldType.OS3)
    ..aI(8, _omitFieldNames ? '' : 'qw', fieldType: $pb.PbFieldType.OS3)
    ..aI(9, _omitFieldNames ? '' : 'qx', fieldType: $pb.PbFieldType.OS3)
    ..aI(10, _omitFieldNames ? '' : 'qy', fieldType: $pb.PbFieldType.OS3)
    ..aI(11, _omitFieldNames ? '' : 'qz', fieldType: $pb.PbFieldType.OS3)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TraceSample clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TraceSample copyWith(void Function(TraceSample) updates) =>
      super.copyWith((message) => updates(message as TraceSample))
          as TraceSample;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use TraceSample() / TraceSample.new instead')
  static TraceSample create() => TraceSample._();
  static $pb.GeneratedMessage $_createMessage() => TraceSample._();
  @$core.override
  TraceSample createEmptyInstance() => TraceSample._();
  @$core.pragma('dart2js:noInline')
  static TraceSample getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<TraceSample>(
          TraceSample.$_createMessage);
  static TraceSample? _defaultInstance;

  @$pb.TagNumber(1)
  $core.int get tOffsetMs => $_getIZ(0);
  @$pb.TagNumber(1)
  set tOffsetMs($core.int value) => $_setUnsignedInt32(0, value);
  @$pb.TagNumber(1)
  $core.bool hasTOffsetMs() => $_has(0);
  @$pb.TagNumber(1)
  void clearTOffsetMs() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.double get px => $_getN(1);
  @$pb.TagNumber(2)
  set px($core.double value) => $_setFloat(1, value);
  @$pb.TagNumber(2)
  $core.bool hasPx() => $_has(1);
  @$pb.TagNumber(2)
  void clearPx() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.double get py => $_getN(2);
  @$pb.TagNumber(3)
  set py($core.double value) => $_setFloat(2, value);
  @$pb.TagNumber(3)
  $core.bool hasPy() => $_has(2);
  @$pb.TagNumber(3)
  void clearPy() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.double get pz => $_getN(3);
  @$pb.TagNumber(4)
  set pz($core.double value) => $_setFloat(3, value);
  @$pb.TagNumber(4)
  $core.bool hasPz() => $_has(3);
  @$pb.TagNumber(4)
  void clearPz() => $_clearField(4);

  @$pb.TagNumber(5)
  $core.int get ax => $_getIZ(4);
  @$pb.TagNumber(5)
  set ax($core.int value) => $_setSignedInt32(4, value);
  @$pb.TagNumber(5)
  $core.bool hasAx() => $_has(4);
  @$pb.TagNumber(5)
  void clearAx() => $_clearField(5);

  @$pb.TagNumber(6)
  $core.int get ay => $_getIZ(5);
  @$pb.TagNumber(6)
  set ay($core.int value) => $_setSignedInt32(5, value);
  @$pb.TagNumber(6)
  $core.bool hasAy() => $_has(5);
  @$pb.TagNumber(6)
  void clearAy() => $_clearField(6);

  @$pb.TagNumber(7)
  $core.int get az => $_getIZ(6);
  @$pb.TagNumber(7)
  set az($core.int value) => $_setSignedInt32(6, value);
  @$pb.TagNumber(7)
  $core.bool hasAz() => $_has(6);
  @$pb.TagNumber(7)
  void clearAz() => $_clearField(7);

  @$pb.TagNumber(8)
  $core.int get qw => $_getIZ(7);
  @$pb.TagNumber(8)
  set qw($core.int value) => $_setSignedInt32(7, value);
  @$pb.TagNumber(8)
  $core.bool hasQw() => $_has(7);
  @$pb.TagNumber(8)
  void clearQw() => $_clearField(8);

  @$pb.TagNumber(9)
  $core.int get qx => $_getIZ(8);
  @$pb.TagNumber(9)
  set qx($core.int value) => $_setSignedInt32(8, value);
  @$pb.TagNumber(9)
  $core.bool hasQx() => $_has(8);
  @$pb.TagNumber(9)
  void clearQx() => $_clearField(9);

  @$pb.TagNumber(10)
  $core.int get qy => $_getIZ(9);
  @$pb.TagNumber(10)
  set qy($core.int value) => $_setSignedInt32(9, value);
  @$pb.TagNumber(10)
  $core.bool hasQy() => $_has(9);
  @$pb.TagNumber(10)
  void clearQy() => $_clearField(10);

  @$pb.TagNumber(11)
  $core.int get qz => $_getIZ(10);
  @$pb.TagNumber(11)
  set qz($core.int value) => $_setSignedInt32(10, value);
  @$pb.TagNumber(11)
  $core.bool hasQz() => $_has(10);
  @$pb.TagNumber(11)
  void clearQz() => $_clearField(11);
}

/// Summary statistics for one rep. Angles are degrees, distances metres.
class RepSummary extends $pb.GeneratedMessage {
  factory RepSummary({
    $core.List<$core.int>? repUuid,
    $core.int? tStartMs,
    $core.int? tEndMs,
    $core.int? durationMs,
    $core.double? rom,
    $core.double? peakVelocity,
    $core.double? meanVelocity,
    $core.double? peakAccel,
    $core.double? rollMinDeg,
    $core.double? rollMaxDeg,
    $core.double? pitchMinDeg,
    $core.double? pitchMaxDeg,
    $core.int? traceLen,
    $core.int? chunkCount,
    $core.bool? decimated,
  }) {
    final result = RepSummary._();
    if (repUuid != null) result.repUuid = repUuid;
    if (tStartMs != null) result.tStartMs = tStartMs;
    if (tEndMs != null) result.tEndMs = tEndMs;
    if (durationMs != null) result.durationMs = durationMs;
    if (rom != null) result.rom = rom;
    if (peakVelocity != null) result.peakVelocity = peakVelocity;
    if (meanVelocity != null) result.meanVelocity = meanVelocity;
    if (peakAccel != null) result.peakAccel = peakAccel;
    if (rollMinDeg != null) result.rollMinDeg = rollMinDeg;
    if (rollMaxDeg != null) result.rollMaxDeg = rollMaxDeg;
    if (pitchMinDeg != null) result.pitchMinDeg = pitchMinDeg;
    if (pitchMaxDeg != null) result.pitchMaxDeg = pitchMaxDeg;
    if (traceLen != null) result.traceLen = traceLen;
    if (chunkCount != null) result.chunkCount = chunkCount;
    if (decimated != null) result.decimated = decimated;
    return result;
  }

  RepSummary._();

  factory RepSummary.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      RepSummary()..mergeFromBuffer(data, registry);
  factory RepSummary.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      RepSummary()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'RepSummary',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: RepSummary.$_createMessage)
    ..a<$core.List<$core.int>>(
        1, _omitFieldNames ? '' : 'repUuid', $pb.PbFieldType.OY)
    ..aI(2, _omitFieldNames ? '' : 'tStartMs', fieldType: $pb.PbFieldType.OU3)
    ..aI(3, _omitFieldNames ? '' : 'tEndMs', fieldType: $pb.PbFieldType.OU3)
    ..aI(4, _omitFieldNames ? '' : 'durationMs', fieldType: $pb.PbFieldType.OU3)
    ..aD(5, _omitFieldNames ? '' : 'rom', fieldType: $pb.PbFieldType.OF)
    ..aD(6, _omitFieldNames ? '' : 'peakVelocity',
        fieldType: $pb.PbFieldType.OF)
    ..aD(7, _omitFieldNames ? '' : 'meanVelocity',
        fieldType: $pb.PbFieldType.OF)
    ..aD(8, _omitFieldNames ? '' : 'peakAccel', fieldType: $pb.PbFieldType.OF)
    ..aD(9, _omitFieldNames ? '' : 'rollMinDeg', fieldType: $pb.PbFieldType.OF)
    ..aD(10, _omitFieldNames ? '' : 'rollMaxDeg', fieldType: $pb.PbFieldType.OF)
    ..aD(11, _omitFieldNames ? '' : 'pitchMinDeg',
        fieldType: $pb.PbFieldType.OF)
    ..aD(12, _omitFieldNames ? '' : 'pitchMaxDeg',
        fieldType: $pb.PbFieldType.OF)
    ..aI(13, _omitFieldNames ? '' : 'traceLen', fieldType: $pb.PbFieldType.OU3)
    ..aI(14, _omitFieldNames ? '' : 'chunkCount',
        fieldType: $pb.PbFieldType.OU3)
    ..aOB(15, _omitFieldNames ? '' : 'decimated')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  RepSummary clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  RepSummary copyWith(void Function(RepSummary) updates) =>
      super.copyWith((message) => updates(message as RepSummary)) as RepSummary;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use RepSummary() / RepSummary.new instead')
  static RepSummary create() => RepSummary._();
  static $pb.GeneratedMessage $_createMessage() => RepSummary._();
  @$core.override
  RepSummary createEmptyInstance() => RepSummary._();
  @$core.pragma('dart2js:noInline')
  static RepSummary getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<RepSummary>(RepSummary.$_createMessage);
  static RepSummary? _defaultInstance;

  @$pb.TagNumber(1)
  $core.List<$core.int> get repUuid => $_getN(0);
  @$pb.TagNumber(1)
  set repUuid($core.List<$core.int> value) => $_setBytes(0, value);
  @$pb.TagNumber(1)
  $core.bool hasRepUuid() => $_has(0);
  @$pb.TagNumber(1)
  void clearRepUuid() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.int get tStartMs => $_getIZ(1);
  @$pb.TagNumber(2)
  set tStartMs($core.int value) => $_setUnsignedInt32(1, value);
  @$pb.TagNumber(2)
  $core.bool hasTStartMs() => $_has(1);
  @$pb.TagNumber(2)
  void clearTStartMs() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.int get tEndMs => $_getIZ(2);
  @$pb.TagNumber(3)
  set tEndMs($core.int value) => $_setUnsignedInt32(2, value);
  @$pb.TagNumber(3)
  $core.bool hasTEndMs() => $_has(2);
  @$pb.TagNumber(3)
  void clearTEndMs() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.int get durationMs => $_getIZ(3);
  @$pb.TagNumber(4)
  set durationMs($core.int value) => $_setUnsignedInt32(3, value);
  @$pb.TagNumber(4)
  $core.bool hasDurationMs() => $_has(3);
  @$pb.TagNumber(4)
  void clearDurationMs() => $_clearField(4);

  @$pb.TagNumber(5)
  $core.double get rom => $_getN(4);
  @$pb.TagNumber(5)
  set rom($core.double value) => $_setFloat(4, value);
  @$pb.TagNumber(5)
  $core.bool hasRom() => $_has(4);
  @$pb.TagNumber(5)
  void clearRom() => $_clearField(5);

  @$pb.TagNumber(6)
  $core.double get peakVelocity => $_getN(5);
  @$pb.TagNumber(6)
  set peakVelocity($core.double value) => $_setFloat(5, value);
  @$pb.TagNumber(6)
  $core.bool hasPeakVelocity() => $_has(5);
  @$pb.TagNumber(6)
  void clearPeakVelocity() => $_clearField(6);

  @$pb.TagNumber(7)
  $core.double get meanVelocity => $_getN(6);
  @$pb.TagNumber(7)
  set meanVelocity($core.double value) => $_setFloat(6, value);
  @$pb.TagNumber(7)
  $core.bool hasMeanVelocity() => $_has(6);
  @$pb.TagNumber(7)
  void clearMeanVelocity() => $_clearField(7);

  @$pb.TagNumber(8)
  $core.double get peakAccel => $_getN(7);
  @$pb.TagNumber(8)
  set peakAccel($core.double value) => $_setFloat(7, value);
  @$pb.TagNumber(8)
  $core.bool hasPeakAccel() => $_has(7);
  @$pb.TagNumber(8)
  void clearPeakAccel() => $_clearField(8);

  @$pb.TagNumber(9)
  $core.double get rollMinDeg => $_getN(8);
  @$pb.TagNumber(9)
  set rollMinDeg($core.double value) => $_setFloat(8, value);
  @$pb.TagNumber(9)
  $core.bool hasRollMinDeg() => $_has(8);
  @$pb.TagNumber(9)
  void clearRollMinDeg() => $_clearField(9);

  @$pb.TagNumber(10)
  $core.double get rollMaxDeg => $_getN(9);
  @$pb.TagNumber(10)
  set rollMaxDeg($core.double value) => $_setFloat(9, value);
  @$pb.TagNumber(10)
  $core.bool hasRollMaxDeg() => $_has(9);
  @$pb.TagNumber(10)
  void clearRollMaxDeg() => $_clearField(10);

  @$pb.TagNumber(11)
  $core.double get pitchMinDeg => $_getN(10);
  @$pb.TagNumber(11)
  set pitchMinDeg($core.double value) => $_setFloat(10, value);
  @$pb.TagNumber(11)
  $core.bool hasPitchMinDeg() => $_has(10);
  @$pb.TagNumber(11)
  void clearPitchMinDeg() => $_clearField(11);

  @$pb.TagNumber(12)
  $core.double get pitchMaxDeg => $_getN(11);
  @$pb.TagNumber(12)
  set pitchMaxDeg($core.double value) => $_setFloat(11, value);
  @$pb.TagNumber(12)
  $core.bool hasPitchMaxDeg() => $_has(11);
  @$pb.TagNumber(12)
  void clearPitchMaxDeg() => $_clearField(12);

  @$pb.TagNumber(13)
  $core.int get traceLen => $_getIZ(12);
  @$pb.TagNumber(13)
  set traceLen($core.int value) => $_setUnsignedInt32(12, value);
  @$pb.TagNumber(13)
  $core.bool hasTraceLen() => $_has(12);
  @$pb.TagNumber(13)
  void clearTraceLen() => $_clearField(13);

  @$pb.TagNumber(14)
  $core.int get chunkCount => $_getIZ(13);
  @$pb.TagNumber(14)
  set chunkCount($core.int value) => $_setUnsignedInt32(13, value);
  @$pb.TagNumber(14)
  $core.bool hasChunkCount() => $_has(13);
  @$pb.TagNumber(14)
  void clearChunkCount() => $_clearField(14);

  @$pb.TagNumber(15)
  $core.bool get decimated => $_getBF(14);
  @$pb.TagNumber(15)
  set decimated($core.bool value) => $_setBool(14, value);
  @$pb.TagNumber(15)
  $core.bool hasDecimated() => $_has(14);
  @$pb.TagNumber(15)
  void clearDecimated() => $_clearField(15);
}

/// A slice of a rep's trace: as stored in the log, and as sent over BLE.
class TraceChunk extends $pb.GeneratedMessage {
  factory TraceChunk({
    $core.List<$core.int>? repUuid,
    $core.int? index,
    $core.int? firstTOffsetMs,
    $core.Iterable<TraceSample>? samples,
    $core.int? totalChunks,
  }) {
    final result = TraceChunk._();
    if (repUuid != null) result.repUuid = repUuid;
    if (index != null) result.index = index;
    if (firstTOffsetMs != null) result.firstTOffsetMs = firstTOffsetMs;
    if (samples != null) result.samples.addAll(samples);
    if (totalChunks != null) result.totalChunks = totalChunks;
    return result;
  }

  TraceChunk._();

  factory TraceChunk.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      TraceChunk()..mergeFromBuffer(data, registry);
  factory TraceChunk.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      TraceChunk()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'TraceChunk',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: TraceChunk.$_createMessage)
    ..a<$core.List<$core.int>>(
        1, _omitFieldNames ? '' : 'repUuid', $pb.PbFieldType.OY)
    ..aI(2, _omitFieldNames ? '' : 'index', fieldType: $pb.PbFieldType.OU3)
    ..aI(3, _omitFieldNames ? '' : 'firstTOffsetMs',
        fieldType: $pb.PbFieldType.OU3)
    ..pPM<TraceSample>(4, _omitFieldNames ? '' : 'samples',
        subBuilder: TraceSample.$_createMessage)
    ..aI(5, _omitFieldNames ? '' : 'totalChunks',
        fieldType: $pb.PbFieldType.OU3)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TraceChunk clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TraceChunk copyWith(void Function(TraceChunk) updates) =>
      super.copyWith((message) => updates(message as TraceChunk)) as TraceChunk;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use TraceChunk() / TraceChunk.new instead')
  static TraceChunk create() => TraceChunk._();
  static $pb.GeneratedMessage $_createMessage() => TraceChunk._();
  @$core.override
  TraceChunk createEmptyInstance() => TraceChunk._();
  @$core.pragma('dart2js:noInline')
  static TraceChunk getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<TraceChunk>(TraceChunk.$_createMessage);
  static TraceChunk? _defaultInstance;

  @$pb.TagNumber(1)
  $core.List<$core.int> get repUuid => $_getN(0);
  @$pb.TagNumber(1)
  set repUuid($core.List<$core.int> value) => $_setBytes(0, value);
  @$pb.TagNumber(1)
  $core.bool hasRepUuid() => $_has(0);
  @$pb.TagNumber(1)
  void clearRepUuid() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.int get index => $_getIZ(1);
  @$pb.TagNumber(2)
  set index($core.int value) => $_setUnsignedInt32(1, value);
  @$pb.TagNumber(2)
  $core.bool hasIndex() => $_has(1);
  @$pb.TagNumber(2)
  void clearIndex() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.int get firstTOffsetMs => $_getIZ(2);
  @$pb.TagNumber(3)
  set firstTOffsetMs($core.int value) => $_setUnsignedInt32(2, value);
  @$pb.TagNumber(3)
  $core.bool hasFirstTOffsetMs() => $_has(2);
  @$pb.TagNumber(3)
  void clearFirstTOffsetMs() => $_clearField(3);

  @$pb.TagNumber(4)
  $pb.PbList<TraceSample> get samples => $_getList(3);

  /// Chunks in the stream this record belongs to. Set only when a trace is being
  /// streamed over BLE; a record stored in the log leaves it 0, because there the
  /// summary's chunk_count already describes the trace and the device re-chunks
  /// to fit the MTU when streaming. Without this the client cannot tell "more
  /// chunks are coming" from "the device has forgotten this rep and will stay
  /// silent".
  @$pb.TagNumber(5)
  $core.int get totalChunks => $_getIZ(4);
  @$pb.TagNumber(5)
  set totalChunks($core.int value) => $_setUnsignedInt32(4, value);
  @$pb.TagNumber(5)
  $core.bool hasTotalChunks() => $_has(4);
  @$pb.TagNumber(5)
  void clearTotalChunks() => $_clearField(5);
}

enum RepRecord_Body { summary, trace, notSet }

/// One log record: a rep summary or one of its trace chunks. Keeping both
/// under one wire type makes the log a single self-describing stream.
class RepRecord extends $pb.GeneratedMessage {
  factory RepRecord({
    RepSummary? summary,
    TraceChunk? trace,
  }) {
    final result = RepRecord._();
    if (summary != null) result.summary = summary;
    if (trace != null) result.trace = trace;
    return result;
  }

  RepRecord._();

  factory RepRecord.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      RepRecord()..mergeFromBuffer(data, registry);
  factory RepRecord.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      RepRecord()..mergeFromJson(json, registry);

  static const $core.Map<$core.int, RepRecord_Body> _RepRecord_BodyByTag = {
    1: RepRecord_Body.summary,
    2: RepRecord_Body.trace,
    0: RepRecord_Body.notSet
  };
  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'RepRecord',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: RepRecord.$_createMessage)
    ..oo(0, [1, 2])
    ..aOM<RepSummary>(1, _omitFieldNames ? '' : 'summary',
        subBuilder: RepSummary.$_createMessage)
    ..aOM<TraceChunk>(2, _omitFieldNames ? '' : 'trace',
        subBuilder: TraceChunk.$_createMessage)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  RepRecord clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  RepRecord copyWith(void Function(RepRecord) updates) =>
      super.copyWith((message) => updates(message as RepRecord)) as RepRecord;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use RepRecord() / RepRecord.new instead')
  static RepRecord create() => RepRecord._();
  static $pb.GeneratedMessage $_createMessage() => RepRecord._();
  @$core.override
  RepRecord createEmptyInstance() => RepRecord._();
  @$core.pragma('dart2js:noInline')
  static RepRecord getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<RepRecord>(RepRecord.$_createMessage);
  static RepRecord? _defaultInstance;

  @$pb.TagNumber(1)
  @$pb.TagNumber(2)
  RepRecord_Body whichBody() => _RepRecord_BodyByTag[$_whichOneof(0)]!;
  @$pb.TagNumber(1)
  @$pb.TagNumber(2)
  void clearBody() => $_clearField($_whichOneof(0));

  @$pb.TagNumber(1)
  RepSummary get summary => $_getN(0);
  @$pb.TagNumber(1)
  set summary(RepSummary value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasSummary() => $_has(0);
  @$pb.TagNumber(1)
  void clearSummary() => $_clearField(1);
  @$pb.TagNumber(1)
  RepSummary ensureSummary() => $_ensure(0);

  @$pb.TagNumber(2)
  TraceChunk get trace => $_getN(1);
  @$pb.TagNumber(2)
  set trace(TraceChunk value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasTrace() => $_has(1);
  @$pb.TagNumber(2)
  void clearTrace() => $_clearField(2);
  @$pb.TagNumber(2)
  TraceChunk ensureTrace() => $_ensure(1);
}

/// Sent by the app to request a rep's trace. `chunk_index` is ignored unless
/// `all_chunks` is false.
class TraceRequest extends $pb.GeneratedMessage {
  factory TraceRequest({
    $core.List<$core.int>? repUuid,
    $core.bool? allChunks,
    $core.int? chunkIndex,
  }) {
    final result = TraceRequest._();
    if (repUuid != null) result.repUuid = repUuid;
    if (allChunks != null) result.allChunks = allChunks;
    if (chunkIndex != null) result.chunkIndex = chunkIndex;
    return result;
  }

  TraceRequest._();

  factory TraceRequest.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      TraceRequest()..mergeFromBuffer(data, registry);
  factory TraceRequest.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      TraceRequest()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'TraceRequest',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: TraceRequest.$_createMessage)
    ..a<$core.List<$core.int>>(
        1, _omitFieldNames ? '' : 'repUuid', $pb.PbFieldType.OY)
    ..aOB(2, _omitFieldNames ? '' : 'allChunks')
    ..aI(3, _omitFieldNames ? '' : 'chunkIndex', fieldType: $pb.PbFieldType.OU3)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TraceRequest clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TraceRequest copyWith(void Function(TraceRequest) updates) =>
      super.copyWith((message) => updates(message as TraceRequest))
          as TraceRequest;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use TraceRequest() / TraceRequest.new instead')
  static TraceRequest create() => TraceRequest._();
  static $pb.GeneratedMessage $_createMessage() => TraceRequest._();
  @$core.override
  TraceRequest createEmptyInstance() => TraceRequest._();
  @$core.pragma('dart2js:noInline')
  static TraceRequest getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<TraceRequest>(
          TraceRequest.$_createMessage);
  static TraceRequest? _defaultInstance;

  @$pb.TagNumber(1)
  $core.List<$core.int> get repUuid => $_getN(0);
  @$pb.TagNumber(1)
  set repUuid($core.List<$core.int> value) => $_setBytes(0, value);
  @$pb.TagNumber(1)
  $core.bool hasRepUuid() => $_has(0);
  @$pb.TagNumber(1)
  void clearRepUuid() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.bool get allChunks => $_getBF(1);
  @$pb.TagNumber(2)
  set allChunks($core.bool value) => $_setBool(1, value);
  @$pb.TagNumber(2)
  $core.bool hasAllChunks() => $_has(1);
  @$pb.TagNumber(2)
  void clearAllChunks() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.int get chunkIndex => $_getIZ(2);
  @$pb.TagNumber(3)
  set chunkIndex($core.int value) => $_setUnsignedInt32(2, value);
  @$pb.TagNumber(3)
  $core.bool hasChunkIndex() => $_has(2);
  @$pb.TagNumber(3)
  void clearChunkIndex() => $_clearField(3);
}

enum SyncMessage_Body { summary, trace, status, notSet }

/// Everything sent device -> app on the sync service.
class SyncMessage extends $pb.GeneratedMessage {
  factory SyncMessage({
    RepSummary? summary,
    TraceChunk? trace,
    SyncStatus? status,
  }) {
    final result = SyncMessage._();
    if (summary != null) result.summary = summary;
    if (trace != null) result.trace = trace;
    if (status != null) result.status = status;
    return result;
  }

  SyncMessage._();

  factory SyncMessage.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      SyncMessage()..mergeFromBuffer(data, registry);
  factory SyncMessage.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      SyncMessage()..mergeFromJson(json, registry);

  static const $core.Map<$core.int, SyncMessage_Body> _SyncMessage_BodyByTag = {
    1: SyncMessage_Body.summary,
    2: SyncMessage_Body.trace,
    3: SyncMessage_Body.status,
    0: SyncMessage_Body.notSet
  };
  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'SyncMessage',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: SyncMessage.$_createMessage)
    ..oo(0, [1, 2, 3])
    ..aOM<RepSummary>(1, _omitFieldNames ? '' : 'summary',
        subBuilder: RepSummary.$_createMessage)
    ..aOM<TraceChunk>(2, _omitFieldNames ? '' : 'trace',
        subBuilder: TraceChunk.$_createMessage)
    ..aOM<SyncStatus>(3, _omitFieldNames ? '' : 'status',
        subBuilder: SyncStatus.$_createMessage)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  SyncMessage clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  SyncMessage copyWith(void Function(SyncMessage) updates) =>
      super.copyWith((message) => updates(message as SyncMessage))
          as SyncMessage;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use SyncMessage() / SyncMessage.new instead')
  static SyncMessage create() => SyncMessage._();
  static $pb.GeneratedMessage $_createMessage() => SyncMessage._();
  @$core.override
  SyncMessage createEmptyInstance() => SyncMessage._();
  @$core.pragma('dart2js:noInline')
  static SyncMessage getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<SyncMessage>(
          SyncMessage.$_createMessage);
  static SyncMessage? _defaultInstance;

  @$pb.TagNumber(1)
  @$pb.TagNumber(2)
  @$pb.TagNumber(3)
  SyncMessage_Body whichBody() => _SyncMessage_BodyByTag[$_whichOneof(0)]!;
  @$pb.TagNumber(1)
  @$pb.TagNumber(2)
  @$pb.TagNumber(3)
  void clearBody() => $_clearField($_whichOneof(0));

  @$pb.TagNumber(1)
  RepSummary get summary => $_getN(0);
  @$pb.TagNumber(1)
  set summary(RepSummary value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasSummary() => $_has(0);
  @$pb.TagNumber(1)
  void clearSummary() => $_clearField(1);
  @$pb.TagNumber(1)
  RepSummary ensureSummary() => $_ensure(0);

  @$pb.TagNumber(2)
  TraceChunk get trace => $_getN(1);
  @$pb.TagNumber(2)
  set trace(TraceChunk value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasTrace() => $_has(1);
  @$pb.TagNumber(2)
  void clearTrace() => $_clearField(2);
  @$pb.TagNumber(2)
  TraceChunk ensureTrace() => $_ensure(1);

  @$pb.TagNumber(3)
  SyncStatus get status => $_getN(2);
  @$pb.TagNumber(3)
  set status(SyncStatus value) => $_setField(3, value);
  @$pb.TagNumber(3)
  $core.bool hasStatus() => $_has(2);
  @$pb.TagNumber(3)
  void clearStatus() => $_clearField(3);
  @$pb.TagNumber(3)
  SyncStatus ensureStatus() => $_ensure(2);
}

class SyncStatus extends $pb.GeneratedMessage {
  factory SyncStatus({
    $core.int? repCount,
    $core.List<$core.int>? oldestRepUuid,
    $core.List<$core.int>? newestRepUuid,
    $core.bool? traceUnavailable,
  }) {
    final result = SyncStatus._();
    if (repCount != null) result.repCount = repCount;
    if (oldestRepUuid != null) result.oldestRepUuid = oldestRepUuid;
    if (newestRepUuid != null) result.newestRepUuid = newestRepUuid;
    if (traceUnavailable != null) result.traceUnavailable = traceUnavailable;
    return result;
  }

  SyncStatus._();

  factory SyncStatus.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      SyncStatus()..mergeFromBuffer(data, registry);
  factory SyncStatus.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      SyncStatus()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'SyncStatus',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: SyncStatus.$_createMessage)
    ..aI(1, _omitFieldNames ? '' : 'repCount', fieldType: $pb.PbFieldType.OU3)
    ..a<$core.List<$core.int>>(
        2, _omitFieldNames ? '' : 'oldestRepUuid', $pb.PbFieldType.OY)
    ..a<$core.List<$core.int>>(
        3, _omitFieldNames ? '' : 'newestRepUuid', $pb.PbFieldType.OY)
    ..aOB(4, _omitFieldNames ? '' : 'traceUnavailable')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  SyncStatus clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  SyncStatus copyWith(void Function(SyncStatus) updates) =>
      super.copyWith((message) => updates(message as SyncStatus)) as SyncStatus;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use SyncStatus() / SyncStatus.new instead')
  static SyncStatus create() => SyncStatus._();
  static $pb.GeneratedMessage $_createMessage() => SyncStatus._();
  @$core.override
  SyncStatus createEmptyInstance() => SyncStatus._();
  @$core.pragma('dart2js:noInline')
  static SyncStatus getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<SyncStatus>(SyncStatus.$_createMessage);
  static SyncStatus? _defaultInstance;

  @$pb.TagNumber(1)
  $core.int get repCount => $_getIZ(0);
  @$pb.TagNumber(1)
  set repCount($core.int value) => $_setUnsignedInt32(0, value);
  @$pb.TagNumber(1)
  $core.bool hasRepCount() => $_has(0);
  @$pb.TagNumber(1)
  void clearRepCount() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.List<$core.int> get oldestRepUuid => $_getN(1);
  @$pb.TagNumber(2)
  set oldestRepUuid($core.List<$core.int> value) => $_setBytes(1, value);
  @$pb.TagNumber(2)
  $core.bool hasOldestRepUuid() => $_has(1);
  @$pb.TagNumber(2)
  void clearOldestRepUuid() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.List<$core.int> get newestRepUuid => $_getN(2);
  @$pb.TagNumber(3)
  set newestRepUuid($core.List<$core.int> value) => $_setBytes(2, value);
  @$pb.TagNumber(3)
  $core.bool hasNewestRepUuid() => $_has(2);
  @$pb.TagNumber(3)
  void clearNewestRepUuid() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.bool get traceUnavailable => $_getBF(3);
  @$pb.TagNumber(4)
  set traceUnavailable($core.bool value) => $_setBool(3, value);
  @$pb.TagNumber(4)
  $core.bool hasTraceUnavailable() => $_has(3);
  @$pb.TagNumber(4)
  void clearTraceUnavailable() => $_clearField(4);
}

const $core.bool _omitFieldNames =
    $core.bool.fromEnvironment('protobuf.omit_field_names');
const $core.bool _omitMessageNames =
    $core.bool.fromEnvironment('protobuf.omit_message_names');
