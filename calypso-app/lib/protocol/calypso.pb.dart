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

import 'package:fixnum/fixnum.dart' as $fixnum;
import 'package:protobuf/protobuf.dart' as $pb;

import 'calypso.pbenum.dart';

export 'package:protobuf/protobuf.dart' show GeneratedMessageGenericExtensions;

export 'calypso.pbenum.dart';

/// One point of a rep's plotted path.
class TraceComputedPoint extends $pb.GeneratedMessage {
  factory TraceComputedPoint({
    $core.int? tOffsetMs,
    $core.double? px,
    $core.double? py,
    $core.double? pz,
    $core.double? qw,
    $core.double? qx,
    $core.double? qy,
    $core.double? qz,
  }) {
    final result = TraceComputedPoint._();
    if (tOffsetMs != null) result.tOffsetMs = tOffsetMs;
    if (px != null) result.px = px;
    if (py != null) result.py = py;
    if (pz != null) result.pz = pz;
    if (qw != null) result.qw = qw;
    if (qx != null) result.qx = qx;
    if (qy != null) result.qy = qy;
    if (qz != null) result.qz = qz;
    return result;
  }

  TraceComputedPoint._();

  factory TraceComputedPoint.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      TraceComputedPoint()..mergeFromBuffer(data, registry);
  factory TraceComputedPoint.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      TraceComputedPoint()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'TraceComputedPoint',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: TraceComputedPoint.$_createMessage)
    ..aI(1, _omitFieldNames ? '' : 'tOffsetMs', fieldType: $pb.PbFieldType.OU3)
    ..aD(2, _omitFieldNames ? '' : 'px', fieldType: $pb.PbFieldType.OF)
    ..aD(3, _omitFieldNames ? '' : 'py', fieldType: $pb.PbFieldType.OF)
    ..aD(4, _omitFieldNames ? '' : 'pz', fieldType: $pb.PbFieldType.OF)
    ..aD(5, _omitFieldNames ? '' : 'qw', fieldType: $pb.PbFieldType.OF)
    ..aD(6, _omitFieldNames ? '' : 'qx', fieldType: $pb.PbFieldType.OF)
    ..aD(7, _omitFieldNames ? '' : 'qy', fieldType: $pb.PbFieldType.OF)
    ..aD(8, _omitFieldNames ? '' : 'qz', fieldType: $pb.PbFieldType.OF)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TraceComputedPoint clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TraceComputedPoint copyWith(void Function(TraceComputedPoint) updates) =>
      super.copyWith((message) => updates(message as TraceComputedPoint))
          as TraceComputedPoint;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use TraceComputedPoint() / TraceComputedPoint.new instead')
  static TraceComputedPoint create() => TraceComputedPoint._();
  static $pb.GeneratedMessage $_createMessage() => TraceComputedPoint._();
  @$core.override
  TraceComputedPoint createEmptyInstance() => TraceComputedPoint._();
  @$core.pragma('dart2js:noInline')
  static TraceComputedPoint getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<TraceComputedPoint>(
          TraceComputedPoint.$_createMessage);
  static TraceComputedPoint? _defaultInstance;

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
  $core.double get qw => $_getN(4);
  @$pb.TagNumber(5)
  set qw($core.double value) => $_setFloat(4, value);
  @$pb.TagNumber(5)
  $core.bool hasQw() => $_has(4);
  @$pb.TagNumber(5)
  void clearQw() => $_clearField(5);

  @$pb.TagNumber(6)
  $core.double get qx => $_getN(5);
  @$pb.TagNumber(6)
  set qx($core.double value) => $_setFloat(5, value);
  @$pb.TagNumber(6)
  $core.bool hasQx() => $_has(5);
  @$pb.TagNumber(6)
  void clearQx() => $_clearField(6);

  @$pb.TagNumber(7)
  $core.double get qy => $_getN(6);
  @$pb.TagNumber(7)
  set qy($core.double value) => $_setFloat(6, value);
  @$pb.TagNumber(7)
  $core.bool hasQy() => $_has(6);
  @$pb.TagNumber(7)
  void clearQy() => $_clearField(7);

  @$pb.TagNumber(8)
  $core.double get qz => $_getN(7);
  @$pb.TagNumber(8)
  set qz($core.double value) => $_setFloat(7, value);
  @$pb.TagNumber(8)
  $core.bool hasQz() => $_has(7);
  @$pb.TagNumber(8)
  void clearQz() => $_clearField(8);
}

/// One sample of a rep's raw stream: the sensor's own reading, as the filter took it.
class TraceRawPoint extends $pb.GeneratedMessage {
  factory TraceRawPoint({
    $core.int? tOffsetMs,
    $core.int? ax,
    $core.int? ay,
    $core.int? az,
    $core.int? gx,
    $core.int? gy,
    $core.int? gz,
  }) {
    final result = TraceRawPoint._();
    if (tOffsetMs != null) result.tOffsetMs = tOffsetMs;
    if (ax != null) result.ax = ax;
    if (ay != null) result.ay = ay;
    if (az != null) result.az = az;
    if (gx != null) result.gx = gx;
    if (gy != null) result.gy = gy;
    if (gz != null) result.gz = gz;
    return result;
  }

  TraceRawPoint._();

  factory TraceRawPoint.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      TraceRawPoint()..mergeFromBuffer(data, registry);
  factory TraceRawPoint.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      TraceRawPoint()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'TraceRawPoint',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: TraceRawPoint.$_createMessage)
    ..aI(1, _omitFieldNames ? '' : 'tOffsetMs', fieldType: $pb.PbFieldType.OU3)
    ..aI(2, _omitFieldNames ? '' : 'ax', fieldType: $pb.PbFieldType.OS3)
    ..aI(3, _omitFieldNames ? '' : 'ay', fieldType: $pb.PbFieldType.OS3)
    ..aI(4, _omitFieldNames ? '' : 'az', fieldType: $pb.PbFieldType.OS3)
    ..aI(5, _omitFieldNames ? '' : 'gx', fieldType: $pb.PbFieldType.OS3)
    ..aI(6, _omitFieldNames ? '' : 'gy', fieldType: $pb.PbFieldType.OS3)
    ..aI(7, _omitFieldNames ? '' : 'gz', fieldType: $pb.PbFieldType.OS3)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TraceRawPoint clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  TraceRawPoint copyWith(void Function(TraceRawPoint) updates) =>
      super.copyWith((message) => updates(message as TraceRawPoint))
          as TraceRawPoint;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use TraceRawPoint() / TraceRawPoint.new instead')
  static TraceRawPoint create() => TraceRawPoint._();
  static $pb.GeneratedMessage $_createMessage() => TraceRawPoint._();
  @$core.override
  TraceRawPoint createEmptyInstance() => TraceRawPoint._();
  @$core.pragma('dart2js:noInline')
  static TraceRawPoint getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<TraceRawPoint>(
          TraceRawPoint.$_createMessage);
  static TraceRawPoint? _defaultInstance;

  @$pb.TagNumber(1)
  $core.int get tOffsetMs => $_getIZ(0);
  @$pb.TagNumber(1)
  set tOffsetMs($core.int value) => $_setUnsignedInt32(0, value);
  @$pb.TagNumber(1)
  $core.bool hasTOffsetMs() => $_has(0);
  @$pb.TagNumber(1)
  void clearTOffsetMs() => $_clearField(1);

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
  $core.int get gx => $_getIZ(4);
  @$pb.TagNumber(5)
  set gx($core.int value) => $_setSignedInt32(4, value);
  @$pb.TagNumber(5)
  $core.bool hasGx() => $_has(4);
  @$pb.TagNumber(5)
  void clearGx() => $_clearField(5);

  @$pb.TagNumber(6)
  $core.int get gy => $_getIZ(5);
  @$pb.TagNumber(6)
  set gy($core.int value) => $_setSignedInt32(5, value);
  @$pb.TagNumber(6)
  $core.bool hasGy() => $_has(5);
  @$pb.TagNumber(6)
  void clearGy() => $_clearField(6);

  @$pb.TagNumber(7)
  $core.int get gz => $_getIZ(6);
  @$pb.TagNumber(7)
  set gz($core.int value) => $_setSignedInt32(6, value);
  @$pb.TagNumber(7)
  $core.bool hasGz() => $_has(6);
  @$pb.TagNumber(7)
  void clearGz() => $_clearField(7);
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
    $core.double? rollMinDeg,
    $core.double? rollMaxDeg,
    $core.double? pitchMinDeg,
    $core.double? pitchMaxDeg,
    $core.int? computedLen,
    $core.int? computedChunks,
    $fixnum.Int64? tUnixMs,
    RepState? state,
    RepConfig? config,
    $core.int? rawLen,
    $core.int? rawChunks,
  }) {
    final result = RepSummary._();
    if (repUuid != null) result.repUuid = repUuid;
    if (tStartMs != null) result.tStartMs = tStartMs;
    if (tEndMs != null) result.tEndMs = tEndMs;
    if (durationMs != null) result.durationMs = durationMs;
    if (rom != null) result.rom = rom;
    if (peakVelocity != null) result.peakVelocity = peakVelocity;
    if (rollMinDeg != null) result.rollMinDeg = rollMinDeg;
    if (rollMaxDeg != null) result.rollMaxDeg = rollMaxDeg;
    if (pitchMinDeg != null) result.pitchMinDeg = pitchMinDeg;
    if (pitchMaxDeg != null) result.pitchMaxDeg = pitchMaxDeg;
    if (computedLen != null) result.computedLen = computedLen;
    if (computedChunks != null) result.computedChunks = computedChunks;
    if (tUnixMs != null) result.tUnixMs = tUnixMs;
    if (state != null) result.state = state;
    if (config != null) result.config = config;
    if (rawLen != null) result.rawLen = rawLen;
    if (rawChunks != null) result.rawChunks = rawChunks;
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
    ..aD(9, _omitFieldNames ? '' : 'rollMinDeg', fieldType: $pb.PbFieldType.OF)
    ..aD(10, _omitFieldNames ? '' : 'rollMaxDeg', fieldType: $pb.PbFieldType.OF)
    ..aD(11, _omitFieldNames ? '' : 'pitchMinDeg',
        fieldType: $pb.PbFieldType.OF)
    ..aD(12, _omitFieldNames ? '' : 'pitchMaxDeg',
        fieldType: $pb.PbFieldType.OF)
    ..aI(13, _omitFieldNames ? '' : 'computedLen',
        fieldType: $pb.PbFieldType.OU3)
    ..aI(14, _omitFieldNames ? '' : 'computedChunks',
        fieldType: $pb.PbFieldType.OU3)
    ..a<$fixnum.Int64>(
        16, _omitFieldNames ? '' : 'tUnixMs', $pb.PbFieldType.OU6,
        defaultOrMaker: $fixnum.Int64.ZERO)
    ..aOM<RepState>(17, _omitFieldNames ? '' : 'state',
        subBuilder: RepState.$_createMessage)
    ..aOM<RepConfig>(18, _omitFieldNames ? '' : 'config',
        subBuilder: RepConfig.$_createMessage)
    ..aI(19, _omitFieldNames ? '' : 'rawLen', fieldType: $pb.PbFieldType.OU3)
    ..aI(20, _omitFieldNames ? '' : 'rawChunks', fieldType: $pb.PbFieldType.OU3)
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

  /// Peak-to-peak vertical travel: the range of motion, on Z. Z is the axis the fusion pins to gravity; X and Y are the mount's heading and drift with yaw, so the largest excursion is not the rep's range of motion and may be the estimate's error rather than the bar.
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

  @$pb.TagNumber(9)
  $core.double get rollMinDeg => $_getN(6);
  @$pb.TagNumber(9)
  set rollMinDeg($core.double value) => $_setFloat(6, value);
  @$pb.TagNumber(9)
  $core.bool hasRollMinDeg() => $_has(6);
  @$pb.TagNumber(9)
  void clearRollMinDeg() => $_clearField(9);

  @$pb.TagNumber(10)
  $core.double get rollMaxDeg => $_getN(7);
  @$pb.TagNumber(10)
  set rollMaxDeg($core.double value) => $_setFloat(7, value);
  @$pb.TagNumber(10)
  $core.bool hasRollMaxDeg() => $_has(7);
  @$pb.TagNumber(10)
  void clearRollMaxDeg() => $_clearField(10);

  @$pb.TagNumber(11)
  $core.double get pitchMinDeg => $_getN(8);
  @$pb.TagNumber(11)
  set pitchMinDeg($core.double value) => $_setFloat(8, value);
  @$pb.TagNumber(11)
  $core.bool hasPitchMinDeg() => $_has(8);
  @$pb.TagNumber(11)
  void clearPitchMinDeg() => $_clearField(11);

  @$pb.TagNumber(12)
  $core.double get pitchMaxDeg => $_getN(9);
  @$pb.TagNumber(12)
  set pitchMaxDeg($core.double value) => $_setFloat(9, value);
  @$pb.TagNumber(12)
  $core.bool hasPitchMaxDeg() => $_has(9);
  @$pb.TagNumber(12)
  void clearPitchMaxDeg() => $_clearField(12);

  @$pb.TagNumber(13)
  $core.int get computedLen => $_getIZ(10);
  @$pb.TagNumber(13)
  set computedLen($core.int value) => $_setUnsignedInt32(10, value);
  @$pb.TagNumber(13)
  $core.bool hasComputedLen() => $_has(10);
  @$pb.TagNumber(13)
  void clearComputedLen() => $_clearField(13);

  @$pb.TagNumber(14)
  $core.int get computedChunks => $_getIZ(11);
  @$pb.TagNumber(14)
  set computedChunks($core.int value) => $_setUnsignedInt32(11, value);
  @$pb.TagNumber(14)
  $core.bool hasComputedChunks() => $_has(11);
  @$pb.TagNumber(14)
  void clearComputedChunks() => $_clearField(14);

  /// The rep's wall-clock time, unix milliseconds, or 0 when the device had no clock for the boot that recorded it.
  ///
  /// **Set when a summary is streamed, 0 in the log.** The time is a property of the boot, not of the rep, and it is often not known until after the rep has been written: a stored summary therefore carries its uptime (`t_start_ms`) and the boot's clock travels as its own record (`BootClock`).
  @$pb.TagNumber(16)
  $fixnum.Int64 get tUnixMs => $_getI64(12);
  @$pb.TagNumber(16)
  set tUnixMs($fixnum.Int64 value) => $_setInt64(12, value);
  @$pb.TagNumber(16)
  $core.bool hasTUnixMs() => $_has(12);
  @$pb.TagNumber(16)
  void clearTUnixMs() => $_clearField(16);

  /// Where the filter stood where the stored window begins: the whole of its memory, so replaying the window from here reproduces the summary instead of approximating it. This is what a `RawChunk` trace is re-run from.
  @$pb.TagNumber(17)
  RepState get state => $_getN(13);
  @$pb.TagNumber(17)
  set state(RepState value) => $_setField(17, value);
  @$pb.TagNumber(17)
  $core.bool hasState() => $_has(13);
  @$pb.TagNumber(17)
  void clearState() => $_clearField(17);
  @$pb.TagNumber(17)
  RepState ensureState() => $_ensure(13);

  /// What the sensor was set to, and which detection logic closed the rep: what the numbers in either trace mean.
  @$pb.TagNumber(18)
  RepConfig get config => $_getN(14);
  @$pb.TagNumber(18)
  set config(RepConfig value) => $_setField(18, value);
  @$pb.TagNumber(18)
  $core.bool hasConfig() => $_has(14);
  @$pb.TagNumber(18)
  void clearConfig() => $_clearField(18);
  @$pb.TagNumber(18)
  RepConfig ensureConfig() => $_ensure(14);

  /// The raw trace's size, or both 0 when this device does not store one. The counts are the answer to "can I replay this rep"; there is no flag for it, because two places to ask is one place to disagree.
  @$pb.TagNumber(19)
  $core.int get rawLen => $_getIZ(15);
  @$pb.TagNumber(19)
  set rawLen($core.int value) => $_setUnsignedInt32(15, value);
  @$pb.TagNumber(19)
  $core.bool hasRawLen() => $_has(15);
  @$pb.TagNumber(19)
  void clearRawLen() => $_clearField(19);

  @$pb.TagNumber(20)
  $core.int get rawChunks => $_getIZ(16);
  @$pb.TagNumber(20)
  set rawChunks($core.int value) => $_setUnsignedInt32(16, value);
  @$pb.TagNumber(20)
  $core.bool hasRawChunks() => $_has(16);
  @$pb.TagNumber(20)
  void clearRawChunks() => $_clearField(20);
}

/// Everything needed to read either trace's numbers.
class RepConfig extends $pb.GeneratedMessage {
  factory RepConfig({
    $core.int? accelRangeG,
    $core.int? gyroRangeDps,
    $core.int? accelBandwidthHz,
    $core.int? gyroBandwidthHz,
    $core.int? sampleRateHz,
    $core.int? detectorVersion,
  }) {
    final result = RepConfig._();
    if (accelRangeG != null) result.accelRangeG = accelRangeG;
    if (gyroRangeDps != null) result.gyroRangeDps = gyroRangeDps;
    if (accelBandwidthHz != null) result.accelBandwidthHz = accelBandwidthHz;
    if (gyroBandwidthHz != null) result.gyroBandwidthHz = gyroBandwidthHz;
    if (sampleRateHz != null) result.sampleRateHz = sampleRateHz;
    if (detectorVersion != null) result.detectorVersion = detectorVersion;
    return result;
  }

  RepConfig._();

  factory RepConfig.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      RepConfig()..mergeFromBuffer(data, registry);
  factory RepConfig.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      RepConfig()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'RepConfig',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: RepConfig.$_createMessage)
    ..aI(1, _omitFieldNames ? '' : 'accelRangeG',
        fieldType: $pb.PbFieldType.OU3)
    ..aI(2, _omitFieldNames ? '' : 'gyroRangeDps',
        fieldType: $pb.PbFieldType.OU3)
    ..aI(3, _omitFieldNames ? '' : 'accelBandwidthHz',
        fieldType: $pb.PbFieldType.OU3)
    ..aI(4, _omitFieldNames ? '' : 'gyroBandwidthHz',
        fieldType: $pb.PbFieldType.OU3)
    ..aI(5, _omitFieldNames ? '' : 'sampleRateHz',
        fieldType: $pb.PbFieldType.OU3)
    ..aI(6, _omitFieldNames ? '' : 'detectorVersion',
        fieldType: $pb.PbFieldType.OU3)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  RepConfig clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  RepConfig copyWith(void Function(RepConfig) updates) =>
      super.copyWith((message) => updates(message as RepConfig)) as RepConfig;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use RepConfig() / RepConfig.new instead')
  static RepConfig create() => RepConfig._();
  static $pb.GeneratedMessage $_createMessage() => RepConfig._();
  @$core.override
  RepConfig createEmptyInstance() => RepConfig._();
  @$core.pragma('dart2js:noInline')
  static RepConfig getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<RepConfig>(RepConfig.$_createMessage);
  static RepConfig? _defaultInstance;

  @$pb.TagNumber(1)
  $core.int get accelRangeG => $_getIZ(0);
  @$pb.TagNumber(1)
  set accelRangeG($core.int value) => $_setUnsignedInt32(0, value);
  @$pb.TagNumber(1)
  $core.bool hasAccelRangeG() => $_has(0);
  @$pb.TagNumber(1)
  void clearAccelRangeG() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.int get gyroRangeDps => $_getIZ(1);
  @$pb.TagNumber(2)
  set gyroRangeDps($core.int value) => $_setUnsignedInt32(1, value);
  @$pb.TagNumber(2)
  $core.bool hasGyroRangeDps() => $_has(1);
  @$pb.TagNumber(2)
  void clearGyroRangeDps() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.int get accelBandwidthHz => $_getIZ(2);
  @$pb.TagNumber(3)
  set accelBandwidthHz($core.int value) => $_setUnsignedInt32(2, value);
  @$pb.TagNumber(3)
  $core.bool hasAccelBandwidthHz() => $_has(2);
  @$pb.TagNumber(3)
  void clearAccelBandwidthHz() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.int get gyroBandwidthHz => $_getIZ(3);
  @$pb.TagNumber(4)
  set gyroBandwidthHz($core.int value) => $_setUnsignedInt32(3, value);
  @$pb.TagNumber(4)
  $core.bool hasGyroBandwidthHz() => $_has(3);
  @$pb.TagNumber(4)
  void clearGyroBandwidthHz() => $_clearField(4);

  @$pb.TagNumber(5)
  $core.int get sampleRateHz => $_getIZ(4);
  @$pb.TagNumber(5)
  set sampleRateHz($core.int value) => $_setUnsignedInt32(4, value);
  @$pb.TagNumber(5)
  $core.bool hasSampleRateHz() => $_has(4);
  @$pb.TagNumber(5)
  void clearSampleRateHz() => $_clearField(5);

  /// Which detection logic closed this rep (`DETECTOR_VERSION` in `calypso_core::rep`).
  /// A summary recomputed by newer logic is a different summary, and this is what says so rather than leaving two numbers to be compared as if they were one.
  @$pb.TagNumber(6)
  $core.int get detectorVersion => $_getIZ(5);
  @$pb.TagNumber(6)
  set detectorVersion($core.int value) => $_setUnsignedInt32(5, value);
  @$pb.TagNumber(6)
  $core.bool hasDetectorVersion() => $_has(5);
  @$pb.TagNumber(6)
  void clearDetectorVersion() => $_clearField(6);
}

/// Where the filter stood when a stored window began.
///
/// The state is the filter's whole memory: a filter started from it and fed the window lands exactly where the one that recorded the rep did.
class RepState extends $pb.GeneratedMessage {
  factory RepState({
    $core.double? vx,
    $core.double? vy,
    $core.double? vz,
    $core.double? qw,
    $core.double? qx,
    $core.double? qy,
    $core.double? qz,
    $core.double? accelBiasX,
    $core.double? accelBiasY,
    $core.double? accelBiasZ,
    $core.double? gyroBiasX,
    $core.double? gyroBiasY,
    $core.double? gyroBiasZ,
    $core.bool? settled,
  }) {
    final result = RepState._();
    if (vx != null) result.vx = vx;
    if (vy != null) result.vy = vy;
    if (vz != null) result.vz = vz;
    if (qw != null) result.qw = qw;
    if (qx != null) result.qx = qx;
    if (qy != null) result.qy = qy;
    if (qz != null) result.qz = qz;
    if (accelBiasX != null) result.accelBiasX = accelBiasX;
    if (accelBiasY != null) result.accelBiasY = accelBiasY;
    if (accelBiasZ != null) result.accelBiasZ = accelBiasZ;
    if (gyroBiasX != null) result.gyroBiasX = gyroBiasX;
    if (gyroBiasY != null) result.gyroBiasY = gyroBiasY;
    if (gyroBiasZ != null) result.gyroBiasZ = gyroBiasZ;
    if (settled != null) result.settled = settled;
    return result;
  }

  RepState._();

  factory RepState.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      RepState()..mergeFromBuffer(data, registry);
  factory RepState.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      RepState()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'RepState',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: RepState.$_createMessage)
    ..aD(1, _omitFieldNames ? '' : 'vx', fieldType: $pb.PbFieldType.OF)
    ..aD(2, _omitFieldNames ? '' : 'vy', fieldType: $pb.PbFieldType.OF)
    ..aD(3, _omitFieldNames ? '' : 'vz', fieldType: $pb.PbFieldType.OF)
    ..aD(4, _omitFieldNames ? '' : 'qw', fieldType: $pb.PbFieldType.OF)
    ..aD(5, _omitFieldNames ? '' : 'qx', fieldType: $pb.PbFieldType.OF)
    ..aD(6, _omitFieldNames ? '' : 'qy', fieldType: $pb.PbFieldType.OF)
    ..aD(7, _omitFieldNames ? '' : 'qz', fieldType: $pb.PbFieldType.OF)
    ..aD(8, _omitFieldNames ? '' : 'accelBiasX', fieldType: $pb.PbFieldType.OF)
    ..aD(9, _omitFieldNames ? '' : 'accelBiasY', fieldType: $pb.PbFieldType.OF)
    ..aD(10, _omitFieldNames ? '' : 'accelBiasZ', fieldType: $pb.PbFieldType.OF)
    ..aD(11, _omitFieldNames ? '' : 'gyroBiasX', fieldType: $pb.PbFieldType.OF)
    ..aD(12, _omitFieldNames ? '' : 'gyroBiasY', fieldType: $pb.PbFieldType.OF)
    ..aD(13, _omitFieldNames ? '' : 'gyroBiasZ', fieldType: $pb.PbFieldType.OF)
    ..aOB(14, _omitFieldNames ? '' : 'settled')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  RepState clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  RepState copyWith(void Function(RepState) updates) =>
      super.copyWith((message) => updates(message as RepState)) as RepState;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use RepState() / RepState.new instead')
  static RepState create() => RepState._();
  static $pb.GeneratedMessage $_createMessage() => RepState._();
  @$core.override
  RepState createEmptyInstance() => RepState._();
  @$core.pragma('dart2js:noInline')
  static RepState getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<RepState>(RepState.$_createMessage);
  static RepState? _defaultInstance;

  @$pb.TagNumber(1)
  $core.double get vx => $_getN(0);
  @$pb.TagNumber(1)
  set vx($core.double value) => $_setFloat(0, value);
  @$pb.TagNumber(1)
  $core.bool hasVx() => $_has(0);
  @$pb.TagNumber(1)
  void clearVx() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.double get vy => $_getN(1);
  @$pb.TagNumber(2)
  set vy($core.double value) => $_setFloat(1, value);
  @$pb.TagNumber(2)
  $core.bool hasVy() => $_has(1);
  @$pb.TagNumber(2)
  void clearVy() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.double get vz => $_getN(2);
  @$pb.TagNumber(3)
  set vz($core.double value) => $_setFloat(2, value);
  @$pb.TagNumber(3)
  $core.bool hasVz() => $_has(2);
  @$pb.TagNumber(3)
  void clearVz() => $_clearField(3);

  @$pb.TagNumber(4)
  $core.double get qw => $_getN(3);
  @$pb.TagNumber(4)
  set qw($core.double value) => $_setFloat(3, value);
  @$pb.TagNumber(4)
  $core.bool hasQw() => $_has(3);
  @$pb.TagNumber(4)
  void clearQw() => $_clearField(4);

  @$pb.TagNumber(5)
  $core.double get qx => $_getN(4);
  @$pb.TagNumber(5)
  set qx($core.double value) => $_setFloat(4, value);
  @$pb.TagNumber(5)
  $core.bool hasQx() => $_has(4);
  @$pb.TagNumber(5)
  void clearQx() => $_clearField(5);

  @$pb.TagNumber(6)
  $core.double get qy => $_getN(5);
  @$pb.TagNumber(6)
  set qy($core.double value) => $_setFloat(5, value);
  @$pb.TagNumber(6)
  $core.bool hasQy() => $_has(5);
  @$pb.TagNumber(6)
  void clearQy() => $_clearField(6);

  @$pb.TagNumber(7)
  $core.double get qz => $_getN(6);
  @$pb.TagNumber(7)
  set qz($core.double value) => $_setFloat(6, value);
  @$pb.TagNumber(7)
  $core.bool hasQz() => $_has(6);
  @$pb.TagNumber(7)
  void clearQz() => $_clearField(7);

  @$pb.TagNumber(8)
  $core.double get accelBiasX => $_getN(7);
  @$pb.TagNumber(8)
  set accelBiasX($core.double value) => $_setFloat(7, value);
  @$pb.TagNumber(8)
  $core.bool hasAccelBiasX() => $_has(7);
  @$pb.TagNumber(8)
  void clearAccelBiasX() => $_clearField(8);

  @$pb.TagNumber(9)
  $core.double get accelBiasY => $_getN(8);
  @$pb.TagNumber(9)
  set accelBiasY($core.double value) => $_setFloat(8, value);
  @$pb.TagNumber(9)
  $core.bool hasAccelBiasY() => $_has(8);
  @$pb.TagNumber(9)
  void clearAccelBiasY() => $_clearField(9);

  @$pb.TagNumber(10)
  $core.double get accelBiasZ => $_getN(9);
  @$pb.TagNumber(10)
  set accelBiasZ($core.double value) => $_setFloat(9, value);
  @$pb.TagNumber(10)
  $core.bool hasAccelBiasZ() => $_has(9);
  @$pb.TagNumber(10)
  void clearAccelBiasZ() => $_clearField(10);

  @$pb.TagNumber(11)
  $core.double get gyroBiasX => $_getN(10);
  @$pb.TagNumber(11)
  set gyroBiasX($core.double value) => $_setFloat(10, value);
  @$pb.TagNumber(11)
  $core.bool hasGyroBiasX() => $_has(10);
  @$pb.TagNumber(11)
  void clearGyroBiasX() => $_clearField(11);

  @$pb.TagNumber(12)
  $core.double get gyroBiasY => $_getN(11);
  @$pb.TagNumber(12)
  set gyroBiasY($core.double value) => $_setFloat(11, value);
  @$pb.TagNumber(12)
  $core.bool hasGyroBiasY() => $_has(11);
  @$pb.TagNumber(12)
  void clearGyroBiasY() => $_clearField(12);

  @$pb.TagNumber(13)
  $core.double get gyroBiasZ => $_getN(12);
  @$pb.TagNumber(13)
  set gyroBiasZ($core.double value) => $_setFloat(12, value);
  @$pb.TagNumber(13)
  $core.bool hasGyroBiasZ() => $_has(12);
  @$pb.TagNumber(13)
  void clearGyroBiasZ() => $_clearField(13);

  /// Whether the bar was already at rest when this was taken. Not a convenience:
  /// "at rest" is a fact about the last 400 ms, so a replay without it has to earn it back and spends its first samples integrating a drift the original had already taken out.
  @$pb.TagNumber(14)
  $core.bool get settled => $_getBF(13);
  @$pb.TagNumber(14)
  set settled($core.bool value) => $_setBool(13, value);
  @$pb.TagNumber(14)
  $core.bool hasSettled() => $_has(13);
  @$pb.TagNumber(14)
  void clearSettled() => $_clearField(14);
}

/// A slice of a rep's plotted trace: as stored in the log, and as sent over BLE.
class TraceChunk extends $pb.GeneratedMessage {
  factory TraceChunk({
    $core.List<$core.int>? repUuid,
    $core.int? index,
    $core.int? firstTOffsetMs,
    $core.Iterable<TraceComputedPoint>? samples,
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
    ..pPM<TraceComputedPoint>(4, _omitFieldNames ? '' : 'samples',
        subBuilder: TraceComputedPoint.$_createMessage)
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
  $pb.PbList<TraceComputedPoint> get samples => $_getList(3);

  /// Chunks in the stream this record belongs to. Set only when a trace is being streamed over BLE; a record stored in the log leaves it 0, because there the summary's `computed_chunks` already describes the trace and the device re-chunks to fit the MTU when streaming. Without this the client cannot tell "more chunks are coming" from "the device has forgotten this rep and will stay silent".
  @$pb.TagNumber(5)
  $core.int get totalChunks => $_getIZ(4);
  @$pb.TagNumber(5)
  set totalChunks($core.int value) => $_setUnsignedInt32(4, value);
  @$pb.TagNumber(5)
  $core.bool hasTotalChunks() => $_has(4);
  @$pb.TagNumber(5)
  void clearTotalChunks() => $_clearField(5);
}

/// A slice of a rep's raw trace: the same framing, at full rate.
///
/// Stored in the log by a record-mode build, and read back only by a client that asks for `TRACE_KIND_RAW`.
class RawChunk extends $pb.GeneratedMessage {
  factory RawChunk({
    $core.List<$core.int>? repUuid,
    $core.int? index,
    $core.int? firstTOffsetMs,
    $core.Iterable<TraceRawPoint>? samples,
    $core.int? totalChunks,
  }) {
    final result = RawChunk._();
    if (repUuid != null) result.repUuid = repUuid;
    if (index != null) result.index = index;
    if (firstTOffsetMs != null) result.firstTOffsetMs = firstTOffsetMs;
    if (samples != null) result.samples.addAll(samples);
    if (totalChunks != null) result.totalChunks = totalChunks;
    return result;
  }

  RawChunk._();

  factory RawChunk.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      RawChunk()..mergeFromBuffer(data, registry);
  factory RawChunk.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      RawChunk()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'RawChunk',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: RawChunk.$_createMessage)
    ..a<$core.List<$core.int>>(
        1, _omitFieldNames ? '' : 'repUuid', $pb.PbFieldType.OY)
    ..aI(2, _omitFieldNames ? '' : 'index', fieldType: $pb.PbFieldType.OU3)
    ..aI(3, _omitFieldNames ? '' : 'firstTOffsetMs',
        fieldType: $pb.PbFieldType.OU3)
    ..pPM<TraceRawPoint>(4, _omitFieldNames ? '' : 'samples',
        subBuilder: TraceRawPoint.$_createMessage)
    ..aI(5, _omitFieldNames ? '' : 'totalChunks',
        fieldType: $pb.PbFieldType.OU3)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  RawChunk clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  RawChunk copyWith(void Function(RawChunk) updates) =>
      super.copyWith((message) => updates(message as RawChunk)) as RawChunk;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use RawChunk() / RawChunk.new instead')
  static RawChunk create() => RawChunk._();
  static $pb.GeneratedMessage $_createMessage() => RawChunk._();
  @$core.override
  RawChunk createEmptyInstance() => RawChunk._();
  @$core.pragma('dart2js:noInline')
  static RawChunk getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<RawChunk>(RawChunk.$_createMessage);
  static RawChunk? _defaultInstance;

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
  $pb.PbList<TraceRawPoint> get samples => $_getList(3);

  @$pb.TagNumber(5)
  $core.int get totalChunks => $_getIZ(4);
  @$pb.TagNumber(5)
  set totalChunks($core.int value) => $_setUnsignedInt32(4, value);
  @$pb.TagNumber(5)
  $core.bool hasTotalChunks() => $_has(4);
  @$pb.TagNumber(5)
  void clearTotalChunks() => $_clearField(5);
}

/// A slice of the live IMU stream: what the sampler read, sent while it reads it.
///
/// One notification is one chunk, so the number of samples a chunk carries is the device's choice and is bounded by the characteristic's `max_len`. The stream has no acknowledgement: a notification is best-effort at the host, and `first_t_ms` plus `seq` are what let the client see a gap instead of silently interpolating one.
///
/// This is the same sample format as `RawChunk`, at the same fixed-point scales, but it is not a rep: the stream runs from the moment `DevMode` is enabled until it is disabled or the link drops, and it has no UUID, window, or total. The phone app's record button writes each one into a `.pblog` as a `RepRecord.imu`, behind the `BootClock` that dates the boot.
class ImuChunk extends $pb.GeneratedMessage {
  factory ImuChunk({
    $core.int? firstTMs,
    $core.int? sampleRateHz,
    $core.int? seq,
    $core.Iterable<TraceRawPoint>? samples,
  }) {
    final result = ImuChunk._();
    if (firstTMs != null) result.firstTMs = firstTMs;
    if (sampleRateHz != null) result.sampleRateHz = sampleRateHz;
    if (seq != null) result.seq = seq;
    if (samples != null) result.samples.addAll(samples);
    return result;
  }

  ImuChunk._();

  factory ImuChunk.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      ImuChunk()..mergeFromBuffer(data, registry);
  factory ImuChunk.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      ImuChunk()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'ImuChunk',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: ImuChunk.$_createMessage)
    ..aI(1, _omitFieldNames ? '' : 'firstTMs', fieldType: $pb.PbFieldType.OU3)
    ..aI(2, _omitFieldNames ? '' : 'sampleRateHz',
        fieldType: $pb.PbFieldType.OU3)
    ..aI(3, _omitFieldNames ? '' : 'seq', fieldType: $pb.PbFieldType.OU3)
    ..pPM<TraceRawPoint>(4, _omitFieldNames ? '' : 'samples',
        subBuilder: TraceRawPoint.$_createMessage)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  ImuChunk clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  ImuChunk copyWith(void Function(ImuChunk) updates) =>
      super.copyWith((message) => updates(message as ImuChunk)) as ImuChunk;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use ImuChunk() / ImuChunk.new instead')
  static ImuChunk create() => ImuChunk._();
  static $pb.GeneratedMessage $_createMessage() => ImuChunk._();
  @$core.override
  ImuChunk createEmptyInstance() => ImuChunk._();
  @$core.pragma('dart2js:noInline')
  static ImuChunk getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<ImuChunk>(ImuChunk.$_createMessage);
  static ImuChunk? _defaultInstance;

  @$pb.TagNumber(1)
  $core.int get firstTMs => $_getIZ(0);
  @$pb.TagNumber(1)
  set firstTMs($core.int value) => $_setUnsignedInt32(0, value);
  @$pb.TagNumber(1)
  $core.bool hasFirstTMs() => $_has(0);
  @$pb.TagNumber(1)
  void clearFirstTMs() => $_clearField(1);

  @$pb.TagNumber(2)
  $core.int get sampleRateHz => $_getIZ(1);
  @$pb.TagNumber(2)
  set sampleRateHz($core.int value) => $_setUnsignedInt32(1, value);
  @$pb.TagNumber(2)
  $core.bool hasSampleRateHz() => $_has(1);
  @$pb.TagNumber(2)
  void clearSampleRateHz() => $_clearField(2);

  @$pb.TagNumber(3)
  $core.int get seq => $_getIZ(2);
  @$pb.TagNumber(3)
  set seq($core.int value) => $_setUnsignedInt32(2, value);
  @$pb.TagNumber(3)
  $core.bool hasSeq() => $_has(2);
  @$pb.TagNumber(3)
  void clearSeq() => $_clearField(3);

  @$pb.TagNumber(4)
  $pb.PbList<TraceRawPoint> get samples => $_getList(3);
}

/// The device's clock, as of one point in the log: what its uptime zero was worth on the wall clock from here on.
///
/// The device has no RTC, so the only time it can know is the one the client set (`Time`, `proto/calypso_gatt.proto`). That time belongs in the log rather than in RAM because it outlives the boot: reps written while the client was connected keep their time when the device restarts, and a rep that only the RAM remembers a clock for would lose it.
///
/// Written at most twice per boot:
///   - at mount, with `epoch_s` 0 — a boot's records start here, which is what stops a boot that never had a clock from being read as the one before it;
///   - when the client sets the clock, with the epoch. Re-setting it writes another, and the newest one before a record is the one that dates it.
///
/// A record's clock is the newest `BootClock` **at or before it**, which is the only rule both ends can apply: the device can name its own boot's records by the ids it has in RAM, and a dump of the log carries no ids at all, so a reader goes by order.
class BootClock extends $pb.GeneratedMessage {
  factory BootClock({
    $core.int? epochS,
  }) {
    final result = BootClock._();
    if (epochS != null) result.epochS = epochS;
    return result;
  }

  BootClock._();

  factory BootClock.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      BootClock()..mergeFromBuffer(data, registry);
  factory BootClock.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      BootClock()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'BootClock',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: BootClock.$_createMessage)
    ..aI(1, _omitFieldNames ? '' : 'epochS', fieldType: $pb.PbFieldType.OU3)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  BootClock clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  BootClock copyWith(void Function(BootClock) updates) =>
      super.copyWith((message) => updates(message as BootClock)) as BootClock;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use BootClock() / BootClock.new instead')
  static BootClock create() => BootClock._();
  static $pb.GeneratedMessage $_createMessage() => BootClock._();
  @$core.override
  BootClock createEmptyInstance() => BootClock._();
  @$core.pragma('dart2js:noInline')
  static BootClock getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<BootClock>(BootClock.$_createMessage);
  static BootClock? _defaultInstance;

  @$pb.TagNumber(1)
  $core.int get epochS => $_getIZ(0);
  @$pb.TagNumber(1)
  set epochS($core.int value) => $_setUnsignedInt32(0, value);
  @$pb.TagNumber(1)
  $core.bool hasEpochS() => $_has(0);
  @$pb.TagNumber(1)
  void clearEpochS() => $_clearField(1);
}

enum RepRecord_Body { summary, computed, clock, raw, imu, notSet }

/// One log record: a rep summary, one of either trace's chunks, a clock reading, or a
/// live IMU chunk a phone captured.
/// Keeping them under one wire type makes the log a single self-describing stream.
class RepRecord extends $pb.GeneratedMessage {
  factory RepRecord({
    RepSummary? summary,
    TraceChunk? computed,
    BootClock? clock,
    RawChunk? raw,
    ImuChunk? imu,
  }) {
    final result = RepRecord._();
    if (summary != null) result.summary = summary;
    if (computed != null) result.computed = computed;
    if (clock != null) result.clock = clock;
    if (raw != null) result.raw = raw;
    if (imu != null) result.imu = imu;
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
    2: RepRecord_Body.computed,
    3: RepRecord_Body.clock,
    4: RepRecord_Body.raw,
    5: RepRecord_Body.imu,
    0: RepRecord_Body.notSet
  };
  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'RepRecord',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: RepRecord.$_createMessage)
    ..oo(0, [1, 2, 3, 4, 5])
    ..aOM<RepSummary>(1, _omitFieldNames ? '' : 'summary',
        subBuilder: RepSummary.$_createMessage)
    ..aOM<TraceChunk>(2, _omitFieldNames ? '' : 'computed',
        subBuilder: TraceChunk.$_createMessage)
    ..aOM<BootClock>(3, _omitFieldNames ? '' : 'clock',
        subBuilder: BootClock.$_createMessage)
    ..aOM<RawChunk>(4, _omitFieldNames ? '' : 'raw',
        subBuilder: RawChunk.$_createMessage)
    ..aOM<ImuChunk>(5, _omitFieldNames ? '' : 'imu',
        subBuilder: ImuChunk.$_createMessage)
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
  @$pb.TagNumber(3)
  @$pb.TagNumber(4)
  @$pb.TagNumber(5)
  RepRecord_Body whichBody() => _RepRecord_BodyByTag[$_whichOneof(0)]!;
  @$pb.TagNumber(1)
  @$pb.TagNumber(2)
  @$pb.TagNumber(3)
  @$pb.TagNumber(4)
  @$pb.TagNumber(5)
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
  TraceChunk get computed => $_getN(1);
  @$pb.TagNumber(2)
  set computed(TraceChunk value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasComputed() => $_has(1);
  @$pb.TagNumber(2)
  void clearComputed() => $_clearField(2);
  @$pb.TagNumber(2)
  TraceChunk ensureComputed() => $_ensure(1);

  @$pb.TagNumber(3)
  BootClock get clock => $_getN(2);
  @$pb.TagNumber(3)
  set clock(BootClock value) => $_setField(3, value);
  @$pb.TagNumber(3)
  $core.bool hasClock() => $_has(2);
  @$pb.TagNumber(3)
  void clearClock() => $_clearField(3);
  @$pb.TagNumber(3)
  BootClock ensureClock() => $_ensure(2);

  @$pb.TagNumber(4)
  RawChunk get raw => $_getN(3);
  @$pb.TagNumber(4)
  set raw(RawChunk value) => $_setField(4, value);
  @$pb.TagNumber(4)
  $core.bool hasRaw() => $_has(3);
  @$pb.TagNumber(4)
  void clearRaw() => $_clearField(4);
  @$pb.TagNumber(4)
  RawChunk ensureRaw() => $_ensure(3);

  @$pb.TagNumber(5)
  ImuChunk get imu => $_getN(4);
  @$pb.TagNumber(5)
  set imu(ImuChunk value) => $_setField(5, value);
  @$pb.TagNumber(5)
  $core.bool hasImu() => $_has(4);
  @$pb.TagNumber(5)
  void clearImu() => $_clearField(5);
  @$pb.TagNumber(5)
  ImuChunk ensureImu() => $_ensure(4);
}

/// Sent by the app to request a rep's trace, which is always the whole of it.
class TraceRequest extends $pb.GeneratedMessage {
  factory TraceRequest({
    $core.List<$core.int>? repUuid,
    TraceKind? kind,
  }) {
    final result = TraceRequest._();
    if (repUuid != null) result.repUuid = repUuid;
    if (kind != null) result.kind = kind;
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
    ..aE<TraceKind>(4, _omitFieldNames ? '' : 'kind',
        enumValues: TraceKind.values)
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

  @$pb.TagNumber(4)
  TraceKind get kind => $_getN(1);
  @$pb.TagNumber(4)
  set kind(TraceKind value) => $_setField(4, value);
  @$pb.TagNumber(4)
  $core.bool hasKind() => $_has(1);
  @$pb.TagNumber(4)
  void clearKind() => $_clearField(4);
}

/// Turn the live IMU stream on or off, until the device restarts.
///
/// A dev switch, not a setting. The stream is every sample the sampler takes, sent as `ImuChunk` notifications on `Sync.Imu`; it is for watching a sensor or recording a session on the phone, not for a release session, and nothing about it is stored on the device. It lives here rather than on a characteristic of its own because it is an operation rather than a value the app reads back: the client that sets it knows which way it set it, and the device forgets at every boot.
///
/// **Nothing is detected or stored while it is on.** A detected rep ends in a flash write, and the write is a pause in the sampling: it would be a hole in the stream the switch exists to make continuous, at every rep. The switch therefore turns the detector off for as long as the stream runs and starts it again from rest when the stream ends.
///
/// **Off at every boot and not persisted.** A switch left on spends the link and the phone's battery for as long as nobody notices; the cost of remembering it would be a reflash to turn it off.
///
/// A record-mode build still stores raw traces per rep in the log (`RawChunk`); that is a build flag, not this switch.
class DevMode extends $pb.GeneratedMessage {
  factory DevMode({
    $core.bool? enabled,
  }) {
    final result = DevMode._();
    if (enabled != null) result.enabled = enabled;
    return result;
  }

  DevMode._();

  factory DevMode.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      DevMode()..mergeFromBuffer(data, registry);
  factory DevMode.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      DevMode()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'DevMode',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: DevMode.$_createMessage)
    ..aOB(1, _omitFieldNames ? '' : 'enabled')
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  DevMode clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  DevMode copyWith(void Function(DevMode) updates) =>
      super.copyWith((message) => updates(message as DevMode)) as DevMode;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use DevMode() / DevMode.new instead')
  static DevMode create() => DevMode._();
  static $pb.GeneratedMessage $_createMessage() => DevMode._();
  @$core.override
  DevMode createEmptyInstance() => DevMode._();
  @$core.pragma('dart2js:noInline')
  static DevMode getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<DevMode>(DevMode.$_createMessage);
  static DevMode? _defaultInstance;

  @$pb.TagNumber(1)
  $core.bool get enabled => $_getBF(0);
  @$pb.TagNumber(1)
  set enabled($core.bool value) => $_setBool(0, value);
  @$pb.TagNumber(1)
  $core.bool hasEnabled() => $_has(0);
  @$pb.TagNumber(1)
  void clearEnabled() => $_clearField(1);
}

enum SyncRequest_Body { trace, devMode, notSet }

/// Everything sent app -> device on the sync service.
class SyncRequest extends $pb.GeneratedMessage {
  factory SyncRequest({
    TraceRequest? trace,
    DevMode? devMode,
  }) {
    final result = SyncRequest._();
    if (trace != null) result.trace = trace;
    if (devMode != null) result.devMode = devMode;
    return result;
  }

  SyncRequest._();

  factory SyncRequest.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      SyncRequest()..mergeFromBuffer(data, registry);
  factory SyncRequest.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      SyncRequest()..mergeFromJson(json, registry);

  static const $core.Map<$core.int, SyncRequest_Body> _SyncRequest_BodyByTag = {
    1: SyncRequest_Body.trace,
    2: SyncRequest_Body.devMode,
    0: SyncRequest_Body.notSet
  };
  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'SyncRequest',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: SyncRequest.$_createMessage)
    ..oo(0, [1, 2])
    ..aOM<TraceRequest>(1, _omitFieldNames ? '' : 'trace',
        subBuilder: TraceRequest.$_createMessage)
    ..aOM<DevMode>(2, _omitFieldNames ? '' : 'devMode',
        subBuilder: DevMode.$_createMessage)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  SyncRequest clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  SyncRequest copyWith(void Function(SyncRequest) updates) =>
      super.copyWith((message) => updates(message as SyncRequest))
          as SyncRequest;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use SyncRequest() / SyncRequest.new instead')
  static SyncRequest create() => SyncRequest._();
  static $pb.GeneratedMessage $_createMessage() => SyncRequest._();
  @$core.override
  SyncRequest createEmptyInstance() => SyncRequest._();
  @$core.pragma('dart2js:noInline')
  static SyncRequest getDefault() =>
      _defaultInstance ??= $pb.GeneratedMessage.$_defaultFor<SyncRequest>(
          SyncRequest.$_createMessage);
  static SyncRequest? _defaultInstance;

  @$pb.TagNumber(1)
  @$pb.TagNumber(2)
  SyncRequest_Body whichBody() => _SyncRequest_BodyByTag[$_whichOneof(0)]!;
  @$pb.TagNumber(1)
  @$pb.TagNumber(2)
  void clearBody() => $_clearField($_whichOneof(0));

  @$pb.TagNumber(1)
  TraceRequest get trace => $_getN(0);
  @$pb.TagNumber(1)
  set trace(TraceRequest value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasTrace() => $_has(0);
  @$pb.TagNumber(1)
  void clearTrace() => $_clearField(1);
  @$pb.TagNumber(1)
  TraceRequest ensureTrace() => $_ensure(0);

  @$pb.TagNumber(2)
  DevMode get devMode => $_getN(1);
  @$pb.TagNumber(2)
  set devMode(DevMode value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasDevMode() => $_has(1);
  @$pb.TagNumber(2)
  void clearDevMode() => $_clearField(2);
  @$pb.TagNumber(2)
  DevMode ensureDevMode() => $_ensure(1);
}

enum SyncMessage_Body { summary, computed, raw, notSet }

/// Everything sent device -> app on the sync service, on the pipe.
///
/// A response and a push look alike here, and the client tells them apart by the body: `computed` or `raw` belongs to the `TraceRequest` that asked for it (its chunks carry the rep's UUID and the kind that was asked for), while `summary` arrives because the client subscribed, not because it asked.
class SyncMessage extends $pb.GeneratedMessage {
  factory SyncMessage({
    RepSummary? summary,
    TraceChunk? computed,
    RawChunk? raw,
  }) {
    final result = SyncMessage._();
    if (summary != null) result.summary = summary;
    if (computed != null) result.computed = computed;
    if (raw != null) result.raw = raw;
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
    2: SyncMessage_Body.computed,
    4: SyncMessage_Body.raw,
    0: SyncMessage_Body.notSet
  };
  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'SyncMessage',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: SyncMessage.$_createMessage)
    ..oo(0, [1, 2, 4])
    ..aOM<RepSummary>(1, _omitFieldNames ? '' : 'summary',
        subBuilder: RepSummary.$_createMessage)
    ..aOM<TraceChunk>(2, _omitFieldNames ? '' : 'computed',
        subBuilder: TraceChunk.$_createMessage)
    ..aOM<RawChunk>(4, _omitFieldNames ? '' : 'raw',
        subBuilder: RawChunk.$_createMessage)
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
  @$pb.TagNumber(4)
  SyncMessage_Body whichBody() => _SyncMessage_BodyByTag[$_whichOneof(0)]!;
  @$pb.TagNumber(1)
  @$pb.TagNumber(2)
  @$pb.TagNumber(4)
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
  TraceChunk get computed => $_getN(1);
  @$pb.TagNumber(2)
  set computed(TraceChunk value) => $_setField(2, value);
  @$pb.TagNumber(2)
  $core.bool hasComputed() => $_has(1);
  @$pb.TagNumber(2)
  void clearComputed() => $_clearField(2);
  @$pb.TagNumber(2)
  TraceChunk ensureComputed() => $_ensure(1);

  @$pb.TagNumber(4)
  RawChunk get raw => $_getN(2);
  @$pb.TagNumber(4)
  set raw(RawChunk value) => $_setField(4, value);
  @$pb.TagNumber(4)
  $core.bool hasRaw() => $_has(2);
  @$pb.TagNumber(4)
  void clearRaw() => $_clearField(4);
  @$pb.TagNumber(4)
  RawChunk ensureRaw() => $_ensure(2);
}

/// What the device says when something goes wrong, pushed on the `Reports` characteristic (`proto/calypso_gatt.proto`).
///
/// **A kind and nothing else.** The device's own sentence stays in its console log; the app turns the kind into words. Text was the other shape considered and it loses: a sentence long enough to be useful does not fit the payload every link can carry, so it would have to be truncated to whatever the current link takes, and a sentence that differs by link is one the app cannot label, count or recognise. The detail underneath a kind (which flash error, which rep) is what a serial console is for.
///
/// These are **events, not state**, which is why the characteristic is notify-only: a fault is reported when it happens, there is no "and it is fine now", and nothing reads a latest one back — a latch the device has to call current is a state it cannot know. A client that was not subscribed hears nothing, so the device holds what it raised while nobody was listening and sends it when a subscriber appears.
///
/// The app owns every sentence, so a kind names a **class** of failure and not an incident: adding detail on the device side must not change which kind it reports. The app counts what arrives.
class Report extends $pb.GeneratedMessage {
  factory Report({
    ReportKind? kind,
  }) {
    final result = Report._();
    if (kind != null) result.kind = kind;
    return result;
  }

  Report._();

  factory Report.fromBuffer($core.List<$core.int> data,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      Report()..mergeFromBuffer(data, registry);
  factory Report.fromJson($core.String json,
          [$pb.ExtensionRegistry registry = $pb.ExtensionRegistry.EMPTY]) =>
      Report()..mergeFromJson(json, registry);

  static final $pb.BuilderInfo _i = $pb.BuilderInfo(
      _omitMessageNames ? '' : 'Report',
      package: const $pb.PackageName(_omitMessageNames ? '' : 'calypso'),
      createEmptyInstance: Report.$_createMessage)
    ..aE<ReportKind>(1, _omitFieldNames ? '' : 'kind',
        enumValues: ReportKind.values)
    ..hasRequiredFields = false;

  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Report clone() => deepCopy();
  @$core.Deprecated('See https://github.com/google/protobuf.dart/issues/998.')
  Report copyWith(void Function(Report) updates) =>
      super.copyWith((message) => updates(message as Report)) as Report;

  @$core.override
  $pb.BuilderInfo get info_ => _i;

  @$core.pragma('dart2js:noInline')
  @$core.Deprecated('Use Report() / Report.new instead')
  static Report create() => Report._();
  static $pb.GeneratedMessage $_createMessage() => Report._();
  @$core.override
  Report createEmptyInstance() => Report._();
  @$core.pragma('dart2js:noInline')
  static Report getDefault() => _defaultInstance ??=
      $pb.GeneratedMessage.$_defaultFor<Report>(Report.$_createMessage);
  static Report? _defaultInstance;

  @$pb.TagNumber(1)
  ReportKind get kind => $_getN(0);
  @$pb.TagNumber(1)
  set kind(ReportKind value) => $_setField(1, value);
  @$pb.TagNumber(1)
  $core.bool hasKind() => $_has(0);
  @$pb.TagNumber(1)
  void clearKind() => $_clearField(1);
}

const $core.bool _omitFieldNames =
    $core.bool.fromEnvironment('protobuf.omit_field_names');
const $core.bool _omitMessageNames =
    $core.bool.fromEnvironment('protobuf.omit_message_names');
