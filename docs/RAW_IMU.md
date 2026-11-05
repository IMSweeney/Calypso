# Raw IMU

The device stops asking the BNO055 for a fused orientation and reads the accelerometer and
gyroscope directly, and the firmware filters them itself. This is the plan for that change.

## Why

Four things, in the order they cost us time, and then the price.

**Rest has no absolute reference in fusion mode.** The fused output's linear acceleration is
zero at rest *by definition* — that is what "gravity removed" means. So the only way to tell a
still bar from a sensor offset is whether the number is *constant*, which is why the detector's
stillness test is a spread over 400 ms rather than a magnitude. That works, but it cannot
answer "is this accelerometer reading right", and a residue the fusion left behind — a fraction
of a metre per second squared pointing at the floor, different in the body frame at every
orientation — made the detector believe it was moving for an entire session. A raw
accelerometer reads 1 g at rest, always, and that is a physical fact no estimate can drift out
of.

**Tilt comes partly from the accelerometer, exactly when the accelerometer is wrong.** In IMU
mode roll and pitch are corrected by gravity, so a stroke's own acceleration pulls the estimate
off by however many degrees `a/g` is worth. In one capture pitch swings 47–82° inside a single
rep and roll crosses ±180°, and five of fourteen reps had less vertical travel than sideways —
so ROM is measured on Z alone, because Z is the only axis that can be wrong about a
real quantity rather than accidentally right about an imaginary one. With a gyro, tilt comes
from rotation, and the accelerometer only has to supply gravity's direction — which a vertical
stroke does not tilt, so trusting it mid-stroke is safe.

**The chip's calibration is not applied in raw mode, and it was never worth much.** See below.

**It is the prerequisite for anything at 1 kHz.** Ring-down analysis, if it is ever pursued,
cannot be done on fused output. Nothing else in this plan needs 1 kHz; this is the only thing
that ever would.

**What it costs.** The quaternion: we compute it, and the app can recompute it from the trace.
The annotated captures: the gyroscope was never recorded, so no existing `.pblog` can be
replayed through the new filter, and the labels that came with them — the expensive part, not
the traces — stop being reachable by the test suite. Accepted knowingly: the archive is wiped
rather than versioned, and `sim/` keeps one format instead of two.

## What the chip gives in ACCGYRO mode

The data registers are fixed scale, *independent of the configured range*:

| quantity | register | scale | source |
|---|---|---|---|
| acceleration | `ACC_DATA_<x,y,z>` | 1 m/s² = 100 LSB | §3.6.5.1, table 3-17 |
| angular rate | `GYR_DATA_<x,y,z>` | 1 dps = 16 LSB | §3.6.5.3, table 3-22 |
| orientation | — | not available in non-fusion modes | §3.6.5.5 |
| calibration status | `CALIB_STAT` | does not advance | §3.11 |

The range setting changes two other things and neither of them is the output scale: the clip
point, and the **internal** sensor's LSB, which does scale (3.91 mg in the 2 g range, 62.5 mg
in the 16 g range, per the datasheet's interrupt-threshold scaling notes). So the range is
chosen for clipping headroom against noise, and nothing else. Both are set once at boot in
CONFIG mode — where the sensor configuration is writable, which it is not in any fusion mode
(§3.5) — and the mode change costs milliseconds, once per boot.

Chosen: **±8 g and ±500 dps.** ±8 g clips at 78 m/s², which is past a rep and past a rack
impact but not past a dropped bar; ±4 g clips at 39 m/s², which a hard rack can reach. On the
gyro, ±500 dps is past anything a barbell does and the internal sensor is quietest at the low
end. Neither choice affects the scale the trace is decoded with.

### Calibration does not exist here

Three statements from the datasheet, and one from the kernel driver:

- §3.6: "Depending on the selected operating mode the device will output either **un-calibrated
  sensor data (in non-fusion mode)** or calibrated / fused data (in fusion mode)."
- §3.6.5.1 and §3.6.5.3: in non-fusion mode the acceleration and angular-rate registers hold
  "**uncompensated**" data; only in fusion mode does "the fusion algorithm output offset
  compensated" values.
- §3.11 describes calibration as something "the sensor fusion software runs … in the
  background", and the profile-restore procedure in §3.11.4 ends by switching to a **fusion**
  mode.
- The Linux IIO driver states it directly: autocalibration runs "if (and only if) operating in
  fusion mode", and in non-fusion mode the offsets are exposed for the **host** to apply.

So in ACCGYRO there is no autocalibration, `CALIB_STAT` cannot advance, and the offsets are not
applied for us. The offset registers still exist and a saved profile can still be written and
subtracted by firmware — but a full calibration left a residue big enough to keep the detector
from bootstrapping its own bias, so the offsets are not carried over. We estimate them from the
sensor's own stillness instead, which costs nothing and is measured in the conditions the bar is
actually in.

What replaces it is better than a routine: at rest the accelerometer reads 1 g on the up axis,
so gravity's direction and the accelerometer's offset arrive from a still window with no user
participation, and the gyroscope's bias arrives from the direction of gravity, which stays still
whether the bar does or not.

The class of bug that ate a session — a gravity vector the fusion failed to subtract — cannot
be represented, because the subtraction is ours and the direction is known.

## The filter

One struct, in `calypso-core`, shared by the device and the simulator.

```
Fusion::update(&Sample) -> Filtered {
    t_ms,
    linear_accel,   // the specific force with gravity and the offset taken out, world frame
    still,          // the bar is at rest
    state,          // the whole of its memory, which is what a replay starts from
}
```

State: a quaternion, an accelerometer offset, a gyroscope offset, a velocity, and whether the
bar was at rest when the state was taken.

- **Attitude** integrates the gyro every sample, and is pulled toward the direction the
  accelerometer measures: hard while the bar is at rest, gently while it moves. A vertical
  stroke's acceleration is along gravity and does not tilt the measured direction, so the
  accelerometer is still a gravity reference mid-stroke; a bench's bar path and a swing are the
  only error, and a slow pull averages them. Correcting only while still lets a degree per
  second of gyro bias walk the vertical several degrees across a set, which is metres per second
  of horizontal velocity that the bar never had. At a boot there is no attitude worth
  protecting, so it is **set** rather than filtered — one exact rotation onto the measured
  gravity — because a filter leaves degrees of error and a degree of tilt is a sixth of a metre
  per second squared of gravity left in the horizontal.
- **Velocity** integrates the linear acceleration, and is pulled back to zero whenever the bar is
  at rest (ZUPT). That is what bounds drift to within one rep, and a lift guarantees the
  conditions at both ends of every stroke.
- **Offsets** are learned from stillness, but stillness is a different question for each. The
  accelerometer's is a window where the reading has stopped changing; the gyroscope's is a
  window where gravity's direction in the sensor's frame has not moved, since a bias leaves it
  there and a real rotation does not. A bar moving up and down is still for the gyro's purpose
  even when it is not for the accelerometer's, so a set with no rest in it still learns its
  bias.

**Four predicates, and the difference between them is the design.** Landing on one question is
what a first attempt at this does, and it fails in both directions at once — recorded here
because the map is what found it.

| predicate | what it asks | what it is for |
|---|---|---|
| quiet | the reading has not changed across 400 ms, and nothing is turning | the accelerometer's offset, and a boot |
| direction quiet | gravity's direction in the sensor's frame has not moved across 400 ms | the gyroscope's offset |
| resting | the reading, with the offset taken out, is 1 g, and nothing is turning | the bar is not accelerating |
| still | resting for 400 ms | the bar is at rest |

Asking "is it 1 g" to decide whether the accelerometer's offset may be learned is circular — an
unknown offset makes a still sensor read something else — so the learning uses the spread
instead. Asking "has it stopped changing" to decide whether the bar is at rest is wrong the
other way: a slow stroke spends a third of a second crossing zero acceleration at its fastest
point, where it reads 1 g and barely moves, and calling that a rest caps the stroke in half.
Only the wait tells them apart, and everything downstream — the trace's anchoring, a leg's
close, the detector's idea of a hold — hangs off `still`.

Nothing here consults the chip's calibration or a config register. It also deletes the
body-frame/world-frame split in `Detector`: bias was learned in the world frame and subtracted
in the body frame, and velocity integrated in the body frame, all to avoid trusting an
orientation estimate that is about to stop being an estimate.

## The trace and its header

A rep carries **two** traces. The plotted one keeps every point a screen can tell apart; the raw
one keeps every sample, and only a device that was asked records it.

```
TraceComputedPoint { t_offset_ms, px/py/pz, qw/qx/qy/qz }   // metres and unit quaternion
TraceRawPoint      { t_offset_ms, ax/ay/az, gx/gy/gz }      // 0.01 m/s² and 1/16 dps per LSB
```

`ax..az` changes meaning from *linear* acceleration to **specific force**. That is the one silent
break in the change — a reader that keeps the old assumption produces plausible-looking numbers —
so the accessors are renamed on both sides rather than reused.

Both traces cover the same window and count from the same origin, the rep's first sample, so
`RepSummary.t_start_ms + t_offset_ms` is a sample's uptime in either of them.

Two messages carry what a summary needs to be reproducible, and to be decoded at all:

```
RepConfig { accel_range_g, gyro_range_dps, accel_bw, gyro_bw, sample_rate_hz, detector_version }
RepState  { v0, accel_bias, gyro_bias, attitude, settled }
```

`RepConfig` is what interprets the numbers. `RepState` is where the filter started: the filter is
deterministic from that state, so a trace plus its header is the summary — which is what a raw trace
is for. `settled` is part of it because "at rest" is a fact about the last 400 ms, and a replay
without it earns that back and skips the first ZUPT.

The summary's counts say what it has: `computed_len`/`computed_chunks` for the plotted trace and
`raw_len`/`raw_chunks` for the raw one, where `raw_chunks > 0` is "this rep can be replayed".
`flags` is gone with its last live bit.

## Stages

### 1. Library and simulator — done

No device, no wire format, no app. The whole of the risk lives here and it is testable in a
closed loop.

- `Sample` becomes raw, with `GYRO_SCALE`. `accel_f32()` becomes `specific_force_f32()`.
- `calypso-core/src/fusion.rs` holds `Fusion`.
- `Detector::update` takes `Filtered`, and loses `BIAS_TAU_STILL_S`, `BIAS_MAX_M_S2`,
  `still_buf`, `seeded` and the body/world split.
- `compute_rep` runs the same `Fusion` over the window, seeded from a `RepState`.
- `sim/src/motion.rs` emits specific force: `accel + 1 g up`, gyro zero, attitude identity. The
  existing scenarios keep their shape, so the maps stay comparable, and two knobs are added —
  a constant mount tilt, and gyro/accel bias.

Verified by: every existing map and pin reading the same on ideal synthetic motion, which is the
check that the filter is not distorting anything; then the things that were impossible before —
a rep at 0°, 30° and 90° about each axis storing the same ROM, a gyro bias not rotating the
ground, an accelerometer bias learned while still and not fabricating motion, and a boot into
motion with no lead-in rest not storing a garbage rep.

**What the implementation changed against this plan.**

- `Filtered` carries `t_ms` and the whole state rather than a bare velocity, because the caller
  has to hand the state on and nothing else knows when it was taken.
- The state is `FusionState` here and becomes `RepState` on the wire in stage 2. It carries
  `settled`, which the plan did not list: whether the bar was at rest is a fact about the last
  400 ms, and a replay without it spends its first sample earning it back and skips the ZUPT.
- The detector's history is 640 ms, not `MARGIN_MS`, and `MARGIN_MS` is 500 ms, not 200: the
  filter only calls a bar at rest after 400 ms of it, so a window with 200 ms of rest either
  side has nothing to anchor to.
- `rep_state()` returns the state *and the moment it was taken*. The detector's history begins
  when it is first fed, which is when the filter becomes ready, so a rep close behind a boot has
  no filtered history `MARGIN_MS` back — and the window must start where the state does.
- `T_REST_MS` is gone: the debounce a rest needs is the filter's, and doing it twice made a rest
  take 800 ms to recognise when it happens in 400.
- `compute_rep` no longer detrends. The position is the filter's velocity integrated, the
  window's rest anchors both ends, and nothing is forced to zero — which is why the ROM a
  summary reports now runs a few percent over the travel rather than under it. The endpoint the
  integration leaves is the drift the ZUPT did not take out, and it is a real property of the
  window rather than an artefact of the maths. On a rep so short and fast that the margins are a
  large fraction of it, that residue is worth tens of percent, which is the trade: the old
  detrend forced the ends together and read short instead.
- The trace keeps its quaternion in this stage, computed from the filter's attitude, so the wire
  format and the app are untouched. Stage 2 swaps it for the gyro.

### 2. Wire format, trace and app — done

**Two traces, not one**, because a rep has two consumers that want opposite things.
`TraceComputed` is where the bar went and which way the sensor faced, decimated to a fixed budget
of 128 points: derived data, and the only thing a plot draws. `TraceRaw` is what the sensor read —
specific force and angular rate, every sample, never decimated — so the filter and the detector can
be run over it again and the summary re-derived. A single message could only serve one of them
badly: a format carrying both is either decimated until it cannot be replayed, or full rate, and the
plot then pays for samples it cannot draw.

The split is also what makes a stored rep smaller. It was 512 points carrying position,
acceleration and attitude; it is now 128 carrying position and attitude, so a rep is about a third
of what it was and the log holds about three times as many. The raw trace costs 8–19 KB on top for
a 4–9 s rep, which is why only a device a client has asked records one (`DevMode`, an arm of
`SyncRequest` rather than a characteristic of its own: it is an operation, and unlike the clock
nothing waits on it).

`RepState` (velocity, attitude, both offsets, and whether the bar was at rest) replaces
`TraceOrigin`, which carried only `v0`; `RepConfig` says what the sensor was set to and which
detection logic closed the rep. `flags` is gone: its one live bit was "the trace was decimated",
which at a fixed budget is true of every rep worth plotting and is derivable from the counts
anyway. `ImuState` gains the angular rate. The `Calibration` characteristic is deleted, with
`CalibrationStatus` and the app's hint, model and state.

The archive's last fixture goes with the archive: a fused-format trace cannot become a raw one, so
`real-deadlift.pblog` and the tests that read it are deleted rather than converted, and the other
fixtures are regenerated from the generators.

**What the implementation changed against this plan.** The raw trace was not recorded by the
device when this stage landed, even with `DevMode` set: its sample had no gyroscope until the chip
was read raw, and a raw trace that cannot be replayed through the filter it exists for is not worth
the log. The write and read paths were in `store` and exercised by the fixtures, so stage 3 had only
the device's own use of them to add — which it did, one stage early: dev mode records and streams a
rep's window end to end, and the buffer one is streamed from is allocated on the first request
rather than at boot.

### 3. Device — done

`ACCGYRO` with the range and bandwidth set in CONFIG mode first; one read of accel and one of
gyro, straight into the sample (`0.01 m/s²` and `1/16 dps` per LSB are the chip's register
scales, so nothing is converted). `load_calibration`, the profile save, the 1 Hz calibration
poll and the `CALIBRATION_SAVED` path all go, along with `calibration.rs`; the recording gate
becomes the fusion's own readiness — seeded from a still window — rather than a level the chip
reports.

**The deliberate A/B:** the detection logic — the turning points, the pairing, the thresholds —
is unchanged, and only what feeds it is new. If the device starts storing reps again, that is
direct evidence the fused output was the whole problem, and it separates "is raw plus filter
sane" from "is the detector right", which have been tangled for the last two sessions.

**Not optional:** clearing the log partition after flashing. The ring survives a reflash, and
without a wipe the app pulls old-format records and plots them as though they were new.

Verified by: a release build, fake reps through the whole path, then a real session.

**What the implementation changed against this plan.**

- **The gyroscope keeps the chip's defaults — 2000 dps at 32 Hz — rather than the ±500
  chosen above.** The driver can set the accelerometer's configuration and not the
  gyroscope's (`GYR_Config`, 0x0F, is not exposed and its register writes are private), and
  the plan's reason for ±500 — the internal sensor is quietest at the low end — buys a few
  milli-dps of quantisation on a signal that never comes near either clip point. The chip's
  2000 dps cannot clip a barbell, and the register's *output* scale does not move with the
  range, so a trace decodes the same either way. It is recorded as 2000, not left at 0.
- **The accelerometer's bandwidth is written explicitly** (62.5 Hz, recorded as 63, which is
  the number the chip's own tables use for that step) even though it is the chip's default:
  a summary that claims a setting is a summary that should not be describing a default
  someone could change. ±8 g is set above it, for the clip headroom the plan argues for, and
  read back before the mode switch — the write reports only that the bus took it.
- **The level verdict moved onto the filter.** A non-fusion mode has no gravity vector to
  read (`GRV_DATA` is a fusion output), so `Sensor is level on the X axis` is now the world's
  up axis carried into the sensor's frame by the estimated attitude. Same number, same
  threshold, from an estimate pulled onto gravity continuously and hard at rest.
- **`CALIBRATION.md` stops being a document**, and its one surviving fact — nothing is
  recorded until the filter has settled, which takes a second of stillness — is in
  [`REP_TRACKING.md`](REP_TRACKING.md) where the sampling path is described.
- **The app needed no change at all.** Its half of the calibration surface went in stage 2,
  and nothing else it reads moved: `ImuState`'s quaternion is the filter's now rather than
  the chip's, at the same scale and in the same frame.
- **The chip hands over a new reading on every poll.** It was open whether a non-fusion mode
  runs at the rate it is polled at, since the datasheet's table pins the rate for fusion modes
  only and a poll that outran the sensor would read the same register twice. Counted on a still
  board: no two consecutive reads were identical in twelve thousand samples, so the 100 Hz poll
  is neither oversampling the chip nor missing an update. Stillness is where that is easiest to
  see — a sensor that repeats gives the same bytes back, and the sensor's own noise is what
  changes them — so it says nothing yet about how the rate behaves through a stroke.

  **The test does not survive raw data.** A still window off the ACCGYRO path repeats its
  previous reading 13–41% of the time per axis, which is what a one-milligram quantiser does to a
  noise of one or two of them; the count was taken while the sample still came from the fused
  output, whose own filtering leaves nothing to quantise. So a repeated reading cannot tell "the
  chip did not update" from "the value landed in the same milligram", and the question is open
  again for this path.

### 4. Reality — todo

One session with its rep counts known before it starts, annotated, turned into fixtures, then
thresholds re-derived and the maps re-run. And the one decision the new format leaves: whether the
detector consumes the filter's velocity.

**The two that used to be here are answered by the split.** `pos` no longer has to leave the trace
— it moved to the one it belongs to, the derived one that only a plot reads — and full rate no
longer has to be afforded against a plot's budget, because the raw trace is where full rate lives
and only a device that was asked records one. The decimation path stays, but only over the plotted
trace, where it is the format rather than a compromise.

## Open

- **Two integrators, deliberately.** The detector keeps its own velocity estimate through stage
  3 and the filter has another, and they will disagree. That is the A/B, and it means the first
  session's numbers are diagnostic rather than final. Folding the detector onto the filter's
  velocity is a stage 4 decision, taken on data.
- **Boot during motion.** The filter needs a still window to seed gravity, so anything before one
  is not recordable. Same gate as today, with a simpler story for the person holding the phone
  than the six-orientation routine. `sim` should pin that a boot into motion recovers at the
  first rest rather than storing a fiction.
- **Yaw is unobservable** and always has been: no magnetometer, by design. A gyro bias leaves a
  slow heading drift, which is the mount's frame and not the room's, so it costs the plot's
  heading and nothing else.
- **The annotated captures are unreachable until stage 4.** In the interim the detector is tuned
  against synthetic motion alone, so stage 4 should not slip far behind stage 3.
- **The gyroscope's range is the chip's default** and cannot be set through the driver. If the
  noise ever matters, it is a register write behind the same trait (`Destroy`, one I2C
  transaction, and the chip's config mode), not a redesign.
- **The `bno055` crate does expose ACCGYRO reads** — `accel_data_fixed` and `gyro_data_fixed`,
  at the scales this sample format already uses. The unknown that answered *no* was the
  gyroscope's configuration, above.

## Status

- **Stage 1: done.** The filter, and the simulator that measures it.
- **Stage 2: done.** Both traces, the summary's header, the sync protocol, the device and the app.
- **Stage 3: done.** The device reads the chip raw, filters it here, and records nothing until
  the filter has settled. The input swap is the A/B, and it is unrun: no session has been
  recorded on the new path yet.
- **Stage 4: todo.** Reality, retune, and whether the detector consumes the filter's velocity.
