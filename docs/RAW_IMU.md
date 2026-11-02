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
from rotation, and the accelerometer only has to be trusted while the bar is still — which is
when it is trustworthy.

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
from bootstrapping its own bias, so the offsets are not carried over. We estimate them at rest
instead, which costs nothing and is measured in the conditions the bar is actually in.

What replaces it is better than a routine: at rest the accelerometer reads 1 g on the up axis,
so gravity's direction and both biases arrive from a still window with no user participation.
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

- **Attitude** integrates the gyro every sample. While the bar is at rest it is pulled toward the
  direction the accelerometer measures, with a time constant around a second: the gyro carries
  tilt through a stroke, and the accelerometer is only believed when it cannot be wrong. At a
  boot there is no attitude worth protecting, so it is **set** rather than filtered — one exact
  rotation onto the measured gravity — because a filter leaves degrees of error and a degree of
  tilt is a sixth of a metre per second squared of gravity left in the horizontal.
- **Velocity** integrates the linear acceleration, and is set to zero whenever the bar is at rest
  (ZUPT). That is what bounds drift to within one rep, and a lift guarantees the conditions at
  both ends of every stroke.
- **Offsets** are learned from a window that has stopped changing, with both time constants set
  by whether the filter has finished booting.

**Three predicates, not one**, and the difference between them is the design. Landing on one
question is what a first attempt at this does, and it fails in both directions at once —
recorded here because the map is what found it.

| predicate | what it asks | what it is for |
|---|---|---|
| quiet | the reading has not changed across 400 ms, and nothing is turning | learning, while the offset is unknown |
| resting | the reading, with the offset taken out, is 1 g, and nothing is turning | the bar is not accelerating |
| still | resting for 400 ms | the bar is at rest |

Asking "is it 1 g" to decide whether the offset may be learned is circular — an unknown offset
makes a still sensor read something else — so the learning uses the spread instead. Asking
"has it stopped changing" to decide whether the bar is at rest is wrong the other way: a slow
stroke spends a third of a second crossing zero acceleration at its fastest point, where it
reads 1 g and barely moves, and calling that a rest caps the stroke in half. Only the wait tells
them apart, and everything downstream — the trace's anchoring, a leg's close, the detector's
idea of a hold — hangs off the third.

Nothing here consults the chip's calibration or a config register. It also deletes the
body-frame/world-frame split in `Detector`: bias was learned in the world frame and subtracted
in the body frame, and velocity integrated in the body frame, all to avoid trusting an
orientation estimate that is about to stop being an estimate.

## The trace and its header

The stored point keeps its shape and loses a field:

```
TraceSample { t_offset_ms, px/py/pz, ax/ay/az, gx/gy/gz }   // 0.01 m/s² and 1/16 dps per LSB
TracePoint  { t_ms, pos, accel: [i16;3], gyro: [i16;3] }    // 28 B, down from 32
Sample      { t_ms, accel: [i16;3], gyro: [i16;3] }         // 16 B, down from 20
```

`ax..az` changes meaning from *linear* acceleration to *specific force*. That is the one silent
break in the whole change — a reader that keeps the old assumption produces plausible-looking
numbers — so the accessors are renamed on both sides rather than reused.

Two messages carry what a summary needs to be reproducible, and to be decoded at all:

```
RepConfig { accel_range_g, gyro_range_dps, accel_bw, gyro_bw, sample_rate_hz, detector_version }
RepState  { v0, accel_bias, gyro_bias, attitude }
```

`RepConfig` is what interprets the numbers. `RepState` is where the filter started, which is what
makes a replay reproduce the summary instead of approximating it: the filter is deterministic
from that state, so a trace plus its header is the summary. `RepState` replaces `TraceOrigin`,
which carried only `v0`.

`flags` loses its calibrated bit. `ImuState` becomes raw accel, raw gyro and the derived attitude.
`calypso_gatt.proto` loses the `Calibration` characteristic.

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

### 2. Wire format, trace and app — todo

`TraceSample`, `RepConfig`, `RepState`, `ImuState`, the `flags` bit, the GATT table (regenerate
both sides), and on the app: the trace point model, the plot's orientation gizmo hidden for now,
`pblog` format bumped, and the calibration hint, model and state deleted.

Verified by: core and sim tests, app tests, one regenerated fixture.

### 3. Device — todo

`ACCGYRO` with the range and bandwidth set in CONFIG mode first; one burst read of accel and
gyro. `load_calibration`, the profile save, the 1 Hz calibration poll and the
`CALIBRATION_SAVED` path all go, along with `calibration.rs`; the recording gate becomes the
fusion's own readiness — seeded from a still window — rather than a level the chip reports.

**The deliberate A/B:** the detection logic — the turning points, the pairing, the thresholds —
is unchanged, and only what feeds it is new. If the device starts storing reps again, that is
direct evidence the fused output was the whole problem, and it separates "is raw plus filter
sane" from "is the detector right", which have been tangled for the last two sessions.

**Not optional:** clearing the log partition after flashing. The ring survives a reflash, and
without a wipe the app pulls old-format records and plots them as though they were new.

Verified by: a release build, fake reps through the whole path, then a real session.

The documentation moves with it, because it describes the system rather than the plan:
[`REP_TRACKING.md`](REP_TRACKING.md) for what a sample is and where the frame comes from,
[`APP_SYNC.md`](APP_SYNC.md) for the characteristics, and [`CALIBRATION.md`](CALIBRATION.md),
which becomes the story of a device settling at boot or stops being a document.

### 4. Reality — todo

One session with its rep counts known before it starts, annotated, turned into fixtures, then
thresholds re-derived and the maps re-run. Then two decisions the new format makes cheaper:
whether the trace stores full rate at 100 Hz, and whether the detector consumes the filter's
velocity.

**`pos` leaves the trace, here rather than in stage 2.** It is the only derived field in a point
— every other one is an input — and storing it freezes the trace to the filter that wrote it,
which is the thing the trace exists to avoid. Removing it is what makes full rate affordable:
a point drops to 16 bytes, so 8 s at 100 Hz is smaller in RAM than today's decimated trace,
and the log lands within a few percent of its current capacity. `TRACE_POINTS` then equals
`MAX_REP_MS` of samples, which is a buffer sized to the longest legal rep, so the decimation
path goes rather than being tuned — the stride, the grid anchored at the last sample, and
`REP_FLAG_DECIMATED` with it.

It waits until this stage because it is the app that is forced to integrate: with the gizmo
deferred, `pos` is the only thing the client needs a filter for. Keeping it through stage 3
means the plot draws without new client-side maths, and the first real capture gets a
cross-check that only becomes exact once `RepState` exists — re-run the filter over the stored
window from its own state and compare it against the stored position.

What it costs is a second implementation of the filter, in Dart — a parallel source of truth —
so the port is pinned by vectors generated from the Rust crate and checked in, rather than by
the port agreeing with itself. The alternative is that the device stays the only integrator,
and a trace can never be re-derived with a better one; that is the trade, and it is taken the
way the trace being the archive implies.

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
- **The `bno055` crate may not expose ACCGYRO reads usefully.** Worth checking before stage 3 is
  sized: if there is no raw path, it is a small register read behind the same trait, not a
  redesign.

## Status

- **Stage 1: done.** The filter is in `calypso-core`, the simulator replays it, and the four
  tests that could not be written before are in place. Everything downstream still reads the old
  wire format, so the device builds and the app is untouched.
- **Stage 2: todo.** Wire format, trace header, app.
- **Stage 3: todo.** Device, with the input-swap A/B.
- **Stage 4: todo.** Reality, retune, and the three deferred decisions.
