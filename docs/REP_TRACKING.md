# Rep tracking

How a barbell rep gets from the IMU to a stored, plottable trace.

```
BNO055 (ACCGYRO, 100 Hz)
   │  I²C @ 400 kHz
   ▼
Sampler ──► RAM ring (~13 s of samples)
   │
   ▼
Rep detector (turning points, ZUPT at rest)
   │  rep complete → slice [start-margin, end+margin] out of the ring
   ▼
Trace computation: world-frame displacement + orientation, summary stats
   │
   ▼
Flash log (ring of 4 KB sectors) ──► BLE sync to the app
```

## Sampling

- The BNO055 runs in **ACCGYRO**: its accelerometer and gyroscope registers, raw and
  uncompensated, no fusion and no magnetometer (unreliable next to a barbell's
  steel). There is no orientation from the chip — the firmware computes it, which is
  what makes a still bar read 1 g instead of an estimate's idea of zero
  ([`RAW_IMU.md`](RAW_IMU.md)).
- I²C at 400 kHz. One sample is two 6-byte register reads, so under a millisecond of
  the 10 ms period.
- Sample format, fixed point, 16 bytes:
  `{ t_ms: u32, accel: [i16; 3], gyro: [i16; 3] }` — accel at 0.01 m/s² per LSB and
  gyro at 1/16 dps per LSB, which are the chip's own register scales: the device
  copies the two readings into the sample and converts nothing. The acceleration is
  **specific force**, gravity still in it, so a still sensor reads 1 g.
  `t_ms` is monotonic device uptime; the device has no RTC, so a rep's wall-clock
  time comes from the boot's clock, which the app sets ([`APP_SYNC.md`](APP_SYNC.md)).
- The range settings do not change those scales — only what clips, and how noisy the
  sensor is inside the chip. A rep's summary carries what they were
  (`RepConfig`), which is why a trace can be read without assuming today's build.
- Samples go into a RAM ring of `SAMPLE_RING_LEN` = 13 s (`REP_WINDOW_SAMPLES` = 1000
  = 10 s, which is the longest window a rep can ask for with its 500 ms margins,
  plus 3 s headroom).
- The sampler also logs a level verdict once a second (`Sensor is level on the X
  axis` — how the device gets levelled). The chip has no gravity vector to read in a
  non-fusion mode, so that is the world's up axis carried into the sensor's frame by
  the filter's attitude: the same number the fusion used to report, from an estimate
  corrected only while the bar is still.

**Nothing is recorded until the filter has settled.** It seeds gravity and both
sensor offsets from a still window, so before that there is no vertical for a rep to
be measured against — a trace taken then would be fiction rather than a measurement.
The sampler logs `imu settling` while the gate is shut and `imu settled` when it
opens, both only when the state changes. This is the whole of what used to be a
calibration ritual: hold the bar still for a second, and the filter knows which way
is down.

## Detecting a rep

`calypso_core::rep::Detector`, fed one sample at a time, asks the question in terms of
travel rather than momentum. **A rep is an out-and-back**: the bar goes more than
`MIN_ROM_M` one way, turns, and comes more than `MIN_ROM_M` back.

```
turn: the bar's vertical acceleration has turned around — a trough where it stopped rising, a
      peak where it stopped falling — and has come back TURN_BACK from it; or, for a stroke
      too gentle to move the acceleration, the vertical speed falls below VEL_MIN for T_TURN_MS
leg:  the bar's travel between two turns, and which way it went
rep:  two consecutive legs, opposite directions, each past MIN_ROM_M
```

**A turn is read on the vertical, not on the whole velocity**, because the vertical is the
only axis a rep is measured on. The frame's horizontal residue — a couple of degrees of
attitude, which is what an accelerometer offset leaves behind — integrates to more than
`VEL_MIN` inside one stroke, and a turn read off the whole speed would then never happen.
The bar turns along Z; where the estimate thinks it is heading in X and Y is not part of
the rep.

**And it is read on the acceleration, not on the velocity**, because the velocity is an
integral and the attitude is only corrected while the bar is still. Over one stroke that
residue is a few tenths of a metre per second and the vertical speed still crosses zero; over
a set with no rest in it the same residue accumulates, and the speed passes the whole set
without ever coming near zero. A reading cannot do that: the acceleration swings ten metres
per second squared either side of a turn whatever the integral has drifted to, so its extreme
is the turn, and the turn says which way the leg went. The velocity keeps its say for the one
case the acceleration cannot see — a deliberate negative whose acceleration barely moves —
which is what `VEL_MIN` is still for.

The detector integrates each sample's acceleration (rotated into the world frame) into a
velocity, which is zero wherever the bar stops — at the bottom and at the top of every
rep, whether it dwells there or not. A pause is just a turn that lasts longer, so a set
nobody pauses through splits at each end the same way a slow one does. Tunables are
constants at the top of `calypso-core/src/rep.rs`.

**Travel is part of detection, not a check afterwards.** A leg too short to be half a
rep is never kept, so it cannot pair with anything, and it cannot consume the leg before
it: a bump, a re-rack and a hand on the rack (the one capture had four spans of 0.03–0.09 m
among ten real reps) are rejected by the same measurement that accepts a press, and the few
centimetres a bar settles back at a turnaround are motion that leaves the drive still
waiting for its return. A one-way push tall enough to be a leg is
dropped when the bar rests and then sets off the same way again — that is an unrack, not
a rep. The summary is still checked against `MIN_ROM_M` when it is stored, against the
travel the trace computes rather than the one integrated on the way in.

**A stall mid-drive is one rep with its whole travel.** The bar stopping without coming
down splits a leg in two; two same-direction legs with no rest between them are the same
stroke, so their travel is summed. After a rest they are not — that is a hold, and the
older leg is the one that was already finished.

**What it cannot see is a stroke slower than `VEL_MIN`.** A leg ends on a turn, and a turn
is a *speed*: a return that never reaches 0.07 m/s at its peak never turns, so it never
closes a rep. For a bench that boundary sits between 0.12 and 0.14 m/s, and it is accepted
for now (`ROADMAP.md`, and the test that pins it is the one to delete when it is fixed).

**The velocity is integrated in the world frame.** The offset is learned, and the velocity
anchored to zero, only while the bar is still: a bias learned from motion is the rep taken
out of itself, and a velocity damped mid-rep is the same mistake one step later. The
detector keeps its own integrator rather than taking the filter's, deliberately, so that a
change in what the device stores can be attributed to the detection or to the filter.

## Computing a rep

On rep close, the sampler copies `[start - MARGIN_MS, end + MARGIN_MS]` out of the
ring and calls `rep::compute_rep`, which:

1. Runs the filter ([`fusion.rs`]) over the window from the state the detector recorded
   where the window begins, so the rotation, the offsets and the anchoring are the ones
   the device had, and the replay reproduces the summary rather than approximating it.
2. Integrates it, **anchored at both ends by ZUPT**: velocity is forced to zero at
   rest, so double-integration drift is bounded within one rep rather than
   accumulating over a set. The result is displacement relative to the rep start, in
   metres — which is what a bar-path plot needs. Absolute position is not available
   from a MEMS IMU and is not attempted.
3. Computes the summary stats: duration, ROM (peak-to-peak **vertical** travel), peak
   and mean velocity, peak acceleration, and roll/pitch min/max in degrees.

**ROM is measured on Z, not on whichever axis moved furthest.** A barbell lift is the
bar going up and down, and Z is the only axis that knows which way up is — it is pinned
to gravity. X and Y are the mount's heading, fixed at power-up and drifting with yaw, so
an excursion there is as likely to be the estimate's error as the bar: the ends of the
window are anchored by the filter, which cancels a *constant* bias but cannot remove an
orientation that wanders during the rep — real reps in one capture swung 47–82° of pitch,
and that double-integrates into metres. (The raw path is what stops the estimate from
wandering; see `RAW_IMU.md`.) Five of the fourteen reps in one capture had a horizontal axis as the furthest
from its start, so their ROM described a direction the bar never travelled. Picking the
largest axis picks the error; the horizontal ranges are a check on the frame, not a
range of motion.

Z is not immune — the same capture's largest ROM, 1.28 m, is on Z, and the workout it
came from was bench and deadlift, so the bar it describes travelled about 0.6 m. The
difference is that Z is wrong about a real quantity and shows up in the number, while a
right X is an accident that looks plausible. Where the extra travel comes from is the
hole: that rep is missing 2.3 s of the 5.6 s it claims, and the integration bridges the
gap as a single step, which adds 2.3 s of whatever acceleration sat on either side of it
to the velocity at once. A gap is motion that was not measured — see `ROADMAP.md` and the
can/must distinction this file's estimator does not yet make.

The trace is capped at **512 points** (`TRACE_POINTS`). When a rep is longer than
that, it is decimated and `RepSummary.decimated` is set. The decimation grid is
anchored at the **last** sample, so a rep's end is always stored — the end is one of
the two samples you judge a bar path by.

Both the derived displacement **and** the raw inputs (specific force, attitude) are
stored, so the app can re-derive traces with better algorithms later without a firmware
update.

## The frame

`pos` is in the IMU's **world frame**: the frame the fusion reports the device's
orientation against. What that frame means is decided by gravity and by the clip.

- **Z is up, and a squat travels down it.** Roll and pitch are absolute because
  gravity is, so this is the vertical axis on every device and a descent is negative
  Z. No magnetometer is involved, and the app draws it upward.
- **X is along the bar and Y is forward/back.** Heading is the one angle with no
  absolute reference here: the fusion starts in whatever frame the device was in when
  it started and drifts from there, so the horizontal axes are the mount's, not the
  room's. The device is clipped with its **X axis along the bar**, which is what puts
  the bar's direction in X and the lifter's forward/back in Y. Which of the two ways
  along the bar +X runs is the clip's business as well: a sideways distance is
  meaningful, "left" is not.
- **Which of the device's axes points up is the clip's business, not the trace's.**
  Clipping it with Y up instead of Z is a rotation about the bar, and the quaternion
  takes the motion back out of the body frame, so the trace is the same either way.
  The device's own axes turn up only in the app's orientation gizmo.

The firmware's level verdict reads gravity on the **X** axis — the same convention
from the other end: level means X is horizontal, and X along a horizontal bar is
level. It cannot see *which* horizontal direction X points, so it is the clip's
geometry that puts X along the bar rather than across it.

## What gets stored

One rep is one `RepSummary` record followed by its `computed_chunks` `TraceChunk` records and,
on a device that was asked to record them, its `raw_chunks` `RawChunk` records — all carrying the
same `rep_uuid`:

```
[RepSummary][TraceChunk 0]…[TraceChunk n][RawChunk 0]…[RawChunk m]
```

The plotted chunks come first and the raw ones directly after, which is what lets either trace be
found from the summary's record id alone ([`RAW_IMU.md`](RAW_IMU.md)).

A `BootClock` record sits between the reps: it is what a boot's reps are dated by,
written when the boot mounts and replaced by the client's write when it connects
([`APP_SYNC.md`](APP_SYNC.md)). A record's clock is the newest one at or before it.

- A chunk holds up to 48 points of either trace (`pb::TRACE_CHUNK_SAMPLES`), so an
  encoded record always fits inside one 4 KB erase sector (the log's record limit is
  4076 bytes; a worst-case plotted chunk encodes to ~2 KB, a raw one to ~1.4 KB). A
  plotted trace is at most 128 points — three chunks — and a raw one is one chunk per
  48 samples, so a nine-second rep is nineteen of them.
- Records live in the `log` data partition (0x190000, 2.4 MB), a ring of 4 KB sectors
  that erases the oldest sectors on wrap. At a few KB per rep that is a few hundred
  reps before the oldest fall off.
- A sector erase stops the whole chip for ~45 ms, so the ring is kept a set's worth of
  sectors ahead: already-erased sectors are made during a rest, one per tick, and an
  append reaching them writes and nothing else. `RESERVE_SECTORS` is a minute of samples
  and is also a minute's worth of history given up — those sectors are the oldest.
- Records never span sectors; see `calypso-core/src/flashlog.rs` for the layout, CRC
  and power-loss behaviour.

**Rep identity is a v4 UUID minted on the device when the rep closes.** Random,
because a rep can close before any client has set the device's clock, so an identity
derived from time would need a second form for those; and because random ids need no
coordination between devices. The price is that UUIDs do not sort — a rep's place in
time comes from the clock its boot was given, which the summary carries as
`t_unix_ms`.

Reps only need to survive on the device until the next sync — the app is the archive
of record, so a ring wrap is harmless. See [`APP_SYNC.md`](APP_SYNC.md).

## Delivery to the app

Summaries auto-stream as soon as an app subscribes; traces stream on request, one
stored `TraceChunk` per transfer, cut into pipe chunks and reassembled by the client.
The protocol, the GATT table and the client side are in [`APP_SYNC.md`](APP_SYNC.md).

What streams is only what the device can date: the client writes its clock on every
connection, and until it has, this boot's summaries wait rather than going out
without a time. Reps stored before that write in an earlier boot are dated by it if a
session saw them, and never offered if none did.
