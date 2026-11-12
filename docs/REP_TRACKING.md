# Rep tracking

How a barbell rep gets from the IMU to a stored, plottable trace.

```
BNO055 (ACCGYRO, 100 Hz)
   │  I²C @ 400 kHz
   ▼
Sampler ──► RAM ring (~13 s of samples)
   │
   ▼
Rep detector (a speed's swings, leak-anchored)
   │  rep complete → slice [the turn it opened at, end+margin] out of the ring
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
- A sample past the configured clip is not a reading, and the filter refuses it rather than
  integrating it; the device's stream carries a few corrupt samples a session.
  [`GOTCHAS.md`](GOTCHAS.md) has the evidence.
- Samples go into a RAM ring of `SAMPLE_RING_LEN` = 13 s (`REP_WINDOW_SAMPLES` = 1000
  = 10 s, which is the longest window a rep can ask for with its 500 ms margins,
  plus 3 s headroom).
- The sampler also logs a level verdict once a second (`Sensor is level on the X
  axis` — how the device gets levelled). The chip has no gravity vector to read in a
  non-fusion mode, so that is the world's up axis carried into the sensor's frame by
  the filter's attitude: the same number the fusion used to report, from an estimate
  pulled onto gravity continuously and hard at rest.

**Nothing is recorded until the filter has settled.** It seeds gravity and both
sensor offsets from a still window, so before that there is no vertical for a rep to
be measured against — a trace taken then would be fiction rather than a measurement.
The sampler logs `imu settling` while the gate is shut and `imu settled` when it
opens, both only when the state changes. This is the whole of what used to be a
calibration ritual: hold the bar still for a second, and the filter knows which way
is down.

## Detecting a rep

`calypso_core::rep::Detector`, fed one sample at a time, asks the question in terms of the
bar's **speed**. **A rep is an out-and-back of that speed**: the bar sets off past
`SWING_MPS` one way, turns, and comes back past it the other.

```
speed: the vertical acceleration integrated, and pulled back toward zero with SPEED_TAU_S
swing: two threshold crossings in opposite directions, MIN_SWING_MS .. MAX_SWING_MS apart
rep:   one swing
```

**The leak is what makes the speed worth asking.** An integral cannot know its own error, so
an unfiltered one walks away as soon as the acceleration carries a bias — the fusion allows a
couple of degrees of tilt, and `g·sin 2°` is 0.34 m/s² held for as long as the set lasts. A
set is twenty seconds with no rest in it long enough to correct that, so the residue outgrows
the stroke it is measuring. Pulling the integral toward zero makes the speed a measurement of
*now*: a bar that is not moving has no speed, continuously, without a rest having to be judged
and without waiting for one.

What that costs is the slowest strokes. The leak attenuates them — the slower the stroke, the
less speed it leaves behind — so `SWING_MPS` is a floor under what counts as a rep, and a
stroke the bar makes beneath it is not one. That is the whole of what the detector cannot see,
and it is why the bar is where it is (`ROADMAP.md`, `GOTCHAS.md`).

**The out-and-back is in the measurement, not a check made after it.** The speed cannot pass
one side and then the other without the bar having gone somewhere and come back, so there is no
travel to measure, no turn to name off an acceleration extreme, and no legs to pair. A bump, a
hand on the rack and a re-rack are refused by the same measurement that accepts a press: the
bar does not reach the speed they would need on both sides. `MIN_ROM_M` no longer decides
anything at detection time — it is asked of the trace a stored rep carries, which is what the
app plots.

**A rep is measured on the vertical**, and only the vertical. The frame's horizontal residue is
as much as a couple of degrees of attitude, which is what the leak above is for; where the
estimate thinks the bar is heading sideways is not part of a rep. The detector integrates all
three axes, because the log prints the whole speed, and counts swings on Z alone.

**The turns a replay is anchored at are the velocity's own quiet.** A turn is an instant the bar
was genuinely stopped, which is where a replay can put its velocity back to zero without lying
about the motion, so the detector names three of them from the vertical velocity the replay will
integrate: the turn behind the first half, which is the quietest that velocity has been since the
stroke before it ended — or, for the first stroke the detector sees, in the `MARGIN_MS` before
its crossing; the turn between the halves, which is the quietest since the first half crossed;
and the end, which is the same question asked of the second. This is the one part of a rep a
velocity can name better than a reading can: a minimum is the turn however much the integral has
drifted.

**A one-way push is a rep to it, if the bar comes back.** An unrack that pauses, followed by
the set's first stroke, reads as out, turn, back: the speed crossed the threshold going up, the
bar turned at the top, and it crossed the other way coming down. What separates that from a
rep is the travel the push leaves behind — the bar ends above where it started — and this
detector does not measure travel. `ROADMAP.md` has it as open and `sim/tests/sets.rs` pins it:
the count is right, the stored window and its ROM are not.

**The velocity is integrated in the world frame**, from the acceleration the filter has
already de-biased. Those offsets are learned the way the filter learns them: the
accelerometer's only while the bar is still, because an offset learned from motion is the rep
taken out of itself, and the gyroscope's whenever gravity is not moving in the sensor's frame,
which a set is happy to provide — a bar moving vertically does not turn gravity. The detector
keeps its own integrator rather than reading the filter's, deliberately, so that a change in
what the device stores can be attributed to the detection or to the filter; the leak is its
own, and it is not the filter's damping.

Tunables are constants at the top of `calypso-core/src/rep.rs`.

## Computing a rep

On rep close, the sampler copies `[start, end + MARGIN_MS]` out of the ring — `start` being
the turn the swing opened at, which is where the filter had already stopped the bar — and calls
`rep::compute_rep`, which:

1. Runs the filter ([`fusion.rs`]) over the window from the state the detector recorded
   where the window begins, so the rotation, the offsets and the anchoring are the ones
   the device had, and the replay reproduces the summary rather than approximating it.
2. Integrates it, **anchored at the rep's turns and wherever the filter sees the bar
   still**: velocity is put back to zero there, so double-integration drift is bounded
   to the inside of one leg rather than running across the rep or in from the window
   that preceded it. The result is displacement relative to the rep start, in metres —
   which is what a bar-path plot needs. Absolute position is not available from a MEMS
   IMU and is not attempted.
3. Computes the summary stats over the rep's own span — the rest the window keeps
   either side of it is plot and filter margin, not travel the rep made: duration, ROM
   (peak-to-peak **vertical** travel), peak and mean velocity, peak acceleration, and
   roll/pitch min/max in degrees.

**A turn is where the velocity can be put back to zero for free, and only there.** A
rep's halves meet at an instant where the bar reversed, so its own speed is zero whatever
the estimate says; everywhere else in a stroke the bar is moving and the estimate's error
cannot be told from the motion. So the detector hands the replay the three turns it found,
and the replay zeroes there and nowhere else. The one that is exact is the middle — a minimum of
the velocity is the turn however much the integral that produced it has drifted. The two ends are
the closest thing to a turn the detector has, which is what a bounded drift costs. A rep's start
is zeroed ahead of its sample's step rather than after it: it is where the bar began to move,
not where it stopped.

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

The trace is capped at `TRACE_POINTS`, the plot's budget. When a rep is longer than
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
