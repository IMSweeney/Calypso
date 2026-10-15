# Plan: Rep Tracking & Workout Storage

Status: **draft for review** — no implementation yet.

## Goal

Track the sensor's motion during an exercise set, detect individual reps, and
store per-rep data on the device for later sync to the phone app over BLE.

A key requirement: for each rep, store enough to **plot the bar/sensor path
and orientation afterwards** — e.g. for a squat, whether the bar dipped to
one side (roll) or the lifter leaned forward (pitch). Detection of *faults*
is **not** done on the device; the app analyzes/plots the synced traces.

## Hard constraints (from hardware)

- **No absolute position.** MEMS double-integration drift is meters within
  seconds. What we can produce is a **per-rep, ZUPT-anchored displacement
  trace** (position relative to rep start, drift reset at every rep) — which
  is exactly what bar-path plotting needs. Orientation (quaternion) is
  drift-stable over rep timescales via gyro+accel fusion.
- **BNO055 fused output rate: 100 Hz.** On-chip fusion gives us linear
  acceleration (gravity removed) + quaternion orientation. We'll run the
  sensor in **IMU mode** (accel + gyro, no magnetometer) since the
  magnetometer is unreliable near barbells/rowers. Heading will drift slowly
  over a set; roll/pitch (what we care about) stay accurate, and yaw error
  can be corrected at rep start via ZUPT alignment.
- **4 MB flash** (ESP32-PICO-D4), ~100–200 KB free heap with BLE running.
- Flash endurance ~100k erase cycles/sector — needs a ring-buffer log, not
  random-access rewriting.

## Architecture

```
BNO055 (IMU mode, 100 Hz)
   │  I2C @ 400 kHz
   ▼
Sampler task ──► Ring buffer (RAM, ~30 s window: accel + quaternion)
                    │
                    ▼
              Rep detector (state machine, ZUPT at rest)
                    │ rep complete
                    ▼
     Rep summary + full per-rep time series (displacement + orientation)
                    │
                    ▼
        Flash log (data partition, ring of 4 KB sectors)
                    │ on request
                    ▼
        Phone app (BLE sync, protobuf) ──► plots bar path, tilt over time
```

### 1. Sampler (100 Hz)

- New high-rate sensor task replaces the current 1 Hz poll.
- Per sample: `t_ms` (u32, monotonic), linear accel xyz, quaternion.
- I2C bumped to 400 kHz (read ~14 B/sample ≈ 0.5 ms of the 10 ms period).
- The existing gravity-x BLE characteristic keeps working off the same
  sampler (decimated).

### 2. Ring buffer (RAM)

- Fixed-rate samples ⇒ timestamp implicit except for the base timestamp.
- Sample format: `{ t_ms: u32, accel: [i16; 3], quat: [i16; 4] }` (scaled
  fixed-point, 18 B) — 30 s @ 100 Hz ≈ 54 KB of heap. Acceptable; can shrink
  by dropping `t_ms` (fixed rate) if heap gets tight.
- The detector consumes a sliding window; nothing is integrated free-running.

### 3. Rep detector (in `calypso-core`, host-testable)

State machine:

```
IDLE ──|a| > threshold for T_on──► MOVING ──|a| < threshold for T_rest──► SETTLE
SETTLE ──ZUPT (v := 0), close rep, emit Rep──► IDLE
```

- ZUPT: when the sensor is at rest (low accel magnitude variance + low gyro),
  force velocity to zero — bounds drift to within one rep.
- At rep close, compute the **world-frame trace** for the rep: rotate each
  sample's linear accel by its quaternion, integrate with ZUPT anchoring at
  both ends → displacement(t). Also compute summary stats: duration, ROM
  (peak-to-peak displacement), peak/mean concentric velocity, peak
  acceleration, **roll/pitch range** (min/max tilt from level).
- Tunables (thresholds, debounce times) as constants; developed against
  synthetic fixtures in host tests before touching hardware.

### 4. Per-rep record (what gets stored)

Each rep produces one record containing:

- **Summary** — id, start/end time, duration, ROM, peak/mean velocity, peak
  accel, roll/pitch min/max.
- **Trace** — the full 100 Hz time series for the rep (plus ~200 ms margin
  each side): displacement xyz, **quaternion orientation** (not precomputed
  Euler angles — more flexible for the app, which can derive any angle from
  them), accel xyz. A 3 s rep ≈ 300 samples ≈ 5.4 KB raw; protobuf-packed
  fixed arrays compress this well. (If flash pressure ever matters,
  decimating the stored trace to 25–50 Hz halves/quarters it with no visible
  plot loss.)

Summaries and traces are stored **together** in one record in one log ring —
simpler log, single sequence space, no cross-ring consistency to manage. The trace stored is the **same one the detector computed**
(consistent with the summary stats), and the raw accel+quat is kept so the
app can re-derive traces with better algorithms later without a firmware
update.

### 5. Long-term store (flash)

- Add `partitions.csv`; shrink the app partition and add a data partition:

  | partition | size | notes |
  |---|---|---|
  | app | 1.5 MB | current app is ~0.55 MB; headroom |
  | data (log) | ~2.3 MB | ring of 4 KB sectors |
  | nvs / phy | existing | name + calibration stay |

  (When we get to OTA, this becomes 2×1.1 MB app slots + ~1.4 MB log.)
- The log is a **ring of 4 KB sectors** with monotonic sequence numbers;
  oldest sector erased on wrap. Capacity at ~5.5 KB/rep (summary + full
  trace): ~420 reps (~50–80 typical workouts) before wrap. Since the app
  pulls summaries automatically on every connect (see §7), reps only need to
  survive on-device until the next sync — wrap is fine.
- NVS (name/calibration) is untouched.

### 6. Data format: protobuf

- **`micropb`** (embedded-first, fixed buffers, no alloc) for encoding on
  device; phone app decodes with any protobuf library.
- Sketch (`.proto` shared with the app):

  ```proto
  message Sample {
    sint32 dt_ms = 1;          // delta from rep start
    float pos_x = 2; float pos_y = 3; float pos_z = 4;      // m, rel. rep start
    float qw = 5; float qx = 6; float qy = 7; float qz = 8; // orientation
    float ax = 9; float ay = 10; float az = 11;             // linear accel
  }
  // On-flash record: summary + full trace, stored together.
  message Rep {
    RepSummary summary = 1;
    repeated Sample samples = 2;   // full-rate trace
  }
  // Auto-synced on connect (small; no samples).
  message RepSummary {
    uint32 id = 1;
    uint32 start_ms = 2;       // device time
    uint32 duration_ms = 3;
    float rom_m = 4;
    float peak_vel_mps = 5;
    float mean_vel_mps = 6;
    float peak_accel_mps2 = 7;
    float roll_min_deg = 8; float roll_max_deg = 9;
    float pitch_min_deg = 10; float pitch_max_deg = 11;
  }
  // On-demand per rep; streamed in chunks.
  message TraceRequest { uint32 rep_id = 1; }
  message TraceChunk {
    uint32 rep_id = 1;
    uint32 chunk_index = 2;
    bool last = 3;
    repeated Sample samples = 4;
  }
  ```

### 7. BLE sync to phone

Two tiers:

- **Summaries sync automatically** on connect: the device streams new
  `RepSummary` records (id, stats — no samples) via notify as soon as the
  app subscribes. Summaries are small (~50 B/rep), so this is cheap and the
  app always has an up-to-date workout history.
- **Traces sync on demand**: the app requests a specific rep's full trace
  (`TraceRequest{rep_id}`) when the user opens that rep; the device streams
  the samples chunked over notify, app acks/reassembles. A trace may span
  several notifications.
- Live streaming during a set stays as the existing notify characteristic
  (decimated ~10 Hz), separate from the log.

## Testing strategy

- **calypso-core (host):** rep detector against synthetic accel traces
  (sine/triangle reps at various cadences, rest periods, noise); ZUPT
  integration accuracy; world-frame rotation correctness (known quaternion ×
  known accel); ring buffer wrap; protobuf round-trip; sector-ring log
  against an in-memory flash.
- **On device:** thresholds tuned with real reps later; `embedded-test` for
  the flash partition layer.
- App-side plotting can be developed entirely against fixture reps exported
  from the host tests — no device needed.

## TODO

### 1. Flash log (`calypso-esp32` + `calypso-core`)
- [x] Add `partitions.csv`: shrink app to 1.5 MB, add ~2.3 MB `data` partition
- [x] `calypso-core`: sector-ring log (append, sequence numbers, wrap/erase)
      against a generic `NorFlash` impl (`flashlog::FlashLog`)
- [x] Host tests: in-memory flash, wrap, power-loss mid-write recovery
- [x] Wire into firmware against the real data partition
      (`mount_rep_log` in `calypso-esp32/src/bin/main.rs`; record writes
      land in Phase 4)

### 2. Sampler & ring buffer (`calypso-esp32`)
- [x] Switch BNO055 to IMU mode; I2C to 400 kHz (calibration gate relaxed
      to sys/gyro/accel — the magnetometer never calibrates in IMU mode)
- [x] 100 Hz sampler task: `{ t_ms, accel[i16;3], quat[i16;4] }` into a RAM
      ring buffer (30 s window; `calypso_core::sampler::{Sample, SampleRing}`)
- [x] Feed existing gravity-x BLE characteristic from the sampler (decimated
      to a 10 Hz update, BLE still notifies at 1 Hz)

### 3. Rep detector (`calypso-core`, host-testable)
- [ ] State machine: IDLE → MOVING → SETTLE → ZUPT → close rep
- [ ] World-frame trace: rotate accel by quaternion, ZUPT-anchored
      integration → displacement(t)
- [ ] Summary stats: duration, ROM, peak/mean velocity, peak accel,
      roll/pitch min/max
- [ ] Host tests: synthetic rep fixtures (cadences, noise, rests), known
      quaternion × known accel rotations, ZUPT accuracy

### 4. Data format (`calypso-core` + app)
- [ ] Write the real `.proto` (`Sample`, `Rep`, `RepSummary`, `TraceRequest`,
      `TraceChunk`), shared with the app
- [ ] `micropb` encode/decode in `calypso-core`; round-trip host tests
- [ ] Rep record → flash log on rep close (device)

### 5. BLE sync (`calypso-esp32`)
- [ ] "Log sync" GATT service
- [ ] Auto-stream new `RepSummary` records on subscribe
- [ ] On-demand `TraceRequest{rep_id}` → chunked `TraceChunk` notify stream
      with app ack/reassembly

### 6. App (separate repo, out of scope here)
- [ ] Decode sync stream; workout history from summaries
- [ ] Bar path (displacement) + tilt (quaternion) plots from traces
- [ ] Later: fault detection (bar dip, forward lean) on the traces
