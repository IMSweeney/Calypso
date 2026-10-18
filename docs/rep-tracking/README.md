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
Sampler task ──► Ring buffer (RAM, ~12 s window: accel + quaternion)
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
  fixed-point, 20 B with padding) — 30 s @ 100 Hz was 60 KB of heap, which turned
  out to be 36 KB more than the device could spare (see the heap note under
  *firmware constraints*). It is 12 s now: the detector never asks for more than
  9 s of history.
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
- [x] Switch BNO055 to IMU mode; I2C to 400 kHz (the calibration gate is
      sys/gyro/accel — the magnetometer never calibrates in IMU mode — and as of
      `docs/calibration/` it no longer blocks boot or BLE: reps are simply not
      recorded until the levels are good, and the app says which one is behind)
- [x] 100 Hz sampler task: `{ t_ms, accel[i16;3], quat[i16;4] }` into a RAM
      ring buffer (12 s window; `calypso_core::sampler::{Sample, SampleRing}`)
- [x] Feed existing gravity-x BLE characteristic from the sampler (decimated
      to a 10 Hz update, BLE still notifies at 1 Hz)

### 3. Rep detector (`calypso-core`, host-testable)
- [x] State machine: IDLE → MOVING → SETTLE → ZUPT → close rep
      (`rep::Detector`; thresholds are constants in `calypso-core/src/rep.rs`)
- [x] World-frame trace: rotate accel by quaternion, ZUPT-anchored
      integration → displacement(t) (`rep::compute_rep`; three passes, no
      scratch buffers)
- [x] Summary stats: duration, ROM, peak/mean velocity, peak accel,
      roll/pitch min/max
- [x] Host tests: synthetic rep fixtures (cadences, noise, rests), known
      quaternion × known accel rotations, ZUPT accuracy (constant bias is
      rejected exactly), decimation, plus a detect→ring-slice→compute test
- [x] Device pipeline: the sampler task runs the detector and logs each rep's
      summary (record storage lands in §4)

Tunables live in `calypso-core/src/rep.rs`: on/rest accel thresholds
(0.60/0.25 m/s², hysteresis), debounce (100 ms on / 250 ms rest), rep length
bounds (0.4–8 s), and a 200 ms ZUPT margin each side of the stored trace.

### 4. Data format (`calypso-core` + app)
- [x] Write the real `.proto` (`Sample`, `TraceSample`, `RepSummary`,
      `TraceChunk`, `RepRecord`, `TraceRequest`, `SyncMessage`, `SyncStatus`)
      at `proto/calypso.proto`, shared with the app
- [x] `micropb` encode/decode in `calypso-core` (`pb` module: conversions,
      record encode/decode into caller buffers, generated types); round-trip
      and sizing host tests
- [x] Rep record → flash log on rep close (device): `store::append_rep`
      writes one summary record plus trace chunks; the firmware stores each
      detected rep from the sampler task

Record layout: `[RepSummary][TraceChunk 0][TraceChunk 1]…` sharing one
`rep_id`. Chunks hold up to `pb::TRACE_CHUNK_SAMPLES` (48) trace points so an
encoded record always fits one 4 KB erase sector (the log's record limit is
4076 bytes; a worst-case chunk encodes to ~2 KB). A full 512-point trace is
11 chunks. `proto/` is shared with the app; codegen needs no `protoc` install
(the build uses `protoc-bin-vendored`).

### 5. BLE sync (`calypso-esp32`)
- [x] "Log sync" GATT service (UUIDs in `ble.rs`): summary notify (0xca21),
      trace-request write (0xca22), trace-chunk notify (0xca23)
- [x] Auto-stream new `RepSummary` records on subscribe (polled on the
      existing 1 Hz BLE tick, driven by a `SummaryCursor`)
- [x] On-demand `TraceRequest{rep_id, all_chunks, chunk_index}` → chunked
      `TraceChunk` notify stream

**End of stream:** every streamed `TraceChunk` carries `total_chunks` — the
number of chunks in that stream — so the client knows which chunk is the last
instead of guessing. It cannot be derived from the summary: the log stores
48-point chunks while BLE re-chunks to ~4 points per notification. Stored records
leave the field 0, where the summary's `chunk_count` describes the trace instead.
(Found while writing the client's sync layer: without it, "more chunks coming"
and "this rep is gone, the device will stay silent" look identical from the
outside.)

Flow control: notifications are MTU-bound, so traces are re-chunked on the
way out at `sync::BLE_CHUNK_SAMPLES` (4) points per notification (~200 bytes,
inside a 247-byte MTU's 244-byte payload). The app reassembles by `index`;
the summary count in `RepSummary.chunk_count` is the log's chunk count, so
apps should count received chunks rather than trusting it. "Subscribed" is
inferred from notify success: the cursor only advances on a successful send,
so an unsubscribed app simply means the next tick retries from the same
position (no CCCD bookkeeping). Reps that fell off the ring are reported as
unavailable rather than streamed. Re-requesting is idempotent.

If the boot log says `Rep log unusable`, the firmware now names the cause and, for
a missing partition, dumps the partition table it actually read. The usual answer
is a stale table on the device: it must contain the `log` partition from
`partitions.csv` (`data`/`undefined`, 0x190000, 0x270000), which a plain
`espflash flash` writes along with the bootloader and the app.

**What a client actually sees**, and the three ways that can be wrong silently:

- **The service UUID is advertised**, in the advertisement packet itself rather than
  the scan response: a client that scans *filtered by service* only sees
  advertisements, so a device that does not advertise its service is invisible to the
  app while a generic scanner lists it anyway. (The advertisement carries the service
  and the name together — 30 of the 31 bytes for `Calypso`.)
- **One byte order, both places: wire order, not text order.** Every array in
  `ble.rs` is written reversed relative to the text it means —
  `40 88 13 df … 20 ca 00 00` is `0000ca20-b0cd-11ec-871f-d45ddf138840` to a client
  — and the GATT table and the advertisement use the same array, because a client
  reverses both. Established from one build's logs rather than from the spec, which
  got this wrong twice: the firmware logs its arrays at boot, and the app logs the
  tree it discovered, so the two ends can be compared directly
  (`calypso: device offers …; want …` against `sync service uuid … (wire), …
  (as a client sees it)`). Both suites pin the rule per side.

  A stale client cache is what made this hard: CoreBluetooth caches services and
  characteristics **keyed to the device's Bluetooth address**, so a corrected
  firmware can look unchanged until the address is bumped (see the note where the
  address is set in `main.rs`) or the cache is cleared.
- **A client must discover services before subscribing.** CoreBluetooth's peripheral
  has no characteristics until `discoverServices` has run, and the plugin reports a
  lookup against that empty list as `characteristicNotFound` — naming a UUID that is
  perfectly correct, so it reads as a device fault. See `docs/app/plan.md`.
### Firmware constraints worth knowing

Three things here cost a session each. All three are about the *board*, not the
logic, and none of them are visible from the host tests.

#### A flash write or erase needs the other core parked

The device log could not be formatted, and the error looked like broken flash:
`esp-storage`'s default `MultiCoreStrategy::Error` **refuses every write and erase
while the second core is running** (`FlashStorageError::OtherCoreRunning`), and on
this build CPU1 is up from the bootloader onwards. Reads are unaffected, which is
what makes it so confusing: the NVS reads worked fine (the stored BNO055
calibration was applied, the device name was read) while every erase — including
the log partition's first format — failed.

`FlashStorage::new(..).multicore_auto_park()` is not optional here. It parks the
other core for the duration of each operation and unparks it afterwards.

#### The main stack is a remainder, shared, and invisible to clippy

`main` runs on the linker's stack, which is not a sized buffer: `_stack_end` is
wherever the RWDATA sections ended and `_stack_start` is the top of that region
(`esp-hal`'s `ld/sections/stack.x`), so **every static and every heap region
carved out of DRAM makes the stack smaller.** It was 22,532 bytes, and removing the
Wi-Fi init (below) made it 41,932 — the same change, twice the stack. It has to
hold `main`'s frame, plus every future `join3` polls, plus interrupt contexts. Two
things to know, and one habit:

- **`main` is `#[allow(clippy::large_stack_frames)]`**, so the crate's own lint
  has never looked at the frame that sits under every task. Anything big in
  `main` needs a human eye.
- **`Box::new(v)` builds `v` on the stack first** and then moves it, so a large
  `v` is large stack. Every big buffer here is therefore allocated through
  `AllocVec` + `Box::leak`, never as an owned value — `SampleRing` borrows its
  buffer for the same reason (`rep::TraceBuf` already did).

The current build's frames can be checked without hardware, by reading the `entry
a1, N` immediate out of the ELF (N × 8 bytes):

```console
xtensa-esp32-elf-objdump -d target/xtensa-esp32-none-elf/debug/calypso \
  | grep -oE 'entry\s+a1, [0-9]+' | awk '{print $3}' | sort -n | tail
```

At the time of writing the largest frame in the firmware is 240 units (1920
bytes), and nothing allocates stack dynamically (no `movsp`, no manual `a1`
adjustment) — so no single function here is the problem any more.

The runtime probes, in the order they print:

- `Radio up, heap free N B`, then the setup lines, then **`Stack in use before
  the tasks: N B`** — `main`'s frame, measured just before `join3` starts.
- **`BLE connection established (stack N B)`** — depth inside the GATT task.
- the 1 Hz sampler line, which carries `stack N B` beside the heap figure.

An overflow shows up as esp-hal printing `Detected a write to the stack guard
value`, and by then the task is dead. If those numbers approach ~20 KB, move a
buffer to the heap rather than looking for stack to grow into — there isn't any.

#### The heap is 164 KB and it was full

Two `esp_alloc` regions, 98,768 B (reclaimed DRAM) + 64 KB (static), and **the
buffer set is large**: the sample ring, the 9 s rep window (18 KB), two 512-point
trace buffers (16 KB each), record scratch, the decode scratch, and the summary
batch — ~104 KB of it. Measured from a boot log, the radio took another ~54 KB
(driver buffers plus the Wi-Fi and BLE task stacks) before any of that, which did
not fit, and the way that showed up was *nothing at all*: the log simply stopped,
because `Vec::resize` on an exhausted heap calls `handle_alloc_error`, which
panics, and the panic handler was `loop {}` with no output.

Two things fixed it. The 30 s sample ring was 36 KB more than the device needed
(the detector never asks for more than 9 s), and Wi-Fi — which this device never
used — is gone, giving back most of the radio's 54 KB. From a boot log, the
figures now read: `Radio up` ~109 KB free, `Rep detector ready` ~45 KB (that was
8,968 B), `Sync buffers ready` ~22 KB. So:

- the panic handler now prints (`*** PANIC: ... ***`), because "the log stops"
  and "the device hung" looked identical;
- `Radio up, heap free N B`, `Rep detector ready, heap free N B` and
  `Sync buffers ready, heap free N B` are the three figures worth watching, and
  anything under ~8 KB at the third one is a device that will not accept a
  connection;
- the `Radio up … free` figure is now the one that says whether the radio came up
  cheaply or expensively: it dropped by most of Wi-Fi's cost when that went away,
  and it is the number to compare if a radio dependency is ever added.

#### The radio has its own tasks and stacks

- **The BLE controller runs in its own esp-rtos task with a 4 KB stack by
default** (`esp_radio::ble::Config::task_stack_size`), allocated from the heap.
A fault inside it looks *nothing* like a `main` problem: the stack pointer in the
panic dump points into DRAM (≈`0x3FFD…`), not at the linker stack. This
firmware doubles it to 8 KB — it has the smallest default stack of anything on the
board and is the only one we can size from here.
- **No Wi-Fi controller.** `wifi::new` is not called: this is a BLE-only device,
and initialising Wi-Fi started a task with its own stack plus driver buffers,
most of the ~54 KB the radio took. It came from the esp-generate template, which
wires up both radios. Two things make the removal safe rather than a guess: a
build without it ran and advertised (the controller and host came up fine), and
esp-radio's BLE bring-up only needs it when the `coex` feature is enabled —

  ```rust
  #[cfg(coex)]
  assert!(crate::wifi::coex_init() == 0, "coex_init failed");
  ```

  with `coex_init` needing the adapter functions the Wi-Fi controller registers.
  `coex` is an opt-in esp-radio feature that nothing here enables (checked: the
  panic string is absent from the binary), so that assert is not compiled in.
  **If BLE ever stops coming up, call `wifi::new` again first**, and if the board
  panics with `coex_init failed`, someone turned `coex` on.

### 6. App (`calypso-app/`)
Planning now lives in [`docs/app/plan.md`](../app/plan.md) (phases, decisions,
TODO) with the design in [`docs/app/README.md`](../app/README.md).
- [ ] Decode sync stream; workout history from summaries
- [ ] Bar path (displacement) + tilt (quaternion) plots from traces
- [ ] Later: fault detection (bar dip, forward lean) on the traces
