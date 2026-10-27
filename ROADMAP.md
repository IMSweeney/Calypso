# Roadmap

Where Calypso is and what is left. Feature documentation lives in
[`docs/`](docs/); this file is only scope and status.

## Done

- **Sampling and rep detection** — 100 Hz IMU sampling, RAM ring, rep detector,
  ZUPT-anchored trace and summary stats. See [docs/REP_TRACKING.md](docs/REP_TRACKING.md).
- **Per-rep storage** — a flash log ring holds each rep's summary and trace, and
  survives power loss. See [docs/REP_TRACKING.md](docs/REP_TRACKING.md).
- **IMU calibration routine** — the device stays up and usable while uncalibrated,
  the app explains what to do, and recording is gated. See
  [docs/CALIBRATION.md](docs/CALIBRATION.md).
- **BLE sync** — summaries auto-stream to a subscriber, traces stream on request,
  per-rep UUIDs identify reps across reboots. See [docs/APP_SYNC.md](docs/APP_SYNC.md).
- **A chunked pipe over one characteristic each way** — messages larger than one
  value are framed, chunked and reassembled by both ends, so a stored trace chunk
  is one transfer rather than three points per notification. Written and tested on
  both ends; no bench session yet (see Open).
- **A generated BLE interface** — `proto/calypso_gatt.proto` declares the service
  and its characteristics; `gatt-gen` writes the device's table
  (`calypso-core/src/gatt.rs`) and the app's typed client
  (`calypso-app/lib/gatt/gatt_client.dart`), both committed and both naming the
  command that regenerates them, since nothing checks them against the schema.
  **Not yet checked on hardware.**
- **Client app** — sync, the archive of record, rep list, 3D bar-path plot,
  calibration hint, BLE transport, all testable without hardware.
- **Bench mode** — `cargo run --features fake-reps` stores a synthetic rep every
  30 s, exercising the whole device → log → BLE → app path without lifting. See
  [calypso-esp32/README.md](calypso-esp32/README.md).
- **Device time** — the app writes the device's clock on every connection, so a rep
  carries the time it happened rather than the time the app heard about it: the log
  keeps each boot's epoch as its own record, the device dates every summary it
  streams, and the list is ordered and labelled by that. A boot the app never
  reached has no clock, and its reps are never offered. **Not yet run on
  hardware.** See [docs/APP_SYNC.md](docs/APP_SYNC.md).
- **Diagnostics that can be read back** — the app logs every BLE message to a
  rotating file, and the firmware's boot log carries the heap and stack figures.
  See [calypso-app/README.md](calypso-app/README.md) and
  [docs/GOTCHAS.md](docs/GOTCHAS.md).
- **The device's own faults reach the app** — a failed mount, a rep that could not be
  stored, an IMU that stopped answering, a transfer the link dropped: each is now a
  `Report` kind pushed on its own characteristic, raised once a second however often it
  happens, and shown as a banner the person holding the phone can act on. Anything
  raised while nothing was subscribed is sent when the app connects, so a boot that
  went wrong before the app arrived still says so. **Not yet run on hardware.** See
  [docs/APP_SYNC.md](docs/APP_SYNC.md).
- **A day at a time** — the rep list opens on today and an arrow either side walks it
  back a day per press, headed by the date, so the archive is browsable instead of
  only the current day being reachable. See [docs/APP_SYNC.md](docs/APP_SYNC.md).

## Open

### The recording gate

- [ ] **Decide what "calibrated" means, because `accel == 3` is not reliable.**
      The BNO055's accelerometer status wanders while the device sits still: a
      hardware log reads `acc:3 → acc:1 → acc:0 → acc:0 → acc:1 → acc:3` over
      ~100 s, with the stored profile applied and verified byte-for-byte at boot.
      Rep recording is gated on `accel == 3 && gyro == 3`, so it paused and
      resumed on its own through that window — every dip is a rep not recorded,
      with no explanation on screen. The options, roughly in order of preference:

      1. gate on the gyroscope only (it is the stable one, and its drift is what
         fabricates motion);
      2. treat a profile that read back byte-for-byte as good enough and accept
         `accel < 3`;
      3. keep `accel == 3` and accept that a still device stops recording at random.

      The app's hint and [docs/CALIBRATION.md](docs/CALIBRATION.md) both state the
      rule, so they change with it.
- [ ] **Settle whether the saved calibration is what the chip wants.** Applying
      the profile reads back byte-for-byte, yet `acc` reads 0 afterwards while
      `gyro` returns to 3 on its own, so either the profile is not the thing the
      chip derives `ACC` from or the restore is not complete. `accel < 3` is
      harmless if the gate stops depending on it (above).

### Sync over the pipe

- [ ] **A bench session** for what no host replaces: negotiation, subscription,
      throughput, and whether the trace latency below is actually gone.

### BLE link stability

- [ ] **The link drops while idle.** Four drops in one session, each preceded by
      nothing but the calibration message, ~1.2 s before CoreBluetooth reported
      `The connection has timed out unexpectedly`. That message turned out to be
      repeating at round-trip rate (see [docs/GOTCHAS.md](docs/GOTCHAS.md), "An
      acknowledgement is also a request for the next message"), so the drops
      actually followed a flood, and the levels are a read now — what is left is
      the app's own poll, a read a second. The flash experiment is readable from
      the device's log alone now: a millisecond stamp on every line, both ends and
      the duration of every erase and write, and the reason a disconnect gives —
      `Connection Timeout` is the supervision timer expiring over a radio that
      heard nothing, which is a flash window being felt, while `Remote User
      Terminated Connection` is the app letting go. What is left is the other one:
      does it still drop without `--features fake-reps`?
- [ ] **Measure trace latency now that the sizes are right.** A window of 127 that
      sent a whole trace chunk in one blast, and framing from an ATT MTU of 23
      (macOS never sends the exchange), made a chunk ~150 notifications of 14
      bytes. It is now ~8 notifications of 244, paced by acknowledgements every 4
      chunks — a ten-chunk trace should be ~80 notifications instead of ~1500.
      Watch for `sync … unanswered` warnings and the app's `raw` line lengths.
- [ ] **`universal_ble` throws an unhandled `TimeoutException`** when a command is
      issued against a link that has already gone (our code logs its side as
      `trace …: request failed: TimeoutException`), and nothing the caller can
      catch. Worth not making worse: do not issue commands on a link we know is
      down.
- [ ] **Values that arrive between a teardown and the next subscribe are dropped**
      by the transport: 507 of 2422 `raw` summary values in one session. Harmless
      today, because the summaries are re-streamed. The pipe changes the shape — a
      dropped value becomes a chunk the pipe retries — which is one more reason to
      do the bench session before tuning this.

### First real device capture

- [ ] Run the app against the hardware and check in a capture as a fixture.
      Hardware rep detection already works (a 4,150 ms squat-shaped rep with 416
      trace points has been stored and read back), so what is missing is the
      capture. It is also what settles the frame a trace is in — a real squat should
      travel down the trace's Z, the axis the plot draws upward, and land the right
      way up (see [docs/REP_TRACKING.md](docs/REP_TRACKING.md)).
- [ ] **Re-check the summary stats on real motion.** `mean_velocity` is the mean
      of the *raw* speed while `peak_velocity` is the peak of the *detrended* one,
      so a mean above the peak means a large DC component rather than a maths bug.
      Either make the two comparable or document the difference.

### Protocol gaps

- [ ] **A request has no reply.** `SyncStatus` is defined and encodable in core,
      but the firmware never sends it, so "this trace is no longer in the log" is
      answered by the client timing out. A `SyncRequest` the device will not serve
      (not subscribed, rep already gone, unknown request) is logged on the device
      and silent to the app. Either reply with something that names what it
      answers — a request id, or a status body — or drop `SyncStatus` and accept
      the timeout.

### Battery level

- [ ] **Read the cell over the STEMMA bus, not off the charger's divider.** The
      BFF's divider lands on A2, which is GPIO27 and therefore an ADC2 channel — and
      on `esp32` the radio claims ADC2 for the life of its controller, BLE included,
      so `Adc::new` panics and claiming it first fails `esp_radio::init`. The chosen
      part is a MAX17048, in-line between the cell and the BFF's battery JST (its two
      PH ports are one net), which costs no pins and yields a percentage instead of a
      diode-drop guess. Blocked on the part arriving. What it needs after that:

      1. sharing `I2c0` with the BNO055, which takes ownership of it in
         `Bno055::new` — without putting a lock on the 100 Hz path;
      2. percentage and voltage, polled far slower than the sampler, because a cell
         takes hours to move;
      3. a way for the app to see it — a characteristic of its own, or a field on the
         status path — with `gatt-gen` rerun so the app's side is regenerated rather
         than hand-written.

      The number is only honest off charge: while the charger holds the cell at 4.2 V
      the gauge reports the charger's curve, so it wants to be a hint ("this is going
      to die mid-set") rather than a measurement the app reasons from.

### Firmware

- [ ] **OTA code updates** — the app partition is 1.5 MB with headroom; dual
      slots would mean shrinking the log partition.
- [ ] On-device tests with [`embedded-test`](https://crates.io/crates/embedded-test):
      the flash partition layer first, then the radio path.
- [ ] Tune the detector's thresholds against real reps (`calypso-core/src/rep.rs`);
      they are fitted to synthetic motion.
- [ ] **Trim the static heap region if the deepest stack figure ever asks for it.**
      The 64 KB `heap_allocator!` region is what the linker stack competes with
      most, and its surplus is stack the linker cannot use. Each byte trimmed moves
      one-for-one into the stack and comes off the heap's margin under a
      connection, so the change wants the connection-time `heap free` in hand. See
      [docs/GOTCHAS.md](docs/GOTCHAS.md).

### Client

- [ ] **A rep archived without its trace stays traceless for ever.** The trace
      fetch failing (a dropped link, say) archives the summary anyway — right, it
      is real data — but `contains()` then reports that we hold it, so the next
      sync skips it. Re-requesting traces for archived-but-traceless reps on each
      sync would turn a transient drop into a retry.
- [ ] **Session grouping** — deliberately skipped for v1: the device log is
      undifferentiated, and grouping reps into sets is a judgement better made with
      real data on screen. The list is flat, a day at a time, newest first.
- [ ] Fault detection / coaching hints on the traces (bar dip, forward lean).
- [ ] Real-device polish: reconnect UX, sync progress, permission prompts.
- [ ] Install story: a free Apple ID gives 7-day builds; TestFlight and the App
      Store need a paid account.
