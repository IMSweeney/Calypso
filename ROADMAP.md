# Roadmap

Where Calypso is and what is left. Feature documentation lives in
[`docs/`](docs/); this file is only scope and status.

## Done

- **Sampling and rep detection** — 100 Hz IMU sampling, RAM ring, rep detector,
  ZUPT-anchored trace + summary stats. See [docs/REP_TRACKING.md](docs/REP_TRACKING.md).
- **Per-rep storage** — a flash log ring holds each rep's summary and trace, and
  survives power loss. See [docs/REP_TRACKING.md](docs/REP_TRACKING.md).
- **IMU calibration routine** — the device stays up and usable while
  uncalibrated, the app explains what to do, and recording is gated. See
  [docs/CALIBRATION.md](docs/CALIBRATION.md).
- **BLE sync** — summaries auto-stream to a subscriber, traces stream on request,
  per-rep UUIDs identify reps across reboots. See [docs/APP_SYNC.md](docs/APP_SYNC.md).
- **Level sensor** — one axis of the gravity vector over BLE (read + notify).
- **Client app** — sync, archive, rep list, 3D bar-path plot, calibration hint,
  BLE transport, all testable without hardware.
- **Save calibration between power cycles** — a 22-byte BNO055 profile in NVS.
- **Flash log reads that survive the driver's alignment rule** — `flashlog` read
  records by their length, and `esp-storage`'s `ReadNorFlash::read` refuses a length
  that is not a multiple of `READ_SIZE`, quietly enough that reps stored fine and
  could then never be read back (which broke both mounts and every trace). Reads are
  whole 64-byte blocks now and the test mock enforces `READ_SIZE = 4` as the driver
  does. See [docs/GOTCHAS.md](docs/GOTCHAS.md).
- **The trace-chunk characteristic carries a chunk** — sized to
  `sync::SYNC_CHUNK_MAX` rather than a summary's 128 bytes, which is why traces
  stopped part-way with nothing said at either end. Both characteristics are pinned
  by size tests now.
- **Flash failures name themselves** — a boot or store failure carries the operation
  (phase), the sector and, where the driver lets us, the real `FlashStorageError`,
  plus a raw probe of the failing sector. Two sessions went on "flash read/write
  error" before this. See [docs/GOTCHAS.md](docs/GOTCHAS.md).
- **Every BLE message is logged** — to a rotating file, at the plugin seam (`raw`) and
  at the transport (`rx`/`tx`), with one line per session step, so a session can be
  read back without a console. See [calypso-app/README.md](calypso-app/README.md).
- **The app's archive is the only source of truth** — the UI is served from the log,
  the receipt times beside it are decoration that cannot make a rep exist, and live
  summaries are written to it as they arrive.
- **Bench mode** — `cargo run --features fake-reps` stores a synthetic rep every 10 s,
  exercising the whole device -> log -> BLE -> app path without lifting. See
  [calypso-esp32/README.md](calypso-esp32/README.md).

## Open

### The recording gate

- [ ] **Decide what "calibrated" means, because `accel == 3` is not reliable.**
      The BNO055's accelerometer status wanders while the device sits still: a
      hardware log reads `acc:3 → acc:1 → acc:0 → acc:0 → acc:1 → acc:3` over ~100 s,
      with the stored profile applied and verified byte-for-byte at boot. Rep
      recording is gated on `Calibration::is_ready()` (`accel == 3 && gyro == 3`), so
      it paused and resumed on its own through that window — every dip is a rep not
      recorded, with no explanation on screen. The options, roughly in order of
      preference:

      1. gate on the gyroscope only (it is the one that is stable, and whose drift is
         what fabricates motion);
      2. treat a profile that read back byte-for-byte as good enough, and accept
         `accel < 3` — the offsets are what correct drift, the flag is the chip's
         opinion of them;
      3. keep `accel == 3` and accept that a still device stops recording at random.

      Whichever it is, the app's hint and `docs/CALIBRATION.md` both state the rule, so
      they change with it.
- [ ] **The accelerometer's calibration does not come back from NVS** in a way the
      chip reports as calibrated: `load_calibration` applies the profile and the
      read-back matches, yet `acc` reads 0 afterwards (the gyroscope returns to 3 on
      its own, from stillness). Needs a bench answer before the gate is changed: is it
      the saved profile, or does the chip simply not derive `ACC` from a restore?
- [ ] Confirm on hardware that the profile really does survive a power cycle, given
      the above.

### BLE link stability

- [ ] **The link drops while idle, for no reason we can see.** Four drops in one
      session, each preceded by nothing but the ordinary 1 Hz calibration message —
      still flowing ~1.2 s before CoreBluetooth reported `The connection has timed out
      unexpectedly`. Two experiments will separate our flash traffic from the radio:
      does the *device's* log show a `rep … stored` or `IMU calibration saved` beside a
      drop, and does it still drop running **without** `--features fake-reps` (i.e.
      with no periodic flash writes)?
- [ ] **Trace latency is tens of seconds**, and it is head-of-line blocking: the
      firmware's GATT loop `await`s the ATT response to a trace request, and that waits
      behind the previous rep's ~150 trace notifications in the controller's queue. The
      app now tolerates it (its trace timeout is an *idle* timeout, 30 s of silence,
      reset by every chunk) and logs which shape a failure was, but the latency is
      real. The MTU item under BLE protocol is the lever.
- [ ] `universal_ble`'s own command queue throws an **unhandled** `TimeoutException`
      when a command is issued against a link that has already gone (our code logs its
      side as `trace …: request failed: TimeoutException`). Nothing the caller can
      catch; worth not making worse: don't issue commands on a link we know is down
      (the `disconnect` in `_teardownConnection` is the remaining one).
- [ ] Values that arrive **between a teardown and the next subscribe** are delivered by
      the plugin and dropped by the transport: 507 of 2422 `raw` summary values in one
      session. Harmless today (the summaries are re-streamed), but the `raw` line is
      what makes it visible, and it is not obviously wrong to leave it.

### First real device capture
- [ ] Run the app against the hardware and check in a capture as a fixture.
      Hardware rep detection now works (a 4150 ms squat-shaped rep with 416 trace
      points has been stored and read back), so this is about the *capture*, not the
      plumbing: every fault found so far in the BLE pair was invisible from both ends
      until two real devices described each other (see
      [docs/GOTCHAS.md](docs/GOTCHAS.md)).
- [ ] Settle **axis mapping** once a real squat exists: which of the device's
      world axes is "up"? Until then the plot draws the device frame as-is and
      labels axes generically.
- [ ] Re-check that a stored rep's summary stats are sane on real motion. The bench
      data is not: `mean_velocity` is the mean of the *raw* speed while
      `peak_velocity` is the peak of the *detrended* one, so a mean above the peak is
      a large DC component rather than a maths bug — worth either making the two
      comparable or documenting the difference.

### BLE protocol
- [ ] **Size outgoing chunks to the negotiated MTU** instead of the fixed
      conservative 3 samples (`sync::BLE_CHUNK_SAMPLES`). The device learns the
      MTU per connection; `TraceChunk.total_chunks` already means the client
      copes with any chunk size. ~12 points per notification instead of 3 would cut
      a rep's trace from ~150 packets to ~40, which is most of the latency above.
- [ ] `SyncStatus` is defined in the proto and encodable in core, but the
      firmware never sends it — so "this trace is no longer in the log" is
      answered by the client timing out instead of by a message. Either wire it
      up (`trace_unavailable`, rep counts) or drop it.
- [ ] **Let the app clear the device's log.** Today the only way is `espflash
      erase-parts --partition-table partitions.csv log` from a host, which is the
      wrong shape when iterating on the app. A `SyncMessage` arm (or a control
      characteristic write) calling `FlashLog::format()` and resetting the summary
      cursor would do it, and it destroys reps so it should be deliberate and
      logged.
- [ ] Decide about the device name: currently rename-only via the GAP
      characteristic, persisted in NVS.

### Firmware
- [ ] **OTA code updates** — the app partition is 1.5 MB with headroom; dual
      slots would mean shrinking the log partition.
- [ ] On-device tests with [`embedded-test`](https://crates.io/crates/embedded-test):
      the flash partition layer first, then the radio path.
- [ ] Tune the detector's thresholds against real reps (`calypso-core/src/rep.rs`);
      they are currently fitted to synthetic motion.
- [ ] **The stack is a landmine.** Every flash read allocates `esp-storage`'s
      4096-byte `FlashSectorBuffer` on the stack, and the linker stack is shared with
      the BLE task and interrupts at `Stack in use before the tasks: 39594 B` out of
      ~42 KB. It works, but a few KB of headroom is all there is; the numbers to watch
      are in [docs/GOTCHAS.md](docs/GOTCHAS.md).
- [ ] Dependency hygiene: `esp-radio` is built with `coex` and `wifi`, and
      `smoltcp`/`embassy-net` are declared, while the firmware deliberately does
      not initialise Wi-Fi. Decide whether to trim the features or to document
      why they stay.

### Client
- [ ] **A rep archived without its trace stays traceless for ever.** The trace fetch
      failing (a dropped link, say) archives the summary anyway — right, it is real
      data — but `contains()` then reports that we hold it, so the next sync skips it.
      Re-requesting traces for archived-but-traceless reps on each sync would turn a
      transient drop into a retry.
- [ ] **Session grouping** — deliberately skipped for v1: the device log is
      undifferentiated, and grouping reps into sets is a judgement better made
      with real data on screen. The list is flat, newest first.
- [ ] Fault detection / coaching hints on the traces (bar dip, forward lean).
      The app plots today; the lifter interprets.
- [ ] Real-device polish: reconnect UX, sync progress, permission prompts.
- [ ] Install story: a free Apple ID gives 7-day builds; TestFlight and the App
      Store need a paid account.
