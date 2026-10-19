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

## Open

### First real device capture
- [ ] Run the app against the hardware and check in a capture as a fixture.
      This is the only step that cannot be done on the host, and it is expected
      to be the interesting one: every fault found so far in the BLE pair was
      invisible from both ends until two real devices described each other (see
      [docs/GOTCHAS.md](docs/GOTCHAS.md)).
- [ ] Settle **axis mapping** once a real squat exists: which of the device's
      world axes is "up"? Until then the plot draws the device frame as-is and
      labels axes generically.
- [ ] Confirm on hardware that the calibration profile really does survive a
      power cycle, and that the levels read off a real BNO055 behave as the
      emulated ones do.

### BLE protocol
- [ ] **Size the trace-chunk characteristic to what it can carry.** `ble.rs`
      types it `BleVec<SYNC_MSG_MAX>` (128 bytes), but a 3-sample chunk can
      encode to `sync::SYNC_CHUNK_MAX` (193). A chunk that overflows fails to
      notify and the stream simply stops, mid-trace, with no error. Real chunks
      are ~115–126 bytes, so it is close enough to bite. The fix is a GATT-level
      change plus its table test.
- [ ] **Size outgoing chunks to the negotiated MTU** instead of the fixed
      conservative 3 samples (`sync::BLE_CHUNK_SAMPLES`). The device learns the
      MTU per connection; `TraceChunk.total_chunks` already means the client
      copes with any chunk size.
- [ ] `SyncStatus` is defined in the proto and encodable in core, but the
      firmware never sends it — so "this trace is no longer in the log" is
      answered by the client timing out instead of by a message. Either wire it
      up (`trace_unavailable`, rep counts) or drop it.
- [ ] Decide about the device name: currently rename-only via the GAP
      characteristic, persisted in NVS.

### Firmware
- [ ] **OTA code updates** — the app partition is 1.5 MB with headroom; dual
      slots would mean shrinking the log partition.
- [ ] On-device tests with [`embedded-test`](https://crates.io/crates/embedded-test):
      the flash partition layer first, then the radio path.
- [ ] Tune the detector's thresholds against real reps (`calypso-core/src/rep.rs`);
      they are currently fitted to synthetic motion.
- [ ] Dependency hygiene: `esp-radio` is built with `coex` and `wifi`, and
      `smoltcp`/`embassy-net` are declared, while the firmware deliberately does
      not initialise Wi-Fi. Decide whether to trim the features or to document
      why they stay.

### Client
- [ ] **Session grouping** — deliberately skipped for v1: the device log is
      undifferentiated, and grouping reps into sets is a judgement better made
      with real data on screen. The list is flat, newest first.
- [ ] Fault detection / coaching hints on the traces (bar dip, forward lean).
      The app plots today; the lifter interprets.
- [ ] Real-device polish: reconnect UX, sync progress, permission prompts.
- [ ] Install story: a free Apple ID gives 7-day builds; TestFlight and the App
      Store need a paid account.
