# Plan: IMU calibration routine

Status: **implemented** — all five steps done, see the checklist at the end.
Verified on the host (core, firmware build, app). **Not verified on hardware**:
no device in the session, so the flash save and real BNO055 levels are untested.

## Goal

The device has no display and no buttons, so before this work calibration was a
dead end: the firmware blocked at boot waiting for the BNO055 to calibrate, and
until it did there was no BLE, no app, and no clue about what to do. You could not
even tell whether the bar was being held wrong or the IMU was simply cold.

What we want instead:

- **BLE stays available when un-calibrated.** The app connects, sees the device
  and is told what to do. Nothing waits forever at boot.
- **The app explains the calibration.** A hint in the UI, in terms of the
  exercise — "rest the bar still on the rack", not "0x35 = 0b01".
- **Per-sensor progress.** Read each sensor's calibration level from the BNO055
  so the hint can point at the one that is actually holding things up.
- **Nothing untrustworthy is recorded.** An uncalibrated gyro or accelerometer
  reports drift as motion; the rep detector would fire on it and the stored trace
  would be fiction rather than a measurement.

## What the sensor gives us

The BNO055 reports `CALIB_STAT` (register `0x35`): four levels, each `0..3`,
3 meaning fully calibrated.

| level | what calibrates it |
|---|---|
| `accel` | the chip held in a few different, **still** orientations |
| `gyro` | the chip **perfectly still** for a few seconds |
| `mag` | figure-of-eight motion — **not used here**, see below |
| `sys` | the aggregate the fusion uses; follows the others |

Two facts about *this* build shape the design:

- **IMU mode has no magnetometer.** We run the BNO055 in IMU mode deliberately
  (the magnetometer is unreliable next to a barbell's steel), and in that mode
  the magnetometer level never reaches 3 — the existing firmware gate already
  accounts for this. So `mag` must never appear in a hint as the thing to wait
  for: the user would move the bar in figure eights forever and watch nothing
  happen. It is reported anyway, for debugging, and labelled as unused.
- **The chip can be calibrated from a saved profile.** A 22-byte profile
  written to the chip's calibration registers restores calibration instantly at
  boot. This already exists (NVS blob at offset 0 in the `nvs` partition, and
  "save calibration between power cycles" is done) — the calibration *routine* is
  about the first time, or after the profile is lost.

## The rules

One rule, in `calypso-core::calibration`, so the firmware gate and the app's
hint cannot drift apart:

- `is_ready()` — `sys == 3 && accel == 3 && gyro == 3`. **This is the gate for
  recording a rep.** The magnetometer is excluded on purpose.
- `is_fully_calibrated()` — all four at 3, i.e. what "fully calibrated" means for
  the chip. Reported, but never waited on.

`ready` travels to the app in the protocol, computed on the device, so the app
shows exactly the state the device is acting on rather than re-deriving it.

## Decisions

**Rep recording is gated on `is_ready()`.** A judgement call, and the one thing
here worth pushing back on. The alternative — record anyway and let the app
warn — puts traces in the archive that look like real measurements but are
drift, and the archive is the product. Gating means a user who never calibrates
gets an empty list; the hint is what makes that legible instead of mysterious
("Recording is paused until the IMU is calibrated"). The detector is not fed
while uncalibrated, so its state starts from a real rest once calibration
completes.

**Calibration rides the existing sync stream** (`SyncMessage.calibration` on the
summary characteristic, `0xca21`), pushed once a second and read from the last
sampler poll. The alternative — a dedicated GATT characteristic — costs two more
attributes, a second subscription and a `read` path through the app's `BleLink`
seam, for the same result: the device→app `SyncMessage` envelope exists precisely
to multiplex device messages, and the app is already subscribed.

**The profile is saved as soon as `is_ready()` holds**, not at boot. Same
outcome as the current code (calibration survives a power cycle) without blocking
boot, and it is saved at the moment it is first true rather than after an
indefinite wait.

**The hint is only shown while connected**, and cleared when the link drops: a
stale calibration reading from ten minutes ago is worse than no hint.

## Design

### Protocol (`proto/calypso.proto`)

```proto
message CalibrationStatus {
  uint32 sys = 1;    // 0..3, the aggregate the fusion uses
  uint32 accel = 2;  // 0..3
  uint32 gyro = 3;   // 0..3
  uint32 mag = 4;    // 0..3, unused in IMU mode
  bool ready = 5;    // the device's own recording gate
}

message SyncMessage {
  oneof body {
    ...
    CalibrationStatus calibration = 4;
  }
}
```

### Firmware (`calypso-esp32`)

- `setup_calibration` loses its wait loop: load the NVS profile, apply it, carry
  on. BLE, the flash log and the sampler all start regardless.
- The sampler polls `get_calibration_status()` at 1 Hz (next to the existing 1 Hz
  level-sense log) — I2C is 400 kHz, and this is one register.
- The latest levels live in an `AtomicU32` (`Calibration::to_bits`), which is how
  the BLE task reads them without an I2C round trip of its own.
- The first time `is_ready()` holds, the profile is read back from the chip and
  written to NVS once.
- While not ready, the detector is not fed and no rep is stored; the 1 Hz log
  line says which level is short, so a serial console is enough to debug it.

### App (`calypso-app`)

- Generated `calypso.pb*.dart` regenerated from the proto (checked in, as
  before).
- `lib/model/calibration.dart`: the levels plus the wording — progress line per
  sensor, and the hint text for the sensor that is holding things up.
- `lib/state/calibration.dart`: a `ChangeNotifier` holding the latest reading,
  cleared on disconnect.
- `lib/transport/ble_transport.dart`: handles the new `SyncMessage` body.
- `lib/widgets/calibration_hint.dart`: a strip under the app bar while connected
  and not ready; nothing once ready.

## Steps

1. [x] **Protocol + core.** `CalibrationStatus` in the proto; the
       `calibration` module (levels, `is_ready`, packing) with tests; the
       `pb`/`sync` encoders; the checked-in Dart protobuf regenerated.

       Regenerating makes the app's `switch` over the `SyncMessage` oneof
       non-exhaustive, so the new arm is present in `ble_transport.dart` and
       drops the message — the tree compiles and behaves as before until step 3
       fills it in. Core: 88 tests pass; app: 63 tests pass, analyzer clean.
       (Two clippy warnings remain in `sync.rs`/the fixtures example; both
       pre-date this work.)
2. [x] **Firmware.** Boot no longer waits (`load_calibration` applies a stored
       profile and returns); the sampler polls `CALIB_STAT` at 1 Hz into an
       `AtomicU32` the BLE task reads; the levels go to the app as a
       `CalibrationStatus` on the next 1 Hz tick; the profile is saved once,
       when `is_ready()` first holds; and the detector is only fed while ready.

       Two things found on the way. The NVS blob is the chip's raw register
       dump with no checksum, so erased flash used to be applied to the chip as
       22 bytes of `0xFF` — it is now recognised and skipped. And the calibration
       save is erase-then-write (bits only go 1 to 0), which the old one-shot
       write was not.

       Verified by `cargo build` for the device (the esp toolchain is installed
       here) plus the host suites: core 88 tests, app 63 tests, analyzer clean.
       **Not verified on hardware** — no device in this session, so the flash
       save and the real BNO055 levels are untested.

       `cargo clippy` in `calypso-esp32` was already failing before this work (8
       `large_stack_frames` errors against a 1 KB threshold). Fixing it properly
       turned into a section of its own below: it is clean now.
3. [x] **App.** `lib/model/calibration.dart` (levels, `ready`, and the wording —
       headline, action, per-sensor line), `lib/state/calibration.dart` (a
       `ChangeNotifier` that only notifies on an actual change, since the device
       pushes once a second), the transport handling the new arm and clearing the
       reading when the link drops, and `lib/widgets/calibration_hint.dart`
       mounted above the list in the list screen.

       The hint goes above the list rather than inside it because an uncalibrated
       device is exactly the case where the list is empty: "No reps yet" alone
       reads as a broken app rather than one waiting for the IMU.

       Verified as far as a step boundary can: analyzer clean, 63 existing tests
       pass (the hint renders nothing unless connected and uncalibrated, so
       nothing existing is disturbed). **The new paths are not yet exercised** —
       parsing the message through the transport and the hint appearing and
       disappearing are what step 4 proves.
4. [x] **Tests.** 13 new, app suite 63 → 76, all green.

       - Core (from step 1): the `calibration` module's rules, including the
         IMU-mode magnetometer case and erased flash reading back as `0xFF`
         levels; and a wire round-trip in `tests/sync.rs` asserting every level
         survives and that `ready` ignores `mag`.
       - `test/calibration_test.dart`: the model and notifier — `ready` is the
         device's answer rather than re-derived, the magnetometer is shown but
         never asked for, the action names only the sensors that are short, and
         an identical reading does not notify (the device pushes once a second).
       - `test/ble_transport_test.dart`: a notification reaches
         `transport.calibration`, a repeat does not fire listeners, and a drop
         clears it.
       - `test/rep_list_test.dart`: end to end through the real list screen and a
         fake link — the hint appears with the device's words and levels, goes
         when the IMU is ready, goes when the connection drops, and is absent
         before connecting.

       Two mutations were run to check the tests are load-bearing rather than
       restating the code, and both were caught (hint ignoring `ready`; stale
       reading surviving a disconnect). Reverted and re-run green.
5. [x] **Docs.** The base README TODO is ticked, the rep-tracking note no longer
       describes the gate as blocking, and the `cargo clippy` state below is
       recorded.

## Clippy on `calypso-esp32`

`cargo clippy` was red before this work: 8 `large_stack_frames` errors (denied in
that crate, 1 KB threshold) and 13 warnings. It is now **clean — no errors, no
warnings** — and the fixes were real ones rather than `#[allow]`s:

- Every helper that looked a partition up read the 3 KB partition table into a
  local. The table now lives in one `FlashStore` (flash peripheral + scratch),
  allocated once and owned by the mutex that already serialises every flash
  access. That alone removed 5 of the 8 errors.
- `pb::decode_record` builds a ~2 KB `RepRecord` on the stack, and the log
  scanning helpers called it from inside two async tasks — so a whole trace chunk
  was part of those futures. Decoding now writes into a caller-owned message
  (`pb::decode_record_into`), and the firmware reuses one on the heap.
- The summary stream's batch of 16 summaries (~1.2 KB) was a local in a future;
  it is a heap buffer now.
- The flash-log call graph (mount, sector scan, CRC, decode) is no longer inlined
  into the futures at all: it sits behind `collect_summaries` /
  `read_trace_from_log`, which are sync functions by design (~2.5 KB and ~1.5 KB
  frames → 1.1 KB and 1.4 KB).
- Encoding a summary now writes straight into the notification buffer instead of
  a scratch plus a copy.

Two exemptions remain, both deliberate and both carrying their measured size in
the reason:

- `stream_trace`: 1363 bytes, of which the largest single local is a 193-byte
  encode scratch — the rest is the inlined encoder and log frames. There is no
  buffer left to move, and suppressing inlining across `calypso-core` to satisfy
  the lint would cost more than it saves.
- `leak_record_scratch`: 2152 bytes, which is the temporary `Box::new` needs to
  build the value before moving it to the heap. It is isolated in a leaf so it is
  not part of `main`'s frame; removing it entirely would mean writing the record's
  fields through `MaybeUninit` by hand, trading a known transient for `unsafe` in
  the decode path.

**And one thing the lint could never have caught: `main` carries its own
`#[allow(clippy::large_stack_frames)]`.** Everything allocated in `main` — which
is the bottom of the stack every task shares — was therefore invisible to the
lint, which is how a 3 KB array and a 2.2 KB temporary accumulated there without
a murmur. Any large local in `main` needs a human look, not a lint; the stack
probe described in the rep-tracking notes is what watches it now.

## Found while fixing clippy

**A trace chunk can be too big to notify, and the failure is silent.**
`sync::SYNC_CHUNK_MAX` (193 bytes) is the worst case for a chunk of
`BLE_CHUNK_SAMPLES` (3) samples, but the trace-chunk characteristic is typed
`BleVec<SYNC_MSG_MAX>` — 128 bytes, sized for a *summary*. A chunk whose encoding
exceeds 128 bytes therefore fails to notify, and the stream just stops; a real
3-sample chunk of a squat is ~115-126 bytes, so this is close enough to bite.
The fix is to size the trace-chunk characteristic to `SYNC_CHUNK_MAX` (the
app decodes notifications by length, so it needs no change), which is a GATT-level
change in `ble.rs` plus its table test. Left alone here because it is a protocol
change, not a calibration one — but it should be settled before the first real
capture.

## Open questions

- **Should an uncalibrated device still record, flagged as such?** Gated for now
  (see Decisions). If a real capture shows a user stuck uncalibrated in a gym,
  a `calibrated: false` bit on `RepSummary` and a warning in the app is the
  alternative.
- **How long should the app keep hinting?** Indefinitely while uncalibrated, or
  collapse to a compact line after the first minute? Needs a real session to
  judge.
- **Does the profile actually survive?** The NVS save path is exercised only by
  a power cycle on hardware; the host tests cover the encoding, not the flash
  behaviour.
- **Should the trace-chunk characteristic carry its own worst case?** See "Found
  while fixing clippy" above: today it cannot, and the failure is a silently
  truncated trace.
