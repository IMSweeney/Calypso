# Roadmap

Where Calypso is and what is left. Feature documentation lives in
[`docs/`](docs/); this file is only scope and status.

## Done

- **Sampling and rep detection** — 100 Hz IMU sampling, RAM ring, rep detector,
  ZUPT-anchored trace and summary stats. See [docs/REP_TRACKING.md](docs/REP_TRACKING.md).
- **Per-rep storage** — a flash log ring holds each rep's summary and trace, and
  survives power loss. See [docs/REP_TRACKING.md](docs/REP_TRACKING.md).
- **Raw IMU, filtered in firmware** — the device reads the BNO055's accelerometer and
  gyroscope registers and computes its own attitude, gravity and offsets instead of
  trusting the chip's fusion. Recording is gated on that filter having settled on a
  still window — one second of stillness, no calibration ritual. Dev mode stores a
  rep's window at full rate for replay. See [docs/RAW_IMU.md](docs/RAW_IMU.md).
  **Not yet run on hardware**: the swap is the A/B, and it is what stage 4 opens.
- **BLE sync** — summaries auto-stream to a subscriber, traces stream on request,
  per-rep UUIDs identify reps across reboots. See [docs/APP_SYNC.md](docs/APP_SYNC.md).
- **A chunked pipe over one characteristic each way** — messages larger than one
  value are framed, chunked and reassembled by both ends, so a stored trace chunk
  is one transfer rather than three points per notification. Written and tested on
  both ends, and exercised end to end by a real session — every trace whole, none
  abandoned — but no controlled bench session yet (see Open).
- **A generated BLE interface** — `proto/calypso_gatt.proto` declares the service
  and its characteristics; `gatt-gen` writes the device's table
  (`calypso-core/src/gatt.rs`) and the app's typed client
  (`calypso-app/lib/gatt/gatt_client.dart`), both committed and both naming the
  command that regenerates them, since nothing checks them against the schema.
  **Not yet checked on hardware.**
- **A summary stream that has nothing to read does not read** — the batch remembers
  what the log looked like when it was collected, so a caught-up subscriber no longer
  makes the device walk every record once a second; and a trace request is a walk of
  that rep's own records, from a place the summary scan recorded, rather than of the
  whole log. Both walks were holding the sampler off its tick for seconds (see
  [docs/GOTCHAS.md](docs/GOTCHAS.md)). The fallback for a rep the device has no place
  for is still the walk. **Not yet re-run on hardware**, which is what would close
  [*A rep's timeline has a hole in it*](#firmware).
- **Client app** — sync, the archive of record, rep list, 3D bar-path plot, the
  device's faults, BLE transport, all testable without hardware.
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
  rotating file, the firmware's boot log carries the heap and stack figures, and the
  IMU's own last sample — specific force, the attitude the filter computed, and when
  the sampler read it — is a `read` on its own characteristic, in the fixed point a
  stored trace carries. The app reads it once per connection and logs it (nothing on screen shows
  it yet), and any BLE client can read it. **Not yet run on hardware.** See
  [docs/APP_SYNC.md](docs/APP_SYNC.md).
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
- **A rep is out and back** — the detector closes a rep on the return, not where the bar
  rests, so a bench rep held at lockout is captured as one rep instead of split into a
  drive and a lowering (or lost). A one-way push that pauses (an unrack) is dropped
  rather than merged into the next stroke, as long as that stroke goes the same way;
  the set that starts with a lowering is below. See
  [docs/DETECTOR_TESTS.md](docs/DETECTOR_TESTS.md).
- **A rep is two strokes between turns** — the detector finds the turning points from the
  bar's *vertical* velocity and measures the signed vertical travel between them, so the travel check
  is part of detection instead of a test afterwards: a leg under `MIN_ROM_M` can never
  pair, and a stroke that stalls mid-drive is one rep with its whole travel rather than a
  rep cut short or lost. Chosen over the momentum detector on measurement — 0 cells where
  it stored a rep the old one rejected across 5,292 map cells, no new false positive on
  the non-rep sweep, and ROM at least as close to the travel actually moved. On the one
  real capture it agrees with the detector it replaces on every stored window whose own
  sample times can be replayed at all — 3 spans each — and the windows where they differ
  are the ones with a 2.2 s jump or a catch-up burst inside them
  ([docs/GOTCHAS.md](docs/GOTCHAS.md)). It costs one thing, accepted knowingly: a return
  too gentle to reach `VEL_MIN` never turns, so it is not detected (below). See
  [docs/REP_TRACKING.md](docs/REP_TRACKING.md).

## Open

### The raw IMU

- [ ] **What is left of a recorded window's stall is the writes and `compute_rep`.**
      Measured on one clock: of the 385 ms the sampler loses per window, 268 was six sector erases,
      50 the writes, 38 `compute_rep` and 21 the log mount and the encode. The "130–240 ms before the
      flash" was the host's serial timestamps, not the device. The erases are gone — the ring keeps
      `RESERVE_SECTORS` already-erased sectors ahead of itself, refilled one per tick while the bar
      is still, and `advance_sector` checks the bytes rather than erasing again. What is still on the
      sampler's path is the page writes (~50 ms for 23 records) and `compute_rep` (38 ms, fifteen
      times what the simulator's per-update timing scaled to). Both leave it by writing one record a
      tick and by moving the replay to the second core.
- [ ] **Whether the offsets should re-converge faster after the device is moved.** A bias step of
      about 330 mg — what moving a sensor with a three-percent scale error looks like — settles
      through `TRACK_TAU_S` in roughly ten seconds, and a window recorded in that stretch carries
      centimetres of travel on a still bar. Measured against the maps: shortening the time
      constant fourfold costs the slow end of every long-travel class (a deadlift's one-rep run
      starts at 0.48 m/s instead of 0.29), and learning at `BOOT_TAU_S` whenever the bar is `still`
      improves typical ROM but turns the slowest cells from 75% long into 50% short, because a very
      gradual stroke can satisfy `resting` and be eaten as an offset. What would separate them is a
      gate that also asks whether the *velocity* has stayed near zero, which the filter has and
      nothing consults.
- [ ] **Whether the filter's stillness gates survive the sensor's noise.** The boot gate is
      one unbroken second of a 400 ms window whose per-axis range stays under `STILL_SPREAD`,
      and `resting` is decided one sample at a time; both were written and measured against
      noise-free synthetic motion. On the simulator, white per-sample noise of a few
      centimetres per second squared is enough that the filter never settles at all — so
      nothing is recorded, for a reason that looks exactly like a detector that never fires.
      The noise map in [docs/DETECTOR_TESTS.md](docs/DETECTOR_TESTS.md) is the instrument that
      would say where the sensor actually is: its axes are sized from the part's datasheet
      rather than from one capture, and it needs a still window from a device first, because
      the part's own noise accounts for only half of the rest spread a bar in a rack shows.
- [ ] **A real session with the raw input, and the map re-run against it.** Stage 3 of
      [docs/RAW_IMU.md](docs/RAW_IMU.md) swapped what feeds the detector and deliberately changed
      nothing else, so the first session is evidence rather than tuning: if reps are stored where
      the fused build stored none, the fused output was the problem. What it does not settle is
      whether the detector now reads travel correctly on a live bar — the maps are synthetic and
      the annotated captures are gone — so the session's rep counts are written down before it
      starts, dev mode records the windows, and the fixtures come from them.
- [ ] **Whether the detector consumes the filter's velocity.** Two integrators run in firmware
      today, deliberately, and they disagree. Folding the detector onto the filter's velocity is
      one decision taken on that session's data, not before it.

### Sync over the pipe

- [ ] **A trace waits on an acknowledgement it does not need.** Streaming is not
      the slow part: a chunk is ~8 notifications of 244 bytes and a ten-chunk trace
      ~78, the framing no longer retries, and every rep in the real session arrived
      whole. The hop before it is what costs — the request's acknowledgement lands
      a median 5 s after the write, which is the whole `xferTimeoutMs` budget often
      enough that most requests were re-sent, and the wait before the first chunk is
      ~40% of the time a trace takes. The device acts on the write immediately and
      replies at once, so the seconds are on the air or in the phone. The fix
      wanted is not a bigger number: a chunk for the rep being asked for is proof
      the request arrived, so the app should never wait on a round trip it can
      already see past. Naming which end holds the seconds needs the device's log,
      which stamps milliseconds.
- [ ] **A bench session** for what no host replaces: negotiation, subscription and
      throughput on a link this repo controls.

### BLE link stability

- [ ] **The link drops while idle.** Four drops in one session, each preceded by
      nothing but the IMU characteristic being polled, ~1.2 s before CoreBluetooth reported
      `The connection has timed out unexpectedly`. The flood that was there then is gone (see
      [docs/GOTCHAS.md](docs/GOTCHAS.md), "An acknowledgement is also a request for the next
      message"); what is left is a read a second. The flash experiment is readable from
      the device's log alone now: a millisecond stamp on every line, both ends and
      the duration of every erase and write, and the reason a disconnect gives —
      `Connection Timeout` is the supervision timer expiring over a radio that
      heard nothing, which is a flash window being felt, while `Remote User
      Terminated Connection` is the app letting go. Answered for a real session with
      no `--features fake-reps`: it still drops, four times inside an hour and a
      half, and none of them with a transfer in flight — so the flash window is not
      the one being felt. The sizes are no longer a candidate either: a trace chunk
      is ~8 notifications of 244 bytes now, and a measure of trace latency found the
      framing healthy end to end (see Sync over the pipe).
- [ ] **A device that goes quiet is invisible for minutes.** In the same session the
      device stopped answering GATT reads and the app went on asking, once a second
      for six minutes, every read failing `characteristicNotFound`, with its own
      status still saying connected — iOS only reported the disconnect at the end
      of it. Nothing on screen said anything was wrong. The app needs a failure it
      can believe: a read that cannot reach the device, unanswered often enough, is
      a dead link whatever CoreBluetooth thinks.
- [ ] **`universal_ble` throws an unhandled `TimeoutException`** when a command is
      issued against a link that has already gone (our code logs its side as
      `trace …: request failed: TimeoutException`), and nothing the caller can
      catch. Worth not making worse: do not issue commands on a link we know is
      down.
- [ ] **Values that arrive between a teardown and the next subscribe are dropped**
      by the transport: 507 of 2422 `raw` summary values in one session. Harmless
      today, because the summaries are re-streamed. Not seen since the pipe — the
      real session forwarded every notification it received — but the transport is
      unchanged, so this is unanswered rather than fixed.

### The rep detector

- [ ] **An unrack followed by a lowering is paired as a rep.** A rep is two opposite legs
      past `MIN_ROM_M`, and a one-way push is dropped only when the stroke after it goes the
      same way. A bench or squat set's first stroke after the unrack is the lowering, so the
      unrack pairs with it: the first stored rep is the unrack plus a descent, and every rep
      after that is one stroke out of phase — an ascent and the next rep's descent, which
      reads the right ROM over a window that spans the rest between two reps. With a rest
      long enough to push the shifted pair past `MAX_REP_MS`, the set stores nothing. The
      simulator cannot see it: `Rep` always drives up first, so the unrack merges with the
      first drive there instead. Nothing about the pair separates the two cases — not the
      pause, since a paused rep has one too, and not the directions — so the measurement to
      take on a session is the travel asymmetry between an unrack and a stroke, and the rule
      is a decision rather than an inference.

### First real device capture

- [ ] **A session with its set counts known on the day.** The three real reps checked in
      from a bench session were removed again because every window in it carried a jump in
      its sample times, so a test against them pinned numbers that came from bridging a
      2.2 s gap ([docs/FIXTURES.md](docs/FIXTURES.md)). The sampler's timeline is fixed,
      and the first rep stored after that is in the fixture set as `real-deadlift`
      (`sim/tests/real_capture.rs`): one clean deadlift, 285 points, no hole. What one rep
      cannot say is whether the detector caught a *set* — that needs a session whose rep
      count is known before it starts, so a miss is a count rather than an inference.
- [ ] **Whether the frame can be trusted, now that a rep is measured on it.** ROM is
      the vertical travel on Z rather than the biggest excursion, because Z is the only
      axis pinned to gravity — a rep's horizontal range is the estimate's error, not the
      bar. That makes the horizontal ranges a check that nothing is measuring: on the
      real reps pitch swings 47–82° inside one rep and roll crosses ±180° (the wrap is
      fixed, the instability is not), and five reps have less vertical travel than
      sideways. Either the clip turns during a rep or the fusion's orientation is poor,
      and either way it is the frame that has to be fixed before ROM means anything.
      Still open: whether a squat lands the right way up
      (see [docs/REP_TRACKING.md](docs/REP_TRACKING.md)).
- [ ] **What a rep does with the time it was not sampled.** A hole is motion nobody
      measured, and the trace integration bridges it as one step: the capture's most
      extreme rep is missing 2.3 s of the 5.6 s it claims, and the bridged gap lifts the
      position ~1 m in the 1.1 s after it, which is most of its 1.28 m "ROM" — for a
      deadlift, whose bar travels about 0.6 m. Three ways to answer it and the choice is
      not obvious: cap the step (what the detector already does) and a rep reads short
      instead of long — measured on that capture, five of fourteen fall under `MIN_ROM_M`
      and are lost; treat a gapped window as unusable and lose the rep outright; or fix
      the holes above and leave the integration alone.
- [ ] **The summary stats disagree on real motion.** `mean_velocity` is the mean
      of the *raw* speed while `peak_velocity` is the peak of the *detrended* one,
      so a mean above the peak means a large DC component rather than a maths bug —
      and real reps have it, on the order of five times over. Either make the two
      comparable or document the difference.

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

### Detector tests

- [ ] **A tolerance suite for the detector, driven by synthetic reps.** One motion per
      lift profile — bench, deadlift, squat — described by travel, speed in, speed out and
      a pause at each end, swept so each class's boundary of detectability is a curve
      rather than the few shapes that were captured. The first map measures `is_a_rep`
      alone (would this motion be stored); the noise map adds the impairments the part's
      datasheet allows; the false-positive map aims the same sweep at motion that is not a
      rep. The maps run by hand, `cargo test` pins the deterministic points, and the
      labelled capture stays the reference rather than being replaced. Plan:
      [docs/DETECTOR_TESTS.md](docs/DETECTOR_TESTS.md).

### Firmware

- [ ] **A rep's timeline has a hole in it.** Nine of the fourteen reps in one capture
      carry a ~2.2 s jump in their stored sample times, each followed by a burst of
      samples 1–7 ms apart, and the cause is known: the BLE arm walking the flash log on
      the sampler's thread, which it did on every idle second and on every trace request
      ([docs/GOTCHAS.md](docs/GOTCHAS.md)). Both of those walks are gone; what is left is
      to record a session and confirm the gaps with them, which is also what closes the
      capture item above. A *collect* still walks the whole log once per connection and
      once per batch of summaries, so a freeze of that shape can still land in a rep —
      making that walk incremental is the next step if a capture shows one. The sampler
      logs `sampler hole: … ms without a sample` at the gap and raises
      `REPORT_KIND_SAMPLER_HOLE`, so the boot log says when it happened and the app says a
      rep from then may read long. What to do about a rep that spans a hole is the item
      above.
- [ ] **A stored trace cannot reproduce its own summary.** The summary is a function of
      the rate its window was read at, not of the window: `real-deadlift` stores 0.937 m,
      its own stored `pos` agrees to 0.1 mm, and re-integrating its `accel`/`quat` — every
      second sample, ~20 ms — reads 1.10 m. Seeding that re-integration with the stored
      `v0` rather than zero moves it by 0.9 mm, so what is missing is the samples: the
      trace carries no gyroscope, and it keeps every second point. The state is no longer
      part of the problem — a filter replayed from a recorded state tracks the run that
      produced it (`docs/RAW_IMU.md`, stage 1) — so the raw trace format is what closes
      this, and full rate is stage 4's decision. Nor can the edges recover it — a set is fluid, so a window rarely begins
      or ends at rest, the detrend is always removing a real ramp, and the result is always
      grid-sensitive. That retires "a rep's window must begin and end at rest" as a
      reproducibility fix; if it is wanted at all it is an accuracy question, on its own.
      Storing the window at full rate fixes it, and costs, measured on that same rep:
      11,729 → 23,366 bytes, 6 → 12 chunks, 4 → 7 sectors, **156 → 89 reps in the log**,
      49 → 96 BLE frames (1.6 → 3.2 s of transfer at the ~30 notifications/s a 30 ms
      interval carries, before the handshake's ~40 %). The wall is RAM rather than flash:
      two `TracePoint` buffers of 32 B each grow by 12.4 KB apiece, so +24.8 KB against the
      ~22 KB free once the sync buffers are up — it does not fit as it stands. Cheapest way
      through is to stop storing `pos`, which is 15 of the 40.8 bytes on the wire and 12 of
      the 32 in RAM, and is the one field the other two derive: `TracePoint` drops to 20 B,
      full rate costs +1.6 KB a buffer and the log stays at 156 reps, at the price of the
      app integrating its own plot. Or stream the sync path a chunk at a time so only the
      sampler's buffer grows (+12.4 KB, ~10 KB left, above the ~8 KB cliff). Worst case is
      an 8 s rep at 840 points, ~34 KB, 10 sectors, 62 reps. Until one is chosen, the
      honest stopgap is a device-side self-check: re-integrate the trace it just wrote and
      flag a mismatch over ~10 %, so a rep that cannot be reproduced is known rather than
      plotted as truth.
- [ ] **OTA code updates** — the app partition is 1.5 MB with headroom; dual
      slots would mean shrinking the log partition.
- [ ] On-device tests with [`embedded-test`](https://crates.io/crates/embedded-test):
      the flash partition layer first, then the radio path.
- [ ] **A gentle return is not detected.** The turning-point detector ends a leg on a
      turn, and a turn is a speed: a return that never reaches `VEL_MIN` (0.07 m/s) at its
      peak never turns, so the rep is never stored. On a bench the boundary is between 0.12
      and 0.14 m/s — the very slow end of a deliberate negative. Closing a leg on
      bounded travel as well as on a turn would cover it; the cost of doing so is not yet
      measured against the maps, which is why it is not done yet. Pinned meanwhile by
      `a_gentle_return_is_not_detected_yet`, which fails with instructions when fixed. See
      [docs/GOTCHAS.md](docs/GOTCHAS.md).
- [ ] **Tune the detector against real reps.** A rep now ends where the bar turns
      around rather than at a pause, which is what a set nobody pauses through needed
      (`docs/GOTCHAS.md`), and a synthetic five-rep set with no rest in it is the test
      for that. What is still fitted to generated motion is the rest: the on and rest
      thresholds, their debounces, and the speed a turn must reach. Nothing here has
      been checked against a set whose rep count is known on the day.
- [ ] **Trim the static heap region if the deepest stack figure ever asks for it.**
      The 64 KB `heap_allocator!` region is what the linker stack competes with
      most, and its surplus is stack the linker cannot use. Each byte trimmed moves
      one-for-one into the stack and comes off the heap's margin under a
      connection, so the change wants the connection-time `heap free` in hand. See
      [docs/GOTCHAS.md](docs/GOTCHAS.md).

### Client

- [ ] **Plot performance** - camera panning is super laggy in the rep plot view 
- [ ] **Session grouping** — deliberately skipped for v1: the device log is
      undifferentiated, and grouping reps into sets is a judgement better made with
      real data on screen. The list is flat, a day at a time, newest first.
- [ ] Fault detection / coaching hints on the traces (bar dip, forward lean).
- [ ] Real-device polish: reconnect UX, sync progress, permission prompts.
- [ ] Install story: a free Apple ID gives 7-day builds; TestFlight and the App
      Store need a paid account.
