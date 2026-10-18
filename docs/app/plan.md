# Plan: Calypso iOS App (client)

Status: **in progress** — phases 1-4 done (scaffold, fixtures, decoding, sync
client, 3D plot); phase 4 (rep list, sessions, persistence) is next.

Design lives in [`README.md`](README.md) (architecture). Device side and the
wire protocol: [`docs/rep-tracking/README.md`](../rep-tracking/README.md).

## Goal

An iOS app that syncs reps from the device over BLE, lists what was recorded,
and plots a single rep in 3D so bar dip and forward lean are visible.

v1 scope:

1. **View rep summaries** — a list of reps, grouped into sessions.
2. **Plot a single rep in 3D** — the bar path plus orientation, scrubbable.

Requirements carried over from the earlier rounds of review:

- **Testable without a real device.** This is the binding requirement: the iOS
  Simulator has no CoreBluetooth, so the app must be fully exercisable from
  fixtures.
- Flutter, with `flutter_reactive_ble` for BLE.

## Decisions

**Stack (settled):**

- **Flutter**, because it renders its own UI: the app looks the same on iOS,
  macOS, Android and in tests, and `flutter test` runs widget tests and
  **golden-file image comparisons headlessly on the host**. That is the
  strongest available answer to the no-device requirement, and it covers the
  plot itself, not just the logic.
- **`universal_ble`** for BLE: BSD-3, actively maintained, one API across
  Android/iOS/macOS/Windows/Linux/Web. Chosen over `flutter_reactive_ble` for
  exactly one reason — the Mac has CoreBluetooth and the Simulator does not, so
  this is what makes the BLE path testable without a phone.
- **Plot by hand** (`vector_math` + `CustomPainter`) rather than in a 3D engine.
  The plot is a polyline, a gizmo and a grid; an engine would add a dependency
  and lose control of the look.
- **Dart classes generated from `proto/calypso.proto`**, so the client and the
  firmware share one wire-format source of truth.
- **`ChangeNotifier` + `provider`** for state: small app, small dependency, easy
  to revisit.
- **Client is the archive of record**; the device log is a buffer (it is a ring
  that erases the oldest sectors). Proposed storage: an append-only file of raw
  `RepRecord` bytes — the same `.pblog` format fixtures and captures use.

**Rejected, and why:**

- **Native SwiftUI + CoreBluetooth + SceneKit** — the most "standard" iOS app
  and the best native feel, but it splits the platform story and cannot match
  Flutter's headless visual testing. Still the obvious fallback if Flutter's
  iOS feel disappoints.
- **React Native / Expo** — a genuinely attractive option (three.js /
  `@react-three/fiber` is a far richer 3D toolbox, and `react-native-web` would
  give a website from the same code). Rejected because the 3D path is
  documented as *unreliable in the iOS Simulator* (GL support; the docs say to
  always test 3D on a physical device), which collides with the no-device
  requirement, and because the BLE plugin ecosystem currently has version churn
  (upstream `react-native-ble-plx` at 3.5.1, with a fork needed for Expo
  SDK 57 / RN 0.86 that is asking for maintainers).
- **A website / PWA** — Web Bluetooth is not implemented in Safari/WebKit on
  iOS and WebKit has no plan to ship it, so a website cannot sync from the
  device on an iPhone. (Android Chrome does support it, so a web viewer remains
  possible later — Flutter compiles to web, which would reuse this UI.)
- **Capacitor (web UI in a native shell)** — would keep a web front end with a
  real BLE plugin, but a webview UI plus a bridge is a worse fit than Flutter
  for a small, self-contained app; revisit only if the website becomes the
  primary target.
- **A Rust client core** (uniffi or `flutter_rust_bridge`) — would let the
  client reuse `calypso-core`'s decoding and trace maths. Not worth the bridge
  for v1; the decoding is generated from the same `.proto`, and re-deriving
  traces client-side is a v2 nicety. `flutter_rust_bridge` is the path if it
  ever earns its keep.
- **`flutter_blue_plus`** — the nicest BLE API, but since 2.0 it needs a paid
  licence for for-profit use ($999/yr for an individual) that explicitly covers
  development and testing, versus BSD-3 for `flutter_reactive_ble`.
- **`flutter_reactive_ble`** — BSD-3, well maintained (Philips Hue), federated.
  Rejected only for being Android/iOS only, which would have kept the BLE path
  untestable without a phone.

**Open design details folded in as they were settled:**

- The plot is drawn in the device's own gravity-aligned world frame; the *up*
  axis depends on sensor mounting, so axis mapping waits for real captures.
- Traces arrive 4 points per notification (244-byte payload budget); the client
  reassembles by chunk `index` and treats re-requesting as idempotent.
- Summaries sync incrementally with a cursor; a device-side ring wrap means old
  ids vanish and the client re-syncs from the oldest id it can see.

## Architecture

See [`README.md`](README.md) for the layered design, project layout, plot model,
transport interface, fixture format and testing matrix. Summary:

```
ui ──► state ──► sync client ──► RepTransport ──► { fixture | ble | recording }
                    │
                    └──► protocol (generated from proto/calypso.proto)

plot model (pure Dart) ──► CustomPainter renderer
```

Two invariants make the testability story hold: the plot model never touches the
canvas, and the sync client never touches Bluetooth.

## Phases

### 1. Scaffold and fixtures

`flutter create` the app inside `calypso-app/`, with iOS and macOS targets.
Generate Dart message classes from `proto/calypso.proto` (checked-in generated
output or a `tool/` codegen script, decided here). Add a Rust example in
`calypso-core` that reuses the motion generator from
`calypso-core/tests/rep_store.rs` to write `synthetic-squat.pblog` plus the
edge-case fixtures, and a Dart test that round-trips every fixture through the
decoder. This phase ends with `flutter test` decoding real fixture bytes and no
UI at all.

### 2. Protocol and sync client

`RepRecord` / `SyncMessage` / `TraceChunk` decoding, chunk reassembly by index,
and the sync state machine (summary cursor, incremental sync, wrap re-sync,
trace request/timeout). Driven by a scripted fake transport: out-of-order
chunks, a duplicate chunk, a chunk for another rep, a truncated record, and a
device that wrapped between syncs.

### 3. Plot model and 3D renderer

The pure model first (axis fit, decimation, velocity colour ramp, camera orbit,
projection and hit-testing), unit-tested against fixtures; then the
`CustomPainter` view: grid, axes, path, markers, scrub marker, orientation
gizmo. Golden tests pin the rendering. Ends with one rep plottable from a
fixture on macOS with orbit/zoom and a working scrubber.

### 4. Rep list, sessions and persistence

Rep list from the summary stream (id, duration, ROM, peak velocity, roll/pitch
range), session grouping by gap, rep detail navigation, and the history store
that makes the client the archive (append-only `.pblog` + index, dedupe on
re-sync). All against fixtures.

### 5. BLE transport

`BleTransport` on top of `flutter_reactive_ble`: scan for the service, connect,
subscribe to summaries, request traces, surface connection state, handle
disconnects and re-sync. This is the only phase that needs a device.

### 6. Polish and install

Connection/permission UX, empty and error states, sync progress, the
`NSBluetoothAlwaysUsageDescription` string, app icon, and the install story
(TestFlight vs personal builds vs App Store).

## TODO

### 1. Scaffold and fixtures
- [x] `flutter create` in `calypso-app/` (iOS + macOS targets, org `com.calypso`)
- [x] proto -> Dart codegen set up: generated Dart is checked in, and
      `calypso-app/tool/gen_proto.sh` regenerates it from `proto/calypso.proto`
      (needs `protoc` + `protoc_plugin`; run only when the .proto changes)
- [x] Rust fixture generator: `calypso-core/src/fixtures.rs` +
      `examples/gen_fixtures.rs` write `calypso-app/assets/fixtures/*.pblog`
      (squat set, single point, decimated long rep, ring-dropped summary,
      extreme values, truncated, empty), using the firmware's own encoders
- [x] Dart decode round-trip test over every fixture
      (`calypso-app/test/pblog_test.dart`, 8 tests) plus the dump reader
      (`lib/protocol/pblog.dart`) that implements the framing
- [x] `.gitignore` for Flutter/iOS build output; generated protobuf Dart is
      deliberately checked in; fixtures are checked in (they are test data)

### 2. Protocol and sync client
- [x] Record + chunk decoding into `TracePoint` / `RepSummary`
      (`lib/model/rep.dart`, `lib/protocol/pblog.dart`)
- [x] Trace reassembly by chunk index, idempotent and out-of-order tolerant
      (`lib/sync/trace_reassembly.dart`, with `missingIndices()` so a gap can be
      re-requested)
- [x] Summary merge by rep id + incremental sync, wrap-safe: the client is the
      archive of record and keeps reps the device has forgotten
      (`lib/sync/sync_client.dart`)
- [x] Trace request with timeout -> `TraceResult.unavailable` (a forgotten trace
      is an expected outcome after a wrap, not a failure)
- [x] `FixtureTransport` over a `.pblog`, with paced replay for the streaming UI
- [x] Scripted fake transport + tests (`test/sync_client_test.dart`,
      `test/trace_reassembly_test.dart`)

**Found here, since fixed on the device side:** a streamed trace had **no
end-of-stream signal**. In the log, chunks hold 48 points and the
summary's `chunk_count` describes them, but over BLE the device re-chunks to
~4 points per notification, so `chunk_count` no longer matches the number of
chunks the client should expect, and the client cannot tell "chunks still
coming" from "the device has forgotten this rep and will stay silent". The fix
is a protocol addition — either `uint32 total_chunks` on `TraceChunk` (set only
when streaming) or a status message sent when a stream starts/ends. Until then
the BLE transport has to guess from `trace_len` and the known BLE chunk size.

### 3. Plot model and 3D renderer
- [x] Axis fit to a minimum-size cube (`AxisBounds`, floor of 0.12 m half-extent
      so a tiny rep isn't stretched into a misleading shape)
- [x] Decimation for long traces — and it keeps **both** ends (start and end are
      the samples you judge a bar path by), unlike the firmware's storage
      decimation which anchors on the last sample
- [x] Speed -> colour ramp, derived from consecutive positions (the trace has no
      velocity field): blue (grinding) through green to orange (snappy)
- [x] Camera orbit + fit + projection and hit-testing (`OrbitCamera`,
      `PlotProjection`; `nearestPoint` is the "tap the path to scrub" test)
- [x] `CustomPainter`: ground grid, world axes, path coloured per segment,
      start/end markers, scrub marker
- [x] Orientation gizmo driven by the scrubbed sample's quaternion
- [x] Golden tests: squat, mid-scrub, single point, and the `dip-left` fixture
      end to end (generated by the firmware's encoders, decoded and plotted)

Phase 3 also produced the `dip-left` fixture: a squat whose lateral acceleration
is proportional to the *magnitude* of the vertical one, so the bar drifts one way
through the whole rep. The first attempt made lateral track vertical exactly,
which cancelled out into a straight line traversed twice — a *lean* that showed
as nothing. The asymmetric version bows, which is the thing worth looking at.

Two things learned by building it:

- macOS is sandboxed, so CoreBluetooth needs
  `com.apple.security.device.bluetooth` in `macos/Runner/DebugProfile.entitlements`
  and `Release.entitlements`. Without it the plugin reports `unsupported`, with
  nothing pointing at the entitlement — already set.
- Goldens are host-dependent (anti-aliasing differs across machines), so a golden
  failure on another machine is expected rather than alarming —
  `flutter test --update-goldens` and review the diff.

### 4. Rep list and persistence
Session grouping is **deliberately skipped** (decided in review): the device's log
is undifferentiated, and grouping into sets is a judgement about training
structure better made with real data on screen. v1 is a flat list of reps with
timestamps.

- [x] Rep list screen: newest first, with receipt timestamp, duration, ROM, peak
      speed, tilt range and a decimated marker (`lib/ui/rep_list_screen.dart`)
- [x] Rep detail: summary stats + the 3D plot, showing *why* when a trace is
      missing instead of plotting nothing
- [x] History store: append-only `.pblog` records (so the archive, a capture and
      a fixture are one format), no side-car index — the framing scan is the
      index — and dedupe on re-sync (`lib/store/history_store.dart`)
- [x] Torn-tail recovery: the incomplete tail rep is dropped and the file is
      truncated back to a record boundary before the next append, so a crash
      can't desync the archive — and the dropped rep is re-synced because it was
      never recorded as received
- [x] Widget tests for the list, driven through injection seams: the screen takes
      a `RepArchive` and a `RepTransport`, so tests pass an in-memory archive and
      a transport built from fixture bytes read synchronously. Fixed by
      `lib/store/rep_archive.dart` (the interface, plus `scanBlocks` and
      `assembleTraceFromBlock` shared by both implementations) and
      `lib/store/memory_archive.dart`.

Two things this fixed beyond the tests:

- **Widget tests under a fake clock never complete real I/O.** The original
  failure was `pumpAndSettle timed out`: reading a file or an asset in
  `initState` leaves the spinner running forever, because the fake clock never
  advances the real async work. The seam is the fix; `tester.runAsync` would only
  have papered over it.
- **A layout bug the tests caught:** the plot screen's stats row overflowed by
  73px. It is a `Wrap` now, so five stats reflow on a phone instead of clipping.

### 5. BLE transport
- [x] macOS entitlement (`com.apple.security.device.bluetooth`), so the BLE path
      runs on the Mac at all
- [x] Chunk sizing decided: Apple platforms manage the ATT MTU and historically
      negotiated ~185 bytes, so the device streams 3 points per notification
      (`sync::BLE_CHUNK_SAMPLES`) until it can size to the connection's own MTU
- [x] End-of-stream signal: `TraceChunk.total_chunks`, set on every streamed chunk
- [x] Scan/connect/permission flow with connection state in the UI
      (`BleTransport.connect`, `BleStatusNotifier`, `ConnectionStatusBadge`)
- [x] Subscribe `0xca21`, write `0xca22`, collect `0xca23`, reassembling by chunk
      index and completing exactly on `totalChunks` (with an idle-timeout
      fallback for firmware that predates the field)
- [x] Reconnect + re-sync: in-flight trace requests are failed, not left hanging,
      and archiving skips reps the archive already holds
- [x] Tests without hardware: the plugin sits behind `BleLink`/`UniversalBleLink`
      and a `FakeBleLink` stands in, covering scanning, subscribing, out-of-order
      and duplicate chunks, a stream without `totalChunks`, a silent device, a
      partial stream, a dropped connection, malformed notifications, and the
      whole chain from notifications to a trace read back out of the archive
- [ ] **First real capture** → checked-in fixture. Needs the hardware; the only
      part of this phase that cannot be done on the host

      Worth knowing why that step is not a formality. The first attempt to talk to
      a real device — one afternoon, several flashes between them — turned up four
      faults in this pair of programs, none of which a fake could catch:

      1. **The firmware advertised no service UUID** (only flags and the name), so
         the app's service-filtered scan never saw the device while a generic
         scanner listed it happily. Fixed by `build_advertising`, which now puts the
         service in the advertisement with the name after it, pinned by
         `calypso-core/tests/advertising.rs`.
      2. **The app subscribed without discovering services first.** On iOS and macOS
         the plugin resolves a characteristic against the peripheral's discovered
         list, which CoreBluetooth leaves empty until `discoverServices` runs — and
         it reports a lookup against that empty list as `characteristicNotFound`,
         naming a UUID that is *correct*, so the error points at the device for a
         fault in the app. `BleTransport.connect` now discovers between connecting
         and subscribing, and `FakeBleLink` throws in that order so the suite fails
         if the call is dropped.
      3. **The same UUID needs two byte orders, and I had them backwards.** In the
         GATT tree a client sees the bytes as the firmware writes them, so those are
         canonical text; in advertising data a 128-bit UUID goes on air
         least-significant byte first, so the same UUID must be *reversed* there.
         Chasing (1) I "fixed" the GATT arrays by reversing them, which is how the
         device came to expose every service as `408813df-…` while the app looked
         for `0000ca20-…`. Both directions are now pinned separately
         (`calypso-core/tests/gatt_table.rs`, `tests/advertising.rs`) with the
         observation that settled each, and `reverse16` derives one from the other.

      4. **The Mac had cached the device's old GATT database.** With the firmware
         fixed *and* reflashed, the app still reported the reversed tree: CoreBluetooth
         caches services and characteristics per peripheral, and universal_ble prefers
         that cache to asking again. The cache is keyed to the device's **Bluetooth
         address**, which reflashing does not change — so the firmware now bumps its
         random address whenever the GATT layout changes, and the app's message names
         this cause as well as the stale-firmware one, because the first version of
         that message sent us back to reflash in a circle.

      The thing that actually *found* (3) was making the app report the tree it
      discovered — `device offers …, not …` — after two rounds of guessing from
      messages that named a correct UUID and so said nothing about the device. That
      diagnostic also found (4), by still reporting the old tree after a reflash that
      should have changed it. Every fault in this list was invisible from both ends
      until two real devices were made to describe each other.

**An earlier sketch was thrown away.** The previous session's `ble_transport.dart`
was written against an invented API (`ble.scan(...)`, `conn.subscribe(char,
callback:)`, `ble.ConnectionState`) with fabricated UUIDs, and its
`RecordingTransport` wrote bare `RepSummary` messages rather than `RepRecord`
wrappers — an invalid `.pblog` that could not carry traces either. It is gone;
`BleTransport` is built on the real `universal_ble` API (checked against the
installed package), and a capture is now just a copy of the archive, which is
already `.pblog`.

Two real bugs surfaced while testing it, both worth knowing:

- **Disconnecting could hang.** `_teardownConnection` awaited the notification
  subscription cancels, and a cancel can block behind an event-loop turn that a
  fake clock never delivers (nor a wedged platform channel). The cancels are now
  fire-and-forget, with listeners nulled and straggling messages dropped.
- **A trace request could fail as an unhandled error.** The connection can drop
  between the request write and the await, completing the request before anything
  listened. The listener is attached before the write now.

### 6. Rep identity — one UUID per rep
Decided in review: **a v4 UUID per rep**, minted on the device from its hardware
RNG. Random rather than a counter or a timestamp, because the device has no RTC and
its uptime resets every boot, so neither could be unique across boots; a counter
would also need somewhere durable to live. Randomness needs no coordination and no
persistence, and it makes archives from two devices mergeable later. The price is
that UUIDs do not sort — ordering comes from the archive's append order.

- [x] Proto: `rep_uuid` (16 bytes) replaces `rep_id` on `RepSummary`,
      `TraceChunk` and `TraceRequest`; `SyncStatus` carries uuids too
- [x] Core: `calypso-core::uuid` (`new_uuid` from an injected RNG, `fixture_uuid`
      for deterministic fixtures, `short()` for logs), threaded through `pb`,
      `store`, `fixtures` and `sync`
- [x] Firmware: a UUID minted per rep from `esp_hal::rng::Rng`; trace requests,
      log lookups and log lines all keyed by uuid
- [x] Fixtures regenerated (they carry UUIDs now; the generator's are
      deterministic so fixtures stay diffable)
- [x] Client: `lib/model/uuid.dart` (canonical text <-> bytes, short form), the
      model, transports, archive and sync client all keyed by uuid, and the list
      shows a short uuid with the receipt time rather than "Rep 3"
- [x] **Dart tests migrated to uuids.** Every test file keys reps by uuid now:
      `summary.repId` and integer rep ids are gone from the tests, chunks and
      traces are named by uuid, and the list assertions use the short uuid the UI
      shows. `flutter test` is green (63 tests) and `flutter analyze` is clean.

`test/support/test_uuid.dart` was the one thing the previous session got wrong.
It claimed to mint the fixture generator's uuids, but it built the UUID bytes one
SplitMix64 byte at a time instead of the generator's four big-endian 32-bit words
(and used Dart's arithmetic `>>` where the Rust side shifts a `u64` logically), so
`u(1)` matched no fixture rep. It is a byte-for-byte port of
`calypso_core::uuid::fixture_uuid` now — verified against the checked-in fixtures,
so a test can name a fixture's rep by index (`u(0)` is `synthetic-squat`'s first
rep) and `pblog_test` can assert the identity off the wire.

## Open questions

1. **Device rep ids collide across reboots** — found while wiring the BLE sync, and
   the most consequential thing left. The firmware's counter is in RAM
   (`let mut reps: u32 = 0` in the sampler task), while the client keys its archive
   on rep id. After a reboot the device's new rep 1 collides with the archived
   rep 1: `archive.contains(1)` is true, so the new rep is **silently skipped** and
   a later trace request returns the *old* rep's trace. Every rep recorded after
   the device's first boot would be lost. Options, cheapest first:
   - persist the counter in NVS so ids keep climbing across reboots (small
     firmware change, client stays simple);
   - give the device a session/boot id and key the client on `(session, rep_id)`;
   - seed the counter from wall-clock time at boot (no NVS write, but two devices
     could collide).
   Worth settling before the first real capture, since a capture is what would
   expose it. Tracked as phase 6.
2. **Axis mapping**: which of the device's world axes is "up" for a squat? Needs a
   real capture before the plot can label anything honestly.
3. **Rep-line overlay in v1?** A reference line alongside the measured path is what
   makes dip/lean obvious, and it is cheap given the axis fit — but it is a
   judgement call about what "plot one rep" means.
4. **Android**: in or out? Nothing in the design blocks it — Flutter, and
   `universal_ble` already supports it.
5. **A web viewer**: Flutter compiles to web, so the same plot could be served as
   a page (against fixtures, or against device WiFi later). Scope creep, or a
   cheap future win?
6. **SQLite**: the archive is an append-only `.pblog` and that is enough for a
   list and a plot. Worth revisiting only if querying grows (search across
   sessions, aggregates over months).

## Decisions settled in review

- **Session grouping: skipped.** The device's log is undifferentiated and
  grouping into sets is a judgement about training structure better made with real
  data on screen. The list is flat, with timestamps.
- **Wall-clock time: stamped on receipt.** The device has no RTC, so the client
  records when it took each summary. Known flaw: wrong if the client is closed
  mid-set.
- **Storage: append-only `.pblog`.** The archive, a capture and a fixture are one
  format, so a fixture is a copy of somebody's history.
- **Client is the archive of record.** The device's log is a ring that erases its
  oldest sectors.
