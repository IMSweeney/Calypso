# Plan: Calypso iOS App (client)

Status: **draft for review** — no client code yet. Framework choice has narrowed
to Flutter vs React Native; see the Recommendation section.

## Goal

An iOS app that syncs reps from the device over BLE and shows the lifter what
happened. v1 is deliberately narrow:

1. **View rep summaries** — a list of reps recorded on the device.
2. **Plot a single rep in 3D** — the bar/sensor path plus orientation, so bar
   dip and forward lean are visible.

Fault *detection* stays off the device and out of v1: the app plots the traces
and the lifter interprets them. (Automatic "you dipped left on rep 3" hints are
a natural v1.5 once traces are on screen.)

One hard requirement from review: **it must be testable without a real device.**
(An earlier draft assumed Rust; that requirement was dropped in favour of using
whatever makes the best client — see "How iOS apps are usually written".)

## What we can build on

The device side already defines the contract:

| Already exists | What the client gets from it |
|---|---|
| `proto/calypso.proto` | The wire format, as one source of truth for generated code |
| BLE service + UUIDs (`calypso-core::ble`) | What to scan for and subscribe/write |
| `calypso-core::store` / `flashlog` | Log-record layout the client must decode |
| `calypso-core/tests/rep_store.rs` | Synthetic rep generator -> fixtures |
| `docs/rep-tracking/README.md` §5-6 | The sync protocol in prose (chunking, MTU budget, subscribe semantics) |

Consequences worth calling out:

- The **protocol is already exercised by the firmware's own tests**, so the
  client has something concrete to decode against from day one.
- Because traces keep raw accel + quaternion, the client can re-derive
  displacement later without a firmware update. That is why the raw inputs are
  stored in the record at all.
- The client reimplements decoding rather than reusing `calypso-core`, but the
  single source of truth still holds: Dart classes are generated from
  `proto/calypso.proto`, exactly as the firmware's Rust types are.

## How iOS apps are usually written (2026)

| Approach | What it is | BLE on iOS | Notes |
|---|---|---|---|
| **Native (Swift/SwiftUI)** | Swift + SwiftUI, UIKit drop-ins where SwiftUI struggles | CoreBluetooth, first-class | The default: ~68% of iOS devs name SwiftUI their primary framework, ~29% UIKit-first; most shipped apps mix both. Needs Xcode; free Apple ID builds expire after 7 days, $99/yr for TestFlight/App Store. |
| **Flutter** | Dart UI, own renderer (Impeller) | `flutter_reactive_ble`, `universal_ble`, or `flutter_blue_plus` (paid licence for for-profit use since 2.0) | ~37% of devs use/evaluate it. Self-rendered, so iOS/Android/tests look identical. |
| **React Native / Expo** | JS/TS UI on native widgets | `react-native-ble-plx` (maintained, supports iOS background BLE) | Needs a dev build — not Expo Go — because of the native module. |
| **Capacitor / Ionic** | A real website packaged in a native shell | community BLE plugin (`@capacitor-community/bluetooth-le`) | You keep the web stack *and* get CoreBluetooth. |
| **Pure web / PWA** | Website, optionally installed to the home screen | **No.** See below. | Zero install friction, but no device access on iOS. |

## Can it be a website?

The plotting and browsing parts: absolutely — the web is arguably the best place
to do 3D line plots (three.js, WebGPU in Safari 26+), and the payloads are
tiny. The syncing part is the problem, and it is a hard blocker, not a
polish issue:

- **Web Bluetooth is not implemented in Safari/WebKit on iOS.** As of Safari
  27 (2026) caniuse still shows it unsupported, and WebKit has no stated plan to
  ship it. Chrome/Edge/Opera on Android and desktop *do* support it.
- Workarounds both force the user to install something: **Bluefy** (a free
  third-party Web BLE browser, still maintained) or the **iOSWebBLE** Safari
  extension that polyfills `navigator.bluetooth` via CoreBluetooth. Fine for a
  hobby demo, not something to build a daily-use app on.
- Other iOS PWA limits worth knowing: no background sync, no background fetch,
  ~50MB storage cap with aggressive cache eviction, push notifications only on
  iOS 16.4+ and only when installed, and standalone mode was removed in the EU
  under the DMA.
- Apple's own guidance: if you need Bluetooth/NFC/USB, build a native app.

Two honest ways to still get a web UI:

1. **Capacitor** — the same website, wrapped in a native shell, with BLE via the
   community plugin. Web development loop *and* Bluetooth.
2. **Device-side WiFi** — the firmware already initialises WiFi (it's how BLE
   coex is set up). The device could serve HTTP/WebSocket instead of BLE, at
   which point a plain, install-free website works on any phone. This is real
   firmware work (HTTP server, AP/provisioning UX) and a different sync path
   from the BLE service we already built and tested, but it is the only route to
   "zero install, works everywhere" — worth keeping on the roadmap rather than
   ruling out.

## Recommendation: Flutter (React Native a close second)

Flutter, with the plot drawn by hand rather than in a 3D engine. The reasoning
is mostly about this app's two requirements.

| Dimension | Flutter | React Native / Expo |
|---|---|---|
| Language | Dart | TypeScript/JS |
| 3D line plot | Project the points with `vector_math` and draw in a `CustomPainter`: ~200 lines, GPU-accelerated via Impeller, no engine needed. `flutter_scene` exists but is young. | three.js + `@react-three/fiber` via `expo-gl` — by far the richest 3D toolbox. But the docs warn iOS Simulator GL support is unreliable and can crash; "always test on a physical device". |
| Testing without a device | Best in class for our need: `flutter test` runs headless on the host, and **golden-file image comparison** catches visual regressions in the plot with no simulator. | Jest + Testing Library for logic; headless visual/3D testing is weak; Detox/Maestro need simulators. A `react-native-web` build gives a browser dev loop. |
| BLE | `flutter_reactive_ble` (BSD-3, Philips Hue, actively maintained) or `universal_ble` (BSD-3, also macOS/web). `flutter_blue_plus` has the nicest API but a paid licence for for-profit use since 2.0, covering development and testing. | `react-native-ble-plx` upstream is at 3.5.1; a fork supports Expo SDK 57/RN 0.86 and is *asking for maintainers*. Expect version churn between RN/Expo and the plugin. |
| UI consistency | Draws its own widgets (Impeller), so it looks identical on iOS, Android, macOS, web, and in tests. | Uses platform widgets: a more native feel on iOS, but more platform divergence to reason about. |
| Reusing calypso-core later | `flutter_rust_bridge` is a mature path if we ever want the Rust trace math on the client. | uniffi or a TurboModule; more plumbing. |

Why Flutter for this app:

1. **The 3D plot does not need a 3D engine.** It is a polyline, a small
   orientation gizmo and a ground grid. Projecting with `vector_math` and
   painting it is straightforward, keeps full control over the look, and avoids
   a dependency whose simulator support is documented as flaky.
2. **It gives the strongest answer to "testable without a real device".** The
   rendering is testable headlessly *and* the pixels can be pinned with golden
   files — the plot, the list and the scrubber can all be asserted in CI with no
   simulator, which is precisely what the iOS Simulator's lack of CoreBluetooth
   otherwise forces us to work around.
3. **BLE has a free, mockable option.** `flutter_reactive_ble` is federated over
   a platform interface, which makes a fake transport natural in tests;
   `universal_ble` additionally covers macOS (handy for desktop-side BLE testing
   on the Mac) and web.
4. **One rendering path everywhere**, including `flutter run -d macos` as a fast
   dev loop that needs no simulator.

When React Native would win instead: if you already write React/TS day to day
(velocity beats everything else), if you want the same codebase to run as a
website via `react-native-web` with three.js doing the plot on the web, or if the
plot might grow into real 3D scenes (meshes, a mannequin) where three.js is far
ahead. The simulator-3D caveat would then just mean plotting gets eyeballed on a
real phone.

Cheapest way to settle it: the M0 prototype below, written in whichever
framework you are more productive in — a fixture-loaded 3D polyline is an
afternoon either way, and it will tell you more than this table.

## Architecture (Flutter)

```
calypso-app/
  lib/
    main.dart
    protocol/       decode RepRecord/SyncMessage (generated from proto/)
    transport/      RepTransport + implementations
      fixture.dart    load a .pblog asset (Simulator, desktop, widget tests)
      ble.dart        flutter_reactive_ble (device)
      recording.dart  capture real logs as fixtures
    plot/           plot model (pure Dart) + CustomPainter-based 3D view
    ui/             rep list, rep detail, scrubber, camera controls
  test/             unit, widget and golden tests
  assets/fixtures/  checked-in .pblog sample logs
  ios/              Flutter iOS project
```

The plot model (world points, axis fitting, decimation, colour mapping,
projection maths) stays separate from painting, so it is unit-testable as plain
Dart and the painter stays dumb.

### Transport interface

```dart
abstract class RepTransport {
  /// Summaries already on the device, oldest first.
  Future<List<RepSummary>> summaries();

  /// Follow the device as new reps arrive (BLE notify; replayed on fixtures).
  Stream<RepSummary> watchSummaries();

  /// One rep's trace, reassembled from chunks.
  Future<List<TracePoint>> trace(int repId);
}
```

- `FixtureTransport` — reads a `.pblog` asset; `watchSummaries()` replays it
  with configurable pacing so streaming UI is exercised for real.
- `BleTransport` — scan, connect, subscribe `0xca21`, write `0xca22`, reassemble
  `0xca23` chunks by `index`.
- `RecordingTransport` — wraps another transport and appends everything received
  to a fixture file, which is how real device logs get captured.

## Fixtures: testing without a device

The iOS Simulator cannot use Bluetooth at all (CoreBluetooth has been
unavailable in the Simulator since iOS 7), so fixtures are the only way to run
the app there — and they are also what makes host-side widget and golden tests
possible.

**Format**: a *log dump* — the same `RepRecord` messages the device stores,
length-delimited, without the flash sector framing. Generated with the same
`pb::encode_summary` / `pb::encode_trace_chunk` calls the firmware uses, so a
fixture cannot drift from the wire format.

**Sources**, in the order they appear:

1. *Synthetic* — `calypso-core/tests/rep_store.rs` already builds a squat-like
   motion, runs the detector, computes the trace and encodes the records. A
   small Rust example writes `assets/fixtures/synthetic-squat.pblog`.
   Deterministic, therefore assertable.
2. *Edge cases* — a 1-point trace, a 512-point decimated trace, a rep whose
   trace fell off the ring, extreme values, a truncated record.
3. *Real captures* — recorded from hardware through `RecordingTransport` and
   checked in as golden data. This is what catches "worked in fixtures, broke on
   device".

Dev loop: `flutter run -d macos --dart-define=FIXTURE=synthetic-squat` (or a
debug menu picker) — no BLE, no device, no simulator.

## Features, v1

### Rep list

From the summary stream. Fields available per rep:

- `rep_id`; `t_start_ms` / `t_end_ms` (device uptime — the app maps these to
  wall-clock on receipt, since the device has no RTC)
- `duration_ms`
- `rom` (range of motion, metres), `peak_velocity`, `mean_velocity`,
  `peak_accel`
- `roll_min/max_deg`, `pitch_min/max_deg` — the "did the bar dip / did the
  lifter lean" numbers, useful directly in the list
- `trace_len`, `chunk_count`, `decimated` — whether a trace is available and how
  detailed it is

Display: one row per rep (id, duration, ROM, peak velocity, tilt range), grouped
by session. Tap → rep detail.

### Single-rep 3D plot

Each point carries ZUPT-anchored displacement (metres, world frame), raw accel,
quaternion and timestamp. What v1 draws:

- **Bar path** — the displacement polyline, coloured by velocity magnitude. A
  squat shows a down-and-up excursion; a dip shows the path bowing to one side.
- **Orientation** — a body-axis gizmo at the scrubbed position, driven by that
  sample's quaternion.
- **Ground grid + world axes**, so "level" is legible.
- **Timeline scrubber** — drag through the rep; the gizmo and a marker on the
  path follow. Start/end markers on the path.
- Orbit/zoom camera plus "reset view" (fit the path's bounding box).

Deliberately not in v1: meshes or mannequins, multi-rep overlays, video,
annotations.

### Session grouping

The device stores an undifferentiated rep log, so grouping into sets is an app
concern. Simplest rule that works: a new set starts when successive reps are
more than N minutes apart, unit-tested against fixture timelines.

## Testing strategy

| Layer | How it is tested | Device needed |
|---|---|---|
| `calypso-core` (device side) | the existing 67 host tests | no |
| Fixture generation | Rust example, plus a Dart decode round-trip test over the same bytes | no |
| Protocol decode + chunk reassembly | `flutter test` unit tests over fixtures, including the edge cases | no |
| Plot model (axis fit, decimation, colour map, projection) | `flutter test` — pure Dart over fixture traces | no |
| Plot rendering | golden-file tests (`flutter test --update-goldens`), no simulator | no |
| UI (list, detail, scrubber) | widget tests driven by `FixtureTransport` | no |
| BLE transport | `flutter_reactive_ble` against the real device (Simulator cannot); `universal_ble` on macOS for the same code path | yes, for the real path |
| iOS build / signing | Xcode build, then a manual smoke checklist on the phone | yes |
| Real-device acceptance | manual checklist: connect, sync, plot a rep, background/foreground | yes |

Everything except the BLE implementation is testable without hardware, and the
BLE implementation is thin enough to review by inspection.

## Milestones

- **M0 — plot prototype**: Flutter project + plot model + `CustomPainter` 3D
  view of one rep from a fixture, with a golden test and manual camera controls.
  Runs on the Mac (`flutter run -d macos`) with no Apple mobile toolchain.
- **M1 — list + polish**: rep list, session grouping, timeline scrubber, camera
  presets; all against fixtures including the edge cases.
- **M2 — real fixtures**: capture logs from hardware (a throwaway BLE CLI, or
  Chrome Web Bluetooth on the Mac) and check them in as golden data.
- **M3 — BLE in the app**: `BleTransport` on a real iPhone; Simulator still runs
  fixtures.
- **M4 — polish**: connection state, error handling, sync progress, install
  story (TestFlight / App Store / personal builds), icon, and the
  `NSBluetoothAlwaysUsageDescription` privacy string.

## Open questions for review

1. **Framework**: Flutter (recommended) vs React Native/Expo — see the
   comparison above. Settled by the M0 prototype if it is close.
2. **BLE package**: `flutter_reactive_ble` (BSD-3, mobile only) vs
   `universal_ble` (BSD-3, also macOS/web) vs `flutter_blue_plus` (nicest API,
   paid licence for for-profit use since 2.0, including development/testing).
3. **Plot ambition**: keep it a custom-painted polyline (recommended) or adopt a
   real 3D scene later for meshes/mannequins?
4. **Distribution**: TestFlight/App Store ($99/yr) vs personal Xcode builds
   (7-day expiry on a free account)?
5. **History model**: the device log is a ring that *forgets*. Should the client
   be the archive of record — keeping everything it has synced — and treat the
   device as a buffer? That is the natural answer, but it means the client owns
   durable storage (SQLite or an append-only file) and dedupe on re-sync. Worth
   deciding before M1, since it shapes the data layer.
6. **Rep-line overlay in v1?** Plotting a reference line alongside the measured
   path is what makes dip/lean obvious, and it is cheap given the axis fit — but
   it is a judgement call about what "plot one rep" means.
7. **Session grouping rule**: gap-based only, or should the app let the user
   tag/name sets (which means client-side storage and editing)?
8. **Wall-clock time**: device timestamps are uptime milliseconds. Timestamp
   summaries on receipt (simple, wrong if the app is backgrounded mid-set) or
   infer set start times from the first rep's receipt time?
9. **A web viewer too?** Flutter compiles to web, so the same plot could be
   served as a page (fixtures, then maybe device WiFi later) — worth it, or
   scope creep?
10. **Android**: does it matter? Flutter/RN would both cover it.

## Non-goals for v1

- Fault detection / coaching hints.
- Any data leaving the phone or the user's machine (no accounts, no cloud).
- Android (see open question 8).
- Editing reps/sets, notes, video.
- Live plots during a set (the device emits summaries when a rep closes, which
  is the right granularity anyway).
