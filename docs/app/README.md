# Plan: Calypso iOS App (client)

Status: **draft for review** — no client code yet. The platform question is now
down to "website wrapped for iOS" vs "native SwiftUI"; see
["Recommendation"](#recommendation-website-first-wrap-it-later).

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
- If the client is not Rust, the `calypso-core` *code* is not reusable, but the
  generated-code path still is: both Swift Protobuf and `protobuf-ts` build from
  the same `.proto`. See "What changes if we leave Rust behind".

## How iOS apps are usually written (2026)

| Approach | What it is | BLE on iOS | Notes |
|---|---|---|---|
| **Native (Swift/SwiftUI)** | Swift + SwiftUI, UIKit drop-ins where SwiftUI struggles | CoreBluetooth, first-class | The default: ~68% of iOS devs name SwiftUI their primary framework, ~29% UIKit-first; most shipped apps mix both. Needs Xcode; free Apple ID builds expire after 7 days, $99/yr for TestFlight/App Store. |
| **Flutter** | Dart UI, own renderer | `flutter_blue_plus` | ~37% of devs use/evaluate it. Good widgets, 3D story is young. |
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

## Recommendation: website first, wrap it later

Build the UI as a **website** and get it in front of real data before writing
any Apple-specific code:

1. **M0 — web plot prototype.** A local web app that loads a fixture log
   (see "Fixtures") and renders one rep in 3D with three.js: bar path coloured
   by velocity, a quaternion-driven orientation gizmo, ground grid, timeline
   scrubber, orbit camera. No Xcode, no device, no Apple account. This is the
   fun part, and it is the highest-risk-of-being-boring part, so it goes first.
2. **M1 — the app shell around it.** Wrap the same UI in **Capacitor** to get
   CoreBluetooth on iOS via the community BLE plugin, keeping the fixture
   transport for development in a plain browser and in the Simulator.
3. **Later, if wanted.** Either keep Capacitor, or move to native SwiftUI (the
   web view feels like a web view), or add device-side WiFi so the website works
   with no install at all.

Why this ordering: it satisfies "test it without a device" literally (the whole
UI runs in a browser against fixtures), it avoids the Web Bluetooth dead end,
and it defers the Apple toolchain until there is something worth packaging.

The alternatives, if the priorities are different:

- **Native SwiftUI + CoreBluetooth + SceneKit** if you want the most "normal"
  iOS app, best feel, and are happy writing Swift. Same Simulator limitation,
  so the fixture/mock work is identical.
- **Flutter or React Native** only if Android matters soon; both handle our
  small app, and both cost an extra framework layer for 3D.

### What changes if we leave Rust behind

The device side is unaffected (it stays Rust). On the client:

- We lose the `calypso-core` reuse. Decoding still has a single source of truth
  (`proto/calypso.proto`), but the client generates its own code — Swift
  Protobuf for a Swift app, `protobuf-ts`/`protobuf-es` for a web app.
- The protocol knowledge (log record layout, chunk reassembly, BLE chunk sizing)
  is already documented in `docs/rep-tracking/README.md` §5–§6 and in the
  fixture files, so it is specifiable rather than something to rediscover.
- If we ever want shared logic again (e.g. re-deriving traces from raw
  accel+quat), Rust compiled to WASM is the escape hatch for the web path, and
  uniffi for the native path. Not needed for v1.

## Architecture (web-first path)

```
calypso-app/
  web/              the UI (plain website): three.js plot, rep list, scrubber
    ├── src/protocol/     decode RepRecord/SyncMessage (generated from proto/)
    ├── src/transport/    Transport interface + implementions
    │     ├── fixture.ts    load a fixture log (browser + Simulator dev)
    │     └── ble.ts        Capacitor BLE plugin (iOS/Android)
    ├── src/plot/         plot model (pure, unit-tested) + three.js view
    └── src/ui/           list, detail, settings
  ios/              Capacitor shell + Xcode project
  fixtures/         checked-in sample logs (.pblog)
  desktop/          optional: Tauri/Electron wrapper if a desktop app is wanted
```

The plot model stays separate from the three.js scene graph — world points, axis
fitting, decimation, colour mapping, projection — so it is unit-testable in
Node/vitest without a canvas, exactly as the Rust plan had it.

### Transport interface

Language-agnostic; the point is that the UI only ever talks to this.

```ts
interface RepTransport {
  summaries(): Promise<RepSummary[]>;          // oldest first
  subscribe(cb: (s: RepSummary) => void): Promise<void>;  // live updates
  trace(repId: number): Promise<TracePoint[]>;  // reassembled from chunks
}
```

- `FixtureTransport` — reads a `.pblog`; `subscribe()` replays it with
  configurable pacing so streaming UI is exercised for real.
- `BleTransport` — scan, connect, subscribe `0xca21`, write `0xca22`,
  reassemble `0xca23` chunks by `index`.
- `RecordingTransport` — wraps another transport and appends everything
  received to a fixture file, which is how real device logs get captured.

## Fixtures: testing without a device

The iOS Simulator cannot use Bluetooth at all (CoreBluetooth has been
unavailable in the Simulator since iOS 7), so fixtures are the only way to run
the app there — and they are what makes a browser development loop possible.
macOS *does* have CoreBluetooth, so the desktop browser can even exercise the
real BLE path (Chrome Web Bluetooth on macOS) with no phone involved.

**Format**: a *log dump* — the same `RepRecord` messages the device stores,
length-delimited, without the flash sector framing. Generated with the same
`pb::encode_summary` / `pb::encode_trace_chunk` calls the firmware uses, so a
fixture cannot drift from the wire format.

**Sources**, in the order they appear:

1. *Synthetic* — `calypso-core/tests/rep_store.rs` already builds a squat-like
   motion, runs the detector, computes the trace and encodes the records. A
   small Rust example writes `fixtures/synthetic-squat.pblog`. Deterministic,
   therefore assertable.
2. *Edge cases* — a 1-point trace, a 512-point decimated trace, a rep whose
   trace fell off the ring, extreme values, a truncated record.
3. *Real captures* — recorded from hardware through `RecordingTransport` and
   checked in as golden data. This is what catches "worked in fixtures, broke on
   device".

Dev loop: `npm run dev` then open the page with `?fixture=synthetic-squat` — no
BLE, no device, no Xcode.

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
| Fixture generation | Rust test/example, asserted round-trip through the client decoder | no |
| Client protocol decode + reassembly | Node/vitest over fixtures, including the nasty edge cases | no |
| Plot model (axis fit, decimation, colour map, projection) | vitest — pure functions over fixture traces | no |
| UI | browser runs the fixture transport; snapshot the DOM/scene state | no |
| 3D rendering | headless render (three.js + WebGL offscreen) compared against a golden PNG; plus a human eyeball pass | no |
| BLE transport | Chrome Web Bluetooth on macOS against the real device, or Bluefy on iOS; then the Capacitor shell on device | yes, for the real path |
| iOS shell | Xcode build + a manual smoke checklist: connect, sync, plot, background/foreground | yes |
| Real-device acceptance | manual checklist on the iPhone | yes |

The point of the split: **everything except the BLE implementation is testable
without hardware**, and the BLE implementation is thin enough to review by
inspection.

## Milestones

- **M0 — web plot prototype**: fixture loader + three.js 3D plot of one rep,
  plot model unit-tested. Runs in a browser, no Apple toolchain.
- **M1 — rep list + fixtures polish**: list of summaries, session grouping,
  scrubber, camera presets; all against fixtures including the edge cases.
- **M2 — fixtures from the real device**: capture logs from hardware over BLE
  (Chrome on macOS is the cheapest way to get the first real capture), check
  them in as golden data.
- **M3 — Capacitor shell**: wrap the web UI, wire the BLE transport in the
  plugin, run on the Simulator (fixtures) and then a real iPhone.
- **M4 — polish**: connection state, error handling, sync progress, install
  story (TestFlight / App Store / PWA), icon, `NSBluetoothAlwaysUsageDescription`.

## Open questions for review

1. **Platform**: the recommendation is website-first → Capacitor for iOS. The
   alternative is native SwiftUI (+ CoreBluetooth + SceneKit) if you want the
   most standard, best-feeling iOS app. Pure PWA is ruled out for syncing
   (no Web Bluetooth on iOS) unless we do device-side WiFi.
2. **Distribution**: TestFlight/App Store ($99/yr, and review), or keep it
   personal with Xcode builds (7-day expiry on a free account)? A PWA/web
   viewer sidesteps both.
3. **Device-side WiFi sync**: worth doing eventually? It buys an install-free
   website on any phone, at the cost of firmware work (HTTP server, AP/
   provisioning UX) and a second sync path.
4. **Rep-line overlay in v1?** Plotting a reference line alongside the measured
   path is what makes dip/lean obvious, and it is cheap given the axis fit — but
   it is a judgement call about what "plot one rep" means.
5. **Session grouping rule**: gap-based only, or should the app let the user
   tag/name sets (which means client-side storage and editing)?
6. **History model**: the device log is a ring that *forgets*. Should the client
   be the archive of record — keeping everything it has synced — and treat the
   device as a buffer? That is the natural answer, but it means the client owns
   durable storage (IndexedDB in the browser, SQLite/Core Data natively) and
   dedupe on re-sync. Worth deciding before M1.
7. **Wall-clock time**: device timestamps are uptime milliseconds. Timestamp
   summaries on receipt (simple, wrong if the app is backgrounded mid-set) or
   infer set start times from the first rep's receipt time?
8. **Android**: does it matter? Capacitor/Flutter/RN would cover it; a native
   SwiftUI app would not.

## Non-goals for v1

- Fault detection / coaching hints.
- Any data leaving the phone or the user's machine (no accounts, no cloud).
- Android (see open question 8).
- Editing reps/sets, notes, video.
- Live plots during a set (the device emits summaries when a rep closes, which
  is the right granularity anyway).
