# Plan: Calypso iOS App (client)

Status: **draft for review** — no app code yet (see "Milestones" for the M0 spike
that settles the open shell question).

## Goal

An iOS app that syncs reps from the device over BLE and shows the lifter what
happened. v1 is deliberately narrow:

1. **View rep summaries** — a list of reps recorded on the device.
2. **Plot a single rep in 3D** — the bar/sensor path plus orientation, so bar
   dip and forward lean are visible.

Fault *detection* stays off the device and out of v1: the app plots the traces
and the lifter interprets them. (Automatic "you dipped left on rep 3" hints are
a natural v1.5 once traces are on screen.)

Two hard requirements from review:

- **Rust** for as much of the app as is practical.
- **Testable without a real device.**

## What we can build on

The device side already defines the contract, and it is shared code:

| Already exists | Reusable by the app |
|---|---|
| `proto/calypso.proto` | Same messages both sides; single source of truth |
| `calypso-core::pb` | Decode `RepRecord` / `SyncMessage` / `TraceChunk` |
| `calypso-core::rep` | `TracePoint`, `compute_rep` (re-derive traces later) |
| `calypso-core::store` | Log-record layout, summary/trace read-back |
| `calypso-core/tests/rep_store.rs` | Synthetic rep generator -> fixtures |

Two consequences worth calling out:

- The app decodes **exactly** the bytes the firmware wrote, using the same code
  path the firmware's tests use. No protocol drift, no second parser.
- Because traces keep raw accel + quaternion, the app can re-derive
  displacement later without a firmware update. That is why the raw inputs are
  stored in the record at all.

**Caveat to resolve:** `calypso-core` also carries the BLE GATT table (`ble.rs`
needs `trouble-host`) and the flash log (`flashlog.rs` needs
`embedded-storage`). An iOS app should not link a BLE peripheral stack. Plan:
feature-gate `calypso-core` (`gatt`, `flash`, default-on) so the app depends on
just the data layer, or split out a `calypso-data` crate. Decide in M1.

## Client shell options

The decision that shapes everything else: what draws the UI.

| Option | UI language | iOS status (2026) | 3D | Notes |
|---|---|---|---|---|
| **A. SwiftUI shell + Rust core (uniffi)** | Swift + Rust logic | Native, boring, reliable | SceneKit/RealityKit (free camera, ~50 lines) | Rust does all data/math/sync; Swift is UI glue. Best BLE reliability (CoreBluetooth, Apple-supported). 3D is Swift, but the *plot model* stays Rust. |
| **B. Slint** | Slint markup + Rust | Official iOS docs (XcodeGen + `cargo build --target aarch64-apple-ios`) | Weak — no plotting engine; custom wgpu embedding is unstable/gated | **Licensing: GPL-3.0-only or commercial.** GPL conflicts with App Store distribution unless we get a royalty-free/commercial licence. |
| **C. egui + wgpu (eframe)** | Rust | No official iOS target; runs via `winit`, community examples; `cargo-mobile2` has an egui template | Good — `egui_wgpu::CallbackTrait` makes a custom 3D pass straightforward, and it renders headless on the host for tests | MIT/Apache. Desktop-first workflow is a real advantage for 3D iteration. iOS is the risk. |
| **D. Dioxus / Tauri 2 mobile** | HTML/CSS/JS frontend, Rust backend | Both ship mobile support; Tauri mobile has known gaps (keyboard, lifecycle) | Via JS libs (three.js) | UI is not Rust, which defeats the request. |
| **E. Bevy / Makepad** | Rust | Bevy has iOS examples (`bevy_ios_*`, `cargo-xcode-build-rs`); Makepad ships mobile | Excellent (Bevy) | A game engine for a 13-row list and one line plot is a lot of engine. Keeping on the shelf. |

### Recommendation: Path A for v1, with a Rust-shaped seam

Ship a **thin SwiftUI shell** around a **Rust core** exposed through **uniffi**,
with the 3D plot initially in **SceneKit**.

Rationale, in order of weight:

1. **BLE is the riskiest part of this app, and it is the one part where Rust
   loses to native.** `btleplug` claims iOS support (its CoreBluetooth backend
   is shared with macOS, and the README lists iOS), and `objc2-core-bluetooth`
   exists for direct bindings, but neither is a first-class, well-trodden path
   like CoreBluetooth-in-Swift. Bluetooth is also the bit that can't be tested
   in the Simulator, so a bug there costs a device round-trip every time.
2. **SceneKit is ~50 lines to a camera-controllable 3D line plot.** A custom
   wgpu renderer is days of work (shaders, depth, camera, gestures) and is the
   kind of thing where v1 dies. "One polyline plus a ground grid" needs no
   engine.
3. **The Rust/not-Rust seam is drawn where it pays off.** Decoding, reassembly,
   session grouping, derivation and plot geometry are Rust and host-tested.
   What stays Swift is view code and gesture wiring.
4. **The 3D decision is reversible.** Because the plot model (world points, axis
   fitting, colour mapping, decimation) lives in Rust and is unit-tested,
   swapping SceneKit for a Rust wgpu view later is a renderer change, not a
   rewrite.

Path B is the strongest *all-Rust* alternative and its blocker is licensing
rather than technology — worth a conversation if "all Rust" is non-negotiable.
Path C is attractive if desktop-first iteration matters more than iOS polish;
its risk is that eframe-on-iOS is community-supported, which the spike should
prove before we commit.

### M0 spike (do this first, ~half a day)

Two throwaway prototypes, then decide:

1. **Path A**: Rust staticlib + uniffi → a SwiftUI screen that receives a `Vec`
   of structs from Rust and renders a SceneKit polyline. Confirms the bridge,
   the Xcode/XCFramework flow, and the Simulator story.
2. **Path C**: an `eframe` app rendering an `egui_wgpu` 3D polyline on macOS;
   then an honest attempt to get it running on the Simulator.

Whichever survives, keep. Record the outcome in this document as a decision
section rather than leaving it implicit.

## Architecture (assuming Path A)

```
calypso-app/
  app-core/        Rust: no UI, no BLE. Host-tested with `cargo test`.
    ├── sync.rs      sync client: summary stream + trace reassembly
    ├── session.rs   grouping reps into sets/workouts (gap-based)
    ├── model.rs     app state: rep list, selected rep, loading/error
    ├── plot.rs      plot model: world points, axes fit, decimation, colouring
    └── fixtures.rs  load a fixture log into the same state a device would
  app-ble/         Rust: transport implementations
    ├── transport.rs  `trait RepTransport` (async)
    ├── ble.rs        btleplug (or objc2-core-bluetooth) — device only
    └── fixture.rs    fixture/replay transport — Simulator + host
  app-ios/         Rust: uniffi surface (staticlib) + the Xcode project
  app-desktop/     Rust: host binary for fast iteration (fixtures, and real
                   BLE on macOS, since CoreBluetooth works there)
  fixtures/        checked-in sample logs (see "Fixtures")
```

`app-core` and `app-ble` are ordinary Rust crates that build and test on the
host with no Apple toolchain. Only `app-ios` needs Xcode.

Dependency direction: `app-core` → `calypso-core` (data layer only, per the
feature-gating note above).

### Transport trait

The seam that makes "no device required" real rather than aspirational:

```rust
trait RepTransport {
    /// Summaries already on the device, oldest first.
    async fn summaries(&mut self) -> Result<Vec<RepSummary>, TransportError>;
    /// Follow the device as new reps arrive (BLE notify; replayed on fixtures).
    async fn subscribe(&mut self) -> Result<SummaryStream, TransportError>;
    /// One rep's trace; mirrors the wire protocol's chunking.
    async fn trace(&mut self, rep_id: u32) -> Result<Vec<TracePoint>, TransportError>;
}
```

- `BleTransport` — scan, connect, subscribe `0xca21`, write `0xca22`, reassemble
  `0xca23` chunks by `index`.
- `FixtureTransport` — reads a fixture log; `subscribe()` replays it with
  configurable pacing so streaming UI is exercised for real.
- `RecordingTransport` — wraps another transport and appends everything
  received to a fixture file. This is how real device logs get captured and
  checked in as regression data.

## Fixtures: testing without a device

The iOS Simulator cannot use Bluetooth at all (CoreBluetooth has been
unavailable in the Simulator since iOS 7), so fixtures are the only way to run
the app there — and they also make host tests and CI possible. Meanwhile macOS
*does* have CoreBluetooth, so a desktop build exercises the real BLE path with
no phone in the loop.

**Format**: a *log dump* — the same `RepRecord` messages the device stores,
length-delimited, without the flash sector framing. Generated with
`pb::encode_summary` / `pb::encode_trace_chunk`, i.e. the identical code the
firmware runs, so a fixture cannot drift from the wire format.

**Sources**, in the order they appear:

1. *Synthetic* — `calypso-core/tests/rep_store.rs` already builds a squat-like
   motion, runs the detector, computes the trace and encodes the records. A
   small `app-core` example does the same and writes
   `fixtures/synthetic-squat.pblog`. Deterministic, therefore assertable.
2. *Edge cases* — deliberately nasty fixtures: a 1-point trace, a 512-point
   decimated trace, a rep at the ring's wrap boundary (its trace already
   overwritten), extreme values, a truncated record.
3. *Real captures* — recorded from hardware through `RecordingTransport` and
   checked in as golden data. This is what catches "worked in fixtures, broke
   on device".

CLI: `cargo run -p app-desktop -- --fixtures fixtures/synthetic-squat.pblog`
runs the whole app with no BLE, on macOS.

## Features, v1

### Rep list

From the summary stream. Fields available per rep:

- `rep_id`; `t_start_ms` / `t_end_ms` (device uptime — the app must map these to
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
- **Orientation** — a body-axis gizmo (or a tilted ground plate) at the scrubbed
  position, driven by that sample's quaternion.
- **Ground grid + world axes**, so "level" is legible.
- **Timeline scrubber** — drag through the rep; the gizmo and a marker on the
  path follow. Start/end markers on the path.
- Orbit/zoom camera plus "reset view" (fit the path's bounding box).

Deliberately not in v1: meshes or mannequins, multi-rep overlays, video,
annotations.

### Session grouping

The device stores an undifferentiated rep log, so grouping into sets is an app
concern. Simplest rule that works: a new set starts when successive reps are
more than N minutes apart, computed in `app-core::session` and unit-tested
against fixture timelines.

## Testing strategy

| Layer | How it is tested | Device needed |
|---|---|---|
| `calypso-core` data layer | the existing 67 host tests | no |
| `app-core` sync / reassembly / sessions | `cargo test`, driven by fixtures | no |
| `app-core` plot model (projection, axis fit, decimation, colour map) | `cargo test`, pure functions over fixture traces | no |
| Fixture transport, end to end | desktop binary + Simulator run | no |
| 3D rendering | offscreen wgpu render on the host, compared against a golden PNG; plus a human eyeball pass | no |
| BLE transport | exercised on macOS against the real device; a fake peripheral later if it earns its keep | yes, for the real path |
| iOS shell | Swift XCTest for the small amount of view logic; the rest is Rust | no |
| Real-device smoke test | manual checklist: connect, sync, plot a rep, background/foreground | yes |

The point of the split: **everything except the BLE implementation is testable
without hardware**, and the BLE implementation is thin enough to review by
inspection.

## Milestones

- **M0 — shell spike** (above). Outcome: this document gains a Decision section;
  the repo gains a hello-world app running on the Simulator.
- **M1 — data layer**: feature-gate `calypso-core`; `app-core` sync client,
  reassembly and sessions; fixture format + generator + `FixtureTransport`;
  `RecordingTransport`. All host-tested. No UI.
- **M2 — list UI**: SwiftUI list of summaries from fixtures, then from a real
  device. Smallest end-to-end slice that shows real data.
- **M3 — 3D plot**: plot model in Rust, SceneKit view, scrubbing, camera.
- **M4 — real BLE + polish**: scan/connect UX, "no device connected" state,
  error handling, sync progress, App Store prerequisites (icon, privacy string
  `NSBluetoothAlwaysUsageDescription`; no background modes needed for v1).

## Open questions for review

1. **Shell**: Path A (SwiftUI + Rust core, recommended) vs Path B (all-Rust
   Slint, licensing-dependent) vs Path C (all-Rust egui/wgpu, iOS less proven)?
   The M0 spike is the tiebreaker.
2. **3D rendering**: SceneKit first (fast, native), swapping to Rust/wgpu later
   only if we want one renderer across platforms? Or go straight to wgpu because
   "3D in Rust" was part of the ask?
3. **Rep-line overlay in v1?** Plotting a reference line alongside the measured
   path is what makes dip/lean obvious, and it is cheap given the axis fit — but
   it is a judgement call about what "plot one rep" means.
4. **Session grouping rule**: gap-based only, or should the app let the user
   tag/name sets (which means app-side storage and editing)?
5. **History model**: the device log is a ring that *forgets*. Should the app be
   the archive of record — keeping everything it has synced, forever — and treat
   the device as a buffer? That is the natural answer, but it means the app owns
   durable storage (SQLite vs a flat append-only record file) and dedupe on
   re-sync. Worth deciding before M1, since it shapes `app-core`.
6. **Wall-clock time**: device timestamps are uptime milliseconds. Timestamp
   summaries on receipt (simple, wrong if the app is backgrounded mid-set) or
   infer set start times from the first rep's receipt time?
7. **Android later?** Path A makes the UI a second implementation (SwiftUI →
   Compose). Path B/C would share it. Doesn't change v1, but it is the main
   argument for an all-Rust shell.

## Non-goals for v1

- Fault detection / coaching hints.
- Any data leaving the phone (no accounts, no cloud sync).
- Android.
- Editing reps/sets, notes, video.
- Live plots during a set (the device emits summaries when a rep closes, which
  is the right granularity anyway).
