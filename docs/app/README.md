# Architecture: Calypso iOS app (client)

The client side of Calypso: it syncs reps from the device over BLE, lists rep
summaries, and plots a single rep in 3D.

This document describes the **design** as decided. For the implementation plan,
phases and the TODO checklist, see [`plan.md`](plan.md). For the device side and
the wire protocol, see [`docs/rep-tracking/README.md`](../rep-tracking/README.md).

## Stack (decided)

| Part | Choice | Why |
|---|---|---|
| Language / UI | **Flutter** (Dart) | Self-rendered with Impeller, so iOS, macOS, Android and tests all look identical; `flutter test` runs the whole UI headlessly on the host. |
| BLE | **`universal_ble`** (BSD-3, Navideck) | One API across Android/iOS/macOS/Windows/Linux/Web, so the BLE path can be exercised on the **Mac**, where CoreBluetooth exists — the iOS Simulator has none. Federated, so the plugin layer is mockable too. |
| 3D plotting | **`vector_math` + `CustomPainter`** | The plot is a polyline, a gizmo and a grid — no 3D engine needed. Full control of the look, GPU-accelerated, and golden-testable. |
| Wire format | Dart classes generated from `proto/calypso.proto` | Same source of truth as the firmware, which generates Rust from the same file. |
| State | `ChangeNotifier` + `provider` | Small app, small dependency. Easy to revisit. |
| Tests | `flutter test` (unit + widget) and golden files | No simulator, no device. |

## Hard constraints (from the platform)

- **The iOS Simulator has no CoreBluetooth at all** (dropped in iOS 7). A
  fixture transport is therefore mandatory to run the app at all in the
  Simulator — this is a design constraint, not a testing nicety.
- **BLE notifications are MTU-bound.** At the 247-byte ATT MTU BLE stacks
  commonly negotiate, a payload is 244 bytes, so traces arrive **4 trace points
  per notification**, re-chunked by the device (see rep-tracking plan §5). The
  client reassembles by chunk `index`.
- **The device log is a ring that forgets.** The device keeps the newest ~2.4 MB
  and erases the oldest sectors on wrap. The client is the archive of record;
  the device is a buffer.
- **The device has no RTC.** Timestamps are uptime milliseconds (`t_start_ms`,
  `t_end_ms`, and relative `t_offset_ms` within a trace).
- **Nothing may require a device to develop against.** Any code path that can
  only run on hardware must be thin and isolated behind a transport.

## Layering

```
        UI (Flutter widgets)              lib/ui/
              │  reads state, sends intents
              ▼
        App state / controllers           lib/state/
              │  ChangeNotifier
              ▼
   ┌──────────┴───────────┐
   │                      │
   ▼                      ▼
 Sync client            Plot model        lib/sync/, lib/plot/
 (reassembly, cursors)  (pure Dart: axis fit, decimation,
   │                     colour map, projection, camera)
   ▼                      │
 RepTransport             ▼
   │                   Renderer
   ├── FixtureTransport   (CustomPainter)  lib/plot/
   ├── BleTransport
   └── RecordingTransport
   ▼
 Codec: RepRecord / SyncMessage / TraceChunk   lib/protocol/
 (generated from proto/calypso.proto)
```

Two rules hold the testability story together:

1. **The plot model never touches the canvas.** World points, axis fitting,
   decimation, colour mapping, camera and projection are pure functions over a
   `List<TracePoint>` — unit-testable, and reusable by any renderer.
2. **The sync client never touches Bluetooth.** It talks to `RepTransport`, so
   the entire data path runs in a plain `flutter test` with no platform code.

## Project layout

```
calypso-app/
  lib/
    main.dart
    model/          domain types (RepSummary, TracePoint) converted from protobuf
    protocol/       generated message classes + record/chunk decoding
    sync/           sync client, trace reassembly, session grouping
    transport/      RepTransport + fixture / ble / recording implementations
    plot/           plot model (pure) + CustomPainter renderer + interactions
    state/          change notifiers: rep list, selection, connection
    ui/             screens and widgets
  test/             unit, widget and golden tests
  assets/fixtures/  checked-in .pblog sample logs
  ios/              Flutter iOS project
  tool/             codegen/refresh scripts (proto -> Dart, fixtures)
```

## Protocol client

Decoding is generated: `proto/calypso.proto` → Dart (`protobuf` package), the
same messages the firmware encodes with `micropb`.

Two record kinds arrive from the device:

- **`RepSummary`** (`SyncMessage.summary`) — auto-streamed when a rep closes.
  Keyed by `rep_id`, monotonic from the device. Fields the UI uses: duration,
  `rom`, peak/mean velocity, peak accel, `roll_min/max_deg`,
  `pitch_min/max_deg`, `trace_len`, `chunk_count`, `decimated`.
- **`TraceChunk`** (`SyncMessage.trace`) — on demand in response to a
  `TraceRequest { rep_id, all_chunks, chunk_index }` written to `0xca22`.

Sync state machine:

```
Idle ──connect──► SyncingSummaries ──cursor caught up──► Live
                        │                                 │
                        └── app requests a trace ─────────┘
                                      │
                                      ▼
                              StreamingTrace(rep_id)
                              (reassemble by index, then Idle/Live)
```

- Summaries are **incremental**: the client keeps a cursor of how many it has
  stored, and the device streams the rest. A wrap on the device means old ids
  disappear; the client re-syncs from the oldest id it can see.
- Traces are **idempotent**: a partial stream can simply be requested again.
- Requests for a trace the device no longer holds are answered with a warning on
  the device side, so the client must time out and surface "trace unavailable".

## Transports

```dart
abstract class RepTransport {
  /// Summaries on the device, oldest first.
  Future<List<RepSummary>> summaries();

  /// Follow the device as new reps arrive.
  Stream<RepSummary> watchSummaries();

  /// One rep's trace, reassembled from chunks.
  Future<List<TracePoint>> trace(int repId);
}
```

| Implementation | Used by | Notes |
|---|---|---|
| `FixtureTransport` | Simulator, macOS dev loop, widget/golden tests | Loads a `.pblog` (see below); `watchSummaries()` replays with configurable pacing so the UI sees a realistic stream. |
| `BleTransport` | Real device (and the Mac) | `universal_ble` behind a `BleLink` seam, so the protocol logic is testable with a fake and no radio: scan for the service, connect, **discover services**, subscribe `0xca21` and `0xca23`, write `0xca22`, reassemble chunks by index, complete exactly on `totalChunks` (idle timeout as a fallback for older firmware), and surface connection state. Discovery is not optional: on iOS and macOS the plugin resolves a characteristic against the peripheral's discovered service list, which CoreBluetooth leaves empty until `discoverServices` runs — and it reports that as `characteristicNotFound`, naming a UUID that is perfectly correct. `FakeBleLink` throws in the same order, so the suite catches a regression. |
| (capture) | Capturing golden data | Not a transport: a capture is a copy of the archive file, which is already `.pblog` — `cp history.pblog fixtures/capture.pblog`. |

### Logging

The pipeline is [`package:logging`](https://pub.dev/packages/logging) — levels,
named loggers (which is what a "tag" is) and one `Logger.root.onRecord` stream for
appenders to hang off. The only part it does not ship is the file, and that is
`lib/logging/rotating_file_appender.dart`: one line per record,
`<timestamp> <LEVEL> <loggerName> <message>`, rotating at 10 MB and keeping three
(`calypso.log`, `calypso.log.1`, `calypso.log.2`) under the app support directory,
path printed at startup as `calypso: log at …`.

It is switched on by `lib/logging/setup.dart`, which is also where the one
load-bearing line lives: `Logger.root.level = Level.ALL`. Records below the root
level are dropped *before* any appender sees them, the default is `Level.INFO`, and
the failure mode is a log file that exists, opens and stays empty — pinned by
`logging_test`.

Because the pipeline is global, "no log file in a test" is the absence of a call: a
test that never calls `startFileLogging` attaches nothing and writes nothing, with
no seam to thread through and no real I/O under the fake clock. Tests that want to
read the records call `captureLogs()` (`test/support/logging.dart`), which unhooks
itself at the end — a listener otherwise outlives the test that added it.

**BLE is the only writer so far**: `BleTransport` reports every message it sends
and receives under the `ble` logger, one line each, because a console scrolls away
and the questions this is for ("did the device send anything at all, and what were
the bytes?") get asked on a phone with nothing attached. The appender swallows
every failure — if it cannot open a file, logging is simply a no-op — because a log
that can break the connection it is watching is worse than no log.

## Plot model and 3D view

Input: a rep's `List<TracePoint>` — each with ZUPT-anchored displacement (m),
raw accel, quaternion and `t_offset_ms`.

Plot model (pure Dart):

- **Axis fit** — bounding box of the path, expanded to a cube with a minimum
  size, so a small rep is not stretched into a misleading shape.
- **Decimation** — a trace can be up to 512 points; the model picks a stride
  that keeps rendering cheap on low-end devices without hiding the shape.
- **Colour mapping** — speed → colour ramp, so slow (grinding) and fast (snappy)
  parts of the rep read differently. Derived client-side from consecutive
  positions: the trace carries no velocity field.
- **Camera** — orbit (azimuth, elevation, distance) plus a "fit" command that
  frames the path's bounding box; projection to 2D and back, so hit-testing a
  scrubbed position is testable without a canvas.
- **Level reference** — the "level" plane/grid the path is measured against.

Renderer (`CustomPainter`), drawn in this order: ground grid → axes → path
polyline (per-segment colour) → start/end markers → scrub marker.

Interaction: drag to orbit, pinch to zoom, a timeline scrubber that moves the
scrub marker and drives the orientation gizmo from that sample's quaternion.

**Axis convention caveat**: the trace is in the device's own gravity-aligned
world frame, whose *up* axis depends on how the sensor is mounted. Until real
captures exist, the plot draws that frame as-is and labels axes generically;
an axis-mapping setting is planned once we have looked at real squats (see the
plan's open question 1).

## The device's GATT, as the client sees it

| UUID | Direction | What it carries |
|---|---|---|
| `0000ca20-b0cd-11ec-871f-d45ddf138840` | service | advertised, so the scan can filter on it |
| `0000ca21-…` | notify | `SyncMessage` with a `summary` body (auto-streamed) |
| `0000ca22-…` | write | a bare `TraceRequest` (see the note below) |
| `0000ca23-…` | notify | `SyncMessage` with a `trace` body (on demand) |

Note the asymmetry: both notification characteristics carry `SyncMessage`
envelopes, while the write on `0xca22` is a **bare** `TraceRequest` (the firmware
decodes it directly). Trace chunks complete on `totalChunks`; the client also
tolerates a device that predates that field by treating a quiet stream as ended.

Chunk size is the device's choice, dictated by the ATT MTU it negotiated: Apple
platforms manage the MTU themselves and historically negotiated ~185 bytes, so the
device streams three trace points per notification. The client does not care how
many arrive per chunk — that is what `totalChunks` is for.

## Sessions and history

The device stores an undifferentiated rep log, so grouping is a client concern:

- **Session grouping** — gap-based: a new session starts when consecutive reps
  are more than `session_gap` apart (tunable, default ~10 min).
- **History** — the client is the archive (see constraints). Proposed storage is
  an append-only file of raw `RepRecord` bytes — i.e. **the same `.pblog` format
  the fixtures use** — plus a small index. That keeps one format for live data,
  archives and test fixtures, and makes a captured fixture just a copy of a
  history file. SQLite remains an option if querying needs grow.

## Fixtures (`.pblog`)

A `.pblog` is a *log dump*: the same `RepRecord` messages the device stores,
without the flash sector framing, written as a sequence of
**`[u32 little-endian length][record bytes]`**. Because the records come from the
firmware's own `pb::encode_summary` / `pb::encode_trace_chunk` calls, a fixture
cannot drift from the wire format. The Rust reference implementation of the
framing is `calypso_core::fixtures::read_dump`; the Dart reader must match it,
and must report a trailing partial record (torn capture) rather than ignore it.

Fixtures are generated, not hand-written:

```text
cargo run -p calypso-core --example gen_fixtures
```

which writes `calypso-app/assets/fixtures/`: a normal three-rep set, a
single-point trace, a decimated long rep, a summary whose trace fell off the
device ring, extreme values, a truncated dump, and an empty dump.

Note on decimation: the device keeps at most 512 trace points per rep, and
`compute_rep` anchors its decimation grid at the **last** sample, so a rep's end
is always stored. Trace capacity is also why a long rep can come back with
`decimated = true` — the fixture for that case is built by running the trace
computation directly, because a smooth synthetic 10 s squat has peak
accelerations (~0.2 m/s²) below the detector's 0.6 m/s² start threshold.

Sources:

1. **Synthetic** — a Rust example in `calypso-core` reuses the motion generator
   from `calypso-core/tests/rep_store.rs`, runs the detector, computes the trace,
   and writes `assets/fixtures/synthetic-squat.pblog`. Deterministic, so tests
   can assert against it.
2. **Edge cases** — a 1-point trace, a 512-point decimated trace, a rep whose
   trace fell off the ring, extreme values, and a truncated record.
3. **Real captures** — recorded from hardware via `RecordingTransport` and
   checked in as golden data. This is what catches "worked in fixtures, broke on
   device".

## Testing

| Layer | How | Device |
|---|---|---|
| Plot model | `flutter test` unit tests over fixtures (axis fit, decimation, colour, projection, camera fit) | no |
| Protocol decode + reassembly | `flutter test` over fixture bytes, including the edge cases | no |
| Sync client | `flutter test` with a scripted fake transport (out-of-order chunks, missing trace, re-sync after wrap) | no |
| UI | widget tests driven by `FixtureTransport` | no |
| Plot rendering | golden files (`flutter test --update-goldens`) — pixels compared headlessly | no |
| BLE transport | real iPhone; `flutter_reactive_ble`'s platform interface can be faked for logic tests | yes, for the real path |
| Build/signing | Xcode build + manual smoke checklist | yes |

Everything except the BLE implementation is testable without hardware.

## Dev loop and distribution

- `flutter run -d macos` — fastest loop for UI/plot work; no simulator, no Apple
  mobile toolchain. CoreBluetooth is available on the Mac, so the real BLE path
  runs here too.
- macOS needs the `com.apple.security.device.bluetooth` entitlement (already set
  in `macos/Runner/*.entitlements`); without it the plugin reports `unsupported`
  and nothing names the cause.
- `flutter run --dart-define=FIXTURE=synthetic-squat` (or a debug menu) — the
  whole app on fixtures, including on the Simulator.
- Real device: `flutter run -d <iphone>`; a free Apple ID gives 7-day builds,
  $99/yr gives TestFlight and App Store.
- iOS needs `NSBluetoothAlwaysUsageDescription` in `Info.plist`; no background
  BLE modes for v1.

## Non-goals

- Fault detection or coaching hints (the app plots; the lifter interprets).
- Multi-rep overlays in v1, meshes/mannequins, video, annotations.
- Any data leaving the phone: no accounts, no cloud sync.
- Android for v1 (nothing in this design prevents it later).
- Live plotting during a set (the device emits summaries when a rep closes,
  which is the right granularity anyway).
