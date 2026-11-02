# Fixtures

The app is developed and tested against checked-in device data, so no work on the
client needs hardware.

## What a `.pblog` is

A `.pblog` is a **log dump**: the `RepRecord` messages the device stores, without the
flash sector framing, written as a sequence of

```
[u32 little-endian length][record bytes]
```

The records come from the firmware's own encoders (`pb::encode_summary`,
`pb::encode_trace_chunk`), so a fixture cannot drift from the wire format — they are
the bytes the device would put in flash. The client's history file uses the same
framing, so **a history file, a fixture and a capture are one format**: a capture is
just a copy of somebody's archive.

That includes the `BootClock` record a dump opens with. A stored rep carries its
uptime, not a time: the boot's epoch is a record of its own, and the client resolves
it the way the device does — the newest clock record at or before the summary — so a
fixture's reps are dated by the fixture and not by when a test read it
([`APP_SYNC.md`](APP_SYNC.md)). A summary before its boot's clock record is left
dateless, which is what a capture of a boot that had no clock when those reps were
stored looks like.

A dump may end mid-record (a torn write, or a capture interrupted). A reader must
report that rather than ignore it, and still return the records before it.

## Generating them

The generators are a crate of their own, `sim`, which depends on `calypso-core` and
nothing else does. They drive the real detector, the real trace maths and the real
encoders, so a fixture is the pipeline's own output — and the firmware's library never
carries a generator it does not call.

```sh
cd sim
cargo run --example gen_fixtures
```

## Reading them

In the app, fixtures are consumed through:

- `FixtureTransport` — loads an asset and replays summaries with pacing, so the UI
  sees a realistic stream. The app seeds its archive from
  `--dart-define=FIXTURE=<name>` (default `synthetic-squat`, empty for none), and a
  seeded run **opens on the day the fixture's reps are dated**, so the demo shows the
  session that was recorded instead of an empty today.
  A fixture is dated by its own boot clock — the generators use `FIXTURE_EPOCH_S`,
  1 January 2026 — and nothing shifts it. That is what the fixed epoch buys: the
  seeded page is reproducible, and a widget test that hands the screen the same
  instant as its clock sees the fixture's reps as today's.
- `MemoryArchive` — loads a fixture's records synchronously for widget tests.
- Golden tests — `test/goldens/*.png` pin the squat, a scrubbed squat, a single point
  and the `dip-left` fixture.

## Adding to the set

1. Add or extend a generator in `sim/src/fixtures.rs` (or build the record
   by hand for a degenerate case, as the single-point fixture does).
2. Add it to `sim/examples/gen_fixtures.rs`, with a comment saying what it is for.
3. Run the generator and commit the `.pblog` — fixtures are checked in deliberately.
4. Assert on it from the Rust and/or Dart suite, so it is load-bearing rather than a
   file nobody reads.

**A real capture**: with the app connected to a device, the archive file
(`history.pblog` in the app support directory) *is* a `.pblog`. Copy it into
`assets/fixtures/` and commit.

A session is far more interesting than a whole file is small, so a fixture is usually a
few reps pulled out of one:

```sh
cd sim
cargo run --example extract_capture -- \
  history.pblog ../calypso-app/assets/fixtures/real-reps.pblog 0f64b98f a64bfad6 0dd91dac
```

Every record is copied byte for byte, so a fixture made this way carries the same bytes
the device wrote. Each prefix must match a rep or the command fails.

**`real-deadlift` is the one that is**, and it is one rep rather than a session: the
first rep the device stored after the sampler's timeline was fixed — a 5.7 s deadlift,
285 points with 19–21 ms between them, so its window is motion the device measured. It is
read by `sim/tests/real_capture.rs`.

An older session's windows were all removed again, and the reason is worth keeping: a
stored window is not a stream. The sampler that recorded them stalled for 2.2 s at a time
and then burst to catch up, so the times inside a window jumped and the samples did not
cover the motion the window claimed — 158 ms of missing time inside a 1.4 s rep, 2.3 s
inside an 8.2 s one. A test against those bytes pins numbers derived from bridging the
gap, and the first thing it did was make a detector change look worse than it was.

A clean timeline is necessary but not sufficient. The window is the motion span with
`MARGIN_MS` asked for before it and whatever had been sampled after it — the tail is cut
at the close, so its margin is a few samples at most — and on `real-deadlift` even the
200 ms before the onset are inside motion. Nothing replays such a window as a stream: a
detector fed it cold closes no span, and `compute_rep` re-run over its own trace reads
1.10 m where the summary stored 0.94, because the trace keeps every second sample and
this motion's travel depends on the step it is integrated at.

So audit a session with `replay_dump` — it labels every window with the timeline it found
and the peak acceleration inside it — and extract only what that says is sound. A whole
session with its set counts known on the day is still what the detector wants
([`ROADMAP.md`](../ROADMAP.md), "first real device capture"). The device's own log does
not need the app: `espflash read-flash 0x190000 0x270000 log.bin`, then `cargo run
--example device_log -- log.bin --dump <out.pblog> <uuid-prefix>`, which is how
`real-deadlift` was made.

**Replaying one through the detector**: from `sim/`, `cargo run --example replay_dump
-- <path.pblog>` prints the spans today's detector finds in each stored window, and
whether `MIN_ROM_M` would store them.
