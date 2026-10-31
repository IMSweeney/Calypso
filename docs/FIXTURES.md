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

**None is checked in at the moment**, and the reason is worth keeping: the one session
extracted this way had every window removed again, because a stored window is not a
stream. The sampler that recorded it stalled for 2.2 s at a time and then burst to catch
up, so the times inside a window jump and the samples in the window do not cover the
motion it claims to describe — 158 ms of missing time inside a 1.4 s rep, 2.3 s inside an
8.2 s one. A test against those bytes pins numbers derived from bridging the gap, and the
first thing it did was make a detector change look worse than it was.

Re-extract from a session recorded after the sampler's timeline is fixed
([`ROADMAP.md`](../ROADMAP.md), "a rep's timeline has a hole in it"). Until then the
checked-in fixtures are all generated, and `replay_dump` labels every window it is given
with the timeline it found, so a session can be audited before anything is extracted from
it.

**Replaying one through the detector**: from `sim/`, `cargo run --example replay_dump
-- <path.pblog>` prints the spans today's detector finds in each stored window, and
whether `MIN_ROM_M` would store them.
