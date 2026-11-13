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

## The captures

A capture is not a fixture. A capture is a whole recording — a session, or a boot, or a
device's own archive — and a fixture is what a test or a plot needs out of one. The
bytes of a capture are recorded data: they are far too big to be source, they never
change, and they therefore live in an object store rather than in the repository. What
the repository keeps is `captures/captures.json`, which says what each one is, the
session notes beside it, and the plots drawn from them.

`tools/captures.sh` moves the bytes, and takes the store from `CAPTURES_REMOTE` — an
`mc` alias and a bucket, so no bucket name becomes a fact in here:

```sh
export CAPTURES_REMOTE=r2/calypso-captures
tools/captures.sh ls
tools/captures.sh push                      # every capture in `captures/`
tools/captures.sh pull working-sets-2026-11-10.pblog
```

`calypso_sim::captures` reads the manifest and a capture's bytes, from `captures/` if it
has been pulled and out of the bucket if not, so a test or an example names a capture
and never asks where it is. `sim/tests/captures.rs` runs as part of `cargo test`, so it
needs the capture pulled or `CAPTURES_REMOTE` naming the bucket: with neither it fails
rather than passing on nothing. The two tests in `sim/tests/real_capture.rs` are
`#[ignore]`d and run on purpose with `cargo test -- --ignored`.

The manifest also carries what a session did — the reps in each set — for the captures
where somebody wrote that down at the time. `sim/tests/captures.rs` replays such a
recording through the real filter and detector and asserts the sets it closes are the
sets that were performed, and no others. That is what makes a detector change answerable
against a real session instead of against a fixture.

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
(`history.pblog` in the app support directory) *is* a `.pblog`. The capture itself is recorded
data, so it goes to the bucket with an entry in `captures/captures.json`; what is committed is a
cut of it.

A session is far more interesting than a whole file is small, so a fixture is usually a
few reps pulled out of one:

```sh
cd sim
cargo run --example extract_capture -- \
  history.pblog ../calypso-app/assets/fixtures/real-reps.pblog 0f64b98f a64bfad6 0dd91dac
```

Every record is copied byte for byte, so a fixture made this way carries the same bytes
the device wrote. Each prefix must match a rep or the command fails.

**The one that is** is `captures/sets-2026-11-04.pblog`: three still windows and two
hand-held sets, replays of motion the device measured rather than motion a generator
made. `sim/tests/real_capture.rs` is what reads it.

An older session's windows were all removed again, and the reason is worth keeping: a
stored window is not a stream. The sampler that recorded them stalled for 2.2 s at a time
and then burst to catch up, so the times inside a window jumped and the samples did not
cover the motion the window claimed — 158 ms of missing time inside a 1.4 s rep, 2.3 s
inside an 8.2 s one. A test against those bytes pins numbers derived from bridging the
gap, and the first thing it did was make a detector change look worse than it was.

A clean timeline is necessary but not sufficient. The window is the motion span with
`MARGIN_MS` asked for before it and whatever had been sampled after it — the tail is cut
at the close, so its margin is a few samples at most — and on a window cut tight to its
motion even the 200 ms before the onset are inside it. Nothing replays such a window as a
stream: a detector fed it cold closes no span, and `compute_rep` re-run over its own trace
reports different travel than the summary stored, because the trace keeps every second
sample and travel depends on the step it is integrated at.

So audit a session with `raw_dump` — it labels every window with the timeline it found
and the peak acceleration inside it — and extract only what that says is sound. A whole
session with its set counts known on the day is still what the detector wants
([`ROADMAP.md`](../ROADMAP.md)). The device's own log does
not need the app: `espflash read-flash 0x190000 0x270000 log.bin`, then `cargo run
--example device_log -- log.bin --dump <out.pblog> <uuid-prefix>`, which is how a session
only the device saw becomes a `.pblog` here.

**Replaying one through the detector**: from `sim/`, `cargo run --example replay_capture
-- <path.pblog>` prints the spans today's detector finds in each stored window, and `--spans`
says where each rep's travel came from — the trace's own range, or the state the window was
seeded with.
