# Fixtures

The app is developed and tested against checked-in device data, so no work on the client needs hardware.

## What a `.pblog` is

A `.pblog` is a **log dump**: the `RepRecord` messages the device stores, without the flash sector framing, written as a sequence of

```
[u32 little-endian length][record bytes]
```

The records come from the firmware's own encoders (`pb::encode_summary`, `pb::encode_trace_chunk`), so a fixture cannot drift from the wire format — the same bytes the device would put in flash. The same framing is used for the client's history file, which means **a history file, a fixture and a capture are
one format**: a capture is just a copy of somebody's archive.

A dump may end mid-record (a torn write, or a capture interrupted). A reader must report that rather than ignore it, and still return the records before it.

## Generating them

```sh
cargo run -p calypso-core --example gen_fixtures   # from the repo root
```

## Reading them

In the app, fixtures are consumed through:

- `FixtureTransport` — loads an asset and replays summaries with pacing, so the UI
  sees a realistic stream. The app seeds its archive from
  `--dart-define=FIXTURE=<name>` (default `synthetic-squat`, empty for none).
- `MemoryArchive` — loads a fixture's records synchronously for widget tests
  (real file I/O never completes under a widget test's fake clock).
- Golden tests — `test/goldens/*.png` pin the rendering of the squat, a scrubbed squat, a single point and the `dip-left` fixture.

## Adding to the set

1. Add or extend a generator in `calypso-core/src/fixtures.rs` (or build the
   record by hand for a degenerate case, as the single-point fixture does).
2. Add it to `examples/gen_fixtures.rs` with a comment saying what it is for.
3. Run the generator and commit the `.pblog` file — fixtures are test data and are
   checked in deliberately.
4. Assert on it from the Rust and/or Dart suite, so it is load-bearing rather than
   a file nobody reads.

**Adding a real capture**: with the app connected to a device, the archive file
(`history.pblog` in the app support directory) *is* a `.pblog`. Copy it into
`assets/fixtures/` and commit.
