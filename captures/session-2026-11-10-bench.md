# Session notes — 2026-11-10, live IMU stream through a bench session

First hardware run of the live stream (`Sync.Imu`, 100 Hz) through real working sets on bench,
with the rep detector paused for the duration of the recording.

Recording: `working-sets-2026-11-10.pblog`, 2,387,896 B, beside this note (copied from the app's
`imu/imu-20261110-152435-080.pblog`). The [deadlift session](session-2026-11-10-deadlift.md) was
recorded next on the same boot.
Notes below are in local time (America/Los_Angeles), taken from when each note was typed, so
they are good to roughly half a minute — the recording's own clock is the authoritative one.

| Time | Note |
| --- | --- |
| ~15:24:35 | recording started (file created) |
| ~15:25:27 | device connected, recording |
| ~15:27:33 | mounted on bench; warmup set, 10 reps, bar only |
| ~15:29:21 | warmup set 2: 5 × 135 |
| ~15:31:57 | 5 × 155 |
| ~15:34:09 | 5 × 165 |
| ~15:37:39 | 5 × 165 |
| ~15:41:33 | 4 × 165 |
| ~15:45:39 | 4 × 155 |
| ~15:46:07 | stopped; recording saved (file last written 15:45:44) |

Bench session: six loaded sets, 28 reps at 135–165, plus a 10-rep bar-only warmup.

## What the recording shows

- 1,269.4 s of stream, 15,869 chunks, 126,952 samples.
- 100.006 Hz overall; per-chunk step median 80 ms, min 77, max 81.
- `seq` 0 → 15,868 with no gaps.
- No `first_t_ms` step above 90 ms, so the sampler never stalled — including through all six sets.
- One `BootClock` (epoch 1794353067) at the head, as designed.
- 150.5 B per chunk, 1,881 B/s ≈ 6.8 MB/h.

That is the result the detector pause was meant to produce: reps no longer leave a hole in the
stream. The earlier stationary run had a sampler hole from connection setup; this one has none.

## What the detector makes of it

`cargo run --release --manifest-path sim/Cargo.toml --example replay_stream -- captures/working-sets-2026-11-10.pblog`
feeds the samples through the firmware's own `Fusion` and `Detector`, from a filter that starts cold
at the recording's first sample — the device had been still for the minute before, so its filter was
ahead of this one at the start. It settles 1.5 s in and calls 73 % of the stream still.

Where the lifting is, taken from the accelerometer's own swings, and what closed against it:

| at | real reps | detected | |
| --- | --- | --- | --- |
| 5 s | — | 2 | the device being handled |
| 120–131 s | — | 3 | handling; one span is 7.2 s of it, rom 1.50 m |
| 142–163 s | 10 (bar-only warmup) | 15 | reps split |
| 189–193 s | — | 3 | rest, bench being adjusted |
| 226–274 s | 5 | 11 | reps split |
| 298–320 s | — | 12 | 23 s of motion with no set against it |
| 413–432 s | 5 | 5 | |
| 472–484 s | — | 2 | rest |
| 529–557 s | 5 | 2 | three missed |
| 750–776 s | 5 | 4 | |
| 986–1040 s | 4 | 4 | |
| 1231–1257 s | 4 | 3 | |

38 reps happened, 66 closed.

- **Sets are found where they happened.** Every cluster sits within a couple of seconds of an
  accelerometer activity block, so nothing here is a rep from the wrong part of the session.
- **The failure is per rep, not per set.** Inside a set the spans chain: each one ends on the
  instant the next begins, and their roms alternate (~0.5 m, then ~0.8 m), so one bench press is
  being closed as two — a stroke each. The 10-rep warmup made 15, the first loaded set 11.
- **Handling the bar is motion.** Two spans at setup, three in a rest, and twelve in a 23 s stretch
  at 298–320 s with no set against it.
- **`rom` is not trustworthy on this data.** It reaches 1.50 m, and the filter's own velocity peaks
  at 20.3 m/s — 7.6 m/s vertical, 11.6 m/s horizontal — where the bar never exceeds about 0.6 m/s.
  Tilt excursions reach 170°, so the attitude loses the vertical at moments, and a velocity that
  large is the leak, not the bar.
