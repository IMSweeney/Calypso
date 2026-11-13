# Session notes — 2026-11-10, live IMU stream through a deadlift session

Second hardware run of the live stream (`Sync.Imu`, 100 Hz) through real working sets, recorded
straight after the [bench session](session-2026-11-10-bench.md) on the same boot.

Recording: `deadlift-2026-11-10.pblog`, 1,460,634 B, beside this note (copied from the app's
`imu/imu-20261110-155013-396.pblog`).
Notes below are in local time (America/Los_Angeles), taken from when each note was typed, so
they are good to roughly half a minute — the recording's own clock is the authoritative one.

| Time | Note |
| --- | --- |
| ~15:50:16 | recording started (file created 15:50:13) |
| ~15:51:28 | warmup: 5 × 135 |
| ~15:54:34 | 5 × 185 |
| ~15:56:58 | 5 × 185 |
| ~15:59:10 | 5 × 185 |
| ~16:01:28 | 5 × 185 |
| ~16:03:10 | 5 × 185 |
| ~16:03:36 | stopped; recording saved (file last written 16:03:12) |

The set rows are each set's `noted_at` in `captures/captures.json`, and are after the set they
record.

Five sets of 5 at 185 after a 5 × 135 warmup.

## What the recording shows

- 778.9 s of stream, 9,737 chunks, 77,896 samples.
- 100.009 Hz overall; per-chunk step median 80 ms, min 78, max 81.
- `seq` 15,869 → 25,605 with no gaps.
- No `first_t_ms` step above 90 ms, so the sampler never stalled — including through all five sets.
- 150.0 B per chunk, 1,875 B/s ≈ 6.8 MB/h.

## What the detector makes of it

`cargo run --release --manifest-path sim/Cargo.toml --example replay_stream -- captures/deadlift-2026-11-10.pblog`
run over the samples: the filter settles 2.2 s in, calls 70 % of the stream still, and closes 38
spans where 30 reps were done.

| at | real reps | detected | |
| --- | --- | --- | --- |
| 5 s | — | 2 | the device being handled |
| 20–44 s | 5 (135 warmup) | 8 | reps split |
| 226–247 s | 5 | 5 | |
| 365–386 s | 5 | 8 | reps split |
| 502–526 s | 5 | 4 | one missed |
| 641–666 s | 5 | 7 | reps split |
| 744–766 s | 5 | 4 | one missed |

- Every set is found, and found on time: each cluster starts within a few seconds of its
  accelerometer block. No detections at all in the four rest breaks, which the bench session cannot
  claim.
- `rom` still runs away — up to 1.97 m off a bar that moved about half that — and the filter's
  velocity peaks at 12.0 m/s with the attitude swinging to 170° at 764.8 s, in the last set. The
  detector's spans are therefore places, not measurements.
- The over-count is the same shape as the bench session's: a long cluster whose spans chain, each
  ending where the next starts, is a set whose reps were cut into strokes.

## What carried across from the bench session

Two things span the pair, and a reader should not treat either recording as self-contained:

- **`seq` did not restart at 0.** This recording begins at 15,869, exactly where the bench one
  stopped (15,868): the link stayed up between the sessions and only dev mode toggled, so `seq` is
  per-connection, as `ImuChunk` says, not per-recording. Nothing may assume a `.pblog` starts at 0 —
  that chunk would read as a gap.
- **The device clock is continuous.** Bench ran `first_t_ms` 7,625 → 1,277,062 and deadlift
  1,545,994 → 2,324,872, under the same `BootClock` epoch (1794353067). The 268,932 ms between them
  matches the 269 s between the two files, so both recordings sit on one timeline.
