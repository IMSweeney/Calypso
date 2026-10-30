# Detector tests

## Why

Measure the detector's performance, and keep measuring it while the detector changes.

Three kinds of evidence, and the suite is only worth having if it takes all three:

- **Real measurements** — the labelled session in `captures/history-2026-10-27.pblog`. The
  only ground truth about what a rep actually looks like, and the answer to "did a change
  make a real rep worse".
- **Generated fixtures** — synthetic reps swept over a range of scenarios, so what is
  detectable is a curve we can draw rather than the three shapes we happened to capture.
  What is stored is the generator, not the output: a scenario is built in the test or the
  example that needs it, so a later change is measured against the same motion and there
  is no `.pblog` to regenerate and re-commit. (The app's fixtures are the other case —
  the client loads files at runtime, so those stay checked in.)
- **Error injections** — the impairments the capture showed, added to **synthetic** reps so
  a loss can be attributed to one of them rather than to "the detector got worse". Never
  added to a captured rep: that already carries the device's own error, and adding more to
  it measures the sum of two things instead of one.

The detector has been tuned on a handful of reps and one 14-rep session. This is the
instrument that turns the next tuning change into a measurement instead of a hope.

Everything runs through the real pipeline — `Detector`, `compute_rep`, the real encoders
— and lives in `sim/`, so a scenario is a summary the device would have produced.

**Out of scope.** The sampler's timeline hole (~2.2 s gaps in stored sample times) is a
bug to fix, not an impairment to model; a suite that tolerated it would hide it. The
calibration gate, the pipe and the app are out too.

## 1. Detectability map

**What varies.** A rep is a class plus five numbers:

| knob | what it is | swept over |
|---|---|---|
| travel | the bar's vertical excursion, m | the class's nominal ± its range |
| speed in | peak speed of the drive, m/s | the nominal, and slower |
| speed out | peak speed of the return, m/s | the nominal, and slower |
| pause top | stillness at the turnaround, ms | 0 – 500 |
| pause bottom | stillness at the bottom, ms | 0 – 500 |

| class | travel | speed in | speed out |
|---|---|---|---|
| bench | 0.19 ± 0.03 m | 0.55 m/s | 0.35 m/s |
| deadlift | 0.55 ± 0.07 m | 1.00 m/s | 0.70 m/s |
| squat (provisional) | 0.70 ± 0.10 m | 1.10 m/s | 0.80 m/s |

A stroke is half a cosine at its peak speed, so its peak acceleration is `2v²/A` and its
duration `πA/(2v)`. That relation is the whole map: at the bench nominals the drive
peaks at 3.2 m/s² — the bottom of the 3.3–26 m/s² the capture's reps peaked at — and the
return at 1.3. A press is only measured whole when both strokes clear the thresholds,
and in a real press the lowering is the slower of the two. Each nominal scenario today
is exactly one stored rep — with 150 ms at the turn and 300 ms at the bottom — and the
maps measure what happens as the knobs move away from that centre. The squat class is
provisional: no squat rep has ever been captured.

**What is measured: `is_a_rep`, nothing else.** A cell is one bit — would a summary for
this motion be stored — not ROM, not span boundaries, not velocity. That is the product
question, and `MIN_ROM_M` is the only threshold in the pipeline the outside world set.

**A miss is a result, not a failure.** The map is expected to show that today's detector
stores nothing for a good share of the pause range, and that belongs in the map rather
than in a test that has to pass.

**Drawing it.** Five axes do not plot, so the map is 2-D slices: speed in × speed out at
the nominal pauses, speed in × pause top at the nominal travel and speeds, and so on. The
output is a boundary per class, not a score, and the interesting part is how far each
nominal sits inside it.

**Phase two: motion around the rep.** Several reps with the set's rest between them, and
the unrack and re-rack either side. That turns "one rep" into "the thing the device
actually records", and it is the only way to test that one rep's motion does not merge
with — or hide — its neighbours.

## 2. Noise map

The same map with impairments injected into the synthetic motion — never into a captured
rep — so a cell stops being a bit and becomes a rate over seeds. Brief for now: the axes
and what each does.

| axis | range | what it does | where the number comes from |
|---|---|---|---|
| white acceleration noise, σ | 0 – 0.2 m/s², anchored at 0.13 | changes nothing measurable — the rep still closes and reads the same ROM | the per-sample spread at rest, 13–17 mg |
| bias drift inside a rep | 0 – 0.02 m/s²/s | the residual this map is really about | half-to-half change measured at 0.003–0.02 |
| attitude offset θ | 0 – 3° | leaks the bar's own acceleration sideways: `A·sin θ` of sideways span, and `A·cos θ` of ROM — which is 0.06 % at 2°, too small to measure | the frame the fusion settles on |
| attitude drift while moving | 0 – 1 °/s | a leak that changes within the rep, which the endpoint detrend removes only partly | the gyro's zero-rate spec, and the fusion runs on the gyro while the bar moves |
| constant body bias | 0 – 0.2 m/s² | nothing at all: the velocity detrend removes a constant acceleration exactly, which `constant_bias_is_fully_rejected` already pins | the measured rest offsets, 2–11 mg |

## 3. False positives

The same sweep aimed at motion that is not a rep: a bar being unracked (large acceleration,
little travel), a re-rack, a hand on the rack, a wobble (oscillation with no net travel),
and the calibration transient the capture caught — 122° of roll in 1.4 s at 2.3 m/s², which
no size threshold can reject.

Same five knobs, different classes, and the same measurement inverted: how often is
something stored that is not a rep. With the noise map on top it becomes a rate.

Two things it needs that the detectability map does not:

- **The captured non-reps**, checked in as they are, so a change that starts admitting one
  fails in `cargo test` rather than in the field. `real-wobble` is the first (0.029 m of
  travel over 4.9 s).
- **Phase two's sets**, because the motion *between* reps is where a false positive
  comes from: an unrack, a re-rack, or a partial drive that never came back down.

## Status

Nothing here is built yet. When it is:

- the generators are in `sim/`, and a scenario is built from them where it is used — a
  generated fixture is code, not a file;
- the maps are `sim/examples/` commands — the noise map prints its seed so a run can be
  repeated — and a map's *shape* is written down here; its numbers are not, because they
  move with every tuning change;
- `cargo test` pins the few deterministic points: each nominal scenario is one stored rep,
  and the captured non-reps stay rejected.
