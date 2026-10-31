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

## 1. Detectability map — done

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

**What is measured: how many reps are stored, nothing else.** A cell is one of three —
`.`, nothing stored; `#`, exactly one; `X`, more than one, the detector splitting one rep
into two — not ROM, not span boundaries, not velocity. The count is the product question
(`MIN_ROM_M` is the threshold the outside world set): zero is a miss, one is right, and
the split is the failure a two-state cell used to hide behind a `#`.

**A miss is a result, not a failure.** The map is expected to show that today's detector
stores nothing for a good share of the pause range, and — where the travel is large
enough that each half clears `MIN_ROM_M` — that it stores a paused rep as two: the
deadlift's pause-top slice is all `X` above the rest debounce. Both belong in the map
rather than in a test that has to pass.

**Drawing it.** Five axes do not plot, so the map is 2-D slices, each with travel on the x
axis and one of the other four knobs on y: travel × speed in, travel × speed out, travel
× pause top, travel × pause bottom. Travel is the shared column because it is the
dimension a lifter actually varies, and the nominal cell is marked `*`. The output is a
boundary per class, not a score, and the interesting part is how far each nominal sits
inside it.

**Phase two** — the unrack, the set's reps, and the re-rack assembled into one stream — is
its own section, §4 Sets: it is where one rep's motion can merge with, or hide, its
neighbours.

## 2. Noise map — todo

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

## 3. False positives — done

The same sweep aimed at motion that is not a rep: a bar being unracked (large acceleration,
little travel), a re-rack, a hand on the rack, a wobble (oscillation with no net travel),
and the calibration transient the capture caught — 122° of roll in 1.4 s at 2.3 m/s², which
no size threshold can reject.

Same five knobs, different classes, and the same measurement inverted: how often is
something stored that is not a rep. With the noise map on top it becomes a rate.

**What the first sweep found.** The generator is `sim/examples/false_positive_map.rs`, two
slices over `Motion::Push` and `Motion::Wobble`. A one-way push is nearly never stored:
the velocity detrend halves the ROM of a stroke that never comes back, so an unrack has to
travel about twice `MIN_ROM_M` to register, and only in a band of speeds where the stroke is
long enough to be a rep but still fast enough to start one. A clean oscillation is stored
only while it is slow enough to rest at each turnaround; a fast shake never dwells below the
reversal threshold and produces no span at all.

Two things it needs that the detectability map does not:

- **The captured non-reps**, checked in as they are, so a change that starts admitting one
  fails in `cargo test` rather than in the field. `real-wobble` is the first (0.029 m of
  travel over 4.9 s).
- **§4 Sets**, because the motion *between* reps is where a false positive comes from: an
  unrack, a re-rack, or a partial drive that never came back down.

## 4. Sets — done

One rep in isolation cannot answer the question that matters on the day: whether a set comes
back as a set. A set is the unrack, several reps separated by the set's rest, and the re-rack —
one continuous stream, replayed through the real pipeline. It is the only place two things can be
seen:

- **A rep's neighbours.** One rep's motion can merge with, or hide, its neighbour. A merged span
  exceeds `MAX_REP_MS` and is thrown away, taking every rep in it; the real 5×5 sessions captured
  at most one rep per set.
- **The motion between reps.** The unrack and the re-rack are the false positives that live *next
  to* a rep rather than on their own.

The generator is a `Set` that assembles segments into one timestamp-continuous stream with a
single lead-in at the start:

```
Set { rep, reps, inter_rep_rest_ms, unrack: Option<Push>, rerack: Option<Push> }
```

The lead-in rest is pulled out of the per-rep generator (a `Rep`/`Push` then emits its strokes
without it), the set's rest between reps is each rep's `pause_bottom`, and the unrack/re-rack are
one-way pushes — direction is irrelevant to a detector that keys on magnitude.

**What is measured.** For each assembled set, the stored summaries are judged on four things, not
on `is_a_rep` alone: the **count** (N reps in, N reps out), the **order** (they come back in time
order), the **ROM** (each still reads its travel, the neighbours not having skewed it), and the
**edges** (neither the unrack nor the re-rack is stored). This is pins plus a `set_probe` example
that prints the detected sequence — not a 2-D map, because the result is a discrete "N in → N out",
not a boundary.

**What the probe found.** The set is measured, not fixed. A comfortable set comes back whole and
in time order; an unrack and a re-rack sit right next to a rep without being stored; and a set
nobody pauses through still splits into all five of its reps — the reversal-based close is what
kept it from merging into one span. The one blemish the probe surfaced is ROM fidelity: in a no-rest
set every rep after the first reads high, because the velocity has not settled to zero between reps.
The fix is in: the integration starts from the bar's velocity at the rep's left edge, shipped as `RepSummary.v0` (a `TraceOrigin`) so a reader re-derives from the same origin, which cut the no-rest inflation from ~25 % to a ~9 % residual and left a set with rest between reps unchanged. The residual is a second-order edge effect (the reversal-based close emits the span a few tens of ms before the true turnaround) and is tracked, not fixed, here.

## Status

- **Detectability map: done.** `sim/examples/detector_map.rs` draws the four slices (travel
  × speed in, travel × speed out, travel × pause top, travel × pause bottom) per class, with
  three cell states so an over-split shows rather than reading as a hit;
  `sim/tests/detector_tolerance.rs` pins that each nominal is exactly one stored rep and that
  the generated non-reps stay rejected.
- **Noise map: todo.** `Rng` and `Impairments` exist in `sim/src/motion.rs`, but the noise
  slice of the map and any noise pins are not written yet.
- **False positives: done.** `sim/examples/false_positive_map.rs` sweeps `Push` (the
  unrack/re-rack/bump) and `Wobble` (the oscillation). The captured non-rep stays pinned in
  `sim/tests/real_capture.rs`.
- **Sets: done.** The `Set` generator is in `sim/src/motion.rs`, `sim/examples/set_probe.rs` prints
  the detected sequence, and `sim/tests/sets.rs` pins that a set comes back whole, in order, with the
  unrack/re-rack unstored. The no-rest set splits correctly, and its later reps read their travel now
  that the integration is seeded from the bar's left-edge velocity (`RepSummary.v0`) — a small edge
  residual remains. The calibration transient (a roll, a fusion artifact
  rather than a stroke) is still open.

Throughout, a generated fixture is code, not a file: a scenario is built where it is used, so
only the generators, the map example and the pins are committed. A map's *shape* is written
down here; its numbers are not, because they move with every tuning change.
