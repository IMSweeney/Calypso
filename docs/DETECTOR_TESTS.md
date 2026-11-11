# Detector tests

## Why

Measure the detector's performance, and keep measuring it while the detector changes.

Three kinds of evidence, and the suite is only worth having if it takes all three:

- **Real measurements** — the labelled session in `captures/history-2026-10-27.pblog`, and
  the sessions recorded through the live stream beside it. The only ground truth about what
  a rep actually looks like, and the answer to "did a change make a real rep worse". A
  capture is recorded data and lives in a bucket rather than in git
  ([`FIXTURES.md`](FIXTURES.md)), so it is pulled by name before a test reads it.
- **Generated fixtures** — synthetic reps swept over a range of scenarios, so what is
  detectable is a curve we can draw rather than the three shapes we happened to capture.
  What is stored is the generator, not the output: a scenario is built in the test or the
  example that needs it, so a later change is measured against the same motion and there
  is no `.pblog` to regenerate and re-commit. (The app's fixtures are the other case —
  the client loads files at runtime, so those stay checked in.)
- **Error injections** — the impairments the capture showed and the part's datasheet allows,
  added to **synthetic** reps so a loss can be attributed to one of them rather than to "the
  detector got worse". Never added to a captured rep: that already carries the device's own
  error, and adding more to it measures the sum of two things instead of one.

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
| travel | the bar's vertical excursion, m | 50% either side of the class's nominal |
| speed in | peak speed of the drive, m/s | 0.1 – 2.0 |
| speed out | peak speed of the return, m/s | 0.1 – 2.0 |
| pause top | stillness at the turnaround, ms | 0 – 1000 |
| pause bottom | stillness at the bottom, ms | 0 – 1000 |

| class | travel | speed in | speed out |
|---|---|---|---|
| bench | 0.19 m | 0.55 m/s | 0.35 m/s |
| deadlift | 0.55 m | 1.00 m/s | 0.70 m/s |
| squat (provisional) | 0.70 m | 1.10 m/s | 0.80 m/s |

The ranges are wider than any class's own reps on purpose: the map exists to find where a class
stops being stored, and a boundary is only placed if the grid runs past it. Each class carries
its own sweep of all five knobs (`sim/src/map.rs`), so a class that needs a different grid for one
knob writes it; the ranges above are what every class is built with.

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

**A miss is a result, not a failure.** The map is expected to show regions where the
detector stores nothing. Since the detector was taught to close on the return rather than
on rest, the pause-top slices now read `#` across the swept range — a paused rep is one
rep — instead of splitting (`X`) or vanishing (`.`), and a lowering gentler than `VEL_MIN`
still closes on its rest. The `X` state stays in the map as the instrument that would show
a future re-split.

**Drawing it.** Five axes do not plot, so the map is 2-D slices, each with travel on the x
axis and one of the other four knobs on y: travel × speed in, travel × speed out, travel
× pause top, travel × pause bottom. Travel is the shared column because it is the
dimension a lifter actually varies, and the nominal cell is marked `*`. The output is a
boundary per class, not a score, and the interesting part is how far each nominal sits
inside it.

**Counting it.** `sim/examples/detector_report.rs` counts the same cells — the slices are
computed once by `Slice::scan`, and the drawing and the numbers are two views of them — and
states the result the boundary is for: the contiguous run of one-rep cells through the
nominal, in each knob's own units, with brackets where the run reached the end of the sweep
and the boundary is therefore outside the grid. That interval needs no denominator, so it can
be compared across tuning changes. The percentages in the same report do need one: the grid is
ours, so only two runs over the same grid compare.

**Phase two** — the unrack, the set's reps, and the re-rack assembled into one stream — is
its own section, §4 Sets: it is where one rep's motion can merge with, or hide, its
neighbours.

## 2. Noise map — todo

§1 asks what the detector does with one motion. This asks what it does with the motion a *part*
would have seen, so a cell stops being a bit and becomes a rate over draws. Two things follow,
and they are the whole of the design: a rate is worth nothing without the seed count written
beside it, and the model has to be the device's rather than ours — a map of our own noise is a
map of our own assumptions.

**Three lifetimes, because the errors do not share one.** A gain error belongs to a part and is
wrong all session; a temperature ramp belongs to a session; white noise belongs to a sample.
Drawing them independently per cell would average away exactly the errors that decide an
outcome — a part four percent high on the accelerometer reads every rep four percent long, it is
not sometimes high — so the draw is hierarchical: part, then session, then sample.

| tier | drawn once per | what it carries |
|---|---|---|
| part | part | both sensors' offsets and gains, cross-axis coupling, nonlinearity, the mount's tilt |
| session | session | white noise per sensor, the platform's vibration, both offsets' temperature ramp, the mount's roll through the motion, how long the motion runs without a still to correct the attitude in, sampler stalls, the rest before the first rep |
| sample | sample | the white and platform terms, quantised at the part's range setting |

**A chain, in the part's own order.** The model is not a bag of terms added to a generated
motion: the world goes into the mount's frame, then through cross-axis coupling, gain and
nonlinearity, then past the offsets, the ramp and the platform, then the noise, and only then
through the quantiser — because each is a stage the signal really passes through, and a term
applied in another order is a different sensor. The gyro is added to rather than replaced, since
a mount rolling through a stroke is the signal the attitude is carried by, and the generators
hand over motion in floating point so the quantiser is passed exactly once.

**What the part allows, and what it does to this pipeline.** Every number below is the BNO055's,
from the datasheet's electrical characteristics and register scales — not one of ours. The third
column is the point: each spec converted into the quantity this pipeline actually gates on.

| quantity | the part | what it costs here |
|---|---|---|
| accelerometer noise density | 150 µg/√Hz typ, 190 max | at the firmware's 63 Hz second-order filter — noise bandwidth about twice its corner — **1.8 mg a sample, 2.25 at the maximum** |
| one 400 ms window of that | the expected range of forty normal samples is ~4.3σ | 7.7 mg, 9.7 at the maximum, against the 15.3 mg `STILL_SPREAD` allows: **the boot gate has 2.0× of margin at a typical part and 1.6× at a maximum one** |
| the die's LSB at ±8 g | 3.9 mg in the interrupt thresholds, against the 1 mg of the output register | **nothing, measured.** Consecutive readings off a device move one LSB, so the ±8 g range does not coarsen what this pipeline sees and the quantiser costs the gate none of its margin. The 3.9 mg figure belongs to the die's threshold units, not to this output |
| zero-g offset | ±80 mg typ, ±150 max | **absorbed.** Learning runs for the second `READY_S` asks for, leaving 1.8% of the offset: at the maximum that is 2.8 mg, an eighth of what `resting` tolerates. The sideways part tilts the boot attitude by 8.5°, and cancels exactly — the bias is learned against the attitude it tilted |
| zero-rate offset | ±1 typ, ±3 max dps | learned to 0.05 dps in the boot second, and the boot gate admits 5 dps: 1.7× the part's maximum. What costs is a bias **step** — 1 dps for a second is 1° and 0.17 m/s² of gravity in the horizontal, 2.4× `VEL_MIN` — not its size. It is learned during a set as well, so the cost is the second before that: the nine-second uncorrected figure, 6.6° and 1.1 m/s², is the measurement from before the direction gate existed |
| gyroscope noise | 0.014 °/s/√Hz, 0.1 dps rms at 32 Hz | 0.01° of angle random walk a second, 0.002 m/s² of gravity leaked: **a null control, not an axis** |
| temperature drift | 1–3.5 mg/K, 0.015–0.03 °/s/K | 5–35 mg over a session's warm-up, tracked whenever gravity's direction in the sensor's frame is not moving — which a ramp does not disturb. A pin |
| gain | ±1/±4 %, ±1/±3 % | ±4% of ROM is 7.6 mm at a bench's nominal: less than the overshoot the integration already reads, so it measures reported travel rather than reach. **A device was measured at 3.3% on one axis** — gravity read 967 mg on that axis and 1001 mg split across the other two — which is inside the ±4% maximum and invisible to the bias learning |
| cross-axis, nonlinearity | 1–2 %, 0.5–2 %FS | a metre per second squared of sway leaks 0.02 into the vertical; nonlinearity is bracketed at its spec points rather than swept |
| data-rate tolerance | ±1 % | the chip's rate against the loop's 100 Hz: a sample repeated rather than missed, and a stall duplicates a whole run |

**So the gate is what has to be measured, and it is sized for the part.** The cliff the simulator
already found sits at about 0.03 m/s² a sample — 1.4× the datasheet's *maximum* density. The gate
is right for a BNO055 and has nothing left over for a coarser quantiser, a noisier part, or
anything else in the same window. That is why this section exists.

**And the number it was built on is not the sensor's.** The rest spread the old brief anchored
on — 13–17 mg — cannot be a per-sample deviation: at that σ the gate never opens, which the
simulator shows directly, and the part is three to eight times smaller. A 400 ms peak-to-peak
would fit the arithmetic better, but the part accounts for only 8–10 mg of that, so something
else is contributing. Another observation says the same thing from the other side: on a still
board no two consecutive reads were ever identical. With a deviation of a couple of quanta, two
readings land in the same quantum about one time in seven on a single axis alone, so twelve
thousand samples of a genuinely still sensor would contain hundreds of repeats. Never repeating
means the value is crossing quanta between samples, which is motion. **The rest of that spread is
the platform**, so the boot gate is largely measuring a bar in a rack rather than a sensor — and
the figure itself has no provenance left, because the capture it came from is gone.

**Two of the four axes are not dimensions.** A mount tilt is an attitude, and once gravity is
found the vertical is still the world's: sweeping it spends a dimension to rediscover an
identity, so it stays where it already is, as three pinned angles. The accelerometer's range
setting is a firmware decision rather than a part's error, so it gets one run of its own — ±4 g
against the firmware's ±8 g on identical draws — because what ±8 g buys is clip headroom and what
it costs is a share of the gate's margin: a trade to price once, not a dimension to sweep.

**The axes that remain.** Everything the part allows is either swept, because it can move an
outcome, or pinned at the datasheet's typical and maximum, because the expected result of a pin
is the row above and a pin that fails is therefore a change in this pipeline rather than in the
part.

| axis | swept over | why it is swept |
|---|---|---|
| white noise, per sample | 0.010 – 0.030 m/s², anchored at 0.017 and 0.022 | the one term that decides whether the gate opens |
| platform vibration | 1–10 Hz, to 12 mg amplitude, all three axes | the rest spread's unexplained part, and the only term that is *coloured*: a slow oscillation costs the window twice its amplitude while adding little to a deviation, so it is a different kind of enemy from the row above |
| offset step at motion onset | 0 – 1 dps, 0 – 0.3 m/s² | the bias a rest cannot learn, because the rep has already begun |
| offset ramp | to the datasheet's temperature coefficients over a session | whether tracking keeps up once the bar is warm |
| rest before the first rep | 0.4 – 6 s | the device records from the moment it calls itself ready, and that count is of quiet seconds, not of correct ones |
| sampler stall | 0–2 events, 10–200 ms | a rep is written to flash inside the sampling loop, and a gap past the integration step is travel that never happened |
| mount roll | 0 – 350 °/s through the motion, which a hand-held set measured 220–330 °/s of | the gyro's whole job, and nothing generates it today |
| attitude, uncorrected | 0 – 30 s of motion without a 400 ms still, at 0.2 – 0.7 dps | the pull onto gravity is slow while the bar moves, so a bias learned slowly still runs uncorrected for the length of a set: 0.7 dps for nine seconds is 6.6° of tilt and 1.1 m/s² of leak, sixteen times `VEL_MIN` before a velocity has even been integrated |
| sway, stroke shape, phase | 0–0.5 m/s² at 0.3–2 Hz; a cosine at 0.6–1.6 power; up- or down-first | motion the detector has never been asked about, including the lowering-first rep that pairs with an unrack |

**What is measured — six things, not one.** Whether anything was stored is the product
question, and the least diagnostic of the six: **how long until the filter called itself
ready** (never, for a session that records nothing, which today looks exactly like a quiet
one), the **count** against the reps asked for, the **order**, the **ROM** against the travel
asked for, whether the **unrack and re-rack** were stored, and the **horizontal share of the peak
velocity** — because a bar in a rack puts nothing in the horizontal plane, so speed there is a
tilt and not a motion, and it is the one number that tells a ROM read long from a tilt from a
window that began with speed already in it. A cell is those six counted over seeds.

**Seeds and the denominator.** A cell costs what a §1 cell costs, so the count is a choice rather
than a budget: the slice grid at 64 seeds, the cells on and beside a boundary at 1024, the pins
at 1024 — and the count printed in the run's header, because a rate without it compares with
nothing. Intervals are Wilson's: a cell of 64 seeds has no business quoting a proportion as
though it were exact. A seed derives from the cell and the session, so the worst cells in a
report can be replayed exactly.

**How a run is shaped.** Three passes, cheapest first. A screen moves one axis at a time to five
levels against the nominal motion, because most of the table above is expected to do nothing and
the screen's purpose is to stop paying for it. The map is then the joint draw over whatever
survived, drawn as a density over each §1 slice, with the boundary stated in the knob's own units
so two runs compare and the percentages do not have to. The pins are the datasheet's typical and
maximum parts against every nominal, which is where a regression is caught.

**What it needs first — the numbers off a device.** The model's largest unknown cannot come from
a datasheet: how much of the rest spread is the platform, and what shape it has. One still
capture answers that and three more questions: the histogram of distinct values gives the
quantiser step, which is simultaneously a check that the range setting costs what the table above
says; peak-to-peak against window length separates noise from drift; the spectrum sizes the
vibration; and windows minutes apart give the 1/f and the temperature terms. One build records
exactly that stream and nothing else: `--features record-mode` runs the sampler and the filter
with no detector and no travel check, so a window is stored whatever it measured, raw samples
included, and it says in the log when it is recording and when it is waiting for the next one.
It does wait for the filter to find gravity before the first window: a window that opens first
has no vertical to replay in, and reads hundreds of metres of travel that never happened.
A session with the bar on the rack for a few minutes, `device_log` over the partition image, and
`sim` needs only a reader that prints the four numbers.

**Motion is the other half of it**, and it has to be recorded the same way: a set is the one
motion in which nothing ever asks the accelerometer to correct the attitude, so it is the only
draw in which the largest term in this table can show itself at all. A still session alone would
have gone on calling the gyroscope a null control.

**What the first capture said.** Four stationary windows off one device, two orientations, nine
seconds each. The part is quieter than its datasheet: the 400 ms range off the quiet axis sits at
4–5 mg p50 and 11 mg at its worst, against the 7.7 mg a typical part would predict, and the whole
window's peak-to-peak is 7–12 mg. In the other orientation the same device reads 13–17 mg p50 and
31–48 mg p-p, and there only a third of the 400 ms windows pass the gate at all while the longest
unbroken quiet run is 1.1 s against the 1 s `READY_S` asks for. So the rest spread the old brief anchored on is
reproduced exactly — as a *window's p50*, in one mounting, on one bar — and the part's own share of
it is smaller than the datasheet allows. The gate's margin is the mount's, not the sensor's.

**What the moving capture said.** The same device, five windows in one session
(`captures/sets-2026-11-04.pblog`): three at rest in three mountings, then two hand-held sets.
The still windows are what this section expected — 0.00–0.01 m of reported travel, no speed above
0.03 m/s, and every sample under the gate, so the sensor's own noise is not in question. The two
moving ones report 21.41 m and 28.68 m of travel for a stroke whose own accelerometer peaks at
1.6 g on a 1.1 Hz rhythm — about 0.3 m of range.
Replayed from the state each window was stored with, they are integrating a velocity the filter
already carried: 8.1 m/s and 15.0 m/s at the window's first sample. Replayed from rest the same
samples still reach 2.3 and 10.2 m/s, and the peak *horizontal* speed is 10.10 of that 10.20,
which is the shape of a tilt and not of a bar. Over those nine seconds `still` is true for 17
samples of 901: the accelerometer is only allowed to correct the attitude when the bar is still,
and a set never is, so 0.7 dps of residual gyro error runs uncorrected into 6.6° and 1.1 m/s² of
gravity in the horizontal. The set of four reps closes two legs and stores nothing; the other
set's five legs store one, at 0.63 m of the 0.3 m it travelled.

Two things follow, and they are why this section grew an axis and a measurement. The term that
costs is not the bias — that is learned to 0.05 dps — but **how long it runs uncorrected**, which
only the motion decides, so the still-free stretch is drawn in seconds and not in dps. And ROM
alone cannot separate the two ways a window reads long: a tilt and a window that began with speed
already in it both report travel the bar never did, and only the horizontal share of the velocity
says which one a cell met.

**Since then** the filter learns the gyro's bias from the direction of gravity rather than from a
rest, and pulls the attitude onto gravity while the bar moves as well as at rest. The same two
windows, replayed from rest, peak at 0.8 and 5.5 m/s horizontally instead of 2.3 and 10.2, with
the same four reps. The figures above are the measurement that motivated it; the axis remains,
because a bias learned slowly still runs uncorrected for as long as it is wrong.

**What anchoring the replay at the turns changed.** The travel these two windows report was traced
to two additive terms: the velocity the window arrived carrying (`rom ≈ v_z at the window's edge ×
window length`) and, on top of it, an attitude residue integrating downhill for the length of the
rep. The replay now puts its velocity back to zero at every turn a cue names the instant of
(`docs/REP_TRACKING.md`), and ROM is measured over the rep's own span rather than the plot window.
On this capture that is the whole difference: the two sets read 10.57 / 3.89 / 4.12 / 3.84 m and
5.86 / 7.05 / 9.15 / 4.53 m for reps whose bars travelled a few tenths, and now read 0.44 / 2.35 /
1.93 / 1.29 and 0.55 / 1.72 / 3.94 / 0.60. Replayed from rest they read the same, rep for rep,
except where a turn was named by the speed and could not be anchored — which is the property worth
having: the travel no longer depends on the drift the window carried in. What is left is the
residue integrating *inside* one leg, which is the filter's and is where the remaining 1–4 m lives.

**What it cost on the maps.** Detection did not move: the same 5,292 cells store the same reps,
the non-rep sweep is identical, and the canonical sets read the same spans. ROM fidelity against the
travel asked for moved with it, and not all one way: the bench's slices came in a point or two
closer (+10.9 % → +10.4 % on `travel × speed_in`, +5.3 % → +5.1 % on `travel × pause_top`), while
the deadlift and the squat went out (deadlift `travel × pause_top` +3.6 % → +5.4 %, squat
+3.2 % → +6.5 %), and the worst cell in a slice went from +75 % to +135 %. The drift in the mean is
the trade and it is expected: these reps come off a perfect sensor, so they arrive with no drift
for an anchor to remove and an anchor can only add its own one-sample timing error. The worst cells
are a different fault, and they were already wrong — they are the slow strokes whose shoulder the
*filter's* stillness gate mistakes for a rest (`still` firing mid-stroke), where an onset the
replay treats as a start from rest is really the middle of a stroke in progress: that cell read
+24 % long before this change and +135 % after. The filter is where that one goes, not the anchor.

**What it will not answer.** It measures the detector against a model, so it is worth the fit and
no more: a term left out of the draw is a term the report calls harmless. The archive that could
have calibrated it is gone, so the first run's model is the datasheet's, and the capture above
either confirms the rest spread or replaces the platform axis with what it actually is.

## 3. False positives — done

The same sweep aimed at motion that is not a rep: a bar being unracked (large acceleration,
little travel), a re-rack, a hand on the rack, a wobble (oscillation with no net travel),
and the calibration transient the capture caught — 122° of roll in 1.4 s at 2.3 m/s², which
no size threshold can reject.

Same five knobs, different classes, and the same measurement inverted: how often is
something stored that is not a rep. With the noise map on top it becomes a rate.

**The wobble is where a turn rule shows up.** Reading the turn off the vertical acceleration
reaches farther into this sweep than reading it off the speed: an oscillation of 0.08 m or more at
0.7–2.5 Hz is stored for 38% of the class's cells, where the velocity rule stored 16% of them. A
half-cycle that long and that wide is an out-and-back by every rule the detector has, so the
change did not create the hole — it removed the arithmetic accident that was hiding part of it,
and the class now measures the hole's real size.

**What the sweep found.** The generator is `sim/src/false_positive.rs`, two slices over
`Motion::Push` and `Motion::Wobble`.

A one-way push is not stored anywhere in the swept range — not at three times `MIN_ROM_M`,
not at any speed. That is structural rather than a tuned margin: a rep is two opposite legs,
each past `MIN_ROM_M`, and a push is one leg, so the travel the trace integrates is not the
quantity being asked about. (The earlier note that an unrack had to travel about twice
`MIN_ROM_M` described the detector that keyed on momentum, before a rep became a pair of
turns.)

A clean oscillation is stored only while it is slow enough to rest at each turnaround; a fast
shake never dwells below the reversal threshold and produces no span at all. Inside that slow
band the detector re-opens a span at every turnaround, so most of the cells that store anything
are splits (`X`) — one 3 s hand-on-the-rack shake is several stored reps, not one. The map shows
all three states for that reason.

Two things it needs that the detectability map does not:

- **The captured non-reps**, checked in as they are, so a change that starts admitting one
  fails in `cargo test` rather than in the field. There are none in the set at the moment:
  the one that was there came from a session whose stored windows each carried a broken
  timeline, and it was removed for the reason in [`FIXTURES.md`](FIXTURES.md).
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
**edges** (neither the unrack nor the re-rack is stored). This is pins plus the report's set
table, which prints the detected sequence — not a 2-D map, because the result is a discrete "N in
→ N out", not a boundary.

**What the probe found.** The set is measured, not fixed. A comfortable set comes back whole and
in time order; an unrack and a re-rack sit right next to a rep without being stored; and a set
nobody pauses through still splits into all five of its reps — the reversal-based close is what
kept it from merging into one span. The one blemish the probe surfaced is ROM fidelity: in a no-rest
set every rep after the first reads high, because the velocity has not settled to zero between reps.
The fix is in: the integration starts from the bar's velocity at the rep's left edge, shipped as `RepSummary.v0` (a `TraceOrigin`) so a reader re-derives from the same origin, which cut the no-rest inflation from ~25 % to a ~9 % residual and left a set with rest between reps unchanged. The residual is a second-order edge effect (a span is emitted a few tens of ms before the true turnaround) and is tracked, not fixed, here.

**Since then the detector itself changed** — a rep is now two strokes between turning points, with the travel check inside detection (`docs/REP_TRACKING.md`). On the maps that moved nothing it should not: the same 5,292 cells store the same reps, no new false positive on the non-rep sweep, ROM on shared cells at least as close to the travel moved, and stalls mid-drive gained. The no-rest residual did not move with it either way: the later reps of a five-rep set with no rest in it read 0.201 m before the change and 0.207 m after, against 0.19 m of travel. That residual is not the detector's close — it is the bar genuinely still moving when the next rep starts, and the reference frame the trace starts from deciding how much of it to attribute to the rep. The sweep below is unchanged and still the way a detector change is measured — it is how this one was.

## Status

- **Detectability map: done.** `sim/examples/detector_map.rs` draws the four slices (travel
  × speed in, travel × speed out, travel × pause top, travel × pause bottom) per class, with
  three cell states so an over-split shows rather than reading as a hit;
  `sim/tests/detector_tolerance.rs` pins that each nominal is exactly one stored rep and that
  the generated non-reps stay rejected.
- **One report over all of it: done.** `sim/examples/detector_report.rs` runs the detectability
  sweeps, the non-rep sweeps and the canonical sets in one pass and prints, per slice, the cell
  counts, the ROM error against the travel asked for, and the run of one-rep cells around the
  nominal in the knob's own units; then what the non-rep sweep stores (with the splits counted)
  and how many reps each set comes back with, in what order, and how far its ROM drifted. The
  cells are `Slice::scan`'s, so the report and the drawing cannot disagree; the maps stay the
  way to see the boundary, and the report is the way to see that it moved.
- **Noise map: todo.** The axes, ranges, statistics and the capture it needs are in §2. What
  exists is `Rng`, the four-field `Impairments` in `sim/src/motion.rs` — which cannot express a
  gain, a ramp, a rotation or a platform, overwrites the gyro instead of adding to it, and
  quantises a value the generators have already quantised — and a device build that records
  without a detector (`--features record-mode`), so the capture itself is available. The model
  and the reader are the parts that have to be built before the map means anything.
- **False positives: done.** `sim/examples/false_positive_map.rs` sweeps `Push` (the
  unrack/re-rack/bump) and `Wobble` (the oscillation). There is no captured non-rep left to pin:
  the one capture's windows carry holes in their own timelines, so a test against them would pin
  numbers that came from bridging a gap (see [FIXTURES.md](FIXTURES.md)).
- **Sets: done.** The `Set` generator is in `sim/src/motion.rs`, `sim/examples/detector_report.rs`
  prints the detected sequence, and `sim/tests/sets.rs` pins that a set comes back whole, in order, with the
  unrack/re-rack unstored. The no-rest set splits correctly, and its later reps read their travel now
  that the integration is seeded from the bar's left-edge velocity (`RepSummary.state`) — a small edge
  residual remains. The calibration transient (a roll, a fusion artifact
  rather than a stroke) is still open.

Throughout, a generated fixture is code, not a file: a scenario is built where it is used, so
only the generators, the map example and the pins are committed. A map's *shape* is written
down here; its numbers are not, because they move with every tuning change.
