# Motion

Turns raw accelerometer and gyroscope samples into rep spans, traces, and summaries.

**Status: the layers and the replay are in place.** This describes the module's shape.

## Design

Raw ACCGYRO samples enter at the top and pass through three layers.

- **Fusion** estimates attitude and sensor biases, and reports whether the bar is at rest.
- **Integration** turns vertical acceleration into velocity and position, with zero-velocity constraints at anchors.
- **Detection** finds anchors from integrated motion and from cues that never passed an integrator.

Fusion and integration are the plant; detection is the observer that produces zero-velocity measurements. Detection feeds corrections back to both layers.

World z is up. Vertical quantities are index 2, and `a_world` has gravity and the accelerometer's bias removed.

There is no magnetometer: the bar and plates distort the field, and heading is not needed. The frame is gravity-aligned with arbitrary yaw, so only vertical is trusted. Horizontal position is a plot output, not an accuracy claim.

## Feedforward

```
raw accel · gyro · t
         │
         ▼
┌─────────────────────────────┐
│ FUSION                      │
│ attitude · bias · resting   │
└──────────────┬──────────────┘
               │ a_world · attitude · bias · resting
        ┌──────┴───────┐
        │              │
        ▼              ▼
┌───────────────┐  ┌───────────────────┐
│ CUES          │  │ INTEGRATION       │
│ sign · rate   │  │ v · p · v_cross   │
│ tilt          │  │                   │
└───────┬───────┘  └─────────┬─────────┘
        │ cues               │ v · p · v_cross
        └─────────┬──────────┘
                  ▼
        ┌───────────────────┐
        │ DETECTION         │
        │ anchors · legs    │
        └─────────┬─────────┘
                  │ anchors · half-reps
                  ▼
        ┌───────────────────┐
        │ REPS / SET        │
        │ spans · set state │
        └─────────┬─────────┘
                  ▼
        rep spans · live status
```

Cues are a bypass: detection gets sensor-domain evidence that never passed an integrator. That is what breaks the circularity.

Integration's `Anchor` input comes from feedback and is not drawn here.

The feedforward path ends at spans. Traces and summaries are produced by replay, because they need confirmed anchors.

## Feedback

```
                   ┌──────────────────┐
                   │ DETECTION        │
                   └──┬───────────┬───┘
        (1) confirmed │           │ (2) anchors
            rest      │           │
                      ▼           ▼
            ┌──────────────┐  ┌──────────────────┐
            │ FUSION       │  │ INTEGRATION      │
            │ re-seed bias │  │ ZUPT · replay    │
            │ + gravity    │  │ from rep seed    │
            └──────┬───────┘  └────────┬─────────┘
                   │                   │
                   └─────────┬─────────┘
                             ▼
                   ┌──────────────────┐
                   │ FINALIZE / TRACE │
                   └──────────────────┘

REPS / SET ──(3) set boundary + first confirmed rest──► FUSION
```

| # | From → To | Signal | Mode | Effect |
| --- | --- | --- | --- | --- |
| 1 | Detection → Fusion | confirmed rest window | replay only | re-seed `accel_bias`, `gyro_bias`, and attitude from that window |
| 2 | Detection → Integration | `Anchor { t_ms, kind, apply }` | live: provisional; replay: confirmed | zero `v` before or after the step; replay re-integrates from the rep seed |
| 3 | Reps/Set → Fusion | set boundary plus first confirmed rest | replay only | start a set with near-zero bias error |
| 4 | Reps/Set → Detection | rep period estimate | optional, live | tighten the crossing search and add a periodicity cue |

Live anchors are provisional and never revise an emitted output. Confirmed anchors are replay-only corrections.

Edge 1 is replay-only because fusion cannot wait for detection and still produce a causal sample.

## Block contracts

| Block | Input | Output | State | Inner loop |
| --- | --- | --- | --- | --- |
| **Fusion** | raw `accel[3]`, `gyro[3]`, `t_ms` | `a_world[3]`, `attitude`, `accel_bias`, `gyro_bias`, `resting`, `ready` | attitude quat; both biases; ring of raw samples for the rest window; `prev_t`; `quiet_s`; `ready` | gyro predicts attitude, accel pulls it toward gravity; bias tracked while resting |
| **Cues** | `a_world`, `attitude`, bias-corrected rate | `sign_vert`, `rate_mag`, `tilt`, `sign_change` event | previous sign; previous rate and tilt; last event time | none; comparison only |
| **Integration** | `a_vert`, `dt`, `Anchor` events | `v[3]`, `p[3]`, `v_cross` | `v`, `p`, `prev_t`, per-rep accumulator, anchor cursor, high-pass state if used | trapezoid; ZUPT at anchors; detrend at finish |
| **Detection** | `v`, `v_cross`, `resting`, cues, optional `p` | `Anchor { t_ms, kind, apply }`, `Leg` | crossing side; pending turn; quietest tracker; history of `(t, v, FusionState)`; lag buffer; candidate anchors | hysteresis on crossings; confirmation with cues; rest reset |
| **Reps / Set** | anchors and legs, `resting`, `t_ms` | `RepSpan`, `RepSummary`, set state | pending leg; `resting_since`; `rested`; set open; rep count | duration and ROM validation; inter-rep gap; set end |
| **Finalize / Trace** | span, seed state, samples, confirmed anchors | trace points, ROM, peak velocity, roll and pitch ranges | replay fusion; replay integrator; decimation cursor | replay from seed; ZUPT; detrend; decimate |

## Invariants

- One definition of vertical: a projection of `a_world`, never a second filter.
- One integrator; `v` feeds position, and `v_cross` is the only velocity detection may threshold.
- Detection owns no filter and no integrator.
- An anchor carries its timestamp and whether it applies before or after the integration step.
- `resting` is causal and local; it is not an anchor and makes no zero-velocity claim.
- Bias and gravity corrections from confirmed anchors happen in replay.
- Replay is a pure function of seed, samples, and anchors.
- Bias subtraction at an anchor is the primary drift fix; a high-pass may shape `v_cross` but never `p`.
- Horizontal position is plot-only.

## Files

```
motion/
  mod.rs
  fusion.rs     filter, attitude, biases, resting
  cues.rs       drift-resistant features
  integrate.rs  velocity, position, v_cross
  rep.rs        detection, rep and set state, validation
  trace.rs      replay, detrend, decimate
  pipeline.rs   `Pipeline`: the live entry point through fusion, cues, integration, detection
```
