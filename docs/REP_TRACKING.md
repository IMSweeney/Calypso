# Rep tracking

How a barbell rep gets from the IMU to a stored, plottable trace.

```
BNO055 (IMU mode, 100 Hz)
   │  I²C @ 400 kHz
   ▼
Sampler ──► RAM ring (~12 s of samples)
   │
   ▼
Rep detector (state machine, ZUPT at rest)
   │  rep complete → slice [start-margin, end+margin] out of the ring
   ▼
Trace computation: world-frame displacement + orientation, summary stats
   │
   ▼
Flash log (ring of 4 KB sectors) ──► BLE sync to the app
```

## Sampling

- The BNO055 runs in **IMU mode**: accelerometer + gyroscope fusion, no
  magnetometer (unreliable next to a barbell's steel). Fused output at 100 Hz: linear acceleration with gravity removed, plus a quaternion orientation.
- I²C at 400 kHz. One sample is ~14 bytes, so ~0.5 ms of the 10 ms period.
- Sample format, fixed point, 20 bytes:
  `{ t_ms: u32, accel: [i16; 3], quat: [i16; 4] }` — accel at 0.01 m/s² per LSB
  (BNO055 native), quaternion at 1/2¹⁴ per LSB in `[w, x, y, z]` order.
  `t_ms` is monotonic device uptime; the device has no RTC.
- Samples go into a RAM ring buffer of `SAMPLE_RING_LEN` = 12 s
  (`REP_WINDOW_SAMPLES` = 900 = 9 s, plus 3 s headroom). The detector never asks
  for more than 9 s, so anything larger is wasted heap — see
  [`GOTCHAS.md`](GOTCHAS.md).
- The sampler also reads the gravity vector once a second, for the level verdict
  in the firmware log (`Sensor is level on the X axis` — how the device gets
  levelled). That reading goes nowhere else: the gravity characteristic it used to
  feed is gone.

## Detecting a rep

`calypso_core::rep::Detector`, fed one sample at a time:

```
IDLE ──|a| > on-threshold for T_on──► MOVING ──|a| < rest-threshold for T_rest──► SETTLE
SETTLE ──ZUPT (v := 0), close the rep, emit it──► IDLE
```

Hysteresis between the two thresholds keeps a noisy rest from flapping.
Tunables are constants at the top of `calypso-core/src/rep.rs`.

## Computing a rep

On rep close, the sampler copies `[start - MARGIN_MS, end + MARGIN_MS]` out of the ring and calls `rep::compute_rep`, which:

1. Rotates each sample's linear acceleration into the world frame using that sample's quaternion.
2. Integrates it, **anchored at both ends by ZUPT**: velocity is forced to zero
   at rest, so double-integration drift is bounded within one rep rather than accumulating over a set. The result is displacement relative to the rep start, in metres — which is what a bar-path plot needs. Absolute position is not available from a MEMS IMU and is not attempted.
3. Computes the summary stats: duration, ROM (peak-to-peak displacement on the
   dominant axis), peak and mean velocity, peak acceleration, and roll/pitch min/max in degrees.

The trace is capped at **512 points** (`TRACE_POINTS`). When a rep is longer than that, it is decimated and `RepSummary.decimated` is set. The decimation grid is anchored at the **last** sample, so a rep's end is always stored — the end is one of the two samples you judge a bar path by.

Both the derived displacement **and** the raw inputs (accel, quaternion) are stored, so the app can re-derive traces with better algorithms later without a firmware update.

## What gets stored

One rep is one `RepSummary` record followed by `chunk_count` `TraceChunk` records carrying the same `rep_uuid`:

```
[RepSummary][TraceChunk 0][TraceChunk 1]…
```

- A chunk holds up to 48 trace points (`pb::TRACE_CHUNK_SAMPLES`), so an encoded
  record always fits inside one 4 KB erase sector (the log's record limit is 4076 bytes; a worst-case chunk encodes to ~2 KB). A full 512-point trace is 11 chunks.
- Records are kept in the `log` data partition (0x190000, 2.4 MB), a ring of 4 KB
  sectors that erases the oldest sectors on wrap. At ~5.5 KB per rep that is roughly 420 reps — 50–80 typical workouts before the oldest fall off.
- Records never span sectors; see `calypso-core/src/flashlog.rs` for the layout, CRC and power-loss behaviour.

**Rep identity is a v4 UUID minted on the device when the rep closes.** Random, because the device has no RTC and its uptime resets on every boot, so neither a counter nor a timestamp can be unique across boots. The price is that UUIDs do not sort: ordering comes from the archive's append order on the client.

Reps only need to survive on the device until the next sync — the app is the archive of record, so a ring wrap is harmless. See [`APP_SYNC.md`](APP_SYNC.md).

## Delivery to the app

Summaries auto-stream as soon as an app subscribes; traces are streamed on request, re-chunked to fit a BLE notification (~3 points each). The protocol, the GATT table and the client side are in [`APP_SYNC.md`](APP_SYNC.md).
