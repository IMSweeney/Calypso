# replay

What the pipeline did inside a rep, drawn.

The device only hands down the reps it closed, so a session that reads wrong arrives as numbers:
a rom twice the stroke, a span a turn late. This draws the layers the rep was built from — the
raw sample, the filter's frame, the cues, the velocity the integrator carried, and the position
the rep was given — against the spans the detector closed, so a wrong number has somewhere to
point.

The numbers come from `sim`'s `pipeline_dump`, which replays a recording through the firmware's own
`Pipeline`. Nothing here re-implements the filter or the detector, so a drawing cannot disagree
with the device.

## Running it

```sh
python3 -m venv replay/.venv
replay/.venv/bin/pip install -r replay/requirements.txt
replay/.venv/bin/python replay/plot.py deadlift-2026-11-10.pblog --set 3
```

The capture is a name in `captures/` or one the bucket holds (`tools/captures.sh pull <name>`).
Each run replays the whole recording once and leaves the numbers in `replay/out/<capture>/`.

| option | |
| --- | --- |
| `--set N` | which set of the recording to draw, 1-based |
| `--rep N` | which rep of that set to draw on its own, 1-based |
| `--margin S` | seconds drawn either side of the set |
| `--out DIR` | where the figures go |
| `--dump DIR` | reuse a dump from an earlier run instead of replaying |

## The figures

`set-N.png` is the set, one panel per stage: the raw specific force and rate, the fused linear
acceleration and tilt, the cues, the integrated vertical and horizontal velocity, the flags, and
the trace each rep was given.

`set-N-rep-N.png` is one rep of that set, close in, with its own span and its three anchors.

In both, the grey band is the span the detector closed and the orange dashed lines are the anchors
it recorded: the turn the rep came in from, the turn at full reversal, and the turn it ended on.
The band repeating across every panel is what keeps a stage's reading tied to the rep it belongs
to; a spike outside a band is a stage reacting to something the detector did not call a rep.

## What to look for

- **`vz` at a turn.** A rep starts and ends with the bar still, so `vz` at an entry or exit anchor
  ought to be zero. A steady offset there is velocity the integrator never lost, and it multiplies
  by the span's length into a position that ramps.
- **`resting`.** How often the filter calls the bar still inside a set says whether gravity and
  bias are being re-estimated at all.
- **The trace panel against the band.** A rep whose trace leaves its band's height range is a rep
  whose position came from a velocity that never returned; a rep whose trace only travels one way
  is a span whose ends are not at the same height, which no detrend can fix.
