# Mounting

How the device sits on the bar, and what each axis measures because of it.

The device is strapped to the bar so that its axes line up with it:

- **Y** runs parallel to the bar's long axis, offset a few centimetres off the bar's centre axis.
- **X** runs perpendicular, across the bar.
- **Z** completes the right-handed triad: perpendicular to the bar, so it points up when the bar
  is level.

A level, still bar therefore reads `+1 g` on the Z accelerometer and near-zero angular rate,
which is the rest orientation the filter converges to.

That mounting turns each axis into a named part of a lift:

| axis | reads |
| --- | --- |
| `gy` (Y) | rotation about the bar's long axis — the bar rolling in the hands |
| `gx` (X) | the bar tipping — one end rising while the other falls |
| `gz` (Z) | yaw, rotation in the horizontal plane, which nothing can observe without a magnetometer |

Roll and pitch in the filter are the Euler angles about X and Y, so a bar rolling in the hands
is the Y channel's. The accelerometer reads the stroke along Z, which is also why a rep's travel
is measured on Z alone.
