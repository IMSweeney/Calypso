# Calibration

The device has no display and no buttons, so calibration is mediated entirely by the
app: the device comes up regardless, reports how calibrated each IMU sensor is (a
`CalibrationStatus` characteristic the app reads — see [`APP_SYNC.md`](APP_SYNC.md)),
and does not record reps until it is calibrated.

## What the IMU reports

The BNO055 reports `CALIB_STAT` (register `0x35`): four levels, each `0..3`, 3
meaning fully calibrated.

| Level | What calibrates it |
|---|---|
| `accel` | The chip held in a few different, **still** orientations. |
| `gyro` | The chip **perfectly still** for a few seconds. |
| `mag` | Figure-of-eight motion — **not used here**. |
| `sys` | The aggregate — **counts the magnetometer, so in IMU mode it sits at 0 for ever**, and it is not part of the rule. |

## The rule

`Calibration::can_record()` is `gyro == 3` (`calypso-core::calibration`), and that is
the gate reps are held behind. The gyroscope is the level that has to be trusted — its
drift is what fabricates motion — while the accelerometer's status wanders on its own
while the chip sits still, and waiting for it costs reps at random. `sys` and `mag` are
reported for the console only, and neither can reach 3 on this device.

A rep stores whether the accelerometer was calibrated too, as the `CALIBRATED` bit of
`RepSummary.flags` (`proto/calypso.proto`), so the reps taken while it was low are still
recorded and are marked as such rather than being lost.

The **profile save** still wants `accel == 3` as well: the accelerometer's offsets are
the part that cannot be re-derived, so a profile saved without them restores nothing.
That is the one place the accelerometer is waited for. Whether a *restored* profile
comes back as calibrated is open — see [`../ROADMAP.md`](../ROADMAP.md).

## The calibration profile

A 22-byte BNO055 profile written to the chip's calibration registers restores
calibration instantly, so a calibrated device does not need the routine again after
a reboot.

- Stored in the `nvs` partition at offset `0x00`, in the first sector — the only
  thing in there.
- Saved **once per boot**, on the transition to `profile_worth_saving()` — gyroscope
  and accelerometer both at 3 — not at boot.
