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

`Calibration::is_ready()` is `accel == 3 && gyro == 3` (`calypso-core::calibration`),
and that is the gate reps are held behind. `sys` and `mag` are reported for the
console only, and neither can reach 3 on this device.

The same gate triggers the profile save, so a gate that never opens never saves one.
Whether a *restored* profile comes back as calibrated is open — see
[`../ROADMAP.md`](../ROADMAP.md).

## The calibration profile

A 22-byte BNO055 profile written to the chip's calibration registers restores
calibration instantly, so a calibrated device does not need the routine again after
a reboot.

- Stored in the `nvs` partition at offset `0x00`, in the first sector — the only
  thing in there.
- Saved **once per boot**, on the transition to `is_ready()`, not at boot.
