# Calibration

The device has no display and no buttons, so calibration is mediated entirely by the app: the device comes up regardless, tells the app how calibrated each IMU sensor is and what to do about it, and simply does not record reps until calibrated

## What the IMU reports

The BNO055 reports `CALIB_STAT` (register `0x35`): four levels, each `0..3`,
where 3 means fully calibrated.

| Level | What calibrates it |
|---|---|
| `accel` | The chip held in a few different, **still** orientations. |
| `gyro` | The chip **perfectly still** for a few seconds. |
| `mag` | Figure-of-eight motion — **not used here**. |
| `sys` | The aggregate — **counts the magnetometer, so in IMU mode it sits at 0 for ever** and is not part of the rule below. |

## The rule

`Calibration::is_ready()` is `accel == 3 && gyro == 3` (`calypso-core::calibration`),
and that is the gate reps are held behind. Neither the magnetometer nor `sys` is part
of it, and on this device neither can ever reach 3 — a gate that included `sys` was a
device that never recorded a rep, with a hint on the phone that never cleared.

The same gate decides when the profile is saved, which is how `sys` came to look like
a persistence bug: with the gate never opening, the accelerometer's offsets were never
written, so a reboot lost them. The gyroscope returns at 3 by itself, because the chip
re-derives that from stillness alone — so the two sensors disagreed across a reboot
while the save itself was fine.

## The calibration profile

A 22-byte BNO055 profile written to the chip's calibration registers restores
calibration instantly, so a calibrated device does not need the routine again
after a reboot.

- Stored in the `nvs` partition at offset `0x00` (the device name lives in the second sector, `0x1000`, so erasing one never touches the other).
- Saved **once per boot**, on the transition to `is_ready()`, not at boot.
