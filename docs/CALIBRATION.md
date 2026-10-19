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
| `sys` | The aggregate the fusion uses; follows the others. |

## The calibration profile

A 22-byte BNO055 profile written to the chip's calibration registers restores
calibration instantly, so a calibrated device does not need the routine again
after a reboot.

- Stored in the `nvs` partition at offset `0x00` (the device name lives in the second sector, `0x1000`, so erasing one never touches the other).
- Saved **once per boot**, on the transition to `is_ready()`, not at boot.
