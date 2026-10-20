//! The IMU's calibration state, and the rule for whether a rep may be recorded.
//!
//! The BNO055 reports four 0..3 levels in `CALIB_STAT` (0x35): system, gyro,
//! accelerometer and magnetometer. Three is fully calibrated.
//!
//! **Rep tracking needs `accel` and `gyro` at 3, and nothing else.** Neither the
//! magnetometer nor the chip's aggregate `sys` is part of it: the device runs the
//! BNO055 in IMU mode, which does not use the magnetometer at all (it is unreliable
//! next to a barbell's steel) and in which the magnetometer status never reaches 3 —
//! and **`sys` counts the magnetometer**, so `sys` never reaches 3 either. Waiting on
//! a number that cannot arrive is a device that never records a rep;
//! `is_fully_calibrated` still reports all four, because that is what "fully
//! calibrated" means for the chip.
//!
//! The distinction matters because an uncalibrated gyro or accelerometer reports
//! drift as motion: the rep detector would fire on it and the trace it stored
//! would be fiction rather than a measurement. So [`Calibration::is_ready`] is
//! the gate the firmware holds reps behind, and the same answer travels to the
//! app in `CalibrationStatus.ready` so the two cannot disagree.

/// A level of 3 means that sensor is fully calibrated.
pub const MAX_LEVEL: u8 = 3;

/// The BNO055's calibration levels, each 0..3.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Calibration {
    /// System: the aggregate the fusion uses.
    pub sys: u8,
    pub accel: u8,
    pub gyro: u8,
    pub mag: u8,
}

impl Calibration {
    pub const fn new(sys: u8, accel: u8, gyro: u8, mag: u8) -> Self {
        Calibration {
            sys,
            accel,
            gyro,
            mag,
        }
    }

    /// Every sensor at [`MAX_LEVEL`], as the chip means it.
    pub const fn is_fully_calibrated(&self) -> bool {
        self.sys == MAX_LEVEL
            && self.accel == MAX_LEVEL
            && self.gyro == MAX_LEVEL
            && self.mag == MAX_LEVEL
    }

    /// Whether the device may record reps with this IMU state.
    ///
    /// The two levels rep tracking depends on, at full: the accelerometer and the
    /// gyroscope. **Not the magnetometer, and not [`Self::sys`]** — the aggregate
    /// counts the magnetometer, so in IMU mode it never leaves 0 even with the other
    /// two fully calibrated. The device's own boot log says it:
    /// `imu acc:3 gyro:3 mag:0 sys:0 — rep recording paused until the IMU is
    /// calibrated`, once a second, forever.
    ///
    /// Gating on the aggregate cost two things, and the second is why it took a power
    /// cycle to notice:
    ///
    /// - no rep is ever recorded, because the detector is only fed while ready;
    /// - **the calibration profile is never saved**, because it is saved on the
    ///   transition to ready. So the accelerometer's offsets were dropped on every
    ///   reboot — the gyroscope came back at 3 on its own (the chip re-derives that
    ///   from stillness alone) and the accelerometer came back at 0, which reads as
    ///   "the save is broken" rather than "the save never ran".
    ///
    /// `sys` is still *reported*, like the magnetometer: a serial console that says
    /// which sensor is behind is the point of the whole routine.
    pub const fn is_ready(&self) -> bool {
        self.accel == MAX_LEVEL && self.gyro == MAX_LEVEL
    }

    /// Pack into four bytes for a shared atomic / lock-free hand-off between
    /// tasks. Order is `[sys, accel, gyro, mag]`.
    pub const fn to_bits(&self) -> u32 {
        u32::from_le_bytes([self.sys, self.accel, self.gyro, self.mag])
    }

    pub const fn from_bits(bits: u32) -> Self {
        let [sys, accel, gyro, mag] = bits.to_le_bytes();
        Calibration {
            sys,
            accel,
            gyro,
            mag,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ready_needs_accel_and_gyro() {
        assert!(Calibration::new(0, 3, 3, 0).is_ready());
        assert!(Calibration::new(3, 3, 3, 0).is_ready());
        assert!(!Calibration::new(3, 2, 3, 0).is_ready());
        assert!(!Calibration::new(3, 3, 2, 0).is_ready());
    }

    #[test]
    fn the_magnetometer_never_blocks_rep_recording() {
        let imu_mode = Calibration::new(3, 3, 3, 0);
        assert!(imu_mode.is_ready());
        assert!(!imu_mode.is_fully_calibrated());
    }

    #[test]
    fn fresh_boot_is_not_ready() {
        assert!(!Calibration::default().is_ready());
        assert!(!Calibration::new(0, 0, 0, 0).is_ready());
    }

    #[test]
    fn fully_calibrated_implies_ready() {
        assert!(Calibration::new(3, 3, 3, 3).is_fully_calibrated());
        assert!(Calibration::new(3, 3, 3, 3).is_ready());
    }

    #[test]
    fn pack_round_trips() {
        let c = Calibration::new(3, 2, 1, 0);
        assert_eq!(Calibration::from_bits(c.to_bits()), c);
    }

    #[test]
    fn packed_erased_flash_is_not_ready() {
        let erased = Calibration::from_bits(0xFFFF_FFFF);
        assert_eq!(erased, Calibration::new(0xFF, 0xFF, 0xFF, 0xFF));
        assert!(!erased.is_ready());
    }
}
