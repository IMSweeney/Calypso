//! The IMU's calibration state, and the rule for whether a rep may be recorded.
//!
//! The BNO055 reports four 0..3 levels in `CALIB_STAT` (0x35): system, gyro,
//! accelerometer and magnetometer. Three is fully calibrated.
//!
//! **Rep tracking needs `sys`, `gyro` and `accel` at 3.** The magnetometer is
//! deliberately not part of it: the device runs the BNO055 in IMU mode, which
//! does not use the magnetometer at all (it is unreliable next to a barbell's
//! steel), and in that mode the magnetometer status never reaches 3 — so waiting
//! on it would be waiting forever. `is_fully_calibrated` still reports the
//! magnetometer, because that is what "fully calibrated" means for the chip.
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
    /// Deliberately stricter than "the numbers look plausible": all three of the
    /// levels rep tracking depends on, at full. Note this is *not*
    /// [`Self::is_fully_calibrated`] — the magnetometer is not used in IMU mode.
    pub const fn is_ready(&self) -> bool {
        self.sys == MAX_LEVEL && self.accel == MAX_LEVEL && self.gyro == MAX_LEVEL
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
    fn ready_needs_sys_accel_and_gyro() {
        assert!(Calibration::new(3, 3, 3, 0).is_ready());
        assert!(!Calibration::new(2, 3, 3, 0).is_ready());
        assert!(!Calibration::new(3, 2, 3, 0).is_ready());
        assert!(!Calibration::new(3, 3, 2, 0).is_ready());
    }

    #[test]
    fn the_magnetometer_never_blocks_rep_recording() {
        // IMU mode has no magnetometer: its status sits at 0 forever, and
        // waiting on it would mean never recording a rep.
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
        // 0xFFFFFFFF is what erased flash reads back as: every level maxed,
        // which must not read as a calibrated IMU.
        let erased = Calibration::from_bits(0xFFFF_FFFF);
        assert_eq!(erased, Calibration::new(0xFF, 0xFF, 0xFF, 0xFF));
        assert!(!erased.is_ready());
    }
}
