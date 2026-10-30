//! The IMU's calibration state, and the rule for whether a rep may be recorded.

pub const MAX_LEVEL: u8 = 3;

/// The BNO055's calibration levels, each 0..3.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Calibration {
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

    pub const fn is_fully_calibrated(&self) -> bool {
        self.sys == MAX_LEVEL
            && self.accel == MAX_LEVEL
            && self.gyro == MAX_LEVEL
            && self.mag == MAX_LEVEL
    }

    /// Whether a rep may be recorded: the gyroscope, and only the gyroscope.
    ///
    /// Its drift is what fabricates motion, so it is the level that has to be trusted. The accelerometer's status wanders on its own while the device sits still — a real session had it read 1 for two and a half hours — and gating on it costs reps at random, with nothing on screen to say why.
    pub const fn can_record(&self) -> bool {
        self.gyro == MAX_LEVEL
    }

    /// Whether the accelerometer is calibrated too: what a rep records in its flags, and what makes a profile worth saving.
    pub const fn accel_calibrated(&self) -> bool {
        self.accel == MAX_LEVEL
    }

    /// Whether the calibration in the chip is worth writing to flash: the accelerometer's offsets are the part that is hard to re-derive, so a profile saved without them is a profile that restores nothing.
    pub const fn profile_worth_saving(&self) -> bool {
        self.can_record() && self.accel_calibrated()
    }

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
    fn recording_needs_only_the_gyroscope() {
        assert!(Calibration::new(0, 3, 3, 0).can_record());
        assert!(Calibration::new(0, 0, 3, 0).can_record());
        assert!(!Calibration::new(3, 3, 2, 3).can_record());
        assert!(!Calibration::new(0, 1, 0, 0).can_record());
    }

    #[test]
    fn the_magnetometer_never_blocks_rep_recording() {
        let imu_mode = Calibration::new(3, 3, 3, 0);
        assert!(imu_mode.can_record());
        assert!(!imu_mode.is_fully_calibrated());
    }

    #[test]
    fn fresh_boot_is_not_ready() {
        assert!(!Calibration::default().can_record());
        assert!(!Calibration::new(0, 0, 0, 0).can_record());
    }

    #[test]
    fn a_profile_wants_the_accelerometer_as_well() {
        assert!(Calibration::new(0, 3, 3, 0).profile_worth_saving());
        assert!(!Calibration::new(0, 2, 3, 0).profile_worth_saving());
        assert!(!Calibration::new(0, 3, 2, 0).profile_worth_saving());
    }

    #[test]
    fn fully_calibrated_implies_ready() {
        assert!(Calibration::new(3, 3, 3, 3).is_fully_calibrated());
        assert!(Calibration::new(3, 3, 3, 3).can_record());
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
        assert!(!erased.can_record());
    }
}
