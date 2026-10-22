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

    pub const fn is_ready(&self) -> bool {
        self.accel == MAX_LEVEL && self.gyro == MAX_LEVEL
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
