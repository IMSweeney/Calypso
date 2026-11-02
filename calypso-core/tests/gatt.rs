//! The value characteristics, as values: a sample that outgrows its characteristic cannot be read at all.

use calypso_core::gatt::{max_len, SyncImuValue};
use calypso_core::pb::msg;

/// The widest `ImuState` the device can build, encoded and read back.
///
/// The extremes are the types' own — a sample's acceleration and quaternion are `i16`, the
/// timestamp is a `u32` — so this is the bound the schema's `max_len` has to hold, not an
/// estimate of what a bar does. A value that did not fit would be `None` at the device, and
/// the app would read the last one that did.
#[test]
fn the_widest_imu_sample_fits_its_characteristic() {
    let widest = msg::ImuState {
        t_uptime_ms: u32::MAX,
        ax: i16::MIN as i32,
        ay: i16::MAX as i32,
        az: i16::MIN as i32,
        qw: i16::MAX as i32,
        qx: i16::MIN as i32,
        qy: i16::MAX as i32,
        qz: i16::MIN as i32,
        gx: i16::MAX as i32,
        gy: i16::MIN as i32,
        gz: i16::MAX as i32,
    };

    let value = SyncImuValue::encode(&widest).expect("the widest sample fits the characteristic");
    assert!(value.as_bytes().len() <= max_len::SYNC_IMU);
    assert_eq!(value.decode(), Some(widest));
}
