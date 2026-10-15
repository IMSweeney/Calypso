//! BLE GATT table construction.
//!
//! This lives in `calypso-core` (rather than the firmware crate) because the
//! trouble-host attribute table is platform-independent: building it on the
//! host lets integration tests read and write characteristics exactly as the
//! BLE stack would when a phone connects — no radio required. See
//! `tests/gatt_table.rs`.

use embassy_sync::blocking_mutex::raw::RawMutex;
use trouble_host::prelude::*;

use crate::DEVICE_NAME_MAX;

/// Number of attributes in the table:
/// GAP service (1) + device name (2) + appearance (2) + GATT service (1)
/// + gravity service (1) + gravity-x characteristic (2) + its CCCD (1),
///   plus 2 spare.
pub const GATT_TABLE_MAX: usize = 12;

/// Custom GATT service exposing the IMU's gravity vector.
pub const GRAVITY_SERVICE_UUID: Uuid = Uuid::new_long([
    0x00, 0x00, 0xca, 0x10, 0xb0, 0xcd, 0x11, 0xec, 0x87, 0x1f, 0xd4, 0x5d, 0xdf, 0x13, 0x88, 0x40,
]);
/// X-axis gravity characteristic (m/s^2, f32, read + notify).
pub const GRAVITY_X_UUID: Uuid = Uuid::new_long([
    0x00, 0x00, 0xca, 0x11, 0xb0, 0xcd, 0x11, 0xec, 0x87, 0x1f, 0xd4, 0x5d, 0xdf, 0x13, 0x88, 0x40,
]);

/// The built GATT attribute table plus handles to the characteristics the
/// application cares about.
pub struct GattTable<'d, M: RawMutex> {
    pub table: AttributeTable<'d, M, GATT_TABLE_MAX>,
    /// Device Name (0x2A00), read + write.
    pub device_name: Characteristic<HeaplessString<DEVICE_NAME_MAX>>,
    /// X-axis gravity (m/s^2), read + notify.
    pub gravity_x: Characteristic<f32>,
}

#[derive(Debug, PartialEq, Eq)]
pub enum BuildError {
    /// The device name is empty or longer than [`DEVICE_NAME_MAX`].
    InvalidName,
}

/// Build the GAP service (writable device name + appearance), the mandatory
/// GATT service declaration, and the custom gravity service.
///
/// `device_name_store` and `gravity_x_store` are the backing buffers the
/// server reads/writes; they must outlive the table.
///
/// Note: `GapConfig` is intentionally not used here — it only supports a
/// read-only device name.
pub fn build_gatt_table<'d, M: RawMutex>(
    device_name: &str,
    device_name_store: &'d mut [u8],
    gravity_x_store: &'d mut [u8],
) -> Result<GattTable<'d, M>, BuildError> {
    let mut initial: HeaplessString<DEVICE_NAME_MAX> = HeaplessString::new();
    if device_name.is_empty() || initial.push_str(device_name).is_err() {
        return Err(BuildError::InvalidName);
    }

    let mut table: AttributeTable<'d, M, GATT_TABLE_MAX> = AttributeTable::new();

    let mut gap = table.add_service(Service::new(service::GAP));
    let device_name = gap
        .add_characteristic(
            characteristic::DEVICE_NAME,
            &[CharacteristicProp::Read, CharacteristicProp::Write],
            initial,
            device_name_store,
        )
        .build();
    gap.add_characteristic_ro(characteristic::APPEARANCE, &appearance::sensor::GENERIC_SENSOR);
    gap.build();

    table.add_service(Service::new(service::GATT)).build();

    let gravity_x = table
        .add_service(Service::new(GRAVITY_SERVICE_UUID.clone()))
        .add_characteristic(
            GRAVITY_X_UUID.clone(),
            &[CharacteristicProp::Read, CharacteristicProp::Notify],
            0.0f32,
            gravity_x_store,
        )
        .build();

    Ok(GattTable {
        table,
        device_name,
        gravity_x,
    })
}
