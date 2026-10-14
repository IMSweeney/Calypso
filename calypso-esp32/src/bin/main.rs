#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use bt_hci::controller::ExternalController;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Timer, Delay};
use esp_hal::{Blocking};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::i2c::master::I2c;
use esp_println as _;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;
use esp_bootloader_esp_idf::partitions;
use esp_storage::FlashStorage;
use embedded_storage::{ReadStorage, Storage};
use embedded_storage::nor_flash::NorFlash;

use bno055::{BNO055OperationMode, BNO055Calibration, BNO055_CALIB_SIZE};
use bno055::Bno055;
use calypso_core::DEVICE_NAME_MAX;
use core::sync::atomic::{AtomicU32, Ordering};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
// Signal channel + ATT channel
const L2CAP_CHANNELS_MAX: usize = 2;

const BNO055_CALIBRATION_ADDR: u32 = 0x00;

/// Offset of the BLE device name blob inside the NVS partition. Lives in the
/// partition's second flash sector so erasing it never touches the calibration
/// blob in the first sector.
const BLE_DEVICE_NAME_ADDR: u32 = 0x1000;
const DEFAULT_DEVICE_NAME: &str = "Calypso";

/// Latest X-axis gravity reading (m/s^2), shared between the IMU loop and the
/// BLE task as raw f32 bits.
static GRAVITY_X_BITS: AtomicU32 = AtomicU32::new(0);

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.1.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 1>::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();

    // Everything runs in this task via join3 below; no extra tasks needed.
    let _ = spawner;

    // Setup the bno055 imu
    info!("Setting up i2c");
    // pinout for esp32 pico
    // https://learn.adafruit.com/assets/112309
    let i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default()).unwrap()
    .with_sda(peripherals.GPIO22).with_scl(peripherals.GPIO19);
    info!("Done setting up i2c");

    let mut delay = Delay;

    info!("Setting up Bno055");
    let mut bno055 = Bno055::new(i2c).with_alternative_address();
    bno055.init(&mut delay).unwrap();
    info!("Bno055 initialized");
    bno055.set_mode(BNO055OperationMode::NDOF, &mut delay).unwrap();

    let mut flash = FlashStorage::new(peripherals.FLASH);
    info!("Flash size = {}", flash.capacity());
    setup_calibration(&mut bno055, &mut flash);

    // Load the persisted BLE device name, falling back to the default.
    let device_name = load_device_name(&mut flash);
    info!("BLE device name: {}", device_name.as_str());

    let stack = trouble_host::new(ble_controller, &mut resources)
        .set_random_address(Address::random([0xca, 0xa7, 0x19, 0x50, 0xe4, 0xff]));
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    // GATT table: GAP service with a *writable* Device Name characteristic
    // (0x2A00), a read-only Appearance characteristic, the mandatory GATT
    // service declaration, and a custom gravity service. Built in
    // calypso-core so host integration tests can exercise the exact same
    // table (see calypso-core/tests/gatt_table.rs).
    let mut name_store = [0u8; DEVICE_NAME_MAX];
    let mut gravity_x_store = [0u8; 4];
    let gatt = calypso_core::ble::build_gatt_table::<NoopRawMutex>(
        device_name.as_str(),
        &mut name_store[..],
        &mut gravity_x_store[..],
    )
    .unwrap();
    let name_char = gatt.device_name;
    let gravity_x_char = gatt.gravity_x;
    let server = AttributeServer::<
        NoopRawMutex,
        DefaultPacketPool,
        { calypso_core::ble::GATT_TABLE_MAX },
        1,
        CONNECTIONS_MAX,
    >::new(gatt.table);

    info!("Starting BLE stack");
    join3(
        // HCI runner: must run forever alongside any other BLE tasks.
        async {
            loop {
                if runner.run().await.is_err() {
                    warn!("BLE runner error");
                }
            }
        },
        // Advertising + GATT event handling.
        async {
            let mut name = device_name;
            loop {
                // (Re)advertise with the current name. A name written over GATT
                // shows up in advertising on the next advertising cycle (i.e.
                // after the current connection closes).
                let mut adv_data = [0u8; 31];
                let adv_len = AdStructure::encode_slice(
                    &[
                        AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                        AdStructure::CompleteLocalName(name.as_bytes()),
                    ],
                    &mut adv_data[..],
                )
                .unwrap();

                let advertiser = match peripheral
                    .advertise(
                        &Default::default(),
                        Advertisement::ConnectableScannableUndirected {
                            adv_data: &adv_data[..adv_len],
                            scan_data: &[],
                        },
                    )
                    .await
                {
                    Ok(advertiser) => advertiser,
                    Err(_) => {
                        warn!("failed to start advertising");
                        Timer::after(Duration::from_millis(500)).await;
                        continue;
                    }
                };
                info!("advertising");

                let conn = match advertiser.accept().await {
                    Ok(conn) => match conn.with_attribute_server(&server) {
                        Ok(conn) => conn,
                        Err(_) => continue,
                    },
                    Err(_) => continue,
                };
                info!("BLE connection established");

                loop {
                    // Service GATT events; once a second, push the latest
                    // gravity reading to the client (no-op unless subscribed).
                    let event = match select(conn.next(), Timer::after(Duration::from_secs(1))).await {
                        Either::First(event) => event,
                        Either::Second(()) => {
                            let value = f32::from_bits(GRAVITY_X_BITS.load(Ordering::Relaxed));
                            if gravity_x_char.notify(&conn, &value).await.is_err() {
                                break;
                            }
                            continue;
                        }
                    };
                    match event {
                        GattConnectionEvent::Disconnected { .. } => {
                            info!("BLE disconnected");
                            break;
                        }
                        GattConnectionEvent::Gatt { event } => {
                            let wrote_name =
                                matches!(&event, GattEvent::Write(w) if w.handle() == name_char.handle);
                            if let Ok(reply) = event.accept() {
                                let _ = reply.send().await;
                            }
                            if wrote_name {
                                // The write has been applied to the attribute
                                // store by `accept`; read the new name back,
                                // keep it in memory and persist it to flash.
                                if let Ok(new_name) = server.table().get(&name_char) {
                                    if new_name != name {
                                        name = new_name;
                                        info!("device name set to {}", name.as_str());
                                        save_device_name(&mut flash, name.as_str());
                                    }
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }
        },
        // IMU loop.
        async {
            loop {
                info!("temperature {}C", bno055.temperature().unwrap());
                let level = get_level_sense(&mut bno055);
                GRAVITY_X_BITS.store(level.to_bits(), Ordering::Relaxed);
                Timer::after(Duration::from_secs(1)).await;
            }
        },
    )
    .await;

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
    loop {}
}

/// Load the BLE device name from the NVS partition, falling back to the default.
///
/// Blob layout at [`BLE_DEVICE_NAME_ADDR`]: 1 length byte followed by up to
/// [`DEVICE_NAME_MAX`] name bytes, padded to a 4-byte boundary.
fn load_device_name(flash: &mut FlashStorage) -> HeaplessString<DEVICE_NAME_MAX> {
    let mut fallback: HeaplessString<DEVICE_NAME_MAX> = HeaplessString::new();
    let _ = fallback.push_str(DEFAULT_DEVICE_NAME);

    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let Ok(pt) = partitions::read_partition_table(flash, &mut pt_mem) else {
        return fallback;
    };
    let Ok(Some(nvs)) = pt.find_partition(partitions::PartitionType::Data(
        partitions::DataPartitionSubType::Nvs,
    )) else {
        return fallback;
    };
    let mut nvs_partition = nvs.as_embedded_storage(flash);

    let mut buf = [0u8; calypso_core::DEVICE_NAME_BLOB_SIZE];
    if nvs_partition.read(BLE_DEVICE_NAME_ADDR, &mut buf).is_err() {
        return fallback;
    }
    match calypso_core::decode_device_name(&buf) {
        Some(s) => {
            let mut name = HeaplessString::new();
            if name.push_str(s).is_ok() { name } else { fallback }
        }
        None => fallback,
    }
}

/// Persist the BLE device name to the NVS partition. Flash bits only go from
/// 1 to 0 when writing, so the sector is erased first. The name lives in its
/// own sector, leaving the calibration blob in the first sector untouched.
fn save_device_name(flash: &mut FlashStorage, name: &str) {
    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let Ok(pt) = partitions::read_partition_table(flash, &mut pt_mem) else {
        warn!("NVS: partition table read failed");
        return;
    };
    let Ok(Some(nvs)) = pt.find_partition(partitions::PartitionType::Data(
        partitions::DataPartitionSubType::Nvs,
    )) else {
        warn!("NVS: partition not found");
        return;
    };
    let mut nvs_partition = nvs.as_embedded_storage(flash);

    if nvs_partition
        .erase(BLE_DEVICE_NAME_ADDR, BLE_DEVICE_NAME_ADDR + 0x1000)
        .is_err()
    {
        warn!("NVS: erase failed");
        return;
    }
    let Some(buf) = calypso_core::encode_device_name(name) else {
        warn!("NVS: device name is empty or too long");
        return;
    };
    if Storage::write(&mut nvs_partition, BLE_DEVICE_NAME_ADDR, &buf).is_err() {
        warn!("NVS: write failed");
    }
}

fn setup_calibration(bno055: &mut Bno055<I2c<'_, Blocking>>, flash: &mut FlashStorage) {
    // Set up nvs to store calibration data
    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let pt = partitions::read_partition_table(flash, &mut pt_mem).unwrap();

    let nvs = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Nvs,
        ))
        .unwrap()
        .unwrap();
    let mut nvs_partition = nvs.as_embedded_storage(flash);

    let mut bno055_calibration: [u8; BNO055_CALIB_SIZE] = [0; BNO055_CALIB_SIZE];
    nvs_partition
        .read(BNO055_CALIBRATION_ADDR, &mut bno055_calibration)
        .unwrap();

    let mut delay = Delay;
    // Calibration
    let mut calib = BNO055Calibration::from_buf(&bno055_calibration);
    bno055.set_calibration_profile(calib, &mut delay).unwrap();
    if !bno055.is_fully_calibrated().unwrap() {
        info!("Waiting for BNO055 to calibrate");
        while !bno055.is_fully_calibrated().unwrap() {}
        calib = bno055.calibration_profile(&mut delay).unwrap();
        Storage::write(&mut nvs_partition, BNO055_CALIBRATION_ADDR, &calib.as_bytes()).unwrap();
    }

    info!("BNO055 calibrated");
}

fn get_level_sense(bno055: &mut Bno055<I2c<'_, Blocking>>) -> f32 {
    match bno055.gravity() {
        Ok(grav) => {
            // info!("Gravity vector: x={} y={} z={}", grav.x, grav.y, grav.z);
            if calypso_core::is_level_along_axis(grav.x) {
                info!("Sensor is level on the X axis, gravity only on Y/Z");
            } else {
                info!("Sensor is not level: X-grav={}", grav.x);
            }
            return grav.x;
        },
        Err(_e) => {
            warn!("BNO055 gravity read error");
            return 0.0;
        }
    }
}
