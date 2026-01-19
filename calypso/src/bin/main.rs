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
use embassy_time::{Duration, Timer, Delay};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::i2c::master::I2c;
use esp_println as _;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;
use esp_bootloader_esp_idf::partitions;
use esp_storage::FlashStorage;
use embedded_storage::{ReadStorage, Storage};

use bno055::{BNO055OperationMode, BNO055Calibration, BNO055_CALIB_SIZE};
use bno055::Bno055;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

const BNO055_CALIBRATION_ADDR: u32 = 0x00;

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
    let _stack = trouble_host::new(ble_controller, &mut resources);

    // TODO: Spawn some tasks
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

    // Set up nvs to store calibration data
    let mut flash = FlashStorage::new(peripherals.FLASH);
    info!("Flash size = {}", flash.capacity());

    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let pt = partitions::read_partition_table(&mut flash, &mut pt_mem).unwrap();

    let nvs = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Nvs,
        ))
        .unwrap()
        .unwrap();
    let mut nvs_partition = nvs.as_embedded_storage(&mut flash);

    let mut bno055_calibration: [u8; BNO055_CALIB_SIZE] = [0; BNO055_CALIB_SIZE];
    nvs_partition
        .read(BNO055_CALIBRATION_ADDR, &mut bno055_calibration)
        .unwrap();

    // Calibration
    let mut calib = BNO055Calibration::from_buf(&bno055_calibration);
    bno055.set_calibration_profile(calib, &mut delay).unwrap();
    if !bno055.is_fully_calibrated().unwrap() {
        info!("Waiting for BNO055 to calibrate");
        while !bno055.is_fully_calibrated().unwrap() {}
        calib = bno055.calibration_profile(&mut delay).unwrap();
        nvs_partition
        .write(BNO055_CALIBRATION_ADDR, &calib.as_bytes())
        .unwrap();
    }

    info!("BNO055 calibrated");

    loop {
        info!("temperature {}C", bno055.temperature().unwrap());
        match bno055.quaternion() {
            Ok(quat) => info!(
                "Quaternion: w={} x={} y={} z={}",
                quat.s, quat.v.x, quat.v.y, quat.v.z
            ),
            Err(_e) => warn!("BNO055 quaternion read error"),
        }
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
}
