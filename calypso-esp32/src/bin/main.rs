#![no_std]
#![no_main]
// `a1` is the stack pointer, and `core::arch::asm!` needs this feature to reach it (`esp-hal` enables it too).
#![feature(asm_experimental_arch)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use alloc::vec::Vec as AllocVec;
use bt_hci::controller::ExternalController;
use core::cell::RefCell;
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex as AsyncMutex;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use embedded_storage::ReadStorage;
use embedded_storage::nor_flash::NorFlash;
use esp_bootloader_esp_idf::partitions;
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::I2c;
use esp_hal::rng::Rng;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;
use esp_radio::ble::controller::BleConnector;
use esp_storage::FlashStorage as FlashDevice;
use esp_storage::FlashStorage;
use trouble_host::prelude::*;

use alloc::boxed::Box;
use bno055::Bno055;
use bno055::{AccBandwidth, AccGRange, BNO055OperationMode};
use calypso_core::clock::{self, Boots};
use calypso_core::flashlog::{self, FlashLog};
use calypso_core::fusion::{Fusion, FusionState, GRAVITY, conjugate, rotate_by_quat};
use calypso_core::gatt::{
    CalypsoGatt, PIPE_STORAGE_MAX, SCHEMA_HASH, SyncImuValue, SyncReportsValue,
};
use calypso_core::pb::msg;
use calypso_core::pipe::{self, Session, Stalled};
use calypso_core::rep::{Detector, MARGIN_MS, RepSpan, TraceBuf, TracePoint, compute_rep};
use calypso_core::report::Latch;
use calypso_core::sampler::{SAMPLE_RATE_HZ, Sample, SampleRing};
use calypso_core::store::{self, RECORD_BUF_BYTES};
use calypso_core::sync::{SummaryCursor, dev_mode, trace_request};
use calypso_core::uuid::{self, RepUuid};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // `esp_println` works from a panic handler; defmt's formatter does not.
    esp_println::println!("\n*** PANIC: {} ***", info);
    loop {
        core::hint::spin_loop();
    }
}

/// A millisecond timestamp on every line, counted from boot: `esp-println`'s `timestamp` feature calls this for each one.
///
/// A dropped link is a duration — the phone's `The connection has timed out unexpectedly` is the link-layer supervision timer running out — so lining a drop up against what the firmware was doing needs one clock across both tasks, and this is it. `esp-hal`'s counter, which is up before the first line of `main`.
#[unsafe(no_mangle)]
pub extern "Rust" fn _esp_println_timestamp() -> u64 {
    esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_millis()
}

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
// Signal channel + ATT channel
const L2CAP_CHANNELS_MAX: usize = 2;

/// The BLE device name: a compile-time constant, and read-only over GATT.
const DEVICE_NAME: &str = "Calypso";

/// Whether a client has armed dev mode this boot: every rep stored from here on keeps its raw trace as well as its plotted one.
///
/// Off at every boot and never persisted (`msg::DevMode` says why): a switch left on fills the log three times as fast,
/// and a switch remembered would cost a reflash to turn off. On from the first sample in a record-mode build, because
/// the raw trace is the whole of what that build is for and no client is there to ask for it.
static DEV_MODE: AtomicBool = AtomicBool::new(RECORD_MODE);

/// The wall clock the client last set: unix seconds at this boot's uptime zero, or 0 while nothing has set one.
///
/// RAM only, and thrown away at the next boot: the log's `BootClock` records are what outlive the boot (`calypso_core::clock`), and the client writes this one again every time it connects.
static CLOCK_EPOCH_S: AtomicU32 = AtomicU32::new(0);

/// The id this boot's first record got, which is where this boot's reps start — the one thing that tells them from the boot before (`calypso_core::clock`).
///
/// 0 when it could not be written: the log then cannot say which boot a rep came from, so nothing in it is dated, rather than a boot's reps being dated by its neighbour's clock.
static BOOT_FROM_ID: AtomicU32 = AtomicU32::new(0);

/// The device's own faults, raised and not yet sent to a client (`calypso_core::report`).
///
/// A critical-section mutex rather than an atomic, because a fault is a kind plus the time it went out and taking one and marking it sent is a single step. A `RefCell` and not a `Cell`: `Cell::get` hands back a copy, so the "mark it sent" half would land on the copy and the latch would never clear. Written from both tasks (the sampler's I2C and flash failures, the BLE task's own) and taken only by [`push_report`]; the lock lasts a few instructions.
static REPORTS: critical_section::Mutex<RefCell<Latch>> =
    critical_section::Mutex::new(RefCell::new(Latch::new()));

/// The last sample the sampler read, for the app's read of the `Imu` characteristic.
///
/// A critical-section mutex for the reason [`REPORTS`] is one: the sampler writes it a hundred times a second and the BLE task copies it out on its tick, and a sample assembled from two different reads — an acceleration from one tick beside a quaternion from the next — would be a reading the sensor never gave.
static LAST_SAMPLE: critical_section::Mutex<RefCell<Option<(Sample, [f32; 4])>>> =
    critical_section::Mutex::new(RefCell::new(None));

/// Room for one `BootClock` record: a varint field, its tag, and the record envelope (`proto/calypso.proto`).
const CLOCK_RECORD_BYTES: usize = 16;

/// Bench mode: store a synthetic "rep" every this many seconds instead of detecting real ones (`--features fake-reps`).
/// A build flag rather than a runtime setting, so the device is not one accidental setting away from recording fiction as a measurement.
const FAKE_REPS: bool = cfg!(feature = "fake-reps");

/// Record mode: raw windows of whatever the sensor read, on a timer, with no detector (`--features record-mode`).
///
/// The opposite question to [`FAKE_REPS`]: that one exercises the path with motion that never happened, and this one
/// captures motion that did, whether or not anything in it was a rep — which is the only evidence a session that
/// stored nothing can leave behind. The filter still runs, because its offsets and its attitude are half of what a
/// capture is read for; only the decision that a window is worth keeping is taken away from the detector.
const RECORD_MODE: bool = cfg!(feature = "record-mode");

/// A build asks for one bench behaviour, not two: a device that both fabricated reps and recorded windows would tell a
/// reader nothing about which of them a stored record was.
const _: () = assert!(!(FAKE_REPS && RECORD_MODE));

/// How much of the ring a recorded window holds, and how long after one closes the next opens.
///
/// Nine seconds rather than ten: the window is copied into `REP_WINDOW_SAMPLES`, and ten seconds at 100 Hz is one
/// sample more than that buffer holds. The period leaves the gap the append needs — a flash write stops the sampler
/// for its duration, so it belongs between windows rather than inside one.
const RECORD_WINDOW_MS: u32 = 9_000;
const RECORD_PERIOD_MS: u32 = 30_000;

/// What a stored rep's summary says about the sensor that recorded it.
///
/// The settings are the ones `main` puts the chip in before it starts sampling, and the read-back there is what says the
/// chip agreed. The accelerometer's range and bandwidth are chosen; the gyroscope's are the chip's defaults, because this
/// driver can set only the accelerometer's and 2000 dps is past anything a barbell does.
///
/// None of it changes the scale a trace is decoded with — the registers are fixed at 1 m/s² = 100 LSB and 1 dps = 16 LSB
/// whatever the range — so this is what the numbers were taken under, not how to convert them.
const SAMPLER_CONFIG: calypso_core::sampler::RepConfig = calypso_core::sampler::RepConfig {
    accel_range_g: 8,
    gyro_range_dps: 2000,
    // The chip's step names it 62.5 Hz; its own tables label the same step 63.
    accel_bandwidth_hz: 63,
    gyro_bandwidth_hz: 32,
    sample_rate_hz: SAMPLE_RATE_HZ,
    detector_version: calypso_core::rep::DETECTOR_VERSION,
};

/// Attempts at starting the sensor before the device reboots.
///
/// A reset that lands in the middle of an I²C read leaves the sensor holding SDA low, and the first transaction after it
/// times out rather than answering; the driver clears the bus on that error, so the next attempt is on a clean one. Five
/// is enough for that and for a sensor still coming up, and short enough that a genuinely absent sensor reboots quickly.
const BNO055_INIT_ATTEMPTS: u32 = 5;

/// How long each attempt after the first waits, in milliseconds.
const BNO055_INIT_RETRY_MS: u32 = 100;

/// Bench mode's period, and how much of the ring each synthetic rep covers, which has to leave room for the ZUPT margins inside `REP_WINDOW_SAMPLES` (9 s).
const FAKE_REP_PERIOD_S: u32 = 30;
const FAKE_REP_MS: u32 = 6_000;

/// Ring-buffer window: how much history the detector can be asked for, plus headroom.
///
/// When the detector closes a rep, the sampler copies `[start - margin, end + margin]` out of the ring.
/// The longest rep the detector can close is 8 s and the margins are 500 ms each side — as long as they
/// are because the filter only calls a bar at rest after it has held still for 400 ms — so 9 s is the
/// most that is ever needed and 12 s leaves a third of the ring spare.
///
/// Growing it costs heap the radio also lives in: read `Radio up, heap free N B` and `Sync buffers ready, heap free N B` in the boot log before changing it.
const SAMPLE_RING_LEN: usize = REP_WINDOW_SAMPLES + 3 * SAMPLE_RATE_HZ as usize;

/// The sampler's sample window, on the heap (leaked at startup): [`SAMPLE_RING_LEN`] samples is too much for the DRAM static region.
/// The sampler task both fills it and runs the rep detector over it.
type SampleWindow = RefCell<SampleRing<'static>>;

/// Longest gap between two samples that is still this loop being busy rather than stopped: ten times the tick, and well past the tens of ms a flash write in this loop costs.
const SAMPLER_HOLE_MS: u32 = 500;

/// The flash peripheral, and the scratch a partition lookup needs.
///
/// The partition table is 3 KB, and reading one into a local is the largest stack frame this firmware would have — which is why the crate denies `clippy::large_stack_frames`.
/// Keeping the scratch here allocates it once, with the other big buffers, rather than in every function that touches flash, and it needs no lock: every use is already inside the mutex that owns this.
/// Behind a pointer rather than an array, because an array would put those 3 KB in the frame of whoever builds the `FlashStore` — `main`, which shares the linker stack with the BLE stack.
struct FlashStore {
    flash: FlashDevice<'static>,
    pt_mem: &'static mut [u8],
}

impl FlashStore {
    /// The peripheral and the partition-table scratch, borrowed apart.
    /// One accessor rather than destructuring at each site: with the scratch behind a pointer, `let FlashStore { flash, pt_mem } = store` would hand out `&mut &mut [u8]`.
    fn split(&mut self) -> (&mut FlashDevice<'static>, &mut [u8]) {
        (&mut self.flash, &mut self.pt_mem[..])
    }
}

/// Summaries collected per pass of the summary stream. Each is ~76 bytes, so the batch is a heap buffer rather than a local.
const SUMMARY_BATCH: usize = 16;

/// What the heap must still have free once the raw trace buffer is allocated: the radio is in the same heap, and a
/// device this short of it stops taking connections.
const RAW_TRACE_HEADROOM_BYTES: usize = 8 * 1024;

/// How many records this boot has added to the flash log.
///
/// The summary stream re-reads the whole log every time it fills a batch, and a walk over a full log holds the
/// sampler off its 100 Hz tick for seconds — which is where the sampler's timeline holes come from
/// (docs/GOTCHAS.md). A stream with nothing new to read must therefore not read: every append bumps this, and a
/// batch remembers the value it was collected at.
///
/// A count rather than a flag because the writes are in three places (a rep, a boot's clock, the client's clock
/// write) and the reader only has to tell "same as last time" from "not the same".
static LOG_APPENDS: AtomicU32 = AtomicU32::new(0);

/// Say that the log has changed in a way the summary stream has to see.
fn log_changed() {
    LOG_APPENDS.fetch_add(1, Ordering::Relaxed);
}

/// Window handed to the detector when a rep closes: `MAX_REP_MS` plus the ZUPT margins (9 s at 100 Hz).
///
/// Ten seconds, and not the nine a window can ask for: a window is inclusive at both ends, so the longest
/// one is 901 samples, and a buffer that fits it exactly would drop the newest sample — the one a trace is
/// most worth having.
const REP_WINDOW_SAMPLES: usize = 1000;
/// Trace points kept per rep; longer reps are decimated uniformly.
const TRACE_POINTS: usize = calypso_core::rep::TRACE_POINTS;

// A default app-descriptor, required by the esp-idf bootloader: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
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

    // Before `esp_rtos::start` below, and before anything else that allocates stack: the deepest use is what the log lines report.
    paint_stack();

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);
    // A second region: the firmware's own buffers are ~82 KB before the radio allocates anything (sample ring 24 KB, rep window 18 KB, two trace buffers 16 KB each, plus scratch), and the radio's task stacks come from here too.
    // Both regions are statics, so this one is also 64 KB the linker stack does not get.
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    // **No Wi-Fi controller.** BLE-only device: initialising Wi-Fi starts a task with its own stack and driver buffers — most of the ~54 KB the radio took.
    // Safe to leave out, and the evidence is a build without it that ran and stayed up.
    // **If BLE ever stops coming up, put this line back first.**
    //
    // More examples: https://github.com/embassy-rs/trouble/tree/main/examples/esp32

    // 8 KB rather than the 4 KB default. A crash that pointed at a task stack in DRAM (not the linker stack) is what put this here: the controller task has the smallest default stack of anything on the board and is the only one we can size from here. A fault inside it looks nothing like a bug in `main`, which is why the number is written down.
    let ble_config = esp_radio::ble::Config::default().with_task_stack_size(8 * 1024);
    let transport = BleConnector::new(&radio_init, peripherals.BT, ble_config).unwrap();
    let ble_controller = ExternalController::<_, 1>::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    info!("Radio up, heap free {} B", esp_alloc::HEAP.free());

    // Everything runs in this task via join3 below; no extra tasks needed.
    let _ = spawner;

    // Setup the bno055 imu
    info!("Setting up i2c");
    // Pinout for the esp32 pico: https://learn.adafruit.com/assets/112309
    // 400 kHz so the 100 Hz sampler's ~14 bytes/sample cost ~0.5 ms of the 10 ms period.
    let i2c_config = esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO22)
        .with_scl(peripherals.GPIO19);
    info!("Done setting up i2c");

    let mut delay = Delay;

    info!("Setting up Bno055");
    let mut bno055 = Bno055::new(i2c).with_alternative_address();
    // Retried, and rebooted rather than halted if every attempt fails: this device spends its life in a gym bag, and one
    // that halts on a bad first read needs a person to unplug it. A reboot loop is visible in the log and recovers itself.
    let mut attempt = 0;
    loop {
        attempt += 1;
        match bno055.init(&mut delay) {
            Ok(()) => break,
            Err(e) if attempt < BNO055_INIT_ATTEMPTS => {
                warn!(
                    "Bno055 init attempt {} of {} failed ({:?}), retrying",
                    attempt,
                    BNO055_INIT_ATTEMPTS,
                    defmt::Debug2Format(&e)
                );
                Timer::after(Duration::from_millis(BNO055_INIT_RETRY_MS as u64)).await;
            }
            Err(e) => {
                error!(
                    "Bno055 init failed {} times ({:?}), rebooting",
                    BNO055_INIT_ATTEMPTS,
                    defmt::Debug2Format(&e)
                );
                esp_hal::system::software_reset();
            }
        }
    }
    info!("Bno055 initialized");
    // **The raw sensors, not the chip's fusion.** ACCGYRO reports uncompensated accelerometer and gyroscope registers;
    // the orientation is this firmware's (`fusion.rs`), because the fused output cannot say whether its own reading is
    // right and a residue it left behind kept a session from recording anything.
    //
    // The range is set here because a non-fusion mode is the only one that allows it, and it decides only what clips:
    // the registers' scale is fixed at 1 m/s² = 100 LSB and 1 dps = 16 LSB whatever the range. ±8 g clips at 78 m/s²,
    // past a rep and past a rerack; the chip's 4 g default clips at 39 m/s², which a hard rerack can reach. The
    // bandwidth is written too, so the number a rep's summary carries is one this build chose rather than a default.
    //
    // The gyroscope keeps the chip's defaults — 2000 dps at 32 Hz — because this driver can set only the accelerometer's,
    // and at 2000 dps it cannot clip. Both are recorded in `SAMPLER_CONFIG`.
    let mut acc = bno055.get_acc_config().unwrap();
    acc.set_g_range(AccGRange::G8);
    acc.set_bandwidth(AccBandwidth::Hz62_5);
    bno055.set_acc_config(&acc).unwrap();
    // Read back, because the write reports only that the bus took it: what a rep's summary claims has to be what the
    // chip holds, or a trace is decoded against settings nobody had.
    match bno055.get_acc_config() {
        Ok(after) if after.bits() == acc.bits() => {}
        Ok(after) => warn!(
            "Accelerometer reads back {=u8:02x}, not the {=u8:02x} just written",
            after.bits(),
            acc.bits()
        ),
        Err(_) => warn!("Accelerometer configuration could not be read back"),
    }
    bno055
        .set_mode(BNO055OperationMode::ACC_GYRO, &mut delay)
        .unwrap();
    info!(
        "Bno055 in ACCGYRO: accel {} g at {} Hz, gyro {} dps at {} Hz (chip defaults)",
        SAMPLER_CONFIG.accel_range_g,
        SAMPLER_CONFIG.accel_bandwidth_hz,
        SAMPLER_CONFIG.gyro_range_dps,
        SAMPLER_CONFIG.gyro_bandwidth_hz,
    );
    if RECORD_MODE {
        // Said once and loudly, for the reason fake-reps says its own: a device recording whatever it hears must not be
        // mistaken for one detecting reps, and the log is where a session is read back from.
        warn!(
            "record mode: no detector; {} ms of raw samples every {} ms, dev mode on from boot",
            RECORD_WINDOW_MS, RECORD_PERIOD_MS
        );
    }

    // `multicore_auto_park` is not optional on this chip: esp-storage's default strategy *refuses* every write and erase while the second core is running (`MultiCoreStrategy::Error`), and CPU1 is up in this build. Reads are unaffected, which is what makes it so confusing — the NVS reads worked while every erase, including the log partition's first format, came back as an I/O error.
    let flash = FlashStorage::new(peripherals.FLASH).multicore_auto_park();
    info!("Flash size = {}", flash.capacity());
    // The peripheral and its partition-table scratch, together (see FlashStore).
    // The scratch is 3 KB and goes on the heap: as an array inside the struct it would sit in `main`'s frame, the bottom of the stack the BLE stack shares.
    let pt_mem: &'static mut [u8] =
        Box::leak(alloc::vec![0u8; partitions::PARTITION_TABLE_MAX_LEN].into_boxed_slice());
    let mut store = FlashStore { flash, pt_mem };

    info!("BLE device name: {}", DEVICE_NAME);

    // The UUIDs this build exposes, in both orders, as hex: which firmware is on this board, as seen from the other end of the link.
    info!(
        "sync service uuid {=[u8]:02x} (wire), {} (as a client sees it)",
        calypso_core::gatt::uuid::SYNC_SERVICE_BYTES,
        calypso_core::gatt::uuid::SYNC_SERVICE_TEXT
    );

    // Mount the rep-record flash log.
    match mount_rep_log(&mut store) {
        Ok(sectors) => info!("Rep log mounted: {} sectors", sectors),
        // The reason *named*, not just "unusable": whether the flashed table has the log partition at all, and which flash error it was.
        Err(reason) => {
            warn!("Rep log unusable: {}", reason);
            report(msg::ReportKind::LogUnusable);
        }
    }

    // The sample ring, on the heap and alive for the program's lifetime.
    // The buffer is allocated and leaked separately, and the ring borrows it: a ring that owned the array would be built as a value before `Box::new` could move it to the heap.
    let mut ring_vec = AllocVec::new();
    ring_vec.resize(SAMPLE_RING_LEN, Sample::ZERO);
    let ring_buf: &'static mut [Sample] = Box::leak(ring_vec.into_boxed_slice());
    let samples: &'static SampleWindow =
        Box::leak(Box::new(RefCell::new(SampleRing::new(ring_buf))));
    // Rep-detector scratch, also on the heap: too big for the static region, and built with `vec!` so no large temporary hits the stack.
    let mut window_vec = alloc::vec::Vec::new();
    window_vec.resize(REP_WINDOW_SAMPLES, Sample::ZERO);
    let rep_window: &'static mut [Sample] = Box::leak(window_vec.into_boxed_slice());
    let mut trace_vec = alloc::vec::Vec::new();
    trace_vec.resize(TRACE_POINTS, TracePoint::ZERO);
    let rep_trace_storage: &'static mut [TracePoint] = Box::leak(trace_vec.into_boxed_slice());
    let mut rep_trace = TraceBuf::new(rep_trace_storage);
    // Scratch for encoding one log record at a time (~2 KB).
    let rec_buf: &'static mut [u8] =
        Box::leak(alloc::vec![0u8; RECORD_BUF_BYTES].into_boxed_slice());
    info!("Rep detector ready, heap free {} B", esp_alloc::HEAP.free());

    // The one flash peripheral, shared by the sampler task (appending reps) and the BLE task (reading a trace back out to stream it).
    // An async mutex, not a critical section: erasing a sector takes tens of milliseconds and must not block interrupts.
    let store: &'static AsyncMutex<NoopRawMutex, FlashStore> =
        Box::leak(Box::new(AsyncMutex::new(store)));

    // **This address is the device's identity to every client**, and therefore the key a client's cached GATT database lives under.
    // Two bytes of the schema's hash ride in it, so a build whose table changed is a device no client has cached, and the rule needs nobody to remember it.
    // The alternative is a Service Changed characteristic (0x2A05), which tells a bonded client to drop the cache itself; this device has no need for one yet.
    // The top two bits of the most significant byte are set, as a static random address requires -- hence 0xfd, with the schema's hash below it.
    let stamp = SCHEMA_HASH.to_le_bytes();
    let stack =
        trouble_host::new(ble_controller, &mut resources).set_random_address(Address::random([
            0xca, 0xa7, 0x19, stamp[0], stamp[1], 0xfd,
        ]));
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    // GATT table: generated from `proto/calypso_gatt.proto` by `gatt-gen` (`calypso-core/src/gatt.rs`).
    // GAP comes from the config below: the device name a client can read, and the sensor appearance that makes a scanner show what this is.
    // **One table per process**: the macros back each characteristic with a `static StaticCell`, so this cannot be called twice.
    let gatt =
        calypso_core::gatt::CalypsoGatt::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name: DEVICE_NAME,
            appearance: &appearance::sensor::GENERIC_SENSOR,
        }))
        .expect("the device name fits the GAP name length");

    // Buffers for the sync path: a trace read back from the log, and a reusable record scratch, both on the heap.
    // The trace storage goes to the session rather than staying here: the cursor that streams it lives there too, so the points a request is read into and the transfers they go out as are one thing.
    let mut trace_vec = AllocVec::new();
    trace_vec.resize(TRACE_POINTS, TracePoint::ZERO);
    let sync_trace_storage: &'static mut [TracePoint] = Box::leak(trace_vec.into_boxed_slice());
    let sync_rec_buf: &'static mut [u8] =
        Box::leak(alloc::vec![0u8; RECORD_BUF_BYTES].into_boxed_slice());
    // Decoding a record needs ~2 KB (a full trace chunk is stored inline), and these futures live on the main task's stack, so one buffer is reused for every record of every scan.
    let sync_rec_msg: &'static mut msg::RepRecord = leak_record_scratch();
    // ...and the summary batch, for the same reason: `SUMMARY_BATCH` of them is ~1.2 KB.
    // It carries how far through it the pipe has got, so it is also the queue the pipe's one-message-at-a-time rule needs.
    let sync_batch: &'static mut AllocVec<msg::RepSummary> =
        Box::leak(Box::new(AllocVec::with_capacity(SUMMARY_BATCH)));
    let mut batch = SummaryBatch::new(sync_batch);
    // The notify characteristic's value type owns its buffer, so a sender encodes into it rather than into a buffer of its own.
    // It, the pipe's two message buffers and the frame each chunk is notified from are all on the heap: a task's future lives on the stack, and 8 KB of message buffer has no business in one.
    let sync_outgoing: &'static mut [u8] =
        Box::leak(alloc::vec![0u8; pipe::MESSAGE_MAX].into_boxed_slice());
    let sync_incoming: &'static mut [u8] =
        Box::leak(alloc::vec![0u8; pipe::MESSAGE_MAX].into_boxed_slice());
    let notify_payload: &'static mut BleVec = Box::leak(Box::new(BleVec::new()));
    let session: &'static mut pipe::Session<'static> =
        leak_session(sync_outgoing, sync_incoming, sync_trace_storage);
    let mut sync = Sync {
        session,
        characteristic: &gatt.sync.messages,
        payload: notify_payload,
    };
    info!("Sync buffers ready, heap free {} B", esp_alloc::HEAP.free());
    info!("Starting BLE stack");
    // What the setup above is holding before the tasks start: the total moves whenever a static does, so measure rather than assume.
    info!("Stack before the tasks: {}", Stack);

    // The boot's clock, shared by both tasks: the sampler times its samples from it, and the client's epoch is
    // read against it, so a rep's uptime and the wall time attributed to it are the same frame
    // (`calypso_core::clock`).
    let boot = Instant::now();
    join3(
        // HCI runner: must run forever alongside any other BLE tasks.
        async {
            loop {
                if runner.run().await.is_err() {
                    warn!("BLE runner error");
                    report(msg::ReportKind::RadioFailed);
                }
            }
        },
        // Advertising + GATT event handling.
        async {
            loop {
                // The name is fixed, so the advertisement never changes: rebuilt here only because `advertise` borrows the payload. The payloads are built in `calypso-core` so that what a client sees is host-tested — see `build_advertising` for the layout and why it is that way.
                let advertising = calypso_core::ble::build_advertising(DEVICE_NAME);
                if advertising.adv_len == 0 {
                    // An empty advertisement is invisible to every client, so it is worth saying out loud rather than debugging on the phone.
                    warn!("no advertising data to send");
                    report(msg::ReportKind::RadioFailed);
                }

                let advertiser = match peripheral
                    .advertise(
                        &Default::default(),
                        Advertisement::ConnectableScannableUndirected {
                            adv_data: &advertising.adv_data[..advertising.adv_len],
                            scan_data: &[],
                        },
                    )
                    .await
                {
                    Ok(advertiser) => advertiser,
                    Err(_) => {
                        warn!("failed to start advertising");
                        report(msg::ReportKind::RadioFailed);
                        Timer::after(Duration::from_millis(500)).await;
                        continue;
                    }
                };
                info!("advertising");

                let conn = match advertiser.accept().await {
                    Ok(conn) => match conn.with_attribute_server(&gatt) {
                        Ok(conn) => conn,
                        Err(_) => continue,
                    },
                    Err(_) => continue,
                };
                info!("BLE connection established (stack {})", Stack);
                // What a message is cut into, on this link: the MTU the peer's stack reports, and the frame the framing will use given it. `23` is the spec default and what a central that never sends the exchange leaves behind — it is not the size of the notifications that follow (docs/GOTCHAS.md, "A window of 127 is not flow control").
                info!(
                    "ATT MTU {}: notifications up to {} bytes",
                    conn.raw().att_mtu(),
                    pipe::frame_ceiling(conn.raw().att_mtu() as usize)
                );
                // A read returns the last value published, so both are published before a client can ask for them.
                publish_imu(&gatt);

                // Sync state: how many summaries this subscriber has been sent, the batch they are read from, and the clock the pipe's timeouts are measured against.
                //
                // A fresh session per connection: whatever was in flight is gone with the link, and half a message is not a message.
                let mut cursor = SummaryCursor::new();
                let mut subscribed = false;
                // Said once per connection, so a session where nothing goes out explains itself in the log.
                let mut said_no_clock = false;
                let start = Instant::now();
                batch.reset();
                sync.session.reset();

                let mut ticks: u32 = 0;
                loop {
                    // Service GATT events; on every tenth tick (a second), keep the table's copy of the last IMU sample current.
                    //
                    // The tick itself is 100 ms because the app's read of the `Imu` characteristic is a snapshot of the sensor: at a second it would show a rep's whole stroke as one frame. Everything else in this arm is a second's business.
                    let event = match select(conn.next(), Timer::after(Duration::from_millis(100))).await {
                        Either::First(event) => event,
                        Either::Second(()) => {
                            ticks = ticks.wrapping_add(1);
                            publish_imu(&gatt);
                            if !ticks.is_multiple_of(10) {
                                continue;
                            }
                            let now = now_ms(start);
                                        // Who is listening, asked of the server rather than inferred from a notify result, because a notify result says nothing (see `is_subscribed`). Once a tick is often enough to say when it changes.
                            let listening =
                                is_subscribed(&gatt, &conn, gatt.sync.messages.cccd_handle);
                            if listening != subscribed {
                                subscribed = listening;
                                if subscribed {
                                    info!("app subscribed to sync notifications (summaries, traces)");
                                } else {
                                    info!("app unsubscribed from sync notifications");
                                }
                                // An emptied batch, so a resubscribing app starts from the cursor rather than from the middle of a batch the last one left behind.
                                batch.reset();
                            }
                            // A transfer that has stopped moving is this end's to notice: `tick` sends it again, and gives up on it after three attempts.
                            match sync.session.tick(now) {
                                Stalled::No => {}
                                Stalled::Retrying { attempt } => info!(
                                    "sync {}: no answer, sending it again (attempt {})",
                                    sync.session.kind().name(),
                                    attempt
                                ),
                                Stalled::Abandoned { attempt } => {
                                    warn!(
                                        "sync {}: unanswered after {} attempts, dropped",
                                        sync.session.kind().name(),
                                        attempt
                                    );
                                    report(msg::ReportKind::TransferAbandoned);
                                }
                            }
                            if subscribed {
                                // Nothing goes out to nobody: a transfer nobody acknowledges stalls, retries and is abandoned, and it would hold the pipe for the whole 15 s.
                                // Auto-sync, and the trace the app is waiting on: the cursor advances one per message the pipe took, and a transfer to nobody is abandoned, so streaming with no subscriber would walk the whole log into the void and the app would never see it on that connection.
                                //
                                // Until the client sets the clock, this boot's summaries are held rather than sent (`collect_summaries`), so say that once rather than leaving an app that never sends one with nothing on screen and nothing in the log either.
                                if !said_no_clock && CLOCK_EPOCH_S.load(Ordering::Relaxed) == 0 {
                                    said_no_clock = true;
                                    warn!(
                                        "sync: no clock this boot, so no summary goes out until the app sets one"
                                    );
                                }
                                keep_sending(
                                    store,
                                    sync_rec_buf,
                                    sync_rec_msg,
                                    &mut batch,
                                    &mut sync,
                                    &conn,
                                    &mut cursor,
                                )
                                .await;
                            }
                            // Beside the pipe's own sending, and outside its `subscribed`: a fault is
                            // owed to whoever subscribed to `Reports`, not to the summaries' subscriber.
                            push_report(&gatt, &conn, now).await;
                            continue;
                        }
                    };
                    match event {
                        GattConnectionEvent::Disconnected { reason } => {
                            // The device's own view of why, where the phone has only a string: `Err(Connection Timeout)` is the link's supervision timer running out with nothing heard from the peer — a starved radio — and `Err(Remote User Terminated Connection)` is the peer letting go.
                            info!("BLE disconnected: {}", defmt::Debug2Format(&reason));
                            break;
                        }
                        // How long the link tolerates hearing nothing: the number a flash window has to be read against.
                        GattConnectionEvent::ConnectionParamsUpdated {
                            conn_interval,
                            peripheral_latency,
                            supervision_timeout,
                        } => info!(
                            "connection parameters: interval {} ms, peripheral latency {}, supervision timeout {} ms",
                            conn_interval.as_millis() as u32,
                            peripheral_latency,
                            supervision_timeout.as_millis() as u32
                        ),
                        GattConnectionEvent::Gatt { event } => {
                            // Which characteristic a write was for, decided before it is accepted — accepting consumes
                            // the event, and until it has run the table still holds the value before this write.
                            let written = match &event {
                                GattEvent::Write(write) => Some(write.handle()),
                                _ => None,
                            };
                            if let Ok(reply) = event.accept() {
                                reply.send().await;
                            }
                            // The clock: a value, so it is read out of the table rather than reassembled.
                            if written == Some(gatt.sync.time.handle) {
                                match gatt.get(&gatt.sync.time) {
                                    Ok(now_s) => {
                                        set_clock(store, sync_rec_buf, now_s, now_ms(boot)).await
                                    }
                                    Err(_) => warn!("clock: a write with no stored value"),
                                }
                                continue;
                            }
                            // Everything else the app writes is a chunk of a request on the pipe — which is also
                            // where it acknowledges a message this end sent, and the frame says which.
                            if written == Some(gatt.sync.requests.handle) {
                                let wrote = {
                                    let Ok(value) = gatt.get(&gatt.sync.requests) else {
                                        warn!("sync: a write with no stored value");
                                        continue;
                                    };
                                    sync.session.written(value.as_slice(), now_ms(start))
                                };

                                // The app is owed an acknowledgement, and its sender is waiting on it: notify it before anything else this write set off.
                                if wrote.reply {
                                    sync.flush(&conn).await;
                                }

                                // The app asked for something. `Request` used to decode this out of the table; with a pipe the bytes are reassembled here first.
                                if wrote.message {
                                    let Some(request) = sync.session.request() else {
                                        warn!("sync: a reassembled request that does not decode");
                                        report(msg::ReportKind::RequestRefused);
                                        continue;
                                    };
                                    // The dev switch is an operation rather than a question, so it is answered with a log line rather than with a message.
                                    if let Some(enabled) = dev_mode(&request) {
                                        DEV_MODE.store(enabled, Ordering::Relaxed);
                                        if enabled {
                                            info!("dev mode on: every rep from here keeps its raw trace as well");
                                        } else {
                                            info!("dev mode off: reps store the plotted trace only");
                                        }
                                    } else {
                                        let Some(request) = trace_request(&request) else {
                                            warn!("sync: a request this device will not serve");
                                            report(msg::ReportKind::RequestRefused);
                                            continue;
                                        };
                                        let wanted =
                                            calypso_core::pb::uuid_from_field(&request.rep_uuid);
                                        info!(
                                            "trace request: rep {} ({=str})",
                                            UuidShort(wanted.as_ref()),
                                            trace_kind(request.kind)
                                        );
                                        if is_subscribed(&gatt, &conn, gatt.sync.messages.cccd_handle) {
                                            // The two traces are read differently because they are different sizes:
                                            // the plotted one fits the session's buffer from boot, and a window of
                                            // full-rate samples does not.
                                            if request.kind == msg::TraceKind::Raw {
                                                stream_raw_trace(
                                                    store,
                                                    sync_rec_buf,
                                                    sync_rec_msg,
                                                    &batch,
                                                    &mut sync,
                                                    &conn,
                                                    &request,
                                                )
                                                .await;
                                            } else {
                                                stream_trace(
                                                    store,
                                                    sync_rec_buf,
                                                    sync_rec_msg,
                                                    &batch,
                                                    &mut sync,
                                                    &conn,
                                                    request,
                                                )
                                                .await;
                                            }
                                        } else {
                                            // A transfer nobody acknowledges would stall and be abandoned, and the app would wait for a trace that is not coming.
                                            warn!("trace request ignored: notifications are not subscribed");
                                            report(msg::ReportKind::RequestRefused);
                                        }
                                    }
                                }

                                // Then send whatever that write freed — an acknowledgement of this end's own transfer, most often — with a trace chunk ahead of everything else, since the app is waiting on its trace.
                                if subscribed {
                                    keep_sending(
                                        store,
                                        sync_rec_buf,
                                        sync_rec_msg,
                                        &mut batch,
                                        &mut sync,
                                        &conn,
                                        &mut cursor,
                                    )
                                    .await;
                                }
                                push_report(&gatt, &conn, now_ms(start)).await;
                            }
                        }
                        _ => {}
                    }
                }
            }
        },
        // Sampler loop: 100 Hz IMU samples into the RAM ring buffer.
        async {
            // Rep identities come from the hardware RNG: the device has no RTC and its uptime resets on every boot, so a counter or a timestamp could not be unique across boots.
            let rng = Rng::new();
            let mut ticker = Ticker::every(Duration::from_hz(SAMPLE_RATE_HZ as u64));
            // The detector runs inline at 100 Hz: it's a few comparisons per sample, and a closed rep does one trace computation.
            let mut fusion = Fusion::new();
            let mut detector = Detector::new();
            let mut reps: u32 = 0;
            let mut n: u32 = 0;
            let mut last_t_ms: Option<u32> = None;
            // The first poll of the sensor can land before it has written its registers.
            let mut first_sample = true;
            // Said once, in bench mode, so a session cannot be mistaken for a real one.
            let mut logged_fake_reps = false;
            // Whether the filter had settled, logged only when it changes, so "paused until the filter knows which way is down" is printed when it becomes true rather than 3,600 times an hour.
            let mut logged_ready: Option<bool> = None;
            let mut logged_level: Option<bool> = None;
            // Record mode's open window: when it began, and where the filter stood then, because a window nine
            // seconds old has nothing else to replay `compute_rep` from.
            let mut recording: Option<(u32, FusionState)> = None;
            // The earliest moment the next recorded window may open, set past each append because an append stops
            // the sampler for its duration.
            let mut next_record_ms: u32 = 0;
            loop {
                ticker.next().await;
                let t_ms = boot.elapsed().as_millis() as u32;
                // A hole is invisible in the numbers it distorts: `compute_rep` bridges it as one integration step, so a rep measured across it reports travel that never happened.
                // Logged where it is made, on the same clock as the flash and link lines, because the cause is whichever of them was running.
                if let Some(last) = last_t_ms {
                    let gap = t_ms.saturating_sub(last);
                    if gap > SAMPLER_HOLE_MS {
                        // A recorded window is written between windows, so the pause is a hole by construction —
                        // and only a gap with a window open is something a client should be told about.
                        if RECORD_MODE && recording.is_none() {
                            info!("record: {} ms of append between windows", gap);
                        } else {
                            error!("sampler hole: {} ms without a sample", gap);
                            report(msg::ReportKind::SamplerHole);
                        }
                    }
                }
                last_t_ms = Some(t_ms);

                // The chip's two sensor registers, straight into the sample's fixed point: 1 m/s² = 100 LSB and
                // 1 dps = 16 LSB are this format's scales, so nothing is converted and nothing is rounded twice.
                let Ok(accel) = bno055.accel_data_fixed() else {
                    warn!("BNO055 accelerometer read error");
                    report(msg::ReportKind::ImuReadFailed);
                    continue;
                };
                let Ok(gyro) = bno055.gyro_data_fixed() else {
                    warn!("BNO055 gyroscope read error");
                    report(msg::ReportKind::ImuReadFailed);
                    continue;
                };
                let sample = Sample {
                    t_ms,
                    accel: [accel.x, accel.y, accel.z],
                    gyro: [gyro.x, gyro.y, gyro.z],
                };
                // The chip's acceleration registers are still empty when the first poll lands: that sample
                // reads no specific force at all while the rate beside it is plausible. It is a whole g of
                // step at the head of the first window — enough to integrate metres of travel that never
                // happened — so it is dropped rather than recorded. A still sensor never reads zero, so the
                // test cannot throw away a reading that was real.
                if first_sample {
                    first_sample = false;
                    if sample.accel == [0, 0, 0] {
                        warn!("first sample read no acceleration, so it was not a reading: discarded");
                        continue;
                    }
                }
                samples.borrow_mut().push(sample);
                let filtered = fusion.update(&sample);
                // The attitude the app reads is the filter's: the chip no longer reports one.
                critical_section::with(|cs| {
                    *LAST_SAMPLE.borrow(cs).borrow_mut() =
                        Some((sample, fusion.state().attitude));
                });

                // Detect reps; on close, slice the window out of the ring and compute the ZUPT-anchored trace + summary, then persist it.
                //
                // Only once the filter knows which way is down: it seeds gravity and both offsets from a still window, and before that there is no vertical for a rep to be measured against, so a trace recorded then would be fiction rather than a measurement.
                let window = if RECORD_MODE {
                    // The detector is not asked, so nothing here can decide a window is not a rep. This sample either
                    // opens one or ends one; the close is logged with the moment the next is due, so the log says
                    // plainly which of recording and waiting the device is doing, and when.
                    match recording {
                        // Not before the filter knows which way is down: a window that opens first has no
                        // vertical for `compute_rep` to replay in, and its trace reads hundreds of metres.
                        // The price is that a device whose gate never opens records nothing — which the
                        // sampler's own line says, rather than leaving silence to be interpreted.
                        None if t_ms >= next_record_ms && fusion.ready() => {
                            recording = Some((t_ms, filtered.state));
                            info!(
                                "record: window opened at {} ms, {} ms of samples to go",
                                t_ms, RECORD_WINDOW_MS
                            );
                            None
                        }
                        Some((start_ms, state))
                            if t_ms.saturating_sub(start_ms) >= RECORD_WINDOW_MS =>
                        {
                            recording = None;
                            next_record_ms = t_ms + RECORD_PERIOD_MS;
                            info!(
                                "record: window {}..{} closed, next opens at {} ms",
                                start_ms, t_ms, next_record_ms
                            );
                            Some((
                                RepSpan {
                                    start_ms,
                                    end_ms: t_ms,
                                },
                                Some((start_ms, state)),
                            ))
                        }
                        _ => None,
                    }
                } else if FAKE_REPS {
                    // Bench mode: one synthetic rep per period, covering the last FAKE_REP_MS of samples whatever the motion was. The filter's readiness is deliberately *not* applied — these windows are cut from whatever the sensor read rather than detected in it, and a bench session should not depend on the bar holding still first. Everything downstream is untouched: the records are ordinary ones the app syncs and plots.
                    if !logged_fake_reps {
                        logged_fake_reps = true;
                        warn!(
                            "fake-reps: storing a synthetic rep every {} s, not detecting real ones",
                            FAKE_REP_PERIOD_S
                        );
                        // One synthetic fault as well, raised before any client can have subscribed: what the
                        // app shows is then the latch being handed over, which is the half of this path a
                        // bench session with a real fault could not easily reach.
                        warn!("fake-reps: raising a synthetic report (REP_NOT_STORED)");
                        report(msg::ReportKind::RepNotStored);
                    }
                    if n.is_multiple_of(SAMPLE_RATE_HZ * FAKE_REP_PERIOD_S)
                        && t_ms >= FAKE_REP_MS
                    {
                        Some((
                            RepSpan {
                                start_ms: t_ms - FAKE_REP_MS,
                                end_ms: t_ms,
                            },
                            None,
                        ))
                    } else {
                        None
                    }
                } else if fusion.ready() {
                    // The filter is fed every sample, so its offsets and its attitude keep up with the sensor; only
                    // the detector is gated, and it starts from a real rest whenever the gate opens.
                    detector.update(&filtered).map(|span| (span, None))
                } else {
                    None
                };
                if let Some((span, edge)) = window {
                    // Where the window begins and where the filter stood: the detector's answer for a rep it closed,
                    // the snapshot taken when the window opened for one that was recorded. A detected rep's is not
                    // always `MARGIN_MS` before it, because one close behind the boot has no history that far back.
                    let (from_ms, state) = edge.unwrap_or_else(|| detector.rep_state());
                    let copied = samples
                        .borrow()
                        .copy_window(from_ms, span.end_ms, 0, MARGIN_MS, rep_window);
                    // Logged at the moment the detector closes one, before anything can reject it: without this line a session that stores nothing cannot tell "the detector never fired" from "every rep was thrown away", and the two have different fixes. A recorded window logs its own open and close, so this would only repeat half of it.
                    if !RECORD_MODE {
                        info!(
                            "span closed: {}..{} ({} ms)",
                            span.start_ms,
                            span.end_ms,
                            span.duration_ms()
                        );
                    }
                    if copied == 0 {
                        warn!("rep window unavailable (ring too short)");
                        report(msg::ReportKind::RepWindowUnavailable);
                    } else {
                        // Where the filter stood at the window's edge: the detector's for a real rep,
                        // level for a synthetic one the fake-reps build fabricates wholesale.
                        let state = if FAKE_REPS {
                            calypso_core::fusion::FusionState::LEVEL
                        } else {
                            state
                        };
                        let mut sum = compute_rep(&rep_window[..copied], &state, &mut rep_trace);
                        // A recorded window is kept whatever it measured: a session in which nothing was a rep is
                        // exactly the case it exists to leave evidence of.
                        if !RECORD_MODE && !sum.is_a_rep() {
                            info!(
                                "rep discarded: {} m of travel, under the {} m a rep needs",
                                sum.rom,
                                calypso_core::rep::MIN_ROM_M
                            );
                        } else {
                            reps += 1;
                            let rep_uuid = {
                                let mut rand = || rng.random();
                                uuid::new_uuid(&mut rand)
                            };
                            // Identity is minted when the rep closes, so it is set here rather than computed with the stats.
                            sum.rep_uuid = calypso_core::pb::uuid_to_field(&rep_uuid);
                            // What the sensor was set to, so the trace can be read without assuming today's build.
                            // A recorded window says version 0: no detection logic closed it, and 0 is what the field
                            // says when nothing did.
                            let config = if RECORD_MODE {
                                calypso_core::sampler::RepConfig {
                                    detector_version: 0,
                                    ..SAMPLER_CONFIG
                                }
                            } else {
                                SAMPLER_CONFIG
                            };
                            sum.set_config(calypso_core::pb::config_message(&config));
                            // A dev session keeps what the sensor read on top of what it plotted: the plotted trace is what a screen draws, and only the raw one can be replayed. Same window, so the two cover the same rep.
                            let raw = DEV_MODE
                                .load(Ordering::Relaxed)
                                .then_some(&rep_window[..copied]);
                            // Persist the rep (summary + trace chunks). This happens right after the rest debounce, i.e. while the sensor is at rest, so the tens of ms an erase takes here can't land in the middle of a rep.
                            let stored = {
                                let mut guard = store.lock().await;
                                flash_window("rep append", || {
                                    store_rep(&mut guard, rec_buf, &mut sum, &rep_trace, raw)
                                })
                            };
                            match stored {
                                Ok(n) => {
                                    // A new summary for the stream to read, and a walk it can now skip.
                                    log_changed();
                                    info!(
                                        "rep {} stored as {} log records",
                                        UuidShort(Some(&rep_uuid)),
                                        n
                                    )
                                }
                                Err(e) => {
                                    warn!(
                                        "rep {} could not be stored: {}",
                                        UuidShort(Some(&rep_uuid)),
                                        e
                                    );
                                    report(msg::ReportKind::RepNotStored);
                                }
                            }
                            info!(
                                "rep {}: {}..{} ms ({} ms), {} trace pts, {} raw samples, rom {=f32} m, peak v {=f32} m/s, mean v {=f32} m/s, peak a {=f32} m/s^2, roll {=f32}..{=f32} deg, pitch {=f32}..{=f32} deg",
                                UuidShort(Some(&rep_uuid)),
                                span.start_ms,
                                span.end_ms,
                                sum.duration_ms,
                                sum.computed_len,
                                sum.raw_len,
                                sum.rom,
                                sum.peak_velocity,
                                sum.mean_velocity,
                                sum.peak_accel,
                                sum.roll_min_deg,
                                sum.roll_max_deg,
                                sum.pitch_min_deg,
                                sum.pitch_max_deg,
                            );
                        }
                    }
                }

                n += 1;
                // The level verdict and the sampler figures are *read* at 1 Hz, but logged only when they change, or once a minute. Two lines a second used to say the same thing forever: the verdict only moves when the sensor does, and the ring and heap figures are there to spot a leak, not to be watched. The minute keeps the boot-log figures this device is debugged from (see docs/GOTCHAS.md).
                if n.is_multiple_of(SAMPLE_RATE_HZ) {
                    let (x_grav, is_level) = level_sense(fusion.state().attitude);
                    // Compared on the verdict, not the reading: X-grav moves by a hundredth a second, and the question is only whether it is level.
                    if logged_level != Some(is_level) {
                        logged_level = Some(is_level);
                        if is_level {
                            info!("Sensor is level on the X axis (X-grav={=f32})", x_grav)
                        } else {
                            warn!("Sensor is not level, X-grav={=f32}", x_grav)
                        }
                    }

                    // Every 5 s while recording: what the filter is subtracting, and how much motion it sees.
                    //
                    // A session that stores nothing has two causes that look identical from outside — the sensor's offset never came out of the signal, or the bar really was still — and the once-a-minute sampler line says neither.
                    if fusion.ready() && n.is_multiple_of(SAMPLE_RATE_HZ * 5) {
                        let b = fusion.state().accel_bias;
                        // Centimetres per second squared, because defmt has no precision hint and whole numbers read better than raw floats.
                        let a = calypso_core::fusion::magnitude(filtered.linear_accel);
                        let speed = detector.speed();
                        info!(
                            "detector: bias ({}, {}, {}) cm/s², |a| {} cm/s², |v| {} cm/s, legs {}, open {}",
                            (b[0] * 100.0) as i32,
                            (b[1] * 100.0) as i32,
                            (b[2] * 100.0) as i32,
                            (a * 100.0) as i32,
                            (speed * 100.0) as i32,
                            detector.legs(),
                            detector.leg_open()
                        );
                    }

                    let ready = fusion.ready();
                    if n.is_multiple_of(SAMPLE_RATE_HZ * 60) || logged_ready != Some(ready) {
                        logged_ready = Some(ready);
                        let (len, pushed) = {
                            let r = samples.borrow();
                            (r.len(), r.pushed())
                        };
                        if ready {
                            info!(
                                "sampler: {} Hz, ring {}/{} samples, {} pushed, {} reps, imu settled (stack {}, heap free {} B)",
                                SAMPLE_RATE_HZ,
                                len,
                                SAMPLE_RING_LEN,
                                pushed,
                                reps,
                                Stack,
                                esp_alloc::HEAP.free()
                            );
                        } else {
                            // Warn, not info: nothing is being recorded until the bar holds still for a moment.
                            warn!(
                                "sampler: {} Hz, ring {}/{} samples, {} pushed, imu {=f32} s of stillness — rep recording paused until the filter knows which way is down (stack {})",
                                SAMPLE_RATE_HZ,
                                len,
                                SAMPLE_RING_LEN,
                                pushed,
                                fusion.still_for_s(),
                                Stack
                            );
                        }
                    }
                }
            }
        },
    )
    .await;

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
    //
    // Unreachable: every arm of the join3 above loops forever.
    loop {
        core::hint::spin_loop();
    }
}

/// Mount the rep-record flash log on the `log` data partition (subtype `undefined`, see partitions.csv), and open this boot's clock record. Returns the number of sectors in the ring. Mounting here formats the partition on first boot; individual reps are written later by [`store_rep`].
fn mount_rep_log(store: &mut FlashStore) -> Result<u32, LogError> {
    let (flash, pt_mem) = store.split();
    let pt = partitions::read_partition_table(flash, pt_mem).map_err(LogError::Table)?;
    let Some(log_part) = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Undefined,
        ))
        .map_err(LogError::Table)?
    else {
        // The flashed table has no data/undefined partition. Usually a stale table on the device rather than a broken log, so say what is actually there — otherwise the next step is a guess.
        log_partition_table(&pt);
        return Err(LogError::NoPartition);
    };
    // Scoped so the region's borrow of `flash` ends before the probe below.
    let opened: Result<u32, flashlog::Error<partitions::Error>> = {
        let mut region = log_part.as_embedded_storage(flash);
        match FlashLog::mount(&mut region) {
            Err(e) => Err(e),
            Ok(mut log) => {
                let sectors = log.sectors();
                // Every boot opens with its own clock record, before any rep it will contain
                // (`calypso_core::clock`): it is what stops this boot's reps being dated by the boot before it,
                // and what the client's write replaces later in this boot. Its id is where this boot's reps start.
                let from_record_id = log.next_id();
                let mut buf = [0u8; CLOCK_RECORD_BYTES];
                match store::append_clock(&mut log, &mut buf, 0) {
                    Ok(_) => {
                        BOOT_FROM_ID.store(from_record_id, Ordering::Relaxed);
                        Ok(sectors)
                    }
                    // Without that record nothing in the log can be told apart by age, so it is unusable for
                    // this boot: the same answer as a mount that failed, and the same log line.
                    Err(store::Error::Log(e)) => Err(e),
                    Err(store::Error::Encode(e)) => return Err(LogError::Encode(e)),
                }
            }
        }
    };
    match opened {
        Ok(sectors) => Ok(sectors),
        Err(e) => {
            probe_log_partition(flash, log_part.offset(), failing_sector(&e));
            Err(map_log_error(e))
        }
    }
}

/// One `flashlog::Error` in the terms the log line needs.
fn map_log_error(e: flashlog::Error<partitions::Error>) -> LogError {
    match e {
        flashlog::Error::RegionTooSmall => LogError::RegionTooSmall,
        flashlog::Error::TooLarge => LogError::TooLarge,
        flashlog::Error::Flash {
            phase,
            sector,
            source,
        } => LogError::Region {
            phase,
            sector,
            source,
        },
    }
}

/// A failing flash phase in words, with the sector: `erasing sector 1`, `header scan stopped at sector 212`.
///
/// `calypso-core` owns the vocabulary ([`flashlog::Phase`]) and this firmware owns the sentence, because core is platform-independent and has no defmt.
struct FlashPhase {
    phase: flashlog::Phase,
    sector: u32,
}

impl defmt::Format for FlashPhase {
    fn format(&self, f: defmt::Formatter) {
        match self.phase {
            flashlog::Phase::ScanHeaders => {
                defmt::write!(f, "header scan stopped at sector {}", self.sector)
            }
            flashlog::Phase::ReadRecords => {
                defmt::write!(f, "reading records in sector {}", self.sector)
            }
            flashlog::Phase::Erase => defmt::write!(f, "erasing sector {}", self.sector),
            flashlog::Phase::Write => defmt::write!(f, "writing sector {}", self.sector),
        }
    }
}

/// Read **the whole sector that failed**, in 64-byte raw reads, and say where it stops.
///
/// Only after a failed log operation: the vendor partition wrapper maps every storage failure to `partitions::Error::StorageError`, while the raw `FlashStorage` reports the real `FlashStorageError`, and a read is safe to repeat.
///
/// The sector comes from the failure rather than from sector 0, and walking rather than sampling is the point: it runs from a shallow frame after the failed read has unwound, so "sector N reads end to end" means the bytes are there and the failure is about the context the real read ran in.
///
/// A write is deliberately not probed: repeating an erase to see whether it works is not a diagnostic, it is the bug.
fn probe_log_partition(flash: &mut FlashDevice<'static>, partition_offset: u32, sector: u32) {
    let sector_size = FlashDevice::ERASE_SIZE as u32;
    let base = partition_offset + sector * sector_size;
    let mut buf = [0u8; 64];
    let mut first = [0u8; 8];
    let mut offset = 0u32;
    while offset < sector_size {
        if let Err(e) = flash.read(base + offset, &mut buf) {
            warn!(
                "Rep log probe: sector {} failed a raw read at +{}: {}",
                sector, offset, e
            );
            return;
        }
        if offset == 0 {
            first.copy_from_slice(&buf[..8]);
        }
        offset += buf.len() as u32;
    }
    info!(
        "Rep log probe: sector {} reads end to end from this frame (starts {=[u8]:02x})",
        sector, first
    );
}

/// The sector a failed flash operation was on, or 0 when the error was not ours to place. Read before the error is mapped, because mapping consumes it.
fn failing_sector(e: &flashlog::Error<partitions::Error>) -> u32 {
    match e {
        flashlog::Error::Flash { sector, .. } => *sector,
        _ => 0,
    }
}

/// Why a flash-log operation failed, named.
///
/// Used for both the boot-time mount and a rep append: an append mounts the log itself (see [`store_rep`]), so it fails for the same reasons.
///
/// The underlying errors travel through rather than being folded into a sentence:
///
/// - for the table, `Invalid` means it read fine and is not a table, while `StorageError` means reading it failed — "partition table unreadable" covered both;
/// - for the region, the *phase* says which flash operation failed (a refused erase and a failed read have different repairs — see `flashlog::Phase`), and the `partitions::Error` beside it says how far it got: `StorageError` means the storage layer failed (the specific error is flattened away by the vendor wrapper — see [`probe_log_partition`]), while `OutOfBounds`, `WriteProtected` and `InvalidArgument` mean the operation never reached the flash. "Flash read/write error on the log partition" covered all of them, and cost a session to read.
///
/// `partitions::Error` is `#[non_exhaustive]` and formatted rather than matched, so a future variant prints instead of failing to compile.
enum LogError {
    Table(partitions::Error),
    NoPartition,
    RegionTooSmall,
    TooLarge,
    Region {
        phase: flashlog::Phase,
        sector: u32,
        source: partitions::Error,
    },
    Encode(calypso_core::pb::Error),
}

impl defmt::Format for LogError {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::Table(e) => defmt::write!(f, "partition table unreadable: {}", e),
            Self::NoPartition => {
                defmt::write!(f, "no data/undefined partition in the flashed table")
            }
            Self::RegionTooSmall => {
                defmt::write!(f, "log partition smaller than two erase sectors")
            }
            Self::TooLarge => defmt::write!(f, "a record did not fit one erase sector"),
            Self::Region {
                phase,
                sector,
                source,
            } => defmt::write!(
                f,
                "log partition: {}: {}",
                FlashPhase {
                    phase: *phase,
                    sector: *sector
                },
                source
            ),
            // Ours rather than the flash's: something was too big for a buffer we sized, which is a different repair from a flash that will not write.
            Self::Encode(calypso_core::pb::Error::BufferTooSmall) => {
                defmt::write!(f, "record did not fit the encode buffer")
            }
            Self::Encode(calypso_core::pb::Error::Decode) => {
                defmt::write!(f, "record decode failed")
            }
            Self::Encode(calypso_core::pb::Error::Empty) => {
                defmt::write!(f, "record had no body")
            }
        }
    }
}

/// Log what the flashed partition table contains, entry by entry.
///
/// Only on a failed lookup: the answer to "is the table the one this firmware was built for?" then sits in the log instead of in a guess.
fn log_partition_table(pt: &partitions::PartitionTable<'_>) {
    for entry in pt.iter() {
        let kind = match entry.partition_type() {
            partitions::PartitionType::App(_) => "app",
            partitions::PartitionType::Data(_) => "data",
            partitions::PartitionType::Bootloader(_) => "bootloader",
            partitions::PartitionType::PartitionTable(_) => "ptable",
        };
        info!(
            "  {}: {} subtype {} at {} len {}",
            entry.label_as_str(),
            kind,
            entry.raw_subtype(),
            entry.offset(),
            entry.len()
        );
    }
}

/// The value a pipe characteristic stores: a chunk's frame, in the table's own type.
///
/// `PIPE_STORAGE_MAX` because that is what the generated characteristic is typed as — `notify` takes the type the table holds — and the spare byte over a frame is what keeps a full-frame write out of `trouble-host`'s equal-length path (see `docs/GOTCHAS.md`). What is notified is only ever a frame, `pipe::FRAME_MAX` bytes at most.
type BleVec = heapless::Vec<u8, PIPE_STORAGE_MAX>;

/// The sending side of the sync service: the pipe's session, the one characteristic every message goes out on, and the buffer each frame is built in.
///
/// One characteristic means one payload buffer, and the pipe means one *message* at a time: a summary and a trace chunk are both transfers, and only one of them can be in flight.
struct Sync<'a> {
    session: &'a mut Session<'static>,
    characteristic: &'a Characteristic<BleVec>,
    payload: &'a mut BleVec,
}

impl Sync<'_> {
    /// Notify everything the pipe has ready: the frame it owes for the last write, and the chunks of the message in flight.
    ///
    /// The buffer is reused for each, so it is resized to a whole frame before the pipe writes one into it and truncated back to the frame's length afterwards.
    async fn flush(&mut self, conn: &GattConnection<'_, '_, DefaultPacketPool>) {
        let Sync {
            session,
            characteristic,
            payload,
        } = self;
        let mtu = conn.raw().att_mtu() as usize;
        // Frames of the *message* this call gets out (the reply is not one). A message wider than the window is sent in waves, and a wave that stops short is the thing to see when a transfer stalls (docs/GOTCHAS.md).
        let mut frames = 0u32;

        loop {
            if payload.resize(pipe::FRAME_MAX, 0).is_err() {
                return;
            }
            let len = match session.reply(payload.as_mut_slice()) {
                // An acknowledgement first: the app's sender is waiting on it, and the chunk it is about would otherwise go out ahead of it.
                Some(len) => len,
                None => match session.next_chunk(mtu, payload.as_mut_slice()) {
                    Some(len) => {
                        frames += 1;
                        len
                    }
                    None => break,
                },
            };
            payload.truncate(len);
            // A notification to nobody reports success (see `is_subscribed`), so this result is ever only a radio failure.
            if characteristic.notify(conn, payload).await.is_err() {
                warn!("sync: notify failed");
                report(msg::ReportKind::NotifyFailed);
                return;
            }
        }
        // The wave that stopped short says so, because the alternative is an app waiting for frames that are sitting in this buffer.
        if frames > 0 && session.awaiting_ack() {
            info!(
                "sync {}: {} frame(s) of a {} B message out, window full at {} — the rest goes when the acknowledgement lands",
                session.kind().name(),
                frames,
                session.message_len(),
                pipe::WINDOW,
            );
        }
        payload.clear();
    }

    /// Send `summary` if the pipe is free, and notify its chunks.
    ///
    /// `false` means the pipe is still finishing the message before this one: the caller keeps its message and asks again when the acknowledgement arrives.
    async fn send_summary(
        &mut self,
        summary: &msg::RepSummary,
        conn: &GattConnection<'_, '_, DefaultPacketPool>,
    ) -> bool {
        if !self.session.send_summary(summary) {
            return false;
        }
        self.flush(conn).await;
        true
    }

    /// Send the next chunk of the trace being streamed, if there is one and the pipe is free.
    async fn send_trace_chunk(&mut self, conn: &GattConnection<'_, '_, DefaultPacketPool>) -> bool {
        if !self.session.send_next_trace_chunk() {
            return false;
        }
        self.flush(conn).await;
        true
    }
}

/// The summaries read out of the log and waiting for the pipe, and how far the pipe has got through them.
///
/// One batch is read at a time and drained one *transfer* at a time, because that is what the pipe carries: a batch of sixteen is sixteen round trips, so it is refilled and drained across event loop iterations rather than in one call.
struct SummaryBatch {
    messages: &'static mut AllocVec<msg::RepSummary>,
    next: usize,
    /// What the log looked like when this batch was collected, or `None` if it never has been: a collect that
    /// drained the log is not repeated while the answer cannot have changed.
    collected_at: Option<u32>,
    /// Whether that collect ran out of log rather than out of batch. A batch that was *filled* has more behind it
    /// whatever `LOG_APPENDS` says, which is what keeps a log of exactly one batch's worth of reps from stalling.
    drained: bool,
    /// Where the records of the reps this batch handed out are.
    ///
    /// Not cleared by `reset`: a rep's records do not move, and the app may still ask for a trace of a rep it was
    /// sent on the connection before this one.
    index: BatchIndex,
}

impl SummaryBatch {
    fn new(messages: &'static mut AllocVec<msg::RepSummary>) -> Self {
        messages.clear();
        SummaryBatch {
            messages,
            next: 0,
            collected_at: None,
            drained: false,
            index: BatchIndex::new(),
        }
    }

    fn spent(&self) -> bool {
        self.next >= self.messages.len()
    }

    /// Whether collecting again could find a summary this batch does not hold.
    ///
    /// Two reasons it could: the last collect filled the batch rather than the log, or the log has grown since.
    fn stale(&self) -> bool {
        !self.drained || self.collected_at != Some(LOG_APPENDS.load(Ordering::Relaxed))
    }

    fn peek(&self) -> Option<&msg::RepSummary> {
        self.messages.get(self.next)
    }

    fn advance(&mut self) {
        self.next += 1;
    }

    /// Back to an empty batch: a new connection, or a subscriber that came back.
    fn reset(&mut self) {
        self.messages.clear();
        self.next = 0;
        self.collected_at = None;
        self.drained = false;
    }
}

/// Where the reps this batch handed out sit in the log, so the trace each summary causes is a walk of that rep's
/// records rather than of the whole log (`calypso_core::sync::RepIndex`).
///
/// Not cleared by `reset` (see the field), and never the record of what exists: a miss falls back to the walk.
type BatchIndex = calypso_core::sync::RepIndex;

/// Push any log summaries the subscriber hasn't been sent yet, oldest first.
///
/// **Call this only while the peer is subscribed** ([`is_subscribed`]). The cursor advances on every message the pipe took, and a transfer to nobody is abandoned after three attempts, so calling it with no subscriber walks the log into the void and the app never sees those reps on that connection.
///
/// One summary per call, at most: a message is a transfer, a transfer ends when the app acknowledges it, and the acknowledgement arrives as a *write* — so this is called from the tick, from each write, and from nothing else. Whatever is not sent this time is still in the batch, and the cursor has not moved past it.
async fn stream_new_summaries(
    store: &'static AsyncMutex<NoopRawMutex, FlashStore>,
    rec_buf: &mut [u8],
    rec: &mut msg::RepRecord,
    batch: &mut SummaryBatch,
    sync: &mut Sync<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    cursor: &mut SummaryCursor,
) {
    // The next batch is read only when the last one is spent *and* the pipe is free:
    // reading the log is a flash scan, and running it while a transfer was in flight would hold the event loop for a batch that could not go out yet anyway.
    if batch.spent() {
        if !sync.session.is_idle() {
            return;
        }
        // A collect walks every record in the log, and the sampler stops for as long as it runs (docs/GOTCHAS.md):
        // one that already drained the log is not run again until the log has changed.
        if !batch.stale() {
            return;
        }
        let collected_at = LOG_APPENDS.load(Ordering::Relaxed);
        let collected = {
            let mut guard = store.lock().await;
            collect_summaries(
                &mut guard,
                rec_buf,
                rec,
                cursor,
                batch.messages,
                &mut batch.index,
            )
        };
        let Some(collected) = collected else {
            return;
        };
        // Reps fall off the ring as it wraps; never point past the end.
        cursor.clamp_to(collected.total);
        batch.next = 0;
        batch.drained = collected.drained;
        batch.collected_at = Some(collected_at);
    }

    let Some(summary) = batch.peek() else {
        return;
    };
    if sync.send_summary(summary, conn).await {
        batch.advance();
        cursor.advance(1);
    }
}

/// What one pass of the log found.
struct Collected {
    /// Summaries in the log, the ones behind the cursor included.
    total: usize,
    /// Whether the pass ran out of log rather than out of batch: everything the cursor could be offered is in the
    /// batch, or this boot's clock is still missing and the stream is holding for it. A batch that was *filled* is
    /// not this: there is more behind it, and the cursor has not been sent it.
    drained: bool,
}

/// Fill `batch` with the next summaries the subscriber hasn't seen, returning what the log holds and whether there
/// was more than the batch could take (or `None` if the log couldn't be read).
///
/// Also records where each summary it hands out sits in the log, which is how the trace request that summary causes
/// is answered without reading the log again (see [`BatchIndex`]).
///
/// **Everything that goes out carries a time** (`calypso_core::clock`), and the two kinds of untimed summary are handled differently:
/// one from a boot that never had a clock can never gain one, so it is stepped over for good; one from *this* boot is only waiting for the client to set the clock, so the batch holds there rather than sending a rep the app cannot place in time.
///
/// Sync on purpose. Everything inlined into an async fn becomes part of its future, and a future lives on the task's stack for as long as it is suspended:
/// the flash log is a deep call graph (mount, sector scan, CRC, decode), none of which needs to be in a task's state while it waits for a notification. Keeping it behind this boundary is what holds both of those frames to one buffer apiece.
fn collect_summaries(
    store: &mut FlashStore,
    rec_buf: &mut [u8],
    rec: &mut msg::RepRecord,
    cursor: &mut SummaryCursor,
    batch: &mut AllocVec<msg::RepSummary>,
    index: &mut BatchIndex,
) -> Option<Collected> {
    let (flash, pt_mem) = store.split();
    let pt = partitions::read_partition_table(flash, pt_mem).ok()?;
    let partition = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Undefined,
        ))
        .ok()??;
    let mut region = partition.as_embedded_storage(flash);
    let mut log = FlashLog::mount(&mut region).ok()?;

    // This boot as the log sees it: the id its first record got, and the epoch in RAM. 0 for the id means the
    // mount could not write that record, which leaves the log unable to say which boot a rep came from — so
    // `Boots` dates none of them, rather than dating them by the boot before.
    let our_from_record_id = match BOOT_FROM_ID.load(Ordering::Relaxed) {
        0 => None,
        id => Some(id),
    };
    let mut boots = Boots::new(our_from_record_id, CLOCK_EPOCH_S.load(Ordering::Relaxed));

    batch.clear();
    let skip = cursor.next();
    let mut summaries = 0usize;
    let mut holding = false;
    let mut filled = false;
    if let Err(e) = store::for_each_record(&mut log, rec_buf, rec, |id, record| {
        match &record.body {
            Some(msg::RepRecord_::Body::Clock(clock)) => boots.on_clock(clock),
            Some(msg::RepRecord_::Body::Summary(s)) => {
                // 1-based here, and `skip` counts the summaries already behind the cursor.
                summaries += 1;
                if summaries <= skip || holding {
                    return;
                }
                if batch.len() >= batch.capacity() {
                    filled = true;
                    return;
                }
                let Some(t_unix_ms) = boots.boot_of(id).rep_time_ms(s.t_start_ms) else {
                    if boots.is_ours(id) {
                        // The client has not set this boot's clock yet. Everything after this record is this
                        // boot's too, and one write dates it all, so the batch waits for it.
                        holding = true;
                    } else {
                        // A boot that never had a clock never will, so this rep is never sent: step over it, or
                        // every batch from here on would read it again.
                        cursor.advance(1);
                    }
                    return;
                };
                let mut summary = s.clone();
                summary.t_unix_ms = t_unix_ms;
                // A trace request for this rep comes as soon as this summary lands on the phone, and its chunks are
                // the records that follow it (`store::append_rep` writes them together).
                if let Some(rep_uuid) = calypso_core::pb::summary_uuid(s) {
                    index.record(rep_uuid, id + 1, s.computed_chunks, s.raw_chunks);
                }
                batch.push(summary);
            }
            _ => {}
        }
    }) {
        // A short batch is worse than none: the cursor would advance past summaries that were never sent, and the app would never be offered them again.
        warn!(
            "sync: reading summaries failed: {}",
            defmt::Debug2Format(&e)
        );
        report(msg::ReportKind::LogUnusable);
        return None;
    }
    Some(Collected {
        total: summaries,
        drained: !filled,
    })
}

/// Read a requested rep's trace out of the log into the session, and start streaming it: one `TraceChunk` per transfer, in chunk order.
///
/// The transfers are the tick's business: this reads the trace into the session's storage and points the cursor at it, and the tick — or the app's acknowledgement — sends the next chunk.
async fn stream_trace(
    store: &'static AsyncMutex<NoopRawMutex, FlashStore>,
    rec_buf: &mut [u8],
    rec: &mut msg::RepRecord,
    batch: &SummaryBatch,
    sync: &mut Sync<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    request: msg::TraceRequest,
) {
    let Some(rep_uuid) = calypso_core::pb::uuid_from_field(&request.rep_uuid) else {
        warn!("trace request with a malformed uuid");
        report(msg::ReportKind::RequestRefused);
        return;
    };

    // Read it into the session's storage, which is where the cursor that streams it looks: one copy, and no lifetime dance to hand the points over.
    //
    let (t0_ms, points) = {
        let mut trace = TraceBuf::new(sync.session.trace_storage());
        {
            let mut guard = store.lock().await;
            // The log work stays out of this future: see `collect_summaries`.
            if !read_trace_from_log(&mut guard, rec_buf, rec, rep_uuid, &batch.index, &mut trace) {
                warn!(
                    "trace for rep {} is no longer in the log",
                    UuidShort(Some(&rep_uuid))
                );
                report(msg::ReportKind::TraceUnreadable);
                return;
            }
        }
        let t0_ms = trace.get(0).map(|p| p.t_ms).unwrap_or(0);
        (t0_ms, trace.len())
    };

    log_trace(&rep_uuid, points);

    // A request for one chunk sends one chunk; a request for the trace sends all of it. Which is a cursor position and a stop, not a different stream.
    let only = if request.all_chunks {
        None
    } else {
        Some(request.chunk_index)
    };
    sync.session.start_trace(rep_uuid, t0_ms, points, only);

    // The first transfer goes now; the rest go as the app acknowledges them (see `keep_sending`).
    sync.send_trace_chunk(conn).await;
}

/// Read one rep's trace out of the log into `trace`, returning whether it was there at all (a rep the ring has overwritten reads back empty).
///
/// `index` is where the summary stream saw the rep's records: with it the read is a walk of those records, and
/// without it the rep is found by walking the log (which holds the sampler off its tick for seconds, so it is the
/// answer for a rep this connection has not seen rather than the usual one).
///
/// Sync, and separate from its caller for the same reason as [`collect_summaries`].
fn read_trace_from_log(
    store: &mut FlashStore,
    rec_buf: &mut [u8],
    rec: &mut msg::RepRecord,
    rep_uuid: RepUuid,
    index: &BatchIndex,
    trace: &mut TraceBuf<'_>,
) -> bool {
    let (flash, pt_mem) = store.split();
    let Ok(pt) = partitions::read_partition_table(flash, pt_mem) else {
        return false;
    };
    let Ok(Some(partition)) = pt.find_partition(partitions::PartitionType::Data(
        partitions::DataPartitionSubType::Undefined,
    )) else {
        return false;
    };
    let mut region = partition.as_embedded_storage(flash);
    let Ok(mut log) = FlashLog::mount(&mut region) else {
        return false;
    };
    let read = match index.find(&rep_uuid) {
        Some(place) => store::read_trace_from(
            &mut log,
            rec_buf,
            rec,
            place.first_chunk_id,
            place.computed_chunks,
            rep_uuid,
            trace,
        ),
        None => store::read_trace(&mut log, rec_buf, rec, rep_uuid, trace),
    };
    read.map(|n| n > 0).unwrap_or(false)
}

/// Read a rep's **raw** trace out of the log and start streaming it, the same way the plotted one is.
///
/// The only difference is where the samples are while they are sent: a full-rate window is the one trace too big to
/// hold from boot, so its buffer is allocated on the first request (`ensure_raw_storage`).
///
/// A function of its own rather than an arm of [`stream_trace`] because two awaited futures in one frame is one future
/// holding both, and this crate lints the frame size.
async fn stream_raw_trace(
    store: &'static AsyncMutex<NoopRawMutex, FlashStore>,
    rec_buf: &mut [u8],
    rec: &mut msg::RepRecord,
    batch: &SummaryBatch,
    sync: &mut Sync<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    request: &msg::TraceRequest,
) {
    let Some(rep_uuid) = calypso_core::pb::uuid_from_field(&request.rep_uuid) else {
        return;
    };
    ensure_raw_storage(sync);
    let (t0_ms, samples) = {
        let storage = sync.session.raw_storage();
        if storage.is_empty() {
            refuse_raw_trace(&rep_uuid, "this device cannot hold one");
            report(msg::ReportKind::RequestRefused);
            return;
        }
        let mut guard = store.lock().await;
        // The log work stays out of this future: see `collect_summaries`.
        let read = read_raw_from_log(&mut guard, rec_buf, rec, rep_uuid, &batch.index, storage);
        if read == 0 {
            refuse_raw_trace(&rep_uuid, "not in the log");
            report(msg::ReportKind::TraceUnreadable);
            return;
        }
        // The window's first sample, which is the origin both traces count from.
        (storage[0].t_ms, read)
    };

    log_raw_trace(&rep_uuid, samples);
    let only = if request.all_chunks {
        None
    } else {
        Some(request.chunk_index)
    };
    sync.session.start_raw_trace(rep_uuid, t0_ms, samples, only);

    // The first transfer goes now; the rest go as the app acknowledges them (see `keep_sending`).
    sync.send_trace_chunk(conn).await;
}

/// Read one rep's raw trace out of the log into `samples`, returning how many were recovered — 0 when the rep's raw
/// chunks were never recorded or the ring has overwritten them.
///
/// In its own frame, and never inlined: the partition table it reads into is the largest local in this file.
#[inline(never)]
fn read_raw_from_log(
    store: &mut FlashStore,
    rec_buf: &mut [u8],
    rec: &mut msg::RepRecord,
    rep_uuid: RepUuid,
    index: &BatchIndex,
    samples: &mut [Sample],
) -> usize {
    let (flash, pt_mem) = store.split();
    let Ok(pt) = partitions::read_partition_table(flash, pt_mem) else {
        return 0;
    };
    let Ok(Some(partition)) = pt.find_partition(partitions::PartitionType::Data(
        partitions::DataPartitionSubType::Undefined,
    )) else {
        return 0;
    };
    let mut region = partition.as_embedded_storage(flash);
    let Ok(mut log) = FlashLog::mount(&mut region) else {
        return 0;
    };
    let read = match index.find(&rep_uuid) {
        Some(place) => store::read_raw_from(
            &mut log,
            rec_buf,
            rec,
            place.first_raw_chunk_id(),
            place.raw_chunks,
            rep_uuid,
            samples,
        ),
        None => store::read_raw(&mut log, rec_buf, rec, rep_uuid, samples),
    };
    read.unwrap_or(0)
}

/// Give the session the buffer a raw trace is streamed from, if it has none yet.
///
/// A window of full-rate samples is the largest thing a session can be asked to hold, and only a client in dev mode
/// asks for it, so it is allocated on the first request rather than at boot. The heap check is the radio's: the two
/// share one heap, and a device left too short of it stops accepting connections.
#[inline(never)]
fn ensure_raw_storage(sync: &mut Sync<'_>) {
    if !sync.session.raw_storage().is_empty() {
        return;
    }
    let bytes = REP_WINDOW_SAMPLES * core::mem::size_of::<Sample>();
    let free = esp_alloc::HEAP.free();
    if free < bytes + RAW_TRACE_HEADROOM_BYTES {
        warn!(
            "raw trace storage: {} B wanted, {} B free — refusing",
            bytes, free
        );
        return;
    }
    let mut buf = AllocVec::new();
    buf.resize(REP_WINDOW_SAMPLES, Sample::ZERO);
    sync.session
        .set_raw_storage(Box::leak(buf.into_boxed_slice()));
    info!(
        "raw trace storage: {} B allocated, heap free {} B",
        bytes,
        esp_alloc::HEAP.free()
    );
}

/// A rep UUID for logs: its short form, or a placeholder when absent.
///
/// Formatted through `defmt` directly rather than to a `&str`, so logging a rep costs no allocation.
struct UuidShort<'a>(Option<&'a RepUuid>);

impl defmt::Format for UuidShort<'_> {
    fn format(&self, f: defmt::Formatter) {
        match self.0 {
            Some(uuid) => defmt::write!(f, "{=[u8]:02x}", &uuid::short(uuid)[..]),
            None => defmt::write!(f, "????????"),
        }
    }
}

/// Which trace a request is for, for a log line. The same question the stream is chosen by: anything that is not raw
/// is served as the plotted trace.
fn trace_kind(kind: msg::TraceKind) -> &'static str {
    if kind == msg::TraceKind::Raw {
        "raw"
    } else {
        "plotted"
    }
}

/// Run one flash erase or write with both ends of it on the log, and how long it took.
///
/// Flash and the radio share the chip, and an erase or write parks the other core and runs with the cache off: a window the controller can miss a connection event in, and a dropped link is that window plus the supervision timeout it takes to give up. Both ends on the clock is what makes the two readable against each other.
fn flash_window<T>(what: &'static str, op: impl FnOnce() -> T) -> T {
    let started = Instant::now();
    info!("flash {}: starting", what);
    let out = op();
    info!(
        "flash {}: done after {} ms",
        what,
        started.elapsed().as_millis() as u32
    );
    out
}

/// The client set the clock: keep it in RAM if it is a time at all, and write it into the log so the reps this
/// boot has already stored are dated by it too (`calypso_core::clock`).
///
/// A time the device will not believe is refused rather than stored: dating real reps with a plausible lie is the
/// one outcome nothing downstream can detect.
async fn set_clock(
    store: &'static AsyncMutex<NoopRawMutex, FlashStore>,
    rec_buf: &mut [u8],
    now_s: u32,
    uptime_ms: u32,
) {
    let Some(epoch_s) = clock::epoch_from(now_s, uptime_ms) else {
        warn!(
            "clock: {} s is not a time this device will believe, ignored (uptime {} ms)",
            now_s, uptime_ms
        );
        return;
    };
    CLOCK_EPOCH_S.store(epoch_s, Ordering::Relaxed);
    // Both outcomes change what the summary stream would read: a written record, or the RAM epoch that dates this
    // boot's summaries without one.
    log_changed();

    let written = {
        let mut guard = store.lock().await;
        write_clock_record(&mut guard, rec_buf, epoch_s)
    };
    match written {
        Ok(id) => info!(
            "clock: epoch {} s (uptime {} ms), log record {}",
            epoch_s, uptime_ms, id
        ),
        // The RAM epoch still dates this boot; the record is what dates it after a restart.
        Err(e) => {
            warn!("clock: epoch {} s kept in RAM, not written: {}", epoch_s, e);
            report(msg::ReportKind::ClockNotPersisted);
        }
    }
}

/// Write a clock record into the log. Sync, for the same reason as [`collect_summaries`].
fn write_clock_record(
    store: &mut FlashStore,
    rec_buf: &mut [u8],
    epoch_s: u32,
) -> Result<u32, LogError> {
    let (flash, pt_mem) = store.split();
    let pt = partitions::read_partition_table(flash, pt_mem).map_err(LogError::Table)?;
    let Some(log_partition) = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Undefined,
        ))
        .map_err(LogError::Table)?
    else {
        return Err(LogError::NoPartition);
    };
    let mut region = log_partition.as_embedded_storage(flash);
    let mut log = FlashLog::mount(&mut region).map_err(map_log_error)?;
    store::append_clock(&mut log, rec_buf, epoch_s).map_err(|e| match e {
        store::Error::Log(e) => map_log_error(e),
        store::Error::Encode(e) => LogError::Encode(e),
    })
}

/// Append one rep (summary, plotted trace chunks, and the raw ones when a dev session asked for them) to the flash log partition. Returns the number of records written.
/// The log is mounted per append — a scan of the sector headers, and appends are rare (once per rep, at rest) — so nothing keeps it alive between reps.
/// An append therefore fails for the same reasons the boot-time mount does, and a rep is lost when this returns an error.
fn store_rep(
    store: &mut FlashStore,
    rec_buf: &mut [u8],
    summary: &mut msg::RepSummary,
    trace: &TraceBuf<'_>,
    raw: Option<&[Sample]>,
) -> Result<u32, LogError> {
    let (flash, pt_mem) = store.split();
    let pt = partitions::read_partition_table(flash, pt_mem).map_err(LogError::Table)?;
    let Some(log_partition) = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Undefined,
        ))
        .map_err(LogError::Table)?
    else {
        return Err(LogError::NoPartition);
    };
    let offset = log_partition.offset();
    // Scoped so the region's borrow of `flash` ends before the probe below.
    let appended = {
        let mut region = log_partition.as_embedded_storage(flash);
        FlashLog::mount(&mut region)
            .map_err(store::Error::Log)
            .and_then(|mut log| {
                store::append_rep(&mut log, rec_buf, summary, trace.as_slice(), raw)
            })
    };
    match appended {
        Ok(records) => Ok(records),
        Err(e) => {
            if let store::Error::Log(e) = &e {
                probe_log_partition(flash, offset, failing_sector(e));
            }
            Err(match e {
                store::Error::Log(e) => map_log_error(e),
                store::Error::Encode(e) => LogError::Encode(e),
            })
        }
    }
}

/// Whether the peer has this characteristic's notifications enabled.
///
/// **This is the only way to know, and it is why the logs and the summary cursor are built on it rather than on a notification's result.**
/// `Characteristic::notify` returns `Ok(())` whether or not anybody is subscribed:
/// trouble-host checks its per-connection CCCD table (`should_notify`) and returns early *without sending* when the peer is not in it. So "the notification succeeded" is not evidence of anything, and code that treated it as evidence got it wrong twice here — a line printed once a second to a device that was sending nothing, and a summary cursor that walked the whole log into the void while nobody was listening.
fn is_subscribed(
    server: &CalypsoGatt<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    cccd_handle: Option<u16>,
) -> bool {
    let Some(cccd_handle) = cccd_handle else {
        return false;
    };
    server
        .get_cccd_table(conn.raw())
        .and_then(|table| {
            table
                .inner()
                .iter()
                .find(|(handle, _)| *handle == cccd_handle)
                .map(|(_, cccd)| cccd.should_notify())
        })
        .unwrap_or(false)
}

/// Note a fault for the app. One line beside the `warn!` that says the same thing to a console.
///
/// Cheap and infallible on purpose: it is called from the 100 Hz sampling path and from the arms that are already handling a failure, so it may not block, allocate or fail. Nothing leaves the device from here — [`push_report`] does that, from the two places something might have freed the link.
///
/// A fault raised while no client is subscribed stays latched: that is how a log that would not mount reaches the phone of the person who turns up afterwards.
fn report(kind: msg::ReportKind) {
    critical_section::with(|cs| REPORTS.borrow(cs).borrow_mut().raise(kind));
}

/// Send the fault the app is owed, if there is one and it is listening.
///
/// Called beside `keep_sending`, which is the funnel for "something may have freed the link": a fault raised while a transfer was in flight goes out when that transfer moves. The subscription is asked of the server rather than taken from a notify result, for the reason [`is_subscribed`] exists — a notification to nobody reports success.
///
/// Nothing is retried: `take_due` marks the kind sent as it hands it over (`calypso_core::report`), because a fault the link would not carry now is not one the next attempt says better.
async fn push_report(
    gatt: &CalypsoGatt<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    now_ms: u32,
) {
    if !is_subscribed(gatt, conn, gatt.sync.reports.cccd_handle) {
        return;
    }
    let Some(kind) = critical_section::with(|cs| REPORTS.borrow(cs).borrow_mut().take_due(now_ms))
    else {
        return;
    };
    // A `Report` is one enum field in a characteristic that holds 20 bytes, which is asserted
    // where the schema is (`calypso_core::report`), so this is not a case that can happen.
    let Some(value) = SyncReportsValue::encode(&msg::Report { kind }) else {
        return;
    };
    if gatt.sync.reports.notify(conn, &value).await.is_err() {
        warn!("reports: notify failed on the link");
    }
}

/// Put the last sample the sampler read into the table, which is what the app's read of the `Imu` characteristic returns.
///
/// Called on connection and every 100 ms after, so a client polling the read sees the sensor moving rather than a rep's stroke in one frame.
fn publish_imu(gatt: &CalypsoGatt<'_>) {
    let Some((sample, attitude)) = critical_section::with(|cs| *LAST_SAMPLE.borrow(cs).borrow())
    else {
        return;
    };
    match SyncImuValue::encode(&calypso_core::pb::imu_state_message(&sample, attitude)) {
        Some(value) => {
            if gatt.set(&gatt.sync.imu, &value).is_err() {
                warn!("imu: the table refused the sample");
            }
        }
        None => warn!("imu: the sample is larger than the characteristic"),
    }
}

/// Send everything the app is owed, in the order it is waiting for it.
///
/// A call rather than a loop that runs to completion: the pipe carries one *message* at a time, and a message is finished by an acknowledgement that arrives as a write.
/// So this is called whenever something might have freed the pipe, and does as little as it can get away with.
///
/// The order is the app's: a trace it asked for (and is timing out on), then the summaries, which are the bulk and can wait.
///
/// The exemption is for the composition: two awaited futures in one function is one future holding both, and the lint adds them up.
#[allow(
    clippy::large_stack_frames,
    reason = "the senders' futures, composed because the pipe carries one message at a time"
)]
async fn keep_sending(
    store: &'static AsyncMutex<NoopRawMutex, FlashStore>,
    rec_buf: &mut [u8],
    rec: &mut msg::RepRecord,
    batch: &mut SummaryBatch,
    sync: &mut Sync<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    cursor: &mut SummaryCursor,
) {
    // **A message can be wider than the window.** The window is 8 chunks and a trace chunk is up to 10 frames, so a transfer is usually sent in waves — and the rest of the wave goes out here, when the acknowledgement that made room arrives.
    // Returning early on a busy pipe leaves the message half-sent for ever: nothing else calls `flush`, and the app waits for a chunk that is sitting in the device's buffer (docs/GOTCHAS.md).
    if !sync.session.is_idle() {
        sync.flush(conn).await;
        return;
    }
    // Otherwise: a trace chunk, if one is being streamed, and only then summaries — the app is waiting on its trace.
    if sync.session.streaming() && sync.send_trace_chunk(conn).await {
        return;
    }
    stream_new_summaries(store, rec_buf, rec, batch, sync, conn, cursor).await;
}

/// A decoded-record scratch on the heap.
///
/// In its own function, and never inlined, so the ~2 KB temporary that `Box::new` needs to build the value lives in *this* frame — which is gone by the time it returns — rather than in `main`'s, which is the bottom of the stack the whole BLE stack also runs on.
///
/// `Box::new` does build its argument on the stack first (2152 bytes, per clippy), and there is no way to allocate a `RepRecord` without one short of writing its fields through a `MaybeUninit` by hand.
/// That would trade a bounded, known transient for `unsafe` in the decode path, so the transient stays, in a leaf called before the tasks start.
#[allow(
    clippy::large_stack_frames,
    reason = "2152 B: the `Box::new` temporary, isolated so it is not in main's frame"
)]
#[inline(never)]
fn leak_record_scratch() -> &'static mut msg::RepRecord {
    Box::leak(Box::new(msg::RepRecord::default()))
}

/// The pipe's session on the heap, for the same reason as the record scratch: the buffers it holds are handed over rather than built here, but the struct itself is not small, and `main`'s frame is the bottom of the stack the BLE stack runs on.
#[inline(never)]
fn leak_session(
    outgoing: &'static mut [u8],
    incoming: &'static mut [u8],
    points: &'static mut [TracePoint],
) -> &'static mut Session<'static> {
    Box::leak(Box::new(Session::new(outgoing, incoming, points)))
}

/// What a trace request turned into, in a frame of its own: the defmt formatting state is a local, and `stream_trace`'s frame is on the stack the BLE stack also runs on (`clippy::large_stack_frames`, which is denied here).
#[inline(never)]
fn log_trace(rep_uuid: &RepUuid, points: usize) {
    info!(
        "trace {}: {} points, {} chunks",
        UuidShort(Some(rep_uuid)),
        points,
        calypso_core::pb::chunk_count_for(points),
    );
}

/// A raw trace this end will not send, in a frame of its own for the same reason.
#[inline(never)]
fn refuse_raw_trace(rep_uuid: &RepUuid, reason: &'static str) {
    warn!(
        "raw trace for rep {}: {=str}",
        UuidShort(Some(rep_uuid)),
        reason
    );
}

/// The same for the raw trace, in a frame of its own for the same reason.
#[inline(never)]
fn log_raw_trace(rep_uuid: &RepUuid, samples: usize) {
    info!(
        "raw trace {}: {} samples, {} chunks",
        UuidShort(Some(rep_uuid)),
        samples,
        calypso_core::pb::chunk_count_for(samples),
    );
}

/// Milliseconds since `start`, which is what the pipe's timeouts are measured in.
///
/// Any monotonic reference will do — a transfer's stall timeout is a difference — so this is the connection's own start rather than the device's uptime.
fn now_ms(start: Instant) -> u32 {
    start.elapsed().as_millis() as u32
}

// The linker stack: `Reset` sets the stack pointer to `_stack_start` and the stack grows down from there, so bytes in use are measured from the top and free bytes from `_stack_end` — which is wherever `.data` and `.bss` ended, making the stack what the RWDATA region has left over (see docs/GOTCHAS.md).
unsafe extern "C" {
    static _stack_start: u8;
    static _stack_end: u8;
}

/// What [`paint_stack`] fills the unused stack with.
const STACK_PAINT: u8 = 0xA5;

/// How much of the stack's bottom is neither painted nor scanned: the `stack-protector` guard sits 60 bytes above `_stack_end`, and a byte next to it says nothing about how deep the stack has been.
const STACK_PAINT_FLOOR: usize = 256;

/// The stack pointer, i.e. the lowest address this frame's own storage occupies.
/// Inlined, so `a1` is the caller's: its `entry` has already allocated the frame.
#[inline(always)]
fn stack_pointer() -> usize {
    let sp: usize;
    unsafe { core::arch::asm!("mov {sp}, a1", sp = out(reg) sp) };
    sp
}

fn stack_top() -> usize {
    core::ptr::addr_of!(_stack_start) as usize
}

fn stack_bottom() -> usize {
    core::ptr::addr_of!(_stack_end) as usize
}

/// Bytes of the linker stack in use at this instant, measured from its top.
fn stack_used() -> usize {
    stack_top().saturating_sub(stack_pointer())
}

/// Bytes of the linker stack this boot has been through at its deepest, read back from [`STACK_PAINT`]: the stack region is a buffer nobody zeroes, and the lowest pattern byte still standing is how far down `main`'s frame and the poll chains under it have reached.
///
/// The figure is at least the depth at the moment of painting, and it cannot see the floor below `STACK_PAINT_FLOOR` — a stack that reached in there would have written the guard and faulted first.
fn stack_deepest() -> usize {
    let (low, high) = paint_range();
    let mut bottom = low;
    while bottom < high && unsafe { core::ptr::read_volatile(bottom as *const u8) } == STACK_PAINT {
        bottom += 1;
    }
    stack_top().saturating_sub(bottom)
}

/// Fill everything below the stack pointer with [`STACK_PAINT`].
///
/// Below the stack pointer is free by definition, and this frame's own storage is above it. Called before `esp_rtos::start`: a task switch happens inside an interrupt, and the interrupted context's frame would outlive the paint that wrote over it.
fn paint_stack() {
    let (low, high) = paint_range();
    if high <= low {
        warn!(
            "stack unpainted: less than {} B below the stack pointer",
            STACK_PAINT_FLOOR
        );
        return;
    }
    unsafe { core::ptr::write_bytes(low as *mut u8, STACK_PAINT, high - low) };
}

fn paint_range() -> (usize, usize) {
    (stack_bottom() + STACK_PAINT_FLOOR, stack_pointer())
}

/// The stack figures a log line carries: in use now, the total, and the deepest since [`paint_stack`].
struct Stack;

impl defmt::Format for Stack {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "{} B used of {} B, {} B at the deepest",
            stack_used(),
            stack_top() - stack_bottom(),
            stack_deepest()
        )
    }
}

/// The device's X-axis gravity as the filter sees it, and whether that reads as level.
///
/// A non-fusion mode has no gravity vector to read, so this is the world's up axis carried back into the sensor's frame
/// by the attitude the filter estimated — the same quantity the fusion used to report, from an estimate that is
/// corrected only while the bar is still. Logged by the caller, and only when the verdict changes: this used to print
/// `Sensor is not level` every second and bury everything else.
fn level_sense(attitude: [f32; 4]) -> (f32, bool) {
    let up = rotate_by_quat(conjugate(attitude), [0.0, 0.0, 1.0]);
    let x_grav = up[0] * GRAVITY;
    (x_grav, calypso_core::is_level_along_axis(x_grav))
}
