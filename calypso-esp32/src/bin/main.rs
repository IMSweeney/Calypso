#![no_std]
#![no_main]
// `a1` is the stack pointer and `core::arch::asm!` needs this to reach it (the
// `esp` toolchain is nightly; `esp-hal` enables the same feature).
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
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex as AsyncMutex;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use embedded_storage::nor_flash::NorFlash;
use embedded_storage::{ReadStorage, Storage};
use esp_bootloader_esp_idf::partitions;
use esp_hal::Blocking;
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
use bno055::{BNO055_CALIB_SIZE, BNO055Calibration, BNO055OperationMode};
use calypso_core::calibration::Calibration;
use calypso_core::flashlog::{self, FlashLog};
use calypso_core::gatt::{CalypsoGatt, PIPE_STORAGE_MAX, SCHEMA_HASH, SyncCalibrationValue};
use calypso_core::pb::msg;
use calypso_core::pipe::{self, Session, Stalled};
use calypso_core::rep::{Detector, MARGIN_MS, RepSpan, TraceBuf, TracePoint, compute_rep};
use calypso_core::sampler::{QUAT_SCALE, SAMPLE_RATE_HZ, Sample, SampleRing, quantize};
use calypso_core::store::{self, RECORD_BUF_BYTES};
use calypso_core::sync::{SummaryCursor, trace_request};
use calypso_core::uuid::{self, RepUuid};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // Printed, not silent. A panic here used to just stop the log, which reads
    // exactly like a hang and cost a session to diagnose: the firmware ran out of
    // heap building its sync buffers and said nothing at all. `esp_println` works
    // from a panic handler (defmt's formatter does not).
    esp_println::println!("\n*** PANIC: {} ***", info);
    loop {
        core::hint::spin_loop();
    }
}

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
// Signal channel + ATT channel
const L2CAP_CHANNELS_MAX: usize = 2;

const BNO055_CALIBRATION_ADDR: u32 = 0x00;

/// The BLE device name. A compile-time constant, and read-only over GATT: the app
/// used to be able to rename the device, which meant a name blob in NVS, a
/// writable characteristic and a second thing to persist. One sensor, one name.
const DEVICE_NAME: &str = "Calypso";

/// The IMU's calibration levels (`Calibration::to_bits`), written by the 1 Hz
/// poll in the sampler task and read by the BLE task to serve the app's read of
/// the `Calibration` characteristic. Zero (every level 0) until the first poll,
/// which reads as "not calibrated" — the safe thing to believe.
static CALIBRATION_BITS: AtomicU32 = AtomicU32::new(0);

/// Whether the calibration profile has been written to NVS this boot. Set once,
/// on the transition to calibrated, so the save (a flash erase and write) does
/// not repeat on every poll.
static CALIBRATION_SAVED: AtomicBool = AtomicBool::new(false);

/// Bench mode: store a synthetic "rep" every this many seconds instead of detecting
/// real ones (`--features fake-reps`).
///
/// A build flag rather than a runtime setting on purpose: the device should not be one
/// accidental setting away from recording fiction as a measurement, and a bench build
/// says so in its own log lines (`fake-reps: …`).
const FAKE_REPS: bool = cfg!(feature = "fake-reps");

/// Bench mode's period, and how much of the ring each synthetic rep covers. The window
/// has to leave room for the ZUPT margins inside `REP_WINDOW_SAMPLES` (9 s).
const FAKE_REP_PERIOD_S: u32 = 10;
const FAKE_REP_MS: u32 = 8_000;

/// Ring-buffer window: how much history the detector can be asked for, plus
/// headroom.
///
/// The only thing that reads it is [`rep_window`](REP_WINDOW_SAMPLES): when the
/// detector closes a rep, the sampler copies `[start - margin, end + margin]` out
/// of the ring. The longest rep the detector can close is 8 s and the margins are
/// 200 ms each side, so 9 s is the most that is ever needed and 12 s leaves a
/// third of the ring spare.
///
/// It used to be 30 s — 60 KB of a ~164 KB heap that the radio also lives in —
/// and that was the difference between a device that connects and one that runs
/// out of memory building its own sync buffers (`heap free 8968 B` before it
/// needed ~23 KB more). Do not grow this without checking the heap figures in the
/// boot log; `Radio up, heap free N B` and `Sync buffers ready, heap free N B`
/// are there to be read.
const SAMPLE_RING_LEN: usize = REP_WINDOW_SAMPLES + 3 * SAMPLE_RATE_HZ as usize;

/// The sampler's sample window. Lives on the heap (leaked at startup):
/// [`SAMPLE_RING_LEN`] samples is too much for the DRAM static region, and the
/// heap is where every big buffer here lives. The sampler task both fills it and
/// runs the rep detector over it.
type SampleWindow = RefCell<SampleRing<'static>>;

/// The flash peripheral, and the scratch a partition lookup needs.
///
/// The partition table is 3 KB, and reading one into a local is by far the
/// largest stack frame this firmware would have — the crate denies
/// `clippy::large_stack_frames` for exactly that reason. Keeping the scratch
/// here, alongside the peripheral it describes, means it is allocated once with
/// the other big buffers instead of in every function that touches flash, and it
/// needs no lock of its own: every use is already inside the mutex that owns
/// this.
///
/// It is behind a pointer rather than an array in the struct: an array would put
/// those 3 KB in the frame of whoever *builds* the `FlashStore` — `main`, which
/// shares the linker stack (`_stack_end`, ~42 KB in the current build) with the
/// BLE stack.
struct FlashStore {
    flash: FlashDevice<'static>,
    pt_mem: &'static mut [u8],
}

impl FlashStore {
    /// The peripheral and the partition-table scratch, borrowed apart.
    ///
    /// One accessor instead of destructuring the struct at each site: with the
    /// scratch behind a pointer, `let FlashStore { flash, pt_mem } = store` would
    /// hand out `&mut &mut [u8]` and every caller would have to deref it.
    fn split(&mut self) -> (&mut FlashDevice<'static>, &mut [u8]) {
        (&mut self.flash, &mut self.pt_mem[..])
    }
}

/// Summaries collected per pass of the summary stream. Each is ~76 bytes, so the
/// batch is a heap buffer rather than a local — see [`collect_summaries`].
const SUMMARY_BATCH: usize = 16;

/// Window handed to the detector when a rep closes: `MAX_REP_MS` plus the
/// ZUPT margins (9 s at 100 Hz).
const REP_WINDOW_SAMPLES: usize = 900;
/// Trace points kept per rep; longer reps are decimated uniformly.
const TRACE_POINTS: usize = 512;

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

    // Before `esp_rtos::start` below, and before anything else that allocates
    // stack: the deepest use is what the log lines report.
    paint_stack();

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);
    // A second region, because the firmware's own buffers are ~82 KB before the
    // radio has allocated anything (sample ring 24 KB, rep window 18 KB, two trace
    // buffers 16 KB each, plus scratch), and the radio's task stacks come from
    // here too. Both regions are statics, so this one is also 64 KB the linker
    // stack below does not get — see the heap note in docs/GOTCHAS.md before
    // changing the split.
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    // **No Wi-Fi controller.** This is a BLE-only device, and initialising Wi-Fi
    // starts a Wi-Fi task with its own stack and driver buffers — most of the
    // ~54 KB the radio took out of a 164 KB heap, which is what the firmware ran
    // out of. It came from the esp-generate template, which wires up both radios.
    //
    // It is safe to leave out here, and this is the evidence rather than a
    // reading of the docs: a build without this call ran, printed `advertising`
    // and kept logging, i.e. the BLE controller and host came up fine. esp-radio
    // *would* need it if its `coex` feature were enabled — the BLE bring-up has
    // `#[cfg(coex)] assert!(coex_init() == 0, "coex_init failed")`, and
    // `coex_init` needs the adapter functions the Wi-Fi controller registers.
    // `coex` is an opt-in esp-radio feature that nothing enables (checked: the
    // panic string is not in the binary), so the assert is not compiled in.
    //
    // **If BLE ever stops coming up, put this line back first** — and if the
    // board panics with `coex_init failed`, someone has enabled `coex`.
    //
    // More examples: https://github.com/embassy-rs/trouble/tree/main/examples/esp32

    // 8 KB rather than the 4 KB default. A crash that pointed at a task stack in
    // DRAM (not the linker stack) is what put this here: the controller task has
    // the smallest default stack of anything on the board and is the only one we
    // can size from here. A fault inside it looks nothing like a bug in `main`,
    // which is why the number is written down.
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
    // pinout for esp32 pico
    // https://learn.adafruit.com/assets/112309
    // 400 kHz so the 100 Hz sampler's ~14 bytes/sample cost ~0.5 ms of the
    // 10 ms period.
    let i2c_config = esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO22)
        .with_scl(peripherals.GPIO19);
    info!("Done setting up i2c");

    let mut delay = Delay;

    info!("Setting up Bno055");
    let mut bno055 = Bno055::new(i2c).with_alternative_address();
    bno055.init(&mut delay).unwrap();
    info!("Bno055 initialized");
    // IMU mode: accel + gyro fusion, no magnetometer (unreliable near
    // barbells). Roll/pitch stay accurate; yaw drift is corrected per rep by
    // ZUPT (Phase 3).
    bno055
        .set_mode(BNO055OperationMode::IMU, &mut delay)
        .unwrap();

    // `multicore_auto_park` is not optional on this chip: esp-storage's default
    // strategy *refuses* every write and erase while the second core is running
    // (`MultiCoreStrategy::Error`), and CPU1 is up in this build. Reads are
    // unaffected, which is what makes it so confusing — the NVS reads worked
    // while every erase, including the log partition's first format, came back
    // as an I/O error.
    let flash = FlashStorage::new(peripherals.FLASH).multicore_auto_park();
    info!("Flash size = {}", flash.capacity());
    // The peripheral and its partition-table scratch, together (see FlashStore).
    // The scratch is 3 KB and goes on the heap: as an array inside the struct it
    // would sit in `main`'s frame, the bottom of the stack the BLE stack shares.
    let pt_mem: &'static mut [u8] =
        Box::leak(alloc::vec![0u8; partitions::PARTITION_TABLE_MAX_LEN].into_boxed_slice());
    let mut store = FlashStore { flash, pt_mem };

    // Apply a stored calibration profile if there is one, but never wait for the
    // IMU to calibrate: this used to block boot, which left an uncalibrated
    // device with no BLE, no app and no way to find out why. It now comes up
    // fully and says what it is waiting for (see docs/CALIBRATION.md).
    load_calibration(&mut bno055, &mut store);

    info!("BLE device name: {}", DEVICE_NAME);

    // The UUIDs this build exposes, in both orders, as hex. One line, and it answers
    // the question that cost several flashes: *which* firmware is on this board, as
    // seen from the other end of the link. A client reports a mismatch here as
    // "characteristic not found", naming a UUID that is correct.
    info!(
        "sync service uuid {=[u8]:02x} (wire), {} (as a client sees it)",
        calypso_core::gatt::uuid::SYNC_SERVICE_BYTES,
        calypso_core::gatt::uuid::SYNC_SERVICE_TEXT
    );

    // Mount the rep-record flash log (see docs/REP_TRACKING.md).
    match mount_rep_log(&mut store) {
        Ok(sectors) => info!("Rep log mounted: {} sectors", sectors),
        // The reason, not just "unusable", and the reason *named*: the first
        // question is whether the flashed table has the log partition at all, and
        // the second is which flash error it was — a refused write (`OtherCoreRunning`)
        // and a bad read are different faults with different fixes.
        Err(reason) => warn!("Rep log unusable: {}", reason),
    }

    // The sample ring, on the heap and alive for the program's lifetime (see
    // `SAMPLE_RING_LEN` for why it is this size).
    //
    // The buffer is allocated and leaked separately from the ring, and the ring
    // borrows it. A ring that owned this array would be built as a *value* before
    // `Box::new` could move it to the heap — 60 KB of stack temporary when it was
    // a 30 s window, more than twice this task's whole stack, which is how this
    // used to crash (see docs/GOTCHAS.md).
    let mut ring_vec = AllocVec::new();
    ring_vec.resize(SAMPLE_RING_LEN, Sample::ZERO);
    let ring_buf: &'static mut [Sample] = Box::leak(ring_vec.into_boxed_slice());
    let samples: &'static SampleWindow =
        Box::leak(Box::new(RefCell::new(SampleRing::new(ring_buf))));
    // Rep-detector scratch, also on the heap (too big for the static region,
    // and built with `vec!` so no large temporary hits the stack).
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

    // The one flash peripheral, shared by the sampler task (appending reps) and
    // the BLE task (reading a rep's trace back out to stream it). An async mutex,
    // not a critical section: erasing a sector takes tens of milliseconds and must
    // not block interrupts. All three futures below run in this task, so a no-op
    // raw mutex suffices.
    let store: &'static AsyncMutex<NoopRawMutex, FlashStore> =
        Box::leak(Box::new(AsyncMutex::new(store)));

    // **This address is the device's identity to every client**, so it is also the
    // key a client's cached GATT database lives under. CoreBluetooth caches a
    // peripheral's services and characteristics, and universal_ble prefers that
    // cache to asking again (`if let cachedServices = peripheral.services` — see
    // `UniversalBleAsyncServiceDiscovery`). So a client that cached the *old*
    // services keeps seeing them however many times this firmware is reflashed: the
    // cache is keyed to this address, not to the flash contents.
    //
    // Two bytes of the schema's hash ride in the address for that reason: the table is
    // generated from the schema, so a build whose table changed is a device no client
    // has cached, and the rule needs no one to remember it. It was a literal to
    // remember until it was missed twice -- `ca10`/`ca11` and `ca23` went, then the
    // pipe came -- and a client holding either cached table is what a connection then
    // fails on (docs/GOTCHAS.md).
    //
    // The principled alternative is a GATT Service Changed characteristic (0x2A05)
    // with an indication, which tells a bonded client to drop the cache itself; this
    // device has no need for one yet.
    //
    // The top two bits of the most significant byte are set, as a static random
    // address requires -- hence 0xfd, with the schema's hash in the two bytes below it.
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

    // GATT table: generated from `proto/calypso.proto` by `gatt-gen` at build time
    // (`calypso-core/src/gatt.rs`), which is where the services, the
    // characteristics, their properties and their sizes are declared now. GAP comes
    // from the config below: a device name a client can read and the sensor
    // appearance, which is what makes a scanner show what this is.
    //
    // **One table per process.** The macros that build it back each characteristic
    // with a `static StaticCell`, so this cannot be called twice — a constraint a
    // build script and the host tests live with (see tools/gatt-gen/PLAN.md).
    let gatt =
        calypso_core::gatt::CalypsoGatt::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name: DEVICE_NAME,
            appearance: &appearance::sensor::GENERIC_SENSOR,
        }))
        .expect("the device name fits the GAP name length");

    // Buffers for the sync path: a trace read back from the log, and a
    // reusable record scratch (both heap: too big for the stack/static RAM).
    //
    // The trace storage goes to the session rather than staying here: the cursor
    // that streams it lives there too (see `calypso_core::pipe`), so the points a
    // request is read into and the transfers they go out as are one thing.
    let mut trace_vec = AllocVec::new();
    trace_vec.resize(TRACE_POINTS, TracePoint::ZERO);
    let sync_trace_storage: &'static mut [TracePoint] = Box::leak(trace_vec.into_boxed_slice());
    let sync_rec_buf: &'static mut [u8] =
        Box::leak(alloc::vec![0u8; RECORD_BUF_BYTES].into_boxed_slice());
    // Reading a record back means decoding it, and a decoded `RepRecord` is ~2 KB
    // (a full trace chunk is stored inline). One buffer, reused for every record
    // of every scan, so no decode ever lands on a task's stack — these futures
    // live on the main task's stack, and a 2 KB frame inside one is 2 KB of it.
    let sync_rec_msg: &'static mut msg::RepRecord = leak_record_scratch();
    // ...and the summary batch, for the same reason: `SUMMARY_BATCH` of them is
    // ~1.2 KB, which has no business sitting in a task's future. It carries how far
    // through it the pipe has got, so it is also the queue the pipe's
    // one-message-at-a-time rule needs.
    let sync_batch: &'static mut AllocVec<msg::RepSummary> =
        Box::leak(Box::new(AllocVec::with_capacity(SUMMARY_BATCH)));
    let mut batch = SummaryBatch::new(sync_batch);
    // **One characteristic means one payload buffer.** Every notification goes out on
    // `messages` now, so every sender has to build the same `BleVec` — sized for the
    // largest message, which is a trace chunk, on a task stack that has none to
    // spare. One buffer, on the heap, and the senders encode straight into it (they
    // are awaited from the same loop, so they never overlap).
    // The value type the schema declares for the notify characteristic: it owns the
    // buffer, so a sender encodes into it rather than into a buffer of its own.
    // The pipe's two message buffers and the frame each chunk is notified from.
    // All three on the heap, and leaked: a task's future lives on the stack, and
    // 8 KB of message buffer has no business in one.
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
    // What the setup above is holding before the tasks start: the total moves
    // whenever a static does, so measure rather than assume.
    info!("Stack before the tasks: {}", Stack);
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
            loop {
                // The name is fixed, so the advertisement never changes: rebuilt
                // here only because `advertise` borrows the payload. The payloads
                // are built in `calypso-core` so that what a client sees is
                // host-tested — see `build_advertising` for the layout and why it
                // is that way.
                let advertising = calypso_core::ble::build_advertising(DEVICE_NAME);
                if advertising.adv_len == 0 {
                    // An empty advertisement is invisible to every client, so it is
                    // worth saying out loud rather than debugging on the phone.
                    warn!("no advertising data to send");
                }

                let advertiser = match peripheral
                    .advertise(
                        &Default::default(),
                        Advertisement::ConnectableScannableUndirected {
                            adv_data: &advertising.adv_data[..advertising.adv_len],
                            scan_data: &advertising.scan_data[..advertising.scan_len],
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
                    Ok(conn) => match conn.with_attribute_server(&gatt) {
                        Ok(conn) => conn,
                        Err(_) => continue,
                    },
                    Err(_) => continue,
                };
                info!("BLE connection established (stack {})", Stack);
                // What a message is cut into, on this link: the MTU the peer's stack
                // reports, and the frame the framing will use given it. `23` is the
                // spec default and what a central that never sends the exchange
                // leaves behind — it is not the size of the notifications that follow
                // (docs/GOTCHAS.md, "A window of 127 is not flow control").
                info!(
                    "ATT MTU {}: notifications up to {} bytes",
                    conn.raw().att_mtu(),
                    pipe::frame_ceiling(conn.raw().att_mtu() as usize)
                );
                // A read returns the last value published, so it is published before
                // a client can ask for it.
                publish_calibration(&gatt);

                // Sync state: how many summaries this subscriber has been sent,
                // the batch they are read from, and the clock the pipe's timeouts
                // are measured against.
                //
                // A fresh session per connection: whatever was in flight is gone
                // with the link, and half a message is not a message.
                let mut cursor = SummaryCursor::new();
                let mut subscribed = false;
                let start = Instant::now();
                batch.reset();
                sync.session.reset();

                loop {
                    // Service GATT events; once a second, keep the table's copy of
                    // the IMU's calibration levels current, which is what the app
                    // reads to turn its hint into progress.
                    let event = match select(conn.next(), Timer::after(Duration::from_secs(1))).await {
                        Either::First(event) => event,
                        Either::Second(()) => {
                            let now = now_ms(start);
                            publish_calibration(&gatt);
                            // Who is listening, asked of the server rather than
                            // inferred from a notify result, because a notify result
                            // says nothing (see `is_subscribed`). Once a tick is
                            // often enough to say when it changes.
                            let listening =
                                is_subscribed(&gatt, &conn, gatt.sync.messages.cccd_handle);
                            if listening != subscribed {
                                subscribed = listening;
                                if subscribed {
                                    info!("app subscribed to sync notifications (summaries, traces)");
                                } else {
                                    info!("app unsubscribed from sync notifications");
                                }
                                // An emptied batch, so a resubscribing app starts
                                // from the cursor rather than from the middle of a
                                // batch the last one left behind.
                                batch.reset();
                            }
                            // A transfer that has stopped moving is this end's to
                            // notice: `tick` sends it again, and gives up on it
                            // after three attempts.
                            match sync.session.tick(now) {
                                Stalled::No => {}
                                Stalled::Retrying { attempt } => info!(
                                    "sync {}: no answer, sending it again (attempt {})",
                                    sync.session.kind().name(),
                                    attempt
                                ),
                                Stalled::Abandoned { attempt } => warn!(
                                    "sync {}: unanswered after {} attempts, dropped",
                                    sync.session.kind().name(),
                                    attempt
                                ),
                            }
                            if subscribed {
                                // Nothing goes out to nobody: a transfer nobody
                                // acknowledges stalls, retries and is abandoned, and
                                // it would hold the pipe for the whole 15 s.
                                // Auto-sync, and the trace the app is waiting on: the
                                // cursor advances one per message the pipe took, and a
                                // transfer to nobody is abandoned, so streaming with
                                // no subscriber would walk the whole log into the void
                                // and the app would never see it on that connection.
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
                            continue;
                        }
                    };
                    match event {
                        GattConnectionEvent::Disconnected { .. } => {
                            info!("BLE disconnected");
                            break;
                        }
                        GattConnectionEvent::Gatt { event } => {
                            // A write is a chunk of a request, or the app's
                            // acknowledgement of a message this end sent — the frame
                            // says which, and the session is the only place either is
                            // believed. Read it *after* the event is accepted: until
                            // the server has stored the value, the table still holds
                            // the one before it.
                            let is_write = matches!(event, GattEvent::Write(_));
                            if let Ok(reply) = event.accept() {
                                let _ = reply.send().await;
                            }
                            if is_write {
                                let wrote = {
                                    let Ok(value) = gatt.get(&gatt.sync.requests) else {
                                        warn!("sync: a write with no stored value");
                                        continue;
                                    };
                                    sync.session.written(value.as_slice(), now_ms(start))
                                };

                                // The app is owed an acknowledgement, and its sender
                                // is waiting on it: notify it before anything else
                                // this write set off.
                                if wrote.reply {
                                    sync.flush(&conn).await;
                                }

                                // The app asked for something. `Request` used to
                                // decode this out of the table; with a pipe the
                                // bytes are reassembled here first.
                                if wrote.message {
                                    let Some(request) = sync.session.request() else {
                                        warn!("sync: a reassembled request that does not decode");
                                        continue;
                                    };
                                    let Some(request) = trace_request(&request) else {
                                        warn!("sync: a request this device will not serve");
                                        continue;
                                    };
                                    let wanted =
                                        calypso_core::pb::uuid_from_field(&request.rep_uuid);
                                    info!("trace request: rep {}", UuidShort(wanted.as_ref()));
                                    if is_subscribed(&gatt, &conn, gatt.sync.messages.cccd_handle) {
                                        stream_trace(
                                            store,
                                            sync_rec_buf,
                                            sync_rec_msg,
                                            &mut sync,
                                            &conn,
                                            request,
                                        )
                                        .await;
                                    } else {
                                        // A transfer nobody acknowledges would stall
                                        // and be abandoned, and the app would wait
                                        // for a trace that is not coming.
                                        warn!("trace request ignored: notifications are not subscribed");
                                    }
                                }

                                // Then send whatever that write freed — an
                                // acknowledgement of this end's own transfer, most
                                // often — with a trace chunk ahead of everything
                                // else, since the app is waiting on its trace.
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
                            }
                        }
                        _ => {}
                    }
                }
            }
        },
        // Sampler loop: 100 Hz IMU samples into the RAM ring buffer.
        async {
            let start = Instant::now();
            // Rep identities come from the hardware RNG: the device has no RTC and
            // its uptime resets on every boot, so a counter or a timestamp could
            // not be unique across boots.
            let rng = Rng::new();
            let mut ticker = Ticker::every(Duration::from_hz(SAMPLE_RATE_HZ as u64));
            // The detector runs inline at 100 Hz: it's a few comparisons per
            // sample, and a closed rep does one trace computation.
            let mut detector = Detector::new();
            let mut reps: u32 = 0;
            let mut n: u32 = 0;
            // Said once, in bench mode, so a session cannot be mistaken for a real one.
            let mut logged_fake_reps = false;
            // The IMU's calibration levels, refreshed once a second — and logged
            // only when they change, so "paused until the IMU is calibrated" is
            // printed when it becomes true rather than 3,600 times an hour.
            let mut calibration = Calibration::default();
            let mut logged_calibration: Option<Calibration> = None;
            let mut logged_level: Option<bool> = None;
            // For the profile read-back, which the bno055 crate needs a delay for
            // (it briefly switches the chip to CONFIG mode).
            let mut delay = Delay;
            loop {
                ticker.next().await;
                let t_ms = start.elapsed().as_millis() as u32;

                // 1 Hz: read the calibration levels, publish them for the BLE
                // task, and save the profile the first time they are good enough.
                //
                // Before this tick's sample, for two reasons: the detector's gate
                // is never a state behind, and the save switches the chip's mode
                // for ~40 ms, after which the first sample read back can be odd.
                if n.is_multiple_of(SAMPLE_RATE_HZ) {
                    let (updated, saved) =
                        poll_calibration(&mut bno055, &mut delay, store).await;
                    calibration = updated;
                    if saved {
                        // Drop the detector's state so the post-mode-switch sample
                        // cannot close a rep around itself. It re-arms from the
                        // next real rest.
                        detector = Detector::new();
                    }
                }

                let Ok(accel) = bno055.linear_acceleration_fixed() else {
                    warn!("BNO055 linear accel read error");
                    continue;
                };
                let Ok(q) = bno055.quaternion() else {
                    warn!("BNO055 quaternion read error");
                    continue;
                };
                let sample = Sample {
                    t_ms,
                    accel: [accel.x, accel.y, accel.z],
                    quat: [
                        quantize(q.s, QUAT_SCALE),
                        quantize(q.v.x, QUAT_SCALE),
                        quantize(q.v.y, QUAT_SCALE),
                        quantize(q.v.z, QUAT_SCALE),
                    ],
                };
                samples.borrow_mut().push(sample);

                // Detect reps; on close, slice the window out of the ring and
                // compute the ZUPT-anchored trace + summary, then persist it.
                //
                // Only while the IMU is trustworthy. Uncalibrated, the gyro and
                // accelerometer report drift as motion: the detector would fire on
                // it and the stored trace would be fiction rather than a
                // measurement (see docs/CALIBRATION.md). Not feeding the detector at
                // all also means it starts from a real rest once calibration
                // completes.
                let span = if FAKE_REPS {
                    // Bench mode: one synthetic rep per period, covering the last
                    // FAKE_REP_MS of samples whatever the motion was. The calibration
                    // gate is deliberately *not* applied — the chip's `acc` status
                    // flaps on its own (see docs/GOTCHAS.md), so gating would make
                    // reps appear and vanish at random, and this data is not meant to
                    // be trustworthy anyway. Everything downstream is untouched: the
                    // records are ordinary ones the app syncs and plots.
                    if !logged_fake_reps {
                        logged_fake_reps = true;
                        warn!(
                            "fake-reps: storing a synthetic rep every {} s, not detecting real ones",
                            FAKE_REP_PERIOD_S
                        );
                    }
                    if n.is_multiple_of(SAMPLE_RATE_HZ * FAKE_REP_PERIOD_S)
                        && t_ms >= FAKE_REP_MS
                    {
                        Some(RepSpan {
                            start_ms: t_ms - FAKE_REP_MS,
                            end_ms: t_ms,
                        })
                    } else {
                        None
                    }
                } else if calibration.is_ready() {
                    detector.update(&sample)
                } else {
                    None
                };
                if let Some(span) = span {
                    let copied = samples.borrow().copy_window(
                        span.start_ms,
                        span.end_ms,
                        MARGIN_MS,
                        MARGIN_MS,
                        rep_window,
                    );
                    if copied == 0 {
                        warn!("rep window unavailable (ring too short)");
                    } else {
                        reps += 1;
                        let rep_uuid = {
                            let mut rand = || rng.random();
                            uuid::new_uuid(&mut rand)
                        };
                        let mut sum = compute_rep(&rep_window[..copied], &mut rep_trace);
                        // Identity is minted when the rep closes, so it is set here
                        // rather than computed with the stats.
                        sum.rep_uuid = calypso_core::pb::uuid_to_field(&rep_uuid);
                        // Persist the rep (summary + trace chunks). This
                        // happens right after the rest debounce, i.e. while
                        // the sensor is at rest, so the tens of ms an erase
                        // takes here can't land in the middle of a rep.
                        let stored = {
                            let mut guard = store.lock().await;
                            store_rep(&mut guard, rec_buf, &mut sum, &rep_trace)
                        };
                        match stored {
                            Ok(n) => info!(
                                "rep {} stored as {} log records",
                                UuidShort(Some(&rep_uuid)),
                                n
                            ),
                            Err(e) => warn!(
                                "rep {} could not be stored: {}",
                                UuidShort(Some(&rep_uuid)),
                                e
                            ),
                        }
                        info!(
                            "rep {}: {}..{} ms ({} ms), {} trace pts{}, rom {=f32} m, peak v {=f32} m/s, mean v {=f32} m/s, peak a {=f32} m/s^2, roll {=f32}..{=f32} deg, pitch {=f32}..{=f32} deg",
                            UuidShort(Some(&rep_uuid)),
                            span.start_ms,
                            span.end_ms,
                            sum.duration_ms,
                            sum.trace_len,
                            if sum.decimated { " (decimated)" } else { "" },
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

                n += 1;
                // The level verdict and the calibration levels are *read* at
                // 1 Hz, but logged only when they change, or once a minute. Two
                // lines a second used to say the same thing forever: the levels and
                // the verdict only move when the sensor does, and the ring and heap
                // figures are there to spot a leak, not to be watched. The minute
                // keeps the boot-log figures this device is debugged from (see
                // docs/GOTCHAS.md).
                if n.is_multiple_of(SAMPLE_RATE_HZ) {
                    let level = level_sense(&mut bno055);
                    // Compared on the verdict, not the reading: X-grav moves by a
                    // hundredth a second, and the question is only whether it is
                    // level.
                    let verdict = level.map(|(_, is_level)| is_level);
                    if verdict != logged_level {
                        logged_level = verdict;
                        match level {
                            Some((x, true)) => {
                                info!("Sensor is level on the X axis (X-grav={=f32})", x)
                            }
                            Some((x, false)) => warn!("Sensor is not level, X-grav={=f32}", x),
                            None => warn!("BNO055 gravity read error"),
                        }
                    }

                    if n.is_multiple_of(SAMPLE_RATE_HZ * 60)
                        || logged_calibration != Some(calibration)
                    {
                        logged_calibration = Some(calibration);
                        let (len, pushed) = {
                            let r = samples.borrow();
                            (r.len(), r.pushed())
                        };
                        if calibration.is_ready() {
                            info!(
                                "sampler: {} Hz, ring {}/{} samples, {} pushed, {} reps, imu {} (recording), stack {}, heap free {} B",
                                SAMPLE_RATE_HZ,
                                len,
                                SAMPLE_RING_LEN,
                                pushed,
                                reps,
                                CalibrationLog(calibration),
                                Stack,
                                esp_alloc::HEAP.free()
                            );
                        } else {
                            // Warn, not info: nothing is being recorded, and the
                            // levels say which sensor to work on.
                            warn!(
                                "sampler: {} Hz, ring {}/{} samples, {} pushed, imu {} — rep recording paused until the IMU is calibrated (stack {})",
                                SAMPLE_RATE_HZ,
                                len,
                                SAMPLE_RING_LEN,
                                pushed,
                                CalibrationLog(calibration),
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

/// Mount the rep-record flash log on the `log` data partition (subtype
/// `undefined`, see partitions.csv). Returns the number of sectors in the
/// ring. Mounting here formats the partition on first boot; individual reps
/// are written later by [`store_rep`].
fn mount_rep_log(store: &mut FlashStore) -> Result<u32, LogError> {
    let (flash, pt_mem) = store.split();
    let pt = partitions::read_partition_table(flash, pt_mem).map_err(LogError::Table)?;
    let Some(log_part) = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Undefined,
        ))
        .map_err(LogError::Table)?
    else {
        // The flashed table has no data/undefined partition. Usually a stale
        // table on the device rather than a broken log, so say what is actually
        // there — otherwise the next step is a guess.
        log_partition_table(&pt);
        return Err(LogError::NoPartition);
    };
    // Scoped so the region's borrow of `flash` ends before the probe below.
    let mounted = {
        let mut region = log_part.as_embedded_storage(flash);
        FlashLog::mount(&mut region).map(|log| log.sectors())
    };
    match mounted {
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

/// A failing flash phase in words, with the sector: `erasing sector 1`,
/// `header scan stopped at sector 212`.
///
/// `calypso-core` owns the vocabulary ([`flashlog::Phase`]) and this firmware owns the
/// sentence, because core is platform-independent and has no defmt.
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
/// Only after a failed log operation, because the useful detail is exactly what the
/// vendor partition wrapper throws away: `PartitionEntry::as_embedded_storage` maps
/// every storage failure to `partitions::Error::StorageError`. The raw `FlashStorage`
/// does report the real `FlashStorageError`, and a read is safe to repeat.
///
/// The sector comes from the failure itself rather than from sector 0 — a probe that
/// always looked at sector 0 reported "sector 0 reads fine" for a failure five sectors
/// away. (It happened; that was the first version of this.)
///
/// Walking rather than sampling is the point: this runs from a *shallow* frame, after
/// the failed read has unwound. So "sector N reads end to end" means the bytes are
/// there and readable when nothing else is on the stack, and the failure is about the
/// context the real read ran in — while "failed at +N" with the flash's own error name
/// pins it to the flash or the driver. Those need different repairs, and nothing else
/// distinguishes them.
///
/// A write is deliberately not probed: repeating an erase to see whether it works is
/// not a diagnostic, it is the bug.
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

/// The sector a failed flash operation was on, or 0 when the error was not ours to
/// place. Read before the error is mapped, because mapping consumes it.
fn failing_sector(e: &flashlog::Error<partitions::Error>) -> u32 {
    match e {
        flashlog::Error::Flash { sector, .. } => *sector,
        _ => 0,
    }
}

/// Why a flash-log operation failed, named.
///
/// Used for both the boot-time mount and a rep append, because an append *mounts the
/// log itself* (see [`store_rep`]) and therefore fails for the same reasons — and
/// because reporting them as two different-sounding sentences ("Rep log unusable" /
/// "rep could not be stored") sends you looking for a second bug that is not there.
///
/// The underlying errors travel through rather than being folded into a sentence:
///
/// - for the table, `Invalid` means it read fine and is not a table, while
///   `StorageError` means reading it failed — "partition table unreadable" covered
///   both;
/// - for the region, the *phase* says which flash operation failed (a refused erase
///   and a failed read have different repairs — see `flashlog::Phase`), and the
///   `partitions::Error` beside it says how far it got: `StorageError` means the
///   storage layer failed (the specific error is flattened away by the vendor wrapper
///   — see [`probe_log_partition`]), while `OutOfBounds`, `WriteProtected` and
///   `InvalidArgument` mean the operation never reached the flash. "Flash read/write
///   error on the log partition" covered all of them, and cost a session to read.
///
/// `partitions::Error` is `#[non_exhaustive]` and formatted rather than matched, so a
/// future variant prints instead of failing to compile.
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
            // Ours rather than the flash's: something was too big for a buffer we
            // sized, which is a different repair from a flash that will not write.
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
/// Only on a failed lookup: the answer to "is the table the one this firmware was
/// built for?" then sits in the log instead of in a guess.
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
/// `PIPE_STORAGE_MAX` because that is what the generated characteristic is typed as —
/// `notify` takes the type the table holds — and the spare byte over a frame is what
/// keeps a full-frame write out of `trouble-host`'s equal-length path (see
/// `docs/GOTCHAS.md`). What is notified is only ever a frame, `pipe::FRAME_MAX` bytes
/// at most.
type BleVec = heapless::Vec<u8, PIPE_STORAGE_MAX>;

/// The sending side of the sync service: the pipe's session, the one characteristic
/// every message goes out on, and the buffer each frame is built in.
///
/// One characteristic means one payload buffer, and the pipe means one *message* at a
/// time: a summary and a trace chunk are both transfers, and only one of them can be
/// in flight. (Calibration is not one: it is a characteristic a client reads.)
struct Sync<'a> {
    session: &'a mut Session<'static>,
    characteristic: &'a Characteristic<BleVec>,
    payload: &'a mut BleVec,
}

impl Sync<'_> {
    /// Notify everything the pipe has ready: the frame it owes for the last write,
    /// and the chunks of the message in flight.
    ///
    /// The buffer is reused for each, so it is resized to a whole frame before the
    /// pipe writes one into it and truncated back to the frame's length afterwards.
    async fn flush(&mut self, conn: &GattConnection<'_, '_, DefaultPacketPool>) {
        let Sync {
            session,
            characteristic,
            payload,
        } = self;
        let mtu = conn.raw().att_mtu() as usize;
        // Frames of the *message* this call gets out (the reply is not one). A message
        // wider than the window is sent in waves, and a wave that stops short is the
        // thing to see when a transfer stalls (docs/GOTCHAS.md).
        let mut frames = 0u32;

        loop {
            if payload.resize(pipe::FRAME_MAX, 0).is_err() {
                return;
            }
            let len = match session.reply(payload.as_mut_slice()) {
                // An acknowledgement first: the app's sender is waiting on it, and
                // the chunk it is about would otherwise go out ahead of it.
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
            // A notification to nobody reports success (see `is_subscribed`), so
            // this result is ever only a radio failure.
            if characteristic.notify(conn, payload).await.is_err() {
                warn!("sync: notify failed");
                return;
            }
        }
        // The wave that stopped short says so, because the alternative is an app
        // waiting for frames that are sitting in this buffer.
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
    /// `false` means the pipe is still finishing the message before this one: the
    /// caller keeps its message and asks again when the acknowledgement arrives.
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

    /// Send the next chunk of the trace being streamed, if there is one and the pipe
    /// is free.
    async fn send_trace_chunk(&mut self, conn: &GattConnection<'_, '_, DefaultPacketPool>) -> bool {
        if !self.session.send_next_trace_chunk() {
            return false;
        }
        self.flush(conn).await;
        true
    }
}

/// The summaries read out of the log and waiting for the pipe, and how far the pipe
/// has got through them.
///
/// One batch is read at a time and drained one *transfer* at a time, because that is
/// what the pipe carries: a batch of sixteen is sixteen round trips, so it is refilled
/// and drained across event loop iterations rather than in one call.
struct SummaryBatch {
    messages: &'static mut AllocVec<msg::RepSummary>,
    next: usize,
}

impl SummaryBatch {
    fn new(messages: &'static mut AllocVec<msg::RepSummary>) -> Self {
        messages.clear();
        SummaryBatch { messages, next: 0 }
    }

    fn spent(&self) -> bool {
        self.next >= self.messages.len()
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
    }
}

/// Push any log summaries the subscriber hasn't been sent yet, oldest first.
///
/// **Call this only while the peer is subscribed** ([`is_subscribed`]). The cursor
/// advances on every message the pipe took, and a transfer to nobody is abandoned
/// after three attempts, so calling it with no subscriber walks the log into the void
/// and the app never sees those reps on that connection.
///
/// One summary per call, at most: a message is a transfer, a transfer ends when the
/// app acknowledges it, and the acknowledgement arrives as a *write* — so this is
/// called from the tick, from each write, and from nothing else. Whatever is not sent
/// this time is still in the batch, and the cursor has not moved past it.
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
    // reading the log is a flash scan, and running it while a transfer was in flight
    // would hold the event loop for a batch that could not go out yet anyway.
    if batch.spent() {
        if !sync.session.is_idle() {
            return;
        }
        let total = {
            let mut guard = store.lock().await;
            collect_summaries(&mut guard, rec_buf, rec, cursor.next(), batch.messages)
        };
        let Some(total) = total else {
            return;
        };
        // Reps fall off the ring as it wraps; never point past the end.
        cursor.clamp_to(total);
        batch.next = 0;
    }

    let Some(summary) = batch.peek() else {
        return;
    };
    if sync.send_summary(summary, conn).await {
        batch.advance();
        cursor.advance(1);
    }
}

/// Fill `batch` with the next summaries the subscriber hasn't seen, returning how
/// many summaries the log holds (or `None` if the log couldn't be read).
///
/// Sync on purpose. Everything inlined into an async fn becomes part of its
/// future, and a future lives on the task's stack for as long as it is suspended:
/// the flash log is a deep call graph (mount, sector scan, CRC, decode), none of
/// which needs to be in a task's state while it waits for a notification. Keeping
/// it behind this boundary is what holds both of those frames to one buffer apiece.
fn collect_summaries(
    store: &mut FlashStore,
    rec_buf: &mut [u8],
    rec: &mut msg::RepRecord,
    skip: usize,
    batch: &mut AllocVec<msg::RepSummary>,
) -> Option<usize> {
    let (flash, pt_mem) = store.split();
    let pt = partitions::read_partition_table(flash, pt_mem).ok()?;
    let partition = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Undefined,
        ))
        .ok()??;
    let mut region = partition.as_embedded_storage(flash);
    let mut log = FlashLog::mount(&mut region).ok()?;

    batch.clear();
    let mut seen = 0usize;
    let _ = store::for_each_summary(&mut log, rec_buf, rec, |s| {
        if seen >= skip && batch.len() < batch.capacity() {
            batch.push(s.clone());
        }
        seen += 1;
    });
    Some(seen)
}

/// Read a requested rep's trace out of the log into the session, and start streaming
/// it: one `TraceChunk` per transfer, in chunk order.
///
/// The transfers themselves are the tick's business, and each one is a round trip:
/// this reads the trace into the session's storage and points the cursor at it, and
/// the tick (or the next write, which is the app's acknowledgement) sends the next
/// chunk. That is what the latency item in the roadmap was reaching for — nothing here
/// waits behind a queue of notifications, and the connection's event loop keeps
/// running to the acknowledgements that move it along.
///
/// This used to be the widest frame in the firmware, with an exemption for it: the
/// chunk message was built here (a `SyncMessage` is 2 KB by value, one of its variants
/// being a whole `TraceChunk`). It is built inside the session now — a plain method
/// call, so the 2 KB is a stack frame that is gone before anything is awaited, rather
/// than something a task's future holds — and this fits the lint's budget without
/// help. The log work was already extracted (see [`read_trace_from_log`]).
async fn stream_trace(
    store: &'static AsyncMutex<NoopRawMutex, FlashStore>,
    rec_buf: &mut [u8],
    rec: &mut msg::RepRecord,
    sync: &mut Sync<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    request: msg::TraceRequest,
) {
    let Some(rep_uuid) = calypso_core::pb::uuid_from_field(&request.rep_uuid) else {
        warn!("trace request with a malformed uuid");
        return;
    };

    // Read it into the session's storage, which is where the cursor that streams it
    // looks: one copy, and no lifetime dance to hand the points over.
    let (t0_ms, points) = {
        let mut trace = TraceBuf::new(sync.session.trace_storage());
        {
            let mut guard = store.lock().await;
            // The log work stays out of this future: see `collect_summaries`.
            if !read_trace_from_log(&mut guard, rec_buf, rec, rep_uuid, &mut trace) {
                warn!(
                    "trace for rep {} is no longer in the log",
                    UuidShort(Some(&rep_uuid))
                );
                return;
            }
        }
        let t0_ms = trace.get(0).map(|p| p.t_ms).unwrap_or(0);
        (t0_ms, trace.len())
    };

    log_trace(&rep_uuid, points);

    // A request for one chunk sends one chunk; a request for the trace sends all of
    // it. Which is a cursor position and a stop, not a different stream.
    let only = if request.all_chunks {
        None
    } else {
        Some(request.chunk_index)
    };
    sync.session.start_trace(rep_uuid, t0_ms, points, only);

    // The first transfer goes now; the rest go as the app acknowledges them (see
    // `keep_sending`).
    sync.send_trace_chunk(conn).await;
}

/// Read one rep's trace out of the log into `trace`, returning whether it was
/// there at all (a rep the ring has overwritten reads back empty).
///
/// Sync, and separate from its caller for the same reason as
/// [`collect_summaries`].
fn read_trace_from_log(
    store: &mut FlashStore,
    rec_buf: &mut [u8],
    rec: &mut msg::RepRecord,
    rep_uuid: RepUuid,
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
    store::read_trace(&mut log, rec_buf, rec, rep_uuid, trace)
        .map(|n| n > 0)
        .unwrap_or(false)
}

/// A rep UUID for logs: its short form, or a placeholder when absent.
///
/// Formatted through `defmt` directly rather than to a `&str`, so logging a rep
/// costs no allocation.
struct UuidShort<'a>(Option<&'a RepUuid>);

impl defmt::Format for UuidShort<'_> {
    fn format(&self, f: defmt::Formatter) {
        match self.0 {
            Some(uuid) => defmt::write!(f, "{=[u8]:02x}", &uuid::short(uuid)[..]),
            None => defmt::write!(f, "????????"),
        }
    }
}

/// Append one rep (summary + trace chunks) to the flash log partition.
///
/// Returns the number of log records written. The log is mounted per append:
/// mounting is a scan of the sector headers, and appends are rare (once per
/// rep, at rest), so there's no need to keep it alive between reps.
///
/// **Which is why an append fails for the same reasons the boot-time mount does**, and
/// reports them the same way: a mount that cannot run means no rep can be stored, and
/// nothing retries, so a rep is lost when this returns an error.
fn store_rep(
    store: &mut FlashStore,
    rec_buf: &mut [u8],
    summary: &mut msg::RepSummary,
    trace: &TraceBuf<'_>,
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
            .and_then(|mut log| store::append_rep(&mut log, rec_buf, summary, trace.as_slice()))
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

/// Apply the stored BNO055 calibration profile, if there is a usable one.
///
/// Deliberately does **not** wait for the IMU to calibrate. This is the routine
/// that used to block boot in a loop, which meant an uncalibrated device had no
/// BLE, no app, and nothing to say about why. The device now boots fully and
/// tells the app what to do; it just does not record reps until the levels are
/// good (see `docs/CALIBRATION.md`).
fn load_calibration(bno055: &mut Bno055<I2c<'_, Blocking>>, store: &mut FlashStore) {
    let Some(profile) = read_calibration(store) else {
        info!("No stored IMU calibration; waiting for the sensor to calibrate");
        return;
    };
    let mut delay = Delay;
    if bno055.set_calibration_profile(profile, &mut delay).is_err() {
        warn!("BNO055 rejected the stored calibration profile");
        return;
    }

    // **Read it back, because the write is fire-and-forget.** `set_calibration_profile`
    // is one I2C burst and reports only that the bus took it, so a profile the chip
    // ignored is indistinguishable from one it accepted until its levels are read —
    // which is exactly the ambiguity that "the accelerometer is 0 after every reboot"
    // has been sitting in. The two outcomes need different fixes: bytes that do not
    // come back mean the restore is not landing (our write, or the chip's mode), while
    // bytes that do come back mean the chip holds the offsets and simply does not
    // report `acc` as calibrated from them.
    //
    // Two more mode switches (~40 ms) and 22 bytes, once per boot, only when there is
    // a profile to apply.
    match bno055.calibration_profile(&mut delay) {
        Ok(after) if after == profile => {
            info!("Stored IMU calibration applied, and reads back byte for byte")
        }
        Ok(after) => warn!(
            "Stored IMU calibration applied, but the chip reads back {=[u8]:02x}",
            after.as_bytes()
        ),
        Err(_) => warn!("Stored IMU calibration applied, but could not be read back"),
    }
}

/// The calibration profile stored in NVS, or `None` when there isn't a usable
/// one.
///
/// The blob has no checksum — it is the chip's own register dump, written as-is —
/// so erased flash is recognised by its all-ones pattern. Without that check a
/// fresh device would apply 22 bytes of `0xFF` as calibration offsets, which is
/// worse than starting uncalibrated.
fn read_calibration(store: &mut FlashStore) -> Option<BNO055Calibration> {
    let (flash, pt_mem) = store.split();
    let pt = match partitions::read_partition_table(flash, pt_mem) {
        Ok(pt) => pt,
        Err(e) => {
            warn!("NVS: partition table unreadable: {}", e);
            return None;
        }
    };
    let nvs = match pt.find_partition(partitions::PartitionType::Data(
        partitions::DataPartitionSubType::Nvs,
    )) {
        Ok(Some(nvs)) => nvs,
        Ok(None) => {
            warn!("NVS: no nvs partition in the flashed table");
            return None;
        }
        Err(e) => {
            warn!("NVS: partition lookup failed: {}", e);
            return None;
        }
    };
    let mut nvs_partition = nvs.as_embedded_storage(flash);

    let mut blob = [0u8; BNO055_CALIB_SIZE];
    if let Err(e) = nvs_partition.read(BNO055_CALIBRATION_ADDR, &mut blob) {
        warn!("NVS: calibration read failed: {}", e);
        return None;
    }
    if blob.iter().all(|&b| b == 0xFF) {
        // Distinguishable on purpose: from the caller's one line, "this device has
        // never been calibrated" and "the calibration is there and unreadable" look
        // the same, and they are not the same problem.
        info!("NVS: no stored calibration (the sector is erased)");
        return None;
    }
    Some(BNO055Calibration::from_buf(&blob))
}

/// Persist the IMU's calibration profile, so the next power cycle starts
/// calibrated instead of putting the user through it again.
///
/// Returns whether the write succeeded. The profile lives at offset 0 of the NVS
/// partition's first sector — it is the only thing in there now that the device
/// name is a compile-time constant — and that sector is erased first: NOR flash
/// bits only go from 1 to 0 when written, so a second save over an older profile
/// would AND the two together.
fn save_calibration(store: &mut FlashStore, profile: &BNO055Calibration) -> bool {
    let (flash, pt_mem) = store.split();
    let Ok(pt) = partitions::read_partition_table(flash, pt_mem) else {
        warn!("NVS: partition table read failed");
        return false;
    };
    let Ok(Some(nvs)) = pt.find_partition(partitions::PartitionType::Data(
        partitions::DataPartitionSubType::Nvs,
    )) else {
        warn!("NVS: partition not found");
        return false;
    };
    let mut nvs_partition = nvs.as_embedded_storage(flash);

    // Each error named: `OtherCoreRunning` here means the flash refused because the
    // other core was up, and the erase-then-write below is why that matters — an
    // erase that succeeds and a write that does not leaves no profile at all, so
    // which of the two failed decides whether there is anything left to recover.
    if let Err(e) = nvs_partition.erase(BNO055_CALIBRATION_ADDR, BNO055_CALIBRATION_ADDR + 0x1000) {
        warn!("NVS: calibration erase failed: {}", e);
        return false;
    }
    if let Err(e) = Storage::write(
        &mut nvs_partition,
        BNO055_CALIBRATION_ADDR,
        profile.as_bytes(),
    ) {
        warn!("NVS: calibration write failed: {}", e);
        return false;
    }
    true
}

/// Read the IMU's calibration levels, publish them for the BLE task, and — the
/// first time they are good enough to record with — save the profile.
///
/// Returns the levels and whether the profile was saved just now. Called at 1 Hz
/// from the sampler task, which owns the I2C bus.
///
/// The read-back and save switch the chip to CONFIG mode and back (~40 ms, during
/// which it stops fusing), so the caller resets its detector afterwards.
async fn poll_calibration(
    bno055: &mut Bno055<I2c<'_, Blocking>>,
    delay: &mut Delay,
    store: &'static AsyncMutex<NoopRawMutex, FlashStore>,
) -> (Calibration, bool) {
    let calibration = match bno055.get_calibration_status() {
        Ok(s) => Calibration::new(s.sys, s.acc, s.gyr, s.mag),
        Err(_) => {
            // A transient read error must not look like "calibration lost": keep
            // reporting the levels we last saw.
            warn!("BNO055 calibration status read error");
            Calibration::from_bits(CALIBRATION_BITS.load(Ordering::Relaxed))
        }
    };
    CALIBRATION_BITS.store(calibration.to_bits(), Ordering::Relaxed);

    if !calibration.is_ready() || CALIBRATION_SAVED.load(Ordering::Relaxed) {
        return (calibration, false);
    }

    // `is_ready`, not `is_fully_calibrated`: the magnetometer is unused in IMU mode,
    // its level never reaches 3 there, and the aggregate `sys` counts it — so
    // readiness is the accelerometer and gyroscope, and the rest would be waiting
    // for a number that cannot arrive (see `calypso_core::calibration`).
    let Ok(profile) = bno055.calibration_profile(delay) else {
        warn!("could not read the calibration profile back");
        return (calibration, false);
    };
    let saved = {
        let mut guard = store.lock().await;
        save_calibration(&mut guard, &profile)
    };
    if saved {
        CALIBRATION_SAVED.store(true, Ordering::Relaxed);
        info!("IMU calibration saved; reps will be recorded");
    }
    (calibration, saved)
}

/// Whether the peer has this characteristic's notifications enabled.
///
/// **This is the only way to know, and it is why the logs and the summary cursor
/// are built on it rather than on a notification's result.**
/// `Characteristic::notify` returns `Ok(())` whether or not anybody is subscribed:
/// trouble-host checks its per-connection CCCD table (`should_notify`) and returns
/// early *without sending* when the peer is not in it. So "the notification
/// succeeded" is not evidence of anything, and code that treated it as evidence got
/// it wrong twice here — a `calibration status delivered to the app` line printed
/// once a second to a device that was sending nothing, and a summary cursor that
/// walked the whole log into the void while nobody was listening.
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

/// Put the IMU's calibration state in the table, where a client reads it.
///
/// The `Calibration` characteristic is a value rather than a pipe message — the app
/// reads it when its UI wants the levels — so this end only keeps the table's copy
/// current. Called when a connection is established and once a second after that:
/// the levels come from the chip's own 1 Hz poll ([`CALIBRATION_BITS`]), so there is
/// nothing fresher to serve.
///
/// A message too large for the characteristic's `max_len` is `None` rather than a
/// truncation, and nothing this device holds can be (see `proto/calypso_gatt.proto`);
/// the warn is there so that it would not be silent if it ever were.
fn publish_calibration(gatt: &CalypsoGatt<'_>) {
    let calibration = Calibration::from_bits(CALIBRATION_BITS.load(Ordering::Relaxed));
    let status = calypso_core::pb::calibration_message(&calibration);
    match SyncCalibrationValue::encode(&status) {
        Some(value) => {
            if gatt.set(&gatt.sync.calibration, &value).is_err() {
                warn!("calibration: the table refused the status");
            }
        }
        None => warn!("calibration: the status is larger than the characteristic"),
    }
}

/// Send everything the app is owed, in the order it is waiting for it.
///
/// The whole send path, and the reason it is a call rather than a loop that runs to
/// completion: the pipe carries one *message* at a time, and a message is finished by
/// an acknowledgement that arrives as a write. So this is called whenever something
/// might have freed the pipe — a tick, a write, a trace request — and does as little
/// as it can get away with.
///
/// The order is the app's: a trace it asked for (and is timing out on), then the
/// summaries, which are the bulk and can wait.
///
/// The exemption is the composition: two awaited futures in one function is one
/// future holding both, and the lint adds them up. It is the same stack the
/// event loop held before the pipe — the trace and summary steps were awaited side
/// by side in that block — and the alternative is writing this order down twice,
/// once per call site, for the lint's benefit rather than the device's.
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
    // **A message can be wider than the window.** The window is 8 chunks and a trace
    // chunk is up to 10 frames, so a transfer is usually sent in waves — and the rest
    // of the wave goes out here, when the acknowledgement that made room arrives.
    // Returning early on a busy pipe leaves the message half-sent for ever: nothing
    // else calls `flush`, and the app waits for a chunk that is sitting in the
    // device's buffer (docs/GOTCHAS.md).
    if !sync.session.is_idle() {
        sync.flush(conn).await;
        return;
    }
    // Otherwise: a trace chunk, if one is being streamed, and only then summaries —
    // the app is waiting on its trace.
    if sync.session.streaming() && sync.send_trace_chunk(conn).await {
        return;
    }
    stream_new_summaries(store, rec_buf, rec, batch, sync, conn, cursor).await;
}

/// A decoded-record scratch on the heap.
///
/// In its own function, and never inlined, so the ~2 KB temporary that
/// `Box::new` needs to build the value lives in *this* frame — which is gone by
/// the time it returns — rather than in `main`'s, which is the bottom of the
/// stack the whole BLE stack also runs on.
///
/// `Box::new` really does build its argument on the stack first (clippy reports
/// exactly that: 2152 bytes), and there is no way to allocate a `RepRecord`
/// without one short of writing its fields through a `MaybeUninit` by hand. That
/// would trade a bounded, known transient for `unsafe` in the decode path, so the
/// transient stays — deliberately in a leaf called before the tasks start, which
/// is what the exemption below records.
#[allow(
    clippy::large_stack_frames,
    reason = "2152 B: the `Box::new` temporary, isolated so it is not in main's frame"
)]
#[inline(never)]
fn leak_record_scratch() -> &'static mut msg::RepRecord {
    Box::leak(Box::new(msg::RepRecord::default()))
}

/// The pipe's session on the heap, for the same reason as the record scratch: the
/// buffers it holds are handed over rather than built here, but the struct itself is
/// not small, and `main`'s frame is the bottom of the stack the BLE stack runs on.
#[inline(never)]
fn leak_session(
    outgoing: &'static mut [u8],
    incoming: &'static mut [u8],
    points: &'static mut [TracePoint],
) -> &'static mut Session<'static> {
    Box::leak(Box::new(Session::new(outgoing, incoming, points)))
}

/// What a trace request turned into, in a frame of its own: the defmt formatting state
/// is a local, and `stream_trace`'s frame is on the stack the BLE stack also runs on
/// (`clippy::large_stack_frames`, which is denied here).
#[inline(never)]
fn log_trace(rep_uuid: &RepUuid, points: usize) {
    info!(
        "trace {}: {} points, {} chunks",
        UuidShort(Some(rep_uuid)),
        points,
        calypso_core::pb::chunk_count_for(points),
    );
}

/// Milliseconds since `start`, which is what the pipe's timeouts are measured in.
///
/// Any monotonic reference will do — a transfer's stall timeout is a difference — so
/// this is the connection's own start rather than the device's uptime.
fn now_ms(start: Instant) -> u32 {
    start.elapsed().as_millis() as u32
}

// The linker stack: `Reset` sets the stack pointer to `_stack_start` and the stack
// grows down from there, so bytes in use are measured from the top and free bytes
// from `_stack_end` — which is wherever `.data` and `.bss` ended, making the stack
// what the RWDATA region has left over (see docs/GOTCHAS.md).
unsafe extern "C" {
    static _stack_start: u8;
    static _stack_end: u8;
}

/// What [`paint_stack`] fills the unused stack with.
const STACK_PAINT: u8 = 0xA5;

/// How much of the stack's bottom is neither painted nor scanned: the
/// `stack-protector` guard sits 60 bytes above `_stack_end`, and a byte next to it
/// says nothing about how deep the stack has been.
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

/// Bytes of the linker stack this boot has been through at its deepest, read back
/// from [`STACK_PAINT`]: the stack region is a buffer nobody zeroes, and the lowest
/// pattern byte still standing is how far down `main`'s frame and the poll chains
/// under it have reached.
///
/// The figure is at least the depth at the moment of painting, and it cannot see
/// the floor below `STACK_PAINT_FLOOR` — a stack that reached in there would have
/// written the guard and faulted first.
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
/// Below the stack pointer is free by definition, and this frame's own storage is
/// above it. Called before `esp_rtos::start`: a task switch happens inside an
/// interrupt, and the interrupted context's frame would outlive the paint that
/// wrote over it.
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

/// The stack figures a log line carries: in use now, the total, and the deepest
/// since [`paint_stack`].
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

/// Calibration levels for logs, so a serial console alone says what to work on:
/// `acc:1 gyro:3 mag:0 sys:1`.
struct CalibrationLog(Calibration);

impl defmt::Format for CalibrationLog {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "acc:{} gyro:{} mag:{} sys:{}",
            self.0.accel,
            self.0.gyro,
            self.0.mag,
            self.0.sys
        )
    }
}

/// The IMU's X-axis gravity, and whether that reads as level.
///
/// `None` when the read failed, which is *not* the same as "not level": a failed
/// I2C read is not a measurement, and a bar can be perfectly still with the sensor
/// unreadable. Logged by the caller, and only when the verdict changes — this used
/// to print `Sensor is not level` every second and bury everything else.
fn level_sense(bno055: &mut Bno055<I2c<'_, Blocking>>) -> Option<(f32, bool)> {
    bno055
        .gravity()
        .ok()
        .map(|grav| (grav.x, calypso_core::is_level_along_axis(grav.x)))
}
