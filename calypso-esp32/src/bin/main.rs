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
use embassy_sync::mutex::Mutex as AsyncMutex;
use embassy_time::{Duration, Instant, Ticker, Timer, Delay};
use esp_storage::FlashStorage as FlashDevice;
use alloc::vec::Vec as AllocVec;
use core::cell::RefCell;
use esp_hal::{Blocking};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::i2c::master::I2c;
use esp_hal::rng::Rng;
use esp_hal::time::Rate;
use esp_println as _;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;
use esp_bootloader_esp_idf::partitions;
use esp_storage::FlashStorage;
use embedded_storage::{ReadStorage, Storage};
use embedded_storage::nor_flash::NorFlash;

use bno055::{BNO055OperationMode, BNO055Calibration, BNO055_CALIB_SIZE};
use bno055::Bno055;
use calypso_core::ble::{BleVec, SYNC_MSG_MAX};
use calypso_core::calibration::Calibration;
use calypso_core::flashlog::{self, FlashLog};
use calypso_core::pb::msg;
use calypso_core::sync::{self, decode_trace_request, SummaryCursor, TraceStreamer};
use calypso_core::uuid::{self, RepUuid};
use calypso_core::rep::{compute_rep, Detector, TraceBuf, TracePoint, MARGIN_MS};
use calypso_core::store::{self, RECORD_BUF_BYTES};
use calypso_core::sampler::{quantize, Sample, SampleRing, QUAT_SCALE, SAMPLE_RATE_HZ};
use calypso_core::DEVICE_NAME_MAX;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use alloc::boxed::Box;

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

/// Offset of the BLE device name blob inside the NVS partition. Lives in the
/// partition's second flash sector so erasing it never touches the calibration
/// blob in the first sector.
const BLE_DEVICE_NAME_ADDR: u32 = 0x1000;
const DEFAULT_DEVICE_NAME: &str = "Calypso";

/// Latest X-axis gravity reading (m/s^2), shared between the IMU loop and the
/// BLE task as raw f32 bits.
static GRAVITY_X_BITS: AtomicU32 = AtomicU32::new(0);

/// The IMU's calibration levels (`Calibration::to_bits`), written by the 1 Hz
/// poll in the sampler task and read by the BLE task to tell the app. Zero (every
/// level 0) until the first poll, which reads as "not calibrated" — the safe
/// thing to believe.
static CALIBRATION_BITS: AtomicU32 = AtomicU32::new(0);

/// Whether the calibration profile has been written to NVS this boot. Set once,
/// on the transition to calibrated, so the save (a flash erase and write) does
/// not repeat on every poll.
static CALIBRATION_SAVED: AtomicBool = AtomicBool::new(false);

/// Whether a calibration status has ever made it to a subscriber. Set once, so the
/// first delivery is logged and the rest (one a second) are not.
static CALIBRATION_DELIVERED: AtomicBool = AtomicBool::new(false);

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
    let i2c = I2c::new(peripherals.I2C0, i2c_config).unwrap()
    .with_sda(peripherals.GPIO22).with_scl(peripherals.GPIO19);
    info!("Done setting up i2c");

    let mut delay = Delay;

    info!("Setting up Bno055");
    let mut bno055 = Bno055::new(i2c).with_alternative_address();
    bno055.init(&mut delay).unwrap();
    info!("Bno055 initialized");
    // IMU mode: accel + gyro fusion, no magnetometer (unreliable near
    // barbells). Roll/pitch stay accurate; yaw drift is corrected per rep by
    // ZUPT (Phase 3).
    bno055.set_mode(BNO055OperationMode::IMU, &mut delay).unwrap();

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

    // Load the persisted BLE device name, falling back to the default.
    let device_name = load_device_name(&mut store);
    info!("BLE device name: {}", device_name.as_str());

    // The UUIDs this build exposes, in both orders, as hex. One line, and it answers
    // the question that cost several flashes: *which* firmware is on this board, as
    // seen from the other end of the link. A client reports a mismatch here as
    // "characteristic not found", naming a UUID that is correct.
    info!(
        "sync service uuid {=[u8]:02x} (wire), {=[u8]:02x} (as a client sees it)",
        calypso_core::ble::SYNC_SERVICE_UUID_BYTES,
        calypso_core::ble::reverse16(calypso_core::ble::SYNC_SERVICE_UUID_BYTES)
    );

    // Mount the rep-record flash log (see docs/REP_TRACKING.md).
    match mount_rep_log(&mut store) {
        Ok(sectors) => info!("Rep log mounted: {} sectors", sectors),
        // The reason, not just "unusable": the first question is always whether
        // the flashed partition table has the log partition at all.
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
    let rep_trace_storage: &'static mut [TracePoint] =
        Box::leak(trace_vec.into_boxed_slice());
    let mut rep_trace = TraceBuf::new(rep_trace_storage);
    // Scratch for encoding one log record at a time (~2 KB).
    let rec_buf: &'static mut [u8] =
        Box::leak(alloc::vec![0u8; RECORD_BUF_BYTES].into_boxed_slice());
    info!("Rep detector ready, heap free {} B", esp_alloc::HEAP.free());

    // The one flash peripheral, shared by the BLE task (device name) and the
    // sampler task (rep log). An async mutex, not a critical section: erasing
    // a sector takes tens of milliseconds and must not block interrupts. All
    // three futures below run in this task, so a no-op raw mutex suffices.
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
    // **Bump it whenever the GATT layout changes.** It was last bumped when the UUID
    // byte order was settled, having cost several flashes to work out. The
    // principled alternative is a GATT Service Changed characteristic (0x2A05) with
    // an indication, which tells clients to drop the cache themselves; this device
    // has no need for one yet.
    //
    // The top two bits are set, as a static random address requires.
    let stack = trouble_host::new(ble_controller, &mut resources)
        .set_random_address(Address::random([0xca, 0xa7, 0x19, 0x50, 0xe4, 0xfd]));
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
    let mut sync_msg_store = [0u8; calypso_core::ble::SYNC_MSG_MAX];
    let mut sync_request_store = [0u8; calypso_core::ble::SYNC_REQUEST_MAX];
    let mut sync_chunk_store = [0u8; calypso_core::ble::SYNC_MSG_MAX];
    let gatt = calypso_core::ble::build_gatt_table::<NoopRawMutex>(
        device_name.as_str(),
        &mut name_store[..],
        &mut gravity_x_store[..],
        &mut sync_msg_store[..],
        &mut sync_request_store[..],
        &mut sync_chunk_store[..],
    )
    .unwrap();
    let name_char = gatt.device_name;
    let gravity_x_char = gatt.gravity_x;
    let summary_char = gatt.summary;
    let trace_request_char = gatt.trace_request;
    let trace_chunk_char = gatt.trace_chunk;

    // Buffers for the sync path: a trace read back from the log, and a
    // reusable record scratch (both heap: too big for the stack/static RAM).
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
    // ~1.2 KB, which has no business sitting in a task's future.
    let sync_batch: &'static mut AllocVec<msg::RepSummary> =
        Box::leak(Box::new(AllocVec::with_capacity(SUMMARY_BATCH)));
    info!("Sync buffers ready, heap free {} B", esp_alloc::HEAP.free());
    let server = AttributeServer::<
        NoopRawMutex,
        DefaultPacketPool,
        { calypso_core::ble::GATT_TABLE_MAX },
        1,
        CONNECTIONS_MAX,
    >::new(gatt.table);

    info!("Starting BLE stack");
    // How much of the stack `main`'s own frame is holding before the tasks start.
    // This is the number that says whether the setup above is thrifty with the
    // linker stack or eating it (`_stack_end`; ~42 KB in the current build, and it
    // moves whenever a static does — measure, do not assume).
    info!("Stack in use before the tasks: {} B", stack_used());
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
                // after the current connection closes). The payloads are built in
                // `calypso-core` so that what a client sees is host-tested — see
                // `build_advertising` for the layout and why it is that way.
                let advertising = calypso_core::ble::build_advertising(name.as_str());
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
                    Ok(conn) => match conn.with_attribute_server(&server) {
                        Ok(conn) => conn,
                        Err(_) => continue,
                    },
                    Err(_) => continue,
                };
                info!("BLE connection established (stack {} B)", stack_used());

                // Sync state: how many summaries this subscriber has been
                // sent. Notifications fail while nobody is subscribed, so the
                // cursor only advances on a successful send and the next tick
                // retries — no CCCD bookkeeping needed.
                let mut cursor = SummaryCursor::new();

                loop {
                    // Service GATT events; once a second, push the latest
                    // gravity reading to the client (no-op unless subscribed).
                    let event = match select(conn.next(), Timer::after(Duration::from_secs(1))).await {
                        Either::First(event) => event,
                        Either::Second(()) => {
                            // Calibration first: it is what tells the app whether
                            // to expect reps at all, so it should not queue behind
                            // a burst of summaries.
                            notify_calibration(&summary_char, &conn).await;
                            let value = f32::from_bits(GRAVITY_X_BITS.load(Ordering::Relaxed));
                            // A failed notify means no subscriber yet; still fine
                            // to keep the connection (and try again next tick).
                            let _ = gravity_x_char.notify(&conn, &value).await;
                            // Auto-sync: push any summaries the app hasn't
                            // seen. A failed notify means nobody is
                            // subscribed, so we stop and retry next tick.
                            stream_new_summaries(
                                store,
                                sync_rec_buf,
                                sync_rec_msg,
                                sync_batch,
                                &summary_char,
                                &conn,
                                &mut cursor,
                            )
                            .await;
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
                            let wrote_request = matches!(&event, GattEvent::Write(w)
                                if w.handle() == trace_request_char.handle);
                            if let Ok(reply) = event.accept() {
                                let _ = reply.send().await;
                            }
                            if wrote_request {
                                // The app asked for a rep's trace: read it
                                // back from the log and stream it.
                                if let Ok(request) = server.table().get(&trace_request_char)
                                    && let Some(request) = decode_trace_request(&request)
                                {
                                    let wanted = calypso_core::pb::uuid_from_field(&request.rep_uuid);
                                    info!("trace request: rep {}", UuidShort(wanted.as_ref()));
                                    stream_trace(
                                        store,
                                        sync_rec_buf,
                                        sync_rec_msg,
                                        &mut *sync_trace_storage,
                                        &trace_chunk_char,
                                        &conn,
                                        request,
                                    )
                                    .await;
                                }
                            }
                            if wrote_name {
                                // The write has been applied to the attribute
                                // store by `accept`; read the new name back,
                                // keep it in memory and persist it to flash.
                                if let Ok(new_name) = server.table().get(&name_char)
                                    && new_name != name
                                {
                                    name = new_name;
                                    info!("device name set to {}", name.as_str());
                                    let mut guard = store.lock().await;
                                    save_device_name(&mut guard, name.as_str());
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }
        },
        // Sampler loop: 100 Hz IMU samples into the RAM ring buffer. The
        // gravity-x BLE characteristic is fed off the same samples, decimated.
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
            // The IMU's calibration levels, refreshed once a second.
            let mut calibration = Calibration::default();
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
                let span = if calibration.is_ready() {
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
                        let sum = compute_rep(&rep_window[..copied], &mut rep_trace);
                        // Persist the rep (summary + trace chunks). This
                        // happens right after the rest debounce, i.e. while
                        // the sensor is at rest, so the tens of ms an erase
                        // takes here can't land in the middle of a rep.
                        let stored = {
                            let mut guard = store.lock().await;
                            store_rep(&mut guard, rec_buf, rep_uuid, &sum, &rep_trace)
                        };
                        match stored {
                            Ok(n) => info!(
                                "rep {} stored as {} log records",
                                UuidShort(Some(&rep_uuid)),
                                n
                            ),
                            Err(()) => warn!("rep {} could not be stored", UuidShort(Some(&rep_uuid))),
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
                // Gravity-x: update at 10 Hz (BLE pushes it at 1 Hz).
                if n.is_multiple_of(10)
                    && let Ok(g) = bno055.gravity_fixed()
                {
                    let x = g.x as f32 / 100.0;
                    GRAVITY_X_BITS.store(x.to_bits(), Ordering::Relaxed);
                }
                // Level-sense log at 1 Hz, matching the old loop.
                if n.is_multiple_of(SAMPLE_RATE_HZ) {
                    get_level_sense(&mut bno055);
                    let (len, pushed) = {
                        let r = samples.borrow();
                        (r.len(), r.pushed())
                    };
                    if calibration.is_ready() {
                        info!(
                            "sampler: {} Hz, ring {}/{} samples, {} pushed, {} reps, imu {} (recording), stack {} B, heap free {} B",
                            SAMPLE_RATE_HZ,
                            len,
                            SAMPLE_RING_LEN,
                            pushed,
                            reps,
                            CalibrationLog(calibration),
                            stack_used(),
                            esp_alloc::HEAP.free()
                        );
                    } else {
                        // Warn, not info: nothing is being recorded, and the
                        // levels say which sensor to work on.
                        warn!(
                            "sampler: {} Hz, ring {}/{} samples, {} pushed, imu {} — rep recording paused until the IMU is calibrated (stack {} B)",
                            SAMPLE_RATE_HZ,
                            len,
                            SAMPLE_RING_LEN,
                            pushed,
                            CalibrationLog(calibration),
                            stack_used()
                        );
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
fn mount_rep_log(store: &mut FlashStore) -> Result<u32, &'static str> {
    let (flash, pt_mem) = store.split();
    let pt = partitions::read_partition_table(flash, pt_mem)
        .map_err(|_| "partition table unreadable")?;
    let Some(log_part) = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Undefined,
        ))
        .map_err(|_| "partition table unreadable")?
    else {
        // The flashed table has no data/undefined partition. Usually a stale
        // table on the device rather than a broken log, so say what is actually
        // there — otherwise the next step is a guess.
        log_partition_table(&pt);
        return Err("no data/undefined partition in the flashed table");
    };
    let mut region = log_part.as_embedded_storage(flash);
    let log = FlashLog::mount(&mut region).map_err(|e| match e {
        flashlog::Error::RegionTooSmall => "log partition smaller than two sectors",
        flashlog::Error::TooLarge => "record too large for a sector",
        flashlog::Error::Flash(_) => "flash read/write error on the log partition",
    })?;
    Ok(log.sectors())
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

/// Push any log summaries the subscriber hasn't been sent yet, oldest first.
/// Stops (without advancing the cursor) as soon as a notification fails,
/// which is how "nobody is subscribed" shows up — so the next tick picks up
/// where this one left off.
async fn stream_new_summaries(
    store: &'static AsyncMutex<NoopRawMutex, FlashStore>,
    rec_buf: &mut [u8],
    rec: &mut msg::RepRecord,
    batch: &mut AllocVec<msg::RepSummary>,
    summary_char: &Characteristic<BleVec<SYNC_MSG_MAX>>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    cursor: &mut SummaryCursor,
) {
    // Collect the next batch, then notify them one at a time: notifications are
    // async, so they cannot happen inside the log's sync iteration.
    let total = {
        let mut guard = store.lock().await;
        collect_summaries(&mut guard, rec_buf, rec, cursor.next(), batch)
    };
    let Some(total) = total else {
        return;
    };
    // Reps fall off the ring as it wraps; never point past the end.
    cursor.clamp_to(total);

    let mut payload: BleVec<SYNC_MSG_MAX> = BleVec::new();
    for summary in batch.iter() {
        // Encoded straight into the notification buffer: one buffer instead of a
        // scratch and a copy, which matters because this frame is a future. A
        // summary too large to fit here would not have fitted the characteristic
        // either, so nothing is lost by failing at the encode.
        //
        // A `RepSummary` carries a uuid container now, so it is not `Copy`.
        payload.clear();
        if payload.resize(SYNC_MSG_MAX, 0).is_err() {
            return;
        }
        let Some(n) = sync::encode_summary_message(summary.clone(), &mut payload[..]) else {
            return;
        };
        payload.truncate(n);
        if summary_char.notify(conn, &payload).await.is_err() {
            return; // nobody subscribed (yet)
        }
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

/// Read a requested rep's trace back from the log and notify it to the app in
/// MTU-sized chunks.
///
/// The one frame in this firmware that is knowingly over the crate's
/// `clippy::large_stack_frames` threshold, hence the exemption. It is 1363 bytes
/// of which the largest single local is the 193-byte encode scratch
/// ([`sync::SYNC_CHUNK_MAX`], the worst case for three samples); the rest is the
/// inlined MIR of the encoder and the log calls. There is no buffer left to move
/// to the heap — extracting the log work is already done (see
/// [`read_trace_from_log`]) — and suppressing inlining across `calypso-core` to
/// satisfy the lint would trade a bounded, once-per-trace cost for a real one.
#[allow(
    clippy::large_stack_frames,
    reason = "1363 B: a 193 B encode scratch plus the inlined encoder/log frames"
)]
async fn stream_trace(
    store: &'static AsyncMutex<NoopRawMutex, FlashStore>,
    rec_buf: &mut [u8],
    rec: &mut msg::RepRecord,
    trace_storage: &mut [TracePoint],
    chunk_char: &Characteristic<BleVec<SYNC_MSG_MAX>>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    request: msg::TraceRequest,
) {
    let Some(rep_uuid) = calypso_core::pb::uuid_from_field(&request.rep_uuid) else {
        warn!("trace request with a malformed uuid");
        return;
    };
    let mut trace = TraceBuf::new(trace_storage);
    {
        let mut guard = store.lock().await;
        // The log work stays out of this future: see `collect_summaries`.
        if !read_trace_from_log(&mut guard, rec_buf, rec, rep_uuid, &mut trace) {
            warn!("trace for rep {} is no longer in the log", UuidShort(Some(&rep_uuid)));
            return;
        }
    }

    let t0_ms = trace.get(0).map(|p| p.t_ms).unwrap_or(0);
    let points = trace.as_slice();
    let mut streamer = if request.all_chunks {
        TraceStreamer::all(rep_uuid, t0_ms, points)
    } else {
        TraceStreamer::one(rep_uuid, t0_ms, points, request.chunk_index)
    };

    let mut scratch = [0u8; sync::SYNC_CHUNK_MAX];
    let mut payload: BleVec<SYNC_MSG_MAX> = BleVec::new();
    while let Some(n) = streamer.next_message(&mut scratch) {
        payload.clear();
        if payload.extend_from_slice(&scratch[..n]).is_err() {
            return;
        }
        if chunk_char.notify(conn, &payload).await.is_err() {
            warn!("trace stream interrupted (unsubscribed?)");
            return;
        }
    }
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
fn store_rep(
    store: &mut FlashStore,
    rec_buf: &mut [u8],
    rep_uuid: RepUuid,
    summary: &calypso_core::rep::RepSummary,
    trace: &TraceBuf<'_>,
) -> Result<u32, ()> {
    let (flash, pt_mem) = store.split();
    let pt = partitions::read_partition_table(flash, pt_mem).map_err(|_| ())?;
    let Some(log_partition) = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Undefined,
        ))
        .map_err(|_| ())?
    else {
        return Err(());
    };
    let mut region = log_partition.as_embedded_storage(flash);
    let mut log = FlashLog::mount(&mut region).map_err(|_| ())?;
    store::append_rep(&mut log, rec_buf, rep_uuid, summary, trace.as_slice()).map_err(|_| ())
}

/// Load the BLE device name from the NVS partition, falling back to the default.
///
/// Blob layout at [`BLE_DEVICE_NAME_ADDR`]: 1 length byte followed by up to
/// [`DEVICE_NAME_MAX`] name bytes, padded to a 4-byte boundary.
fn load_device_name(store: &mut FlashStore) -> HeaplessString<DEVICE_NAME_MAX> {
    let (flash, pt_mem) = store.split();
    let mut fallback: HeaplessString<DEVICE_NAME_MAX> = HeaplessString::new();
    let _ = fallback.push_str(DEFAULT_DEVICE_NAME);

    let Ok(pt) = partitions::read_partition_table(flash, pt_mem) else {
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
fn save_device_name(store: &mut FlashStore, name: &str) {
    let (flash, pt_mem) = store.split();
    let Ok(pt) = partitions::read_partition_table(flash, pt_mem) else {
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
    match bno055.set_calibration_profile(profile, &mut delay) {
        Ok(()) => info!("Stored IMU calibration applied"),
        Err(_) => warn!("BNO055 rejected the stored calibration profile"),
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
    let pt = partitions::read_partition_table(flash, pt_mem).ok()?;
    let nvs = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Nvs,
        ))
        .ok()??;
    let mut nvs_partition = nvs.as_embedded_storage(flash);

    let mut blob = [0u8; BNO055_CALIB_SIZE];
    nvs_partition.read(BNO055_CALIBRATION_ADDR, &mut blob).ok()?;
    if blob.iter().all(|&b| b == 0xFF) {
        return None;
    }
    Some(BNO055Calibration::from_buf(&blob))
}

/// Persist the IMU's calibration profile, so the next power cycle starts
/// calibrated instead of putting the user through it again.
///
/// Returns whether the write succeeded. The profile lives at offset 0 of the NVS
/// partition's first sector (the device name is in the second, see
/// [`BLE_DEVICE_NAME_ADDR`]), and that sector is erased first: NOR flash bits only
/// go from 1 to 0 when written, so a second save over an older profile would AND
/// the two together.
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

    if nvs_partition
        .erase(BNO055_CALIBRATION_ADDR, BNO055_CALIBRATION_ADDR + 0x1000)
        .is_err()
    {
        warn!("NVS: calibration erase failed");
        return false;
    }
    if Storage::write(&mut nvs_partition, BNO055_CALIBRATION_ADDR, profile.as_bytes()).is_err()
    {
        warn!("NVS: calibration write failed");
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

    // Note this is `is_ready`, not `is_fully_calibrated`: the magnetometer is
    // unused in IMU mode and its level never reaches 3 there.
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

/// Tell the app the IMU's calibration state, on the summary characteristic.
///
/// The same characteristic summaries stream on: `SyncMessage` exists to
/// multiplex device -> app messages, and the app is already subscribed to this
/// one (see `proto/calypso.proto`). Best-effort — a failed notification means
/// nobody is subscribed, which is how the summary stream reads it too, and the
/// next tick tries again.
async fn notify_calibration(
    summary_char: &Characteristic<BleVec<SYNC_MSG_MAX>>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
) {
    let calibration = Calibration::from_bits(CALIBRATION_BITS.load(Ordering::Relaxed));
    let mut scratch = [0u8; SYNC_MSG_MAX];
    let Some(n) = sync::encode_calibration_message(&calibration, &mut scratch) else {
        return;
    };
    let mut payload: BleVec<SYNC_MSG_MAX> = BleVec::new();
    if payload.extend_from_slice(&scratch[..n]).is_err() {
        return;
    }
    // Not `let _ =`: this is sent once a second and the whole point of it is that a
    // screen somewhere shows it. "The hint never appeared" and "the hint was never
    // sent" look identical from the app, so the device says which it was.
    match summary_char.notify(conn, &payload).await {
        Ok(()) => {
            if !CALIBRATION_DELIVERED.swap(true, Ordering::Relaxed) {
                info!("calibration status delivered to the app ({} bytes)", n);
            }
        }
        // One line a second while nobody is subscribed: noisy, but the alternative
        // is a UI that quietly shows nothing.
        Err(_) => warn!("calibration notify failed (no subscriber?)"),
    }
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

/// Bytes of stack used so far, from the bottom of the main stack.
///
/// Taken from the address of a local, so it is the depth we are really at. On a
/// device with no debugger this is how the stack getting full tells you — nothing
/// else does until the guard word is written and the CPU faults. The total is
/// whatever the linker script left below `_stack_start`.
fn stack_used() -> usize {
    let probe = 0u8;
    let here = core::ptr::addr_of!(probe) as usize;
    // Defined by the linker script (`esp-hal`'s `ld/sections/stack.x`): the
    // lowest address the stack can grow to.
    unsafe extern "C" {
        static _stack_end: u8;
    }
    let bottom = core::ptr::addr_of!(_stack_end) as usize;
    here.saturating_sub(bottom)
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

fn get_level_sense(bno055: &mut Bno055<I2c<'_, Blocking>>) -> f32 {
    match bno055.gravity() {
        Ok(grav) => {
            // info!("Gravity vector: x={} y={} z={}", grav.x, grav.y, grav.z);
            if calypso_core::is_level_along_axis(grav.x) {
                info!("Sensor is level on the X axis, gravity only on Y/Z");
            } else {
                info!("Sensor is not level: X-grav={}", grav.x);
            }
            grav.x
        }
        Err(_e) => {
            warn!("BNO055 gravity read error");
            0.0
        }
    }
}
