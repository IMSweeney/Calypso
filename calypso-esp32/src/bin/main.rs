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
use calypso_core::flashlog::FlashLog;
use calypso_core::pb::msg;
use calypso_core::sync::{self, decode_trace_request, SummaryCursor, TraceStreamer};
use calypso_core::uuid::{self, RepUuid};
use calypso_core::rep::{compute_rep, Detector, TraceBuf, TracePoint, MARGIN_MS};
use calypso_core::store::{self, RECORD_BUF_BYTES};
use calypso_core::sampler::{quantize, Sample, SampleRing, QUAT_SCALE, SAMPLE_RATE_HZ};
use calypso_core::DEVICE_NAME_MAX;
use core::sync::atomic::{AtomicU32, Ordering};
use alloc::boxed::Box;

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

/// Ring-buffer window: 30 s at 100 Hz (60 KB). The rep detector (Phase 3)
/// consumes sliding windows from here; BLE trace sync will read it too.
const SAMPLE_RING_LEN: usize = 30 * SAMPLE_RATE_HZ as usize;

/// The sampler's sample window. Lives on the heap (leaked at startup): 60 KB
/// is too much for the DRAM static region, but fits the ~160 KB heap. The
/// sampler task both fills it and runs the rep detector over it; Phase 4/5
/// will additionally read it from the BLE task (behind a mutex).
type SampleWindow = RefCell<SampleRing<SAMPLE_RING_LEN>>;

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

    let mut flash = FlashStorage::new(peripherals.FLASH);
    info!("Flash size = {}", flash.capacity());
    setup_calibration(&mut bno055, &mut flash);

    // Load the persisted BLE device name, falling back to the default.
    let device_name = load_device_name(&mut flash);
    info!("BLE device name: {}", device_name.as_str());

    // Mount the rep-record flash log (see docs/rep-tracking).
    match mount_rep_log(&mut flash) {
        Ok(sectors) => info!("Rep log mounted: {} sectors", sectors),
        Err(()) => warn!("Rep log: partition missing or unusable"),
    }

    // 30 s of samples on the heap, alive for the program's lifetime.
    let samples: &'static SampleWindow = Box::leak(Box::new(RefCell::new(SampleRing::new())));
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
    let mut rec_vec = AllocVec::new();
    rec_vec.resize(RECORD_BUF_BYTES, 0u8);
    let rec_buf: &'static mut [u8] = Box::leak(rec_vec.into_boxed_slice());
    info!("Rep detector ready, heap free {} B", esp_alloc::HEAP.free());

    // The one flash peripheral, shared by the BLE task (device name) and the
    // sampler task (rep log). An async mutex, not a critical section: erasing
    // a sector takes tens of milliseconds and must not block interrupts. All
    // three futures below run in this task, so a no-op raw mutex suffices.
    let flash: &'static AsyncMutex<NoopRawMutex, FlashDevice> =
        Box::leak(Box::new(AsyncMutex::new(flash)));

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
    let mut sync_rec_vec = AllocVec::new();
    sync_rec_vec.resize(RECORD_BUF_BYTES, 0u8);
    let sync_rec_buf: &'static mut [u8] = Box::leak(sync_rec_vec.into_boxed_slice());
    info!("Sync buffers ready, heap free {} B", esp_alloc::HEAP.free());
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
                            let value = f32::from_bits(GRAVITY_X_BITS.load(Ordering::Relaxed));
                            if let Err(_) = gravity_x_char.notify(&conn, &value).await {
                                // No subscriber yet; still fine to keep the
                                // connection (and try again on the next tick).
                            }
                            // Auto-sync: push any summaries the app hasn't
                            // seen. A failed notify means nobody is
                            // subscribed, so we stop and retry next tick.
                            stream_new_summaries(
                                flash,
                                sync_rec_buf,
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
                                if let Ok(request) = server.table().get(&trace_request_char) {
                                    if let Some(request) = decode_trace_request(&request) {
                                        let wanted = calypso_core::pb::uuid_from_field(&request.rep_uuid);
                                        info!("trace request: rep {}", UuidShort(wanted.as_ref()));
                                        stream_trace(
                                            flash,
                                            sync_rec_buf,
                                            &mut *sync_trace_storage,
                                            &trace_chunk_char,
                                            &conn,
                                            request,
                                        )
                                        .await;
                                    }
                                }
                            }
                            if wrote_name {
                                // The write has been applied to the attribute
                                // store by `accept`; read the new name back,
                                // keep it in memory and persist it to flash.
                                if let Ok(new_name) = server.table().get(&name_char) {
                                    if new_name != name {
                                        name = new_name;
                                        info!("device name set to {}", name.as_str());
                                        let mut f = flash.lock().await;
                                        save_device_name(&mut f, name.as_str());
                                    }
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
            loop {
                ticker.next().await;
                let t_ms = start.elapsed().as_millis() as u32;
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
                // compute the ZUPT-anchored trace + summary. Storing the
                // record in the flash log lands in Phase 4.
                if let Some(span) = detector.update(&sample) {
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
                            let mut f = flash.lock().await;
                            store_rep(&mut f, rec_buf, rep_uuid, &sum, &rep_trace)
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
                if n % 10 == 0 {
                    if let Ok(g) = bno055.gravity_fixed() {
                        let x = g.x as f32 / 100.0;
                        GRAVITY_X_BITS.store(x.to_bits(), Ordering::Relaxed);
                    }
                }
                // Level-sense log at 1 Hz, matching the old loop.
                if n % SAMPLE_RATE_HZ == 0 {
                    get_level_sense(&mut bno055);
                    let (len, pushed) = {
                        let r = samples.borrow();
                        (r.len(), r.pushed())
                    };
                    info!(
                        "sampler: {} Hz, ring {}/{} samples, {} pushed, {} reps, heap free {} B",
                        SAMPLE_RATE_HZ,
                        len,
                        SAMPLE_RING_LEN,
                        pushed,
                        reps,
                        esp_alloc::HEAP.free()
                    );
                }
            }
        },
    )
    .await;

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
    loop {}
}

/// Mount the rep-record flash log on the `log` data partition (subtype
/// `undefined`, see partitions.csv). Returns the number of sectors in the
/// ring. Mounting here formats the partition on first boot; individual reps
/// are written later by [`store_rep`].
fn mount_rep_log(flash: &mut FlashStorage) -> Result<u32, ()> {
    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let pt = partitions::read_partition_table(flash, &mut pt_mem).map_err(|_| ())?;
    let Some(log_part) = pt
        .find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Undefined,
        ))
        .map_err(|_| ())?
    else {
        return Err(());
    };
    let mut region = log_part.as_embedded_storage(flash);
    let log = FlashLog::mount(&mut region).map_err(|_| ())?;
    Ok(log.sectors())
}

/// Push any log summaries the subscriber hasn't been sent yet, oldest first.
/// Stops (without advancing the cursor) as soon as a notification fails,
/// which is how "nobody is subscribed" shows up — so the next tick picks up
/// where this one left off.
async fn stream_new_summaries(
    flash: &'static AsyncMutex<NoopRawMutex, FlashDevice<'static>>,
    rec_buf: &mut [u8],
    summary_char: &Characteristic<BleVec<SYNC_MSG_MAX>>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    cursor: &mut SummaryCursor,
) {
    const BATCH: usize = 16;

    // Collect the next batch of summaries from the log, then notify them one
    // at a time (notifications are async, so they can't happen inside the
    // log's sync iteration).
    let mut batch: heapless::Vec<msg::RepSummary, BATCH> = heapless::Vec::new();
    let total;
    {
        let mut guard = flash.lock().await;
        let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let Ok(pt) = partitions::read_partition_table(&mut *guard, &mut pt_mem) else {
            return;
        };
        let Ok(Some(partition)) = pt.find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Undefined,
        )) else {
            return;
        };
        let mut region = partition.as_embedded_storage(&mut *guard);
        let Ok(mut log) = FlashLog::mount(&mut region) else {
            return;
        };
        let skip = cursor.next();
        let mut seen = 0usize;
        let _ = store::for_each_summary(&mut log, rec_buf, |s| {
            if seen >= skip && batch.len() < BATCH {
                let _ = batch.push(s);
            }
            seen += 1;
        });
        total = seen;
    }
    // Reps fall off the ring as it wraps; never point past the end.
    cursor.clamp_to(total);

    let mut scratch = [0u8; SYNC_MSG_MAX];
    let mut payload: BleVec<SYNC_MSG_MAX> = BleVec::new();
    for summary in batch.iter() {
        // A `RepSummary` carries a uuid container now, so it is not `Copy`.
        let Some(n) = sync::encode_summary_message(summary.clone(), &mut scratch) else {
            return;
        };
        payload.clear();
        if payload.extend_from_slice(&scratch[..n]).is_err() {
            return;
        }
        if summary_char.notify(conn, &payload).await.is_err() {
            return; // nobody subscribed (yet)
        }
        cursor.advance(1);
    }
}

/// Read a requested rep's trace back from the log and notify it to the app in
/// MTU-sized chunks.
async fn stream_trace(
    flash: &'static AsyncMutex<NoopRawMutex, FlashDevice<'static>>,
    rec_buf: &mut [u8],
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
        let mut guard = flash.lock().await;
        let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let Ok(pt) = partitions::read_partition_table(&mut *guard, &mut pt_mem) else {
            return;
        };
        let Ok(Some(partition)) = pt.find_partition(partitions::PartitionType::Data(
            partitions::DataPartitionSubType::Undefined,
        )) else {
            return;
        };
        let mut region = partition.as_embedded_storage(&mut *guard);
        let Ok(mut log) = FlashLog::mount(&mut region) else {
            return;
        };
        if store::read_trace(&mut log, rec_buf, rep_uuid, &mut trace)
            .map(|n| n == 0)
            .unwrap_or(true)
        {
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
    flash: &mut FlashDevice,
    rec_buf: &mut [u8],
    rep_uuid: RepUuid,
    summary: &calypso_core::rep::RepSummary,
    trace: &TraceBuf<'_>,
) -> Result<u32, ()> {
    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let pt = partitions::read_partition_table(flash, &mut pt_mem).map_err(|_| ())?;
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
    // IMU mode has no magnetometer, so the mag status never reaches 3 (and
    // `is_fully_calibrated` requires it). Gate on sys/gyro/accel only.
    let imu_ready = |s: &bno055::BNO055CalibrationStatus| s.sys == 3 && s.gyr == 3 && s.acc == 3;
    let is_calibrated = bno055
        .get_calibration_status()
        .map(|s| imu_ready(&s))
        .unwrap_or(false);
    if !is_calibrated {
        info!("Waiting for BNO055 to calibrate");
        loop {
            if let Ok(s) = bno055.get_calibration_status() {
                if imu_ready(&s) {
                    break;
                }
            }
        }
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
