//! Say what is in a raw dump of the device's `log` partition, without booting the firmware.
//!
//! Usage: `cargo run --example device_log -- <image.bin>`
//!
//! The image is what the bootloader hands back for the partition:
//! `espflash read-flash 0x190000 0x270000 <image.bin>` (the offsets are `calypso-esp32/partitions.csv`).
//!
//! This mounts the image with the firmware's own log and walks it with the firmware's own record decoder, so what it counts is what the device would serve — including the records a torn write or a bad CRC makes unreadable, which a reader of the raw bytes would happily count. Mounting can erase — a head sector whose tail is corrupt sends the ring on to the next one — but that happens to this copy in memory, never to the device.

use std::env;
use std::fs;

use calypso_core::flashlog::FlashLog;
use calypso_core::fusion::{magnitude, roll_pitch_deg};
use calypso_core::pb::msg;
use calypso_core::pb::summary_uuid;
use calypso_core::rep::{TraceBuf, TracePoint};
use calypso_core::sampler::QUAT_SCALE;
use calypso_core::store::{self, RECORD_BUF_BYTES};
use calypso_core::uuid::RepUuid;
use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

#[derive(Debug)]
struct ImageError;

impl NorFlashError for ImageError {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::OutOfBounds
    }
}

/// The dump, as flash: reads are reads, and the two writes the mount scan may make are allowed to land.
struct Image {
    data: Vec<u8>,
}

impl ErrorType for Image {
    type Error = ImageError;
}

impl ReadNorFlash for Image {
    const READ_SIZE: usize = 4;

    fn capacity(&self) -> usize {
        self.data.len()
    }

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let start = offset as usize;
        let end = start + bytes.len();
        if end > self.data.len() {
            return Err(ImageError);
        }
        bytes.copy_from_slice(&self.data[start..end]);
        Ok(())
    }
}

impl NorFlash for Image {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = 4096;

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let start = offset as usize;
        for (i, &b) in bytes.iter().enumerate() {
            self.data[start + i] &= b; // NOR flash: a write can only clear bits
        }
        Ok(())
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.data[from as usize..to as usize].fill(0xFF);
        Ok(())
    }
}

/// Something to write besides the report on screen.
#[derive(Debug)]
enum Output {
    None,
    /// Each rep's trace as `t_ms, x, y, z, ax, ay, az`, so a plot is of what the device stored.
    Csv(String),
    /// The reps as a `.pblog`, the format the app's fixture reader takes, so a rep the device recorded can become a fixture. `prefix` is a hex uuid prefix, or every rep when absent.
    Dump {
        path: String,
        prefix: Option<String>,
    },
}

fn die(why: &str) -> ! {
    eprintln!("{why}");
    eprintln!("usage: device_log <image.bin> [--csv <path>] [--dump <path> [uuid-prefix]]");
    std::process::exit(2);
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    let Some(path) = args.first() else {
        die("no image");
    };

    let mut write = Output::None;
    let mut rest = args.iter().skip(1);
    while let Some(arg) = rest.next() {
        match arg.as_str() {
            "--csv" => match rest.next() {
                Some(p) => write = Output::Csv(p.clone()),
                None => die("--csv wants a path"),
            },
            "--dump" => {
                let Some(p) = rest.next() else {
                    die("--dump wants a path")
                };
                let prefix = rest
                    .next()
                    .filter(|a| !a.starts_with("--"))
                    .map(|a| a.to_lowercase());
                write = Output::Dump {
                    path: p.clone(),
                    prefix,
                };
            }
            other => die(&format!("unknown argument {other}")),
        }
    }
    let csv = match &write {
        Output::Csv(p) => Some(p.clone()),
        _ => None,
    };

    let bytes = fs::read(path).expect("the image reads");
    let mut flash = Image { data: bytes };
    println!(
        "{path}: {} bytes, {} sectors",
        flash.data.len(),
        flash.data.len() / 4096
    );

    let mut log = match FlashLog::mount(&mut flash) {
        Ok(log) => log,
        Err(e) => {
            eprintln!("the image does not mount as a Calypso log: {e:?}");
            std::process::exit(1);
        }
    };
    println!(
        "log: {} sectors, next record id {}",
        log.sectors(),
        log.next_id()
    );

    let mut buf = vec![0u8; RECORD_BUF_BYTES];
    let mut rec = msg::RepRecord::default();
    let mut records = 0usize;
    let mut clocks = 0usize;
    let mut summaries = 0usize;
    let mut chunks = 0usize;
    // The epoch of the boot the walk is in: a rep's time is this boot's epoch plus its uptime, the rule a reader of a dump applies.
    let mut epoch_s: u32 = 0;
    let mut boots = 0usize;
    // Every rep's uuid, so the traces can be read back once the walk is done with the log.
    let mut reps: Vec<(RepUuid, String)> = Vec::new();

    // A boot's records start at its mount clock, so a clock at 0 is where a boot begins and the newest epoch after it dates the reps that follow. Counting clock records as boots would count each boot twice over: the client's set is a second record in the same boot.
    println!("\nlog, oldest first:");
    println!("  {:<6} {:<8} detail", "id", "what");
    store::for_each_record(&mut log, &mut buf, &mut rec, |id, record| {
        records += 1;
        match &record.body {
            Some(msg::RepRecord_::Body::Clock(c)) => {
                clocks += 1;
                epoch_s = c.epoch_s;
                if c.epoch_s == 0 {
                    boots += 1;
                    println!("  {id:<6} {:<8} boot {boots} begins", "clock");
                } else {
                    println!(
                        "  {id:<6} {:<8} boot {boots} clocked {epoch_s} ({})",
                        "clock",
                        utc(u64::from(c.epoch_s) * 1000)
                    );
                }
            }
            Some(msg::RepRecord_::Body::Summary(s)) => {
                summaries += 1;
                if let Some(uuid) = summary_uuid(s) {
                    reps.push((uuid, hex(&s.rep_uuid)));
                }
                let stamp = if epoch_s == 0 {
                    "no clock".to_string()
                } else {
                    utc(u64::from(epoch_s) * 1000 + u64::from(s.t_start_ms))
                };
                // The velocity the window is replayed from: `compute_rep` integrates it forward, so a window whose state already carries speed reads travel that the bar never did.
                let v0 = magnitude([s.state.vx, s.state.vy, s.state.vz]);
                println!(
                    "  {id:<6} {:<8} {} {} ms  rom {:.2} m  v0 {:.1} m/s  {} pts{}  {stamp}",
                    "rep",
                    hex(&s.rep_uuid),
                    s.duration_ms,
                    s.rom,
                    v0,
                    s.computed_len,
                    if s.raw_len == 0 {
                        String::new()
                    } else {
                        format!(", {} raw", s.raw_len)
                    }
                );
            }
            Some(msg::RepRecord_::Body::Computed(_)) => {
                chunks += 1;
            }
            Some(msg::RepRecord_::Body::Raw(_)) => {}
            None => {}
        }
    })
    .expect("the walk reads");

    println!(
        "\n{records} records: {summaries} summaries, {chunks} trace chunks, {clocks} clock readings, {boots} boots"
    );
    println!("=> {summaries} reps stored on the device");

    // Read each trace back through the firmware's own reader, and say how evenly it is sampled: a rep whose window has a hole in it is a window whose motion was not measured, and that shows up as a wide step between two points rather than anywhere in the summary.
    let mut points = [TracePoint::ZERO; 512];
    for (uuid, prefix) in &reps {
        let mut out = TraceBuf::new(&mut points);
        let n = match store::read_trace(&mut log, &mut buf, &mut rec, *uuid, &mut out) {
            Ok(n) => n,
            Err(e) => {
                println!("  trace {prefix}: unreadable ({e:?})");
                continue;
            }
        };
        let mut widest = 0u32;
        let mut narrowest = u32::MAX;
        let mut backwards = 0usize;
        for i in 1..n {
            let (a, b) = (out.get(i - 1).unwrap().t_ms, out.get(i).unwrap().t_ms);
            if b < a {
                backwards += 1;
            } else {
                widest = widest.max(b - a);
                narrowest = narrowest.min(b - a);
            }
        }
        let span = if n > 1 {
            out.get(n - 1).unwrap().t_ms - out.get(0).unwrap().t_ms
        } else {
            0
        };
        if let Some(path) = &csv {
            let mut out_text = format!("# {prefix}\n");
            for i in 0..n {
                let p = out.get(i).unwrap();
                // Time, position, and the two angles the attitude implies. The plotted trace carries no raw reading; that is the rep's raw trace, which only a dev build records.
                let (roll, pitch) = roll_pitch_deg(p.quat.map(|q| q as f32 / QUAT_SCALE as f32));
                out_text.push_str(&format!(
                    "{}, {:.4}, {:.4}, {:.4}, {:.1}, {:.1}\n",
                    p.t_ms, p.pos[0], p.pos[1], p.pos[2], roll, pitch
                ));
            }
            if let Err(e) = std::fs::write(path, out_text) {
                println!("  csv: {e}");
            }
        }
        println!(
            "  trace {prefix}: {n} points over {span} ms, steps {narrowest}-{widest} ms{}",
            if backwards > 0 {
                format!(", {backwards} out of order")
            } else {
                String::new()
            }
        );
    }

    if let Output::Dump { path, prefix } = &write {
        dump(&mut log, &mut buf, path, prefix.as_deref());
    }
}

/// Write the chosen reps as a `.pblog`: each one's summary and its chunks, in log order, behind the clock record that dates them.
///
/// The clock is not optional. A stored summary carries its uptime, not a time (`proto/calypso.proto`), so a fixture without its boot's epoch is a rep that cannot be dated — and the app resolves the time from the records in front of it, not from the file's name.
fn dump(log: &mut FlashLog<Image>, buf: &mut [u8], path: &str, prefix: Option<&str>) {
    let mut out: Vec<u8> = Vec::new();
    // The newest clock that dates the reps that follow it. A clock at 0 is a boot opening, not a time.
    let mut clock: Option<Vec<u8>> = None;
    // The reps chosen so far, so their chunks can be recognised as they come past.
    let mut chosen: Vec<Vec<u8>> = Vec::new();

    log.for_each(buf, |_, payload| {
        match calypso_core::pb::decode_record(payload) {
            Ok(calypso_core::pb::Record::Clock(c)) => {
                if c.epoch_s != 0 {
                    clock = Some(payload.to_vec());
                }
            }
            Ok(calypso_core::pb::Record::Summary(s)) => {
                let uuid = calypso_core::pb::summary_uuid(&s).map(|u| u.to_vec());
                let wanted = prefix.is_none_or(|p| hex(&s.rep_uuid).starts_with(p));
                if !wanted {
                    return;
                }
                if let Some(clock) = &clock {
                    out.extend_from_slice(&(clock.len() as u32).to_le_bytes());
                    out.extend_from_slice(clock);
                }
                out.extend_from_slice(&(payload.len() as u32).to_le_bytes());
                out.extend_from_slice(payload);
                if let Some(uuid) = uuid {
                    chosen.push(uuid);
                }
            }
            Ok(calypso_core::pb::Record::Computed(c)) => {
                let uuid = calypso_core::pb::computed_uuid(&c).map(|u| u.to_vec());
                if uuid.is_some_and(|u| chosen.contains(&u)) {
                    out.extend_from_slice(&(payload.len() as u32).to_le_bytes());
                    out.extend_from_slice(payload);
                }
            }
            Ok(calypso_core::pb::Record::Raw(r)) => {
                let uuid = calypso_core::pb::raw_uuid(&r).map(|u| u.to_vec());
                if uuid.is_some_and(|u| chosen.contains(&u)) {
                    out.extend_from_slice(&(payload.len() as u32).to_le_bytes());
                    out.extend_from_slice(payload);
                }
            }
            Err(e) => println!("  dump: a record does not decode: {e:?}"),
        }
    })
    .expect("the dump walks");

    if chosen.is_empty() {
        die(&match prefix {
            Some(p) => format!("no rep matched the prefix {p}"),
            None => "the log holds no reps to dump".to_string(),
        });
    }
    fs::write(path, &out).expect("the dump writes");
    println!("\n{path}: {} reps, {} bytes", chosen.len(), out.len());
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().take(4).map(|b| format!("{b:02x}")).collect()
}

/// Unix milliseconds as `YYYY-MM-DD HH:MM:SSZ`, so a stamp can be checked against a wall clock by eye.
fn utc(ms: u64) -> String {
    let secs = ms / 1000;
    let (days, rem) = (secs / 86_400, secs % 86_400);
    let (h, m, s) = (rem / 3600, rem % 3600 / 60, rem % 60);
    // Days since the epoch to a civil date, the century-shifting form of Howard Hinnant's algorithm.
    let z = days as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let mo = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(mo <= 2);
    format!("{y:04}-{mo:02}-{d:02} {h:02}:{m:02}:{s:02}Z")
}
