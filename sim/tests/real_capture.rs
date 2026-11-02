//! Tests against real motion: reps a device recorded, kept as fixtures.
//!
//! A synthetic rep is a shape somebody chose; these are the shapes a bar actually made, with the sensor's own
//! noise, bias and mounting on them. The fixtures live with the app's, because a fixture is for the app as much as
//! for here.
//!
//! **What a stored window can and cannot say.** It can say that the device stored this rep, that its trace is
//! whole, and what the summary's numbers are. It cannot be replayed: the trace keeps specific force and an
//! attitude, not the gyroscope the filter integrates, so a window is enough to plot and not enough to filter. A
//! reader that wants a summary re-derived needs a full-rate capture with `RepState` beside it, which is what a
//! fresh session produces and the archive cannot be made to.

use std::fs;
use std::path::PathBuf;

use calypso_core::pb::{self, msg, Record};
use calypso_core::rep::{TracePoint, MAX_REP_MS, MIN_ROM_M};
use calypso_sim::fixtures::read_dump;

fn fixture(name: &str) -> Vec<u8> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../calypso-app/assets/fixtures")
        .join(format!("{name}.pblog"));
    fs::read(&path).unwrap_or_else(|e| panic!("reading {}: {e}", path.display()))
}

/// One rep out of a fixture: its summary, its trace as the device stored it, and the epoch that dates it.
struct RealRep {
    summary: msg::RepSummary,
    points: Vec<TracePoint>,
    /// Unix seconds at the boot's uptime zero, from the newest clock record before the summary.
    epoch_s: u32,
}

/// Read one fixture, checking as it goes that it is a dump a reader can trust: it ends on a record, every record
/// decodes, the trace is complete, and the rep is behind the clock record that dates it.
fn real_rep(name: &str) -> RealRep {
    let bytes = fixture(name);
    let (records, truncated) = read_dump(&bytes);
    assert!(!truncated, "{name}: a fixture should end on a record");

    let mut summary: Option<msg::RepSummary> = None;
    let mut chunks: Vec<msg::TraceChunk> = Vec::new();
    let mut epoch_s = 0u32;
    for record in &records {
        match pb::decode_record(record).expect("a fixture record decodes") {
            Record::Clock(c) => {
                // The newest clock at or before the records that follow it; a clock at 0 is a boot opening, not a
                // time, so it leaves the reps after it dateless (`docs/FIXTURES.md`).
                epoch_s = c.epoch_s;
            }
            Record::Summary(s) => {
                assert!(summary.is_none(), "{name}: one rep per fixture");
                summary = Some(s);
            }
            Record::Trace(c) => chunks.push(c),
        }
    }

    let summary = summary.unwrap_or_else(|| panic!("{name}: no summary"));
    chunks.sort_by_key(|c| c.index);
    assert_eq!(
        chunks.len(),
        summary.chunk_count as usize,
        "{name}: the summary says {} chunks",
        summary.chunk_count
    );
    for (i, chunk) in chunks.iter().enumerate() {
        assert_eq!(chunk.index, i as u32, "{name}: chunk {i} is out of order");
        assert_eq!(
            pb::chunk_uuid(chunk),
            pb::summary_uuid(&summary),
            "{name}: chunk {i} belongs to another rep"
        );
    }

    let points: Vec<TracePoint> = chunks
        .iter()
        .flat_map(|c| {
            c.samples.iter().map(|s| TracePoint {
                t_ms: s.t_offset_ms,
                pos: [s.px, s.py, s.pz],
                accel: [s.ax as i16, s.ay as i16, s.az as i16],
                quat: [s.qw as i16, s.qx as i16, s.qy as i16, s.qz as i16],
            })
        })
        .collect();
    assert_eq!(
        points.len(),
        summary.trace_len as usize,
        "{name}: the trace is incomplete"
    );

    RealRep {
        summary,
        points,
        epoch_s,
    }
}

#[test]
fn a_real_deadlift_is_a_rep() {
    let rep = real_rep("real-deadlift");
    let s = &rep.summary;

    assert!(s.is_a_rep(), "a recorded rep was rejected: {} m", s.rom);
    assert!(
        s.rom > MIN_ROM_M,
        "{} m of travel, under the {MIN_ROM_M} m a rep needs",
        s.rom
    );
    assert!(
        s.duration_ms <= MAX_REP_MS,
        "{} ms is past the {MAX_REP_MS} ms bound",
        s.duration_ms
    );
    assert!(
        s.decimated(),
        "a 5.7 s window does not fit the trace buffer at full rate"
    );
    println!(
        "deadlift: {} ms, rom {:.3} m, {} points, peak v {:.2} m/s, peak a {:.1} m/s², flags {:#x}",
        s.duration_ms, s.rom, s.trace_len, s.peak_velocity, s.peak_accel, s.flags
    );
}

/// The window's own timeline: monotonic, and every step a step the device can take. This is the property that makes
/// a window usable at all — the capture this fixture's predecessors came from had jumps of 2 s inside a rep
/// (`docs/GOTCHAS.md`), and a trace whose times jump describes motion it never measured.
#[test]
fn a_real_window_has_a_timeline() {
    let rep = real_rep("real-deadlift");
    let t: Vec<u32> = rep.points.iter().map(|p| p.t_ms).collect();
    assert!(
        t.windows(2).all(|w| w[1] > w[0]),
        "the window's times must ascend: {t:?}"
    );
    let widest = t.windows(2).map(|w| w[1] - w[0]).max().unwrap_or(0);
    let narrowest = t.windows(2).map(|w| w[1] - w[0]).min().unwrap_or(0);
    assert!(
        widest <= 50,
        "a gap of {widest} ms inside a stored window is motion nobody measured"
    );
    assert!(narrowest >= 4, "a step of {narrowest} ms is a burst");
    println!(
        "timeline: {} points over {} ms, steps {narrowest}-{widest} ms",
        t.len(),
        t[t.len() - 1] - t[0]
    );
}

/// The fixture carries its own time: the rep's uptime plus its boot's epoch is the instant it was lifted, which is
/// what the app shows in place of a UUID. The date is pinned because a fixture whose date depends on when it is
/// read is not a fixture.
#[test]
fn a_real_rep_carries_the_time_it_was_lifted() {
    let rep = real_rep("real-deadlift");
    assert_ne!(rep.epoch_s, 0, "the fixture carries no clock");
    let unix_ms = u64::from(rep.epoch_s) * 1000 + u64::from(rep.summary.t_start_ms);
    assert_eq!(
        unix_ms / 1000,
        1_793_478_656,
        "2026-10-31T20:30:56Z, the session the rep came from"
    );
    println!(
        "lifted {} ms into that second, {} ms into the boot",
        unix_ms % 1000,
        rep.summary.t_start_ms
    );
}
