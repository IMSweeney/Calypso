//! Tests against real motion: reps a device recorded, kept as fixtures.
//!
//! A synthetic rep is a shape somebody chose; these are the shapes a bar actually made, with the sensor's own
//! noise, bias and mounting on them. The fixtures live with the app's, because a fixture is for the app as much as
//! for here.
//!
//! **What a stored window can and cannot say.** It can say that the device stored this rep, that its trace is
//! whole, and what the summary's numbers are. It cannot be replayed as a stream: the window is the motion span with
//! `MARGIN_MS` *requested* before it and whatever had been sampled after it — the tail is cut at the close, so the
//! margin there is a few samples at most. On this rep even the 200 ms before the onset are inside motion (the first
//! sample carries 7.0 m/s², against a 0.6 m/s² on-threshold), and the last carries 1.1, against a 0.25 m/s² rest
//! threshold. Neither end is rest, so a detector fed this window cold has nothing to anchor on and nothing to close
//! against — which is why nothing here asserts what a replay finds. `examples/replay_dump.rs` labels every window
//! it is given with the timeline it found, and the timeline is the half of this that *is* checkable from a file.

use std::fs;
use std::path::PathBuf;

use calypso_core::pb::{self, msg, Record};
use calypso_core::rep::{compute_rep, TraceBuf, TracePoint, MAX_REP_MS, MIN_ROM_M};
use calypso_core::sampler::Sample;
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

impl RealRep {
    /// The trace as the samples the detector would have been fed: the stored points are what `compute_rep` kept of
    /// them, so this is not the stream, only what the device still knows of it.
    fn samples(&self) -> Vec<Sample> {
        self.points
            .iter()
            .map(|p| Sample {
                t_ms: p.t_ms,
                accel: p.accel,
                quat: p.quat,
            })
            .collect()
    }

    fn v0(&self) -> [f32; 3] {
        let v = &self.summary.v0;
        [v.vx, v.vy, v.vz]
    }
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

/// The trace maths re-reads the same window from the bytes the device kept: the summary's `v0` and its own trace,
/// integrated again over real motion rather than over a cosine.
///
/// The window, though, is not the sample set the summary was computed from: the trace keeps every second sample, so
/// the same integration over it reads **1.10 m against the 0.94 m stored** — and every second sample again reads
/// 1.42 m. Travel rises steeply as the step coarsens on this motion, which is what makes the summary a function of
/// the rate it was read at rather than of the window alone. The tolerance is loose enough for that and tight enough
/// to catch a maths change; the duration and the point count, which do not depend on the decimation, are pinned
/// hard.
#[test]
fn the_stored_trace_recomputes_the_stored_window() {
    let rep = real_rep("real-deadlift");
    let samples = rep.samples();
    let mut storage = [TracePoint::ZERO; 512];
    let mut trace = TraceBuf::new(&mut storage);
    let again = compute_rep(&samples, rep.v0(), &mut trace);

    assert_eq!(trace.len(), rep.summary.trace_len as usize);
    assert!(
        again.duration_ms.abs_diff(rep.summary.duration_ms) <= 20,
        "the recomputed span is {} ms against {}",
        again.duration_ms,
        rep.summary.duration_ms
    );
    assert!(
        again.is_a_rep(),
        "{} m of travel was read from a rep that stored {} m",
        again.rom,
        rep.summary.rom
    );
    assert!(
        (again.rom - rep.summary.rom).abs() < rep.summary.rom * 0.25,
        "the recomputed travel {:.3} m is far from the stored {:.3} m",
        again.rom,
        rep.summary.rom
    );
    println!(
        "recomputed: rom {:.3} m (stored {:.3}), duration {} ms (stored {})",
        again.rom, rep.summary.rom, again.duration_ms, rep.summary.duration_ms
    );
}
