//! Tests against reps a real device recorded, kept as fixtures.
//!
//! These are a capture of one session with everything but a few reps dropped (`examples/extract_capture.rs`), so they
//! carry the acceleration, the timestamps and the shape a synthetic generator only imitates. The fixtures live with the
//! app's, because a fixture is for the app and these are as much for it as for here.

use std::fs;
use std::path::PathBuf;

use calypso_core::pb::{decode_record, Record};
use calypso_sim::fixtures::read_dump;

fn fixture(name: &str) -> Vec<u8> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../calypso-app/assets/fixtures")
        .join(format!("{name}.pblog"));
    fs::read(&path).unwrap_or_else(|e| panic!("reading {}: {e}", path.display()))
}

/// The summaries and their trace points, in file order.
fn reps(name: &str) -> Vec<(calypso_core::pb::msg::RepSummary, usize)> {
    let bytes = fixture(name);
    let (records, truncated) = read_dump(&bytes);
    assert!(!truncated, "{name}: a fixture should end on a record");

    let mut out: Vec<(calypso_core::pb::msg::RepSummary, usize)> = Vec::new();
    for record in &records {
        match decode_record(record).expect("a fixture record decodes") {
            Record::Summary(s) => out.push((s, 0)),
            Record::Trace(c) => {
                if let Some((_, points)) = out.iter_mut().find(|(s, _)| s.rep_uuid == c.rep_uuid) {
                    *points += c.samples.len();
                }
            }
            Record::Clock(_) => {}
        }
    }
    out
}

#[test]
fn a_real_rep_is_a_rep() {
    let reps = reps("real-reps");
    assert_eq!(reps.len(), 3, "three reps were kept, not {}", reps.len());
    for (s, points) in &reps {
        assert!(s.is_a_rep(), "a recorded rep was rejected: {} m", s.rom);
        assert_eq!(
            *points, s.trace_len as usize,
            "the trace is incomplete: {points} of {}",
            s.trace_len
        );
        assert!(s.t_unix_ms != 0, "a real rep should carry its time");
    }
    // The three were chosen to be different shapes: the shortest drive, the slowest, and the longest.
    let mut durations: Vec<u32> = reps.iter().map(|(s, _)| s.duration_ms).collect();
    durations.sort_unstable();
    assert_eq!(durations, vec![1369, 5112, 8220]);
    let roms: Vec<f32> = reps.iter().map(|(s, _)| s.rom).collect();
    assert!(roms.iter().all(|r| *r > 0.2 && *r < 0.7), "{roms:?}");
}

#[test]
fn a_real_wobble_is_not_a_rep() {
    let reps = reps("real-wobble");
    assert_eq!(reps.len(), 1);
    let (s, points) = &reps[0];
    assert!(s.t_unix_ms != 0, "the wobble carries its time");
    assert_eq!(*points, s.trace_len as usize);
    // The motion is real enough to close a span — four times the length of the shortest drive — and the sensor went
    // 3 cm. This is the case `MIN_ROM_M` exists for, from the session that produced it.
    assert!(s.duration_ms > 3000, "{} ms", s.duration_ms);
    assert!(!s.is_a_rep(), "{} m of wobble was accepted", s.rom);
    assert!(s.rom < calypso_core::rep::MIN_ROM_M);
}
