//! Pull chosen reps out of a rep log dump into a dump of their own, so a real capture can be checked in as fixtures.
//!
//! Every record is copied byte for byte: the output is the same bytes the device wrote, in the same order, with the
//! reps nobody asked for left out. Nothing is re-encoded, so a fixture made this way cannot drift from what the
//! firmware stored.
//!
//! Usage: `cargo run --example extract_capture -- <capture.pblog> <out.pblog> <uuid-prefix>...`
//!
//! The prefix is hex, at least four characters (`0f64b98f`), and every one given has to match a rep or this fails:
//! a fixture that silently lost the rep it was named for is worse than no fixture.

use std::env;
use std::fs;

use calypso_core::pb::{decode_record, Record};
use calypso_sim::fixtures::read_dump;

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 3 {
        eprintln!("usage: extract_capture <capture.pblog> <out.pblog> <uuid-prefix>...");
        std::process::exit(2);
    }
    let (capture, out) = (&args[0], &args[1]);
    let wanted: Vec<String> = args[2..].iter().map(|p| p.to_lowercase()).collect();

    let bytes = fs::read(capture).expect("the capture reads");
    let (records, truncated) = read_dump(&bytes);
    if truncated {
        eprintln!("{capture}: torn at the end, the last record is being dropped");
    }

    let mut kept: Vec<Vec<u8>> = Vec::new();
    let mut found = vec![false; wanted.len()];
    for record in &records {
        // A clock record belongs to whatever it sits in front of, so it is kept unconditionally: a dump is read by
        // order, and dropping the one that dates a rep would leave that rep undated.
        let keep = match decode_record(record) {
            Ok(Record::Summary(s)) => prefix_matches(&s.rep_uuid, &wanted, &mut found),
            Ok(Record::Computed(c)) => prefix_matches(&c.rep_uuid, &wanted, &mut found),
            Ok(Record::Raw(r)) => prefix_matches(&r.rep_uuid, &wanted, &mut found),
            Ok(Record::Clock(_)) => true,
            Err(e) => panic!("a record in the capture does not decode: {e:?}"),
        };
        if keep {
            kept.push(record.to_vec());
        }
    }

    for (i, hit) in found.iter().enumerate() {
        assert!(*hit, "no rep matched the prefix {}", wanted[i]);
    }

    let mut dump = Vec::new();
    for record in &kept {
        dump.extend_from_slice(&(record.len() as u32).to_le_bytes());
        dump.extend_from_slice(record);
    }
    fs::write(out, &dump).expect("the fixture writes");

    println!("{out}: {} records, {} bytes", kept.len(), dump.len());
    for record in &kept {
        if let Ok(Record::Summary(s)) = decode_record(record) {
            println!(
                "  {} rom {:.3} m, {} ms, {} trace pts, dated {}",
                short(&s.rep_uuid),
                s.rom,
                s.duration_ms,
                s.computed_len,
                if s.t_unix_ms == 0 {
                    "no".to_string()
                } else {
                    format!("{} ms", s.t_unix_ms)
                }
            );
        }
    }
}

/// Whether this record belongs to a wanted rep, marking the ones that matched.
fn prefix_matches(uuid: &[u8], wanted: &[String], found: &mut [bool]) -> bool {
    let hex = uuid.iter().map(|b| format!("{b:02x}")).collect::<String>();
    let mut keep = false;
    for (i, prefix) in wanted.iter().enumerate() {
        if hex.starts_with(prefix.as_str()) {
            found[i] = true;
            keep = true;
        }
    }
    keep
}

fn short(uuid: &[u8]) -> String {
    uuid.iter()
        .take(4)
        .map(|b| format!("{b:02x}"))
        .collect::<String>()
}
