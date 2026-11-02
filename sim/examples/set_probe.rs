//! Set probe — what a set really comes back as. Prints the detected stored reps for the canonical
//! sets, so the measurement (not the fix) is what is on show.
//!
//! `cargo run -p calypso-sim --example detector_report` prints the same sets as one table, with the
//! count, the order and the ROM error against the rep asked for.

use calypso_sim::fixtures::detect_and_trace;
use calypso_sim::motion::{probe_sets, TRACE_POINTS};

fn main() {
    for (name, set) in probe_sets() {
        let reps = detect_and_trace(&set.samples(), TRACE_POINTS);
        let stored: Vec<_> = reps.iter().filter(|(s, _)| s.is_a_rep()).collect();
        println!(
            "{name}: raw spans={}  stored reps={}",
            reps.len(),
            stored.len()
        );
        for (s, _) in &stored {
            println!(
                "    rom={:.4} dur={}ms t0={}ms",
                s.rom, s.duration_ms, s.t_start_ms
            );
        }
    }
}
