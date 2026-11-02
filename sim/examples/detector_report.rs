//! Detector report — phases 1, 3 and 4 of `docs/DETECTOR_TESTS.md`, counted in one pass.
//!
//! The maps draw where the detector works; this adds up the same cells, states the run of one-rep
//! cells around each class's nominal in the knob's own units, counts the non-rep sweep that is
//! stored anyway, and measures each canonical set: how many reps in, how many out, in what order,
//! and how far each ROM drifted from the travel it asked for.
//!
//! Usage: `cargo run -p calypso-sim --example detector_report`

fn main() {
    calypso_sim::stats::print_report();
}
