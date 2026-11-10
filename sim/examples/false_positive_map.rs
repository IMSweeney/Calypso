//! False-positive map.
//!
//! See [`calypso_sim::false_positive`] for the sweeps, the classes and the symbols. The same cells are counted, with the run that is stored and the smallest travel that gets there, by `cargo run -p calypso-sim --example detector_report`.
//!
//! Usage: `cargo run -p calypso-sim --example false_positive_map`

fn main() {
    calypso_sim::false_positive::render();
}
