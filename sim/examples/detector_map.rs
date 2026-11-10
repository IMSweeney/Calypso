//! Detectability map.
//!
//! For one lift class, draw where a synthetic rep is stored and where it is not, as 2-D ASCII slices. See [`calypso_sim::map`] for the slices and the symbols.
//!
//! Usage: `cargo run -p calypso-sim --example detector_map -- bench|deadlift|squat`

use std::env;

use calypso_sim::map;

fn main() {
    let name = env::args().nth(1).unwrap_or_else(|| "bench".to_string());
    map::render_profile(&name);
}
