//! Detectability map — phase 1 of `docs/DETECTOR_TESTS.md`.
//!
//! For one lift class, draw where a synthetic rep is stored and where it is not, as 2-D ASCII
//! slices. A cell is one of three outcomes: `.` nothing stored, `#` exactly one rep stored, `X`
//! more than one — the detector splitting one rep into two, which a boolean could not show. `*`
//! marks the nominal. The map is a boundary, not a score — the interesting part is how far each
//! nominal sits inside it, and the map is expected to show that some of the pause range stores
//! nothing (that is a result, not a failure).
//!
//! Five knobs (travel, speed in, speed out, pause top, pause bottom) do not plot, so the map is
//! four slices. Travel is the x axis on every one of them: it is the dimension a lifter actually
//! varies, and keeping it fixed to the columns makes the slices read the same way across.
//!
//!   1. travel × speed in
//!   2. travel × speed out
//!   3. travel × pause top
//!   4. travel × pause bottom
//!
//! Usage: `cargo run -p calypso-sim --example detector_map -- bench|deadlift|squat`

use std::env;

use calypso_sim::motion::{stored, Motion, Rep};

struct Profile {
    name: &'static str,
    rep: Rep,
    /// The half-width the travel knob is swept over (from the plan's class table).
    travel_range: f32,
}

fn profiles() -> [Profile; 3] {
    [
        Profile {
            name: "bench",
            rep: Rep::bench(),
            travel_range: 0.03,
        },
        Profile {
            name: "deadlift",
            rep: Rep::deadlift(),
            travel_range: 0.07,
        },
        // Provisional: no squat rep has ever been captured.
        Profile {
            name: "squat",
            rep: Rep::squat(),
            travel_range: 0.10,
        },
    ]
}

fn linspace(lo: f32, hi: f32, n: usize) -> Vec<f32> {
    (0..n)
        .map(|i| lo + (hi - lo) * i as f32 / (n - 1) as f32)
        .collect()
}

/// Pause knob from 0 to 500 ms, in 25 ms steps (the plan's sweep range).
fn pause_sweep() -> Vec<f32> {
    (0..=500).step_by(25).map(|v| v as f32).collect()
}

/// Decimals needed to tell one axis value from its neighbour, so travel columns do not all print
/// as the same two-decimal number.
fn decimals_for(vals: &[f32]) -> usize {
    let step = match (vals.first(), vals.get(1)) {
        (Some(a), Some(b)) => (b - a).abs(),
        _ => return 2,
    };
    if step >= 1.0 {
        0
    } else if step >= 0.1 {
        1
    } else if step >= 0.01 {
        2
    } else {
        3
    }
}

fn fmt(v: f32, decimals: usize) -> String {
    format!("{:.*}", decimals, v)
}

/// The grid index closest to `target`. The nominal travel lands exactly on its grid; a knob whose
/// range does not put the nominal on a grid point gets the nearest cell marked instead.
fn nearest_index(vals: &[f32], target: f32) -> usize {
    let mut best = 0;
    let mut best_d = f32::MAX;
    for (i, v) in vals.iter().enumerate() {
        let d = (v - target).abs();
        if d < best_d {
            best_d = d;
            best = i;
        }
    }
    best
}

/// Draw one slice: `build(x, y)` turns the two swept knobs into the rep to test, and `(nominal_x,
/// nominal_y)` is where that class's nominal sits, marked `*`.
fn render(
    xlabel: &str,
    xvals: &[f32],
    ylabel: &str,
    yvals: &[f32],
    nominal_x: f32,
    nominal_y: f32,
    build: &dyn Fn(f32, f32) -> Rep,
) {
    let xd = decimals_for(xvals);
    let yd = decimals_for(yvals);
    let xi = nearest_index(xvals, nominal_x);
    let yi = nearest_index(yvals, nominal_y);

    println!("\n=== {ylabel} (rows) × {xlabel} (columns) ===");
    println!(
        "{xlabel} columns: {}",
        xvals
            .iter()
            .map(|x| fmt(*x, xd))
            .collect::<Vec<_>>()
            .join(" ")
    );
    for (j, y) in yvals.iter().enumerate().rev() {
        let mut line = format!("{:>7} |", fmt(*y, yd));
        for (i, x) in xvals.iter().enumerate() {
            let symbol = if i == xi && j == yi {
                '*'
            } else {
                match stored(&Motion::Rep(build(*x, *y))).len() {
                    0 => '.',
                    1 => '#',
                    _ => 'X',
                }
            };
            line.push(symbol);
        }
        println!("{line}");
    }
    println!(
        "row order top→bottom: {ylabel} {} → {}   ('#' = one rep, 'X' = more than one, '.' = none, '*' = nominal)",
        fmt(*yvals.last().unwrap(), yd),
        fmt(*yvals.first().unwrap(), yd)
    );
}

fn main() {
    let name = env::args().nth(1).unwrap_or_else(|| "bench".to_string());
    let profile = profiles()
        .into_iter()
        .find(|p| p.name == name)
        .unwrap_or_else(|| panic!("unknown profile '{name}' (try bench|deadlift|squat)"));

    let r = profile.rep;
    println!(
        "detectability map: {}  travel {:.2} ± {:.2} m, speed in {:.2} m/s, speed out {:.2} m/s, pauses {:?} / {:?} ms",
        profile.name,
        r.travel,
        profile.travel_range,
        r.speed_in,
        r.speed_out,
        r.pause_top_ms,
        r.pause_bottom_ms
    );

    // Travel on the x axis for every slice; only the y knob changes.
    let travel = linspace(
        r.travel - profile.travel_range,
        r.travel + profile.travel_range,
        21,
    );
    let speed_in = linspace(r.speed_in * 0.4, r.speed_in * 1.1, 21);
    let speed_out = linspace(r.speed_out * 0.4, r.speed_out * 1.1, 21);
    let pauses = pause_sweep();

    render(
        "travel",
        &travel,
        "speed_in",
        &speed_in,
        r.travel,
        r.speed_in,
        &|tr, sin| Rep {
            travel: tr,
            speed_in: sin,
            ..r
        },
    );

    render(
        "travel",
        &travel,
        "speed_out",
        &speed_out,
        r.travel,
        r.speed_out,
        &|tr, sout| Rep {
            travel: tr,
            speed_out: sout,
            ..r
        },
    );

    render(
        "travel",
        &travel,
        "pause_top",
        &pauses,
        r.travel,
        r.pause_top_ms as f32,
        &|tr, ptop| Rep {
            travel: tr,
            pause_top_ms: ptop as u32,
            ..r
        },
    );

    render(
        "travel",
        &travel,
        "pause_bottom",
        &pauses,
        r.travel,
        r.pause_bottom_ms as f32,
        &|tr, pbot| Rep {
            travel: tr,
            pause_bottom_ms: pbot as u32,
            ..r
        },
    );
}
