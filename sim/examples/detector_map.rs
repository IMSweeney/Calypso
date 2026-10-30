//! Detectability map — phase 1 of `docs/DETECTOR_TESTS.md`.
//!
//! For one lift class, draw where a synthetic rep is stored and where it is not, as 2-D ASCII
//! slices. A cell is one bit: `#` if the motion produced a stored rep (`rom >= MIN_ROM_M`),
//! `.` if it did not. The map is a boundary, not a score — the interesting part is how far each
//! nominal sits inside it, and the map is expected to show that some of the pause range stores
//! nothing (that is a result, not a failure).
//!
//! Five knobs (travel, speed in, speed out, pause top, pause bottom) do not plot, so the map is
//! four slices, each fixing three knobs at the nominal and sweeping two:
//!
//!   1. speed in × speed out
//!   2. travel × speed in
//!   3. speed in × pause top
//!   4. speed in × pause bottom
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

fn fmt_val(x: f32) -> String {
    if (x - x.round()).abs() < 1e-6 {
        format!("{x:.0}")
    } else {
        format!("{x:.2}")
    }
}

/// Draw one slice: `build(x, y)` turns the two swept knobs into the rep to test.
fn render(
    xlabel: &str,
    xvals: &[f32],
    ylabel: &str,
    yvals: &[f32],
    build: &dyn Fn(f32, f32) -> Rep,
) {
    println!("\n=== {ylabel} (rows) × {xlabel} (columns) ===");
    println!(
        "{xlabel} columns: {}",
        xvals
            .iter()
            .map(|x| fmt_val(*x))
            .collect::<Vec<_>>()
            .join(" ")
    );
    for y in yvals.iter().rev() {
        let mut line = format!("{:>7} |", fmt_val(*y));
        for x in xvals {
            let rep = build(*x, *y);
            let found = !stored(&Motion::Rep(rep)).is_empty();
            line.push(if found { '#' } else { '.' });
        }
        println!("{line}");
    }
    println!(
        "row order top→bottom: {ylabel} {} → {}   ('#' = rep stored, '.' = not)",
        fmt_val(*yvals.last().unwrap()),
        fmt_val(*yvals.first().unwrap())
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

    // The nominal pauses/travel/speeds are fixed by the profile; each slice overrides two knobs.
    let speed_in = linspace(r.speed_in * 0.4, r.speed_in * 1.1, 21);
    let speed_out = linspace(r.speed_out * 0.4, r.speed_out * 1.1, 21);
    let travel = linspace(
        r.travel - profile.travel_range,
        r.travel + profile.travel_range,
        21,
    );
    let pauses = pause_sweep();

    render(
        "speed_in",
        &speed_in,
        "speed_out",
        &speed_out,
        &|sin, sout| Rep {
            speed_in: sin,
            speed_out: sout,
            ..r
        },
    );

    render("travel", &travel, "speed_in", &speed_in, &|tr, sin| Rep {
        travel: tr,
        speed_in: sin,
        ..r
    });

    render("speed_in", &speed_in, "pause_top", &pauses, &|sin, ptop| {
        Rep {
            speed_in: sin,
            pause_top_ms: ptop as u32,
            ..r
        }
    });

    render(
        "speed_in",
        &speed_in,
        "pause_bottom",
        &pauses,
        &|sin, pbot| Rep {
            speed_in: sin,
            pause_bottom_ms: pbot as u32,
            ..r
        },
    );
}
