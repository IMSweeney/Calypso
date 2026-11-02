//! False-positive map — phase 3 of `docs/DETECTOR_TESTS.md`.
//!
//! The inverse of the detectability map: sweep the motion that is *not* a rep and report where
//! the detector starts storing it. A cell is still one bit — `#` if the motion produced a
//! stored rep (`rom >= MIN_ROM_M`), `.` if it did not. The output is the boundary of how
//! exaggerated a non-rep has to be before the device records it as one.
//!
//! Two classes, both already generators in `sim::motion`:
//!
//!   `Push`   — the unrack, a re-rack, a bump, a partial drive: one stroke, no return.
//!   `Wobble` — a hand on the rack: oscillation with no net travel.
//!
//! The calibration transient (a roll) and the unrack/re-rack assembled *around* a rep both belong
//! to phase two, so they are not here.
//!
//! The knobs are swept with the same [`Sweep`] the detectability map uses, so the two maps are read
//! at the same resolution. The ranges are not the classes': this map's job is the motion that is
//! *not* a rep, and it has to reach past the point where one would be stored, so travel runs from a
//! nudge to three times `MIN_ROM_M` and speed from a slow drag to faster than any pull.
//!
//! Usage: `cargo run -p calypso-sim --example false_positive_map`

use calypso_sim::map::Sweep;
use calypso_sim::motion::{stored, Motion, Push};

/// The bar's excursion: below `MIN_ROM_M`, then well past it.
const TRAVEL: Sweep = Sweep::new(0.02, 0.30, 21);
/// How fast the non-rep moves: a slow drag to faster than a rep is ever driven.
const SPEED: Sweep = Sweep::new(0.30, 2.50, 21);
/// How fast a wobble oscillates: a hand on the rack, to a rattle.
const HZ: Sweep = Sweep::new(0.5, 5.0, 21);

fn fmt_val(x: f32) -> String {
    if (x - x.round()).abs() < 1e-6 {
        format!("{x:.0}")
    } else {
        format!("{x:.2}")
    }
}

/// Draw one slice: `build(x, y)` turns the two swept knobs into the motion to test.
fn render(
    xlabel: &str,
    xvals: &[f32],
    ylabel: &str,
    yvals: &[f32],
    build: &dyn Fn(f32, f32) -> Motion,
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
            let motion = build(*x, *y);
            let found = !stored(&motion).is_empty();
            line.push(if found { '#' } else { '.' });
        }
        println!("{line}");
    }
    println!(
        "row order top→bottom: {ylabel} {} → {}   ('#' = stored, '.' = rejected)",
        fmt_val(*yvals.last().unwrap()),
        fmt_val(*yvals.first().unwrap())
    );
}

fn main() {
    // A push needs rest at the top long enough to close the span (`T_REST_MS`), so the hold is
    // fixed comfortably above it; the wobble is fixed at a few seconds of shaking.
    let hold_ms = 500u32;
    let wobble_ms = 3000u32;

    let travel = TRAVEL.values();
    let speed = SPEED.values();
    let hz = HZ.values();

    println!("false-positive map: how exaggerated must a non-rep be before it is stored?");

    println!("\n### Push (unrack / re-rack / bump / partial drive), hold {hold_ms} ms ###");
    render("travel", &travel, "speed", &speed, &|t, s| {
        Motion::Push(Push {
            travel: t,
            speed: s,
            hold_ms,
        })
    });

    println!("\n### Wobble (hand on rack), {wobble_ms} ms ###");
    render("travel", &travel, "hz", &hz, &|t, h| Motion::Wobble {
        travel: t,
        hz: h,
        ms: wobble_ms,
    });
}
