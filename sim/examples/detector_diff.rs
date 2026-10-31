//! The two detectability maps, cell by cell: the firmware's detector against the leg prototype.
//!
//! Each cell replays one synthetic rep through both pipelines and compares what they stored.
//! The map's own symbols answer "did the device store it"; this answers "would the change move
//! that answer", which is the question a candidate detector has to pass before it is worth any
//! firmware work.
//!
//!   `.` neither stores a rep        `=` both store, same count, ROMs agree
//!   `L` leg only                    `C` current only
//!   `~` both store, ROMs differ by more than [`ROM_TOL`]
//!   `X` both store, different counts
//!
//! With `-- non-reps` it sweeps phase 3 instead: the motions that are *not* a rep, where a `L` is a
//! false positive the prototype would add.
//!
//! Usage: `cargo run -p calypso-sim --example detector_diff -- [bench|deadlift|squat|non-reps]`

use std::env;

use calypso_core::pb::msg;
use calypso_sim::leg::leg_stored;
use calypso_sim::map::{self, Store};
use calypso_sim::motion::{stored, Motion, Push};

/// The ROM disagreement worth a symbol, in metres. Below this the two readouts are the same rep.
const ROM_TOL: f32 = 0.005;

fn count(store: Store<'_>, rep: calypso_sim::motion::Rep) -> Vec<msg::RepSummary> {
    store(&Motion::Rep(rep))
}

fn linspace(lo: f32, hi: f32, n: usize) -> Vec<f32> {
    (0..n)
        .map(|i| lo + (hi - lo) * i as f32 / (n - 1) as f32)
        .collect()
}

/// Phase 3, asked of both detectors: the sweep the false-positive map uses, counted rather than
/// drawn in full. A `L` here is a non-rep the prototype calls a rep.
fn non_reps() {
    println!("detector diff: non-reps (`.` neither, `=` both store, `L` leg only = new false positive, `C` current only)");
    let travel = linspace(0.02, 0.30, 21);
    let speed = linspace(0.30, 2.5, 21);
    let hz = linspace(0.5, 5.0, 21);
    let grid = [
        (
            "Push (unrack / re-rack / bump), hold 500 ms, speed (rows) × travel",
            speed.clone(),
            Box::new(|t: f32, s: f32| {
                Motion::Push(Push {
                    travel: t,
                    speed: s,
                    hold_ms: 500,
                })
            }) as Box<dyn Fn(f32, f32) -> Motion>,
        ),
        (
            "Wobble (hand on rack), 3000 ms, hz (rows) × travel",
            hz.clone(),
            Box::new(|t: f32, h: f32| Motion::Wobble {
                travel: t,
                hz: h,
                ms: 3000,
            }),
        ),
    ];

    for (label, rows, build) in grid {
        println!("\n=== {label} ===");
        let (mut l, mut c) = (0, 0);
        for y in rows.iter().rev() {
            let mut line = String::new();
            for x in travel.iter() {
                let m = build(*x, *y);
                let cur = !stored(&m).is_empty();
                let leg = !leg_stored(&m).is_empty();
                line.push(match (cur, leg) {
                    (false, false) => '.',
                    (true, true) => '=',
                    (false, true) => {
                        l += 1;
                        'L'
                    }
                    (true, false) => {
                        c += 1;
                        'C'
                    }
                });
            }
            println!("{line}");
        }
        println!("  leg-only {l}, current-only {c}");
    }
}

fn main() {
    let name = env::args().nth(1).unwrap_or_else(|| "bench".to_string());
    if name == "non-reps" {
        non_reps();
        return;
    }
    let p = map::profile(&name);
    println!(
        "detector diff: {}  (`.` same nothing, `=` same rep, `L` leg only, `C` current only, `~` ROM differs > {:.3} m, `X` count differs)",
        p.name, ROM_TOL
    );

    let (mut same, mut leg_only, mut cur_only, mut rom_diff, mut count_diff) = (0, 0, 0, 0, 0);
    let mut worst_rom = 0.0f32;

    for slice in map::slices(&p) {
        println!(
            "\n=== {} (rows) × {} (columns) ===",
            slice.ylabel, slice.xlabel
        );
        println!("{} columns: {}", slice.xlabel, map::column_labels(&slice));
        let (mut s_leg, mut s_cur, mut s_x, mut s_rom) = (0, 0, 0, 0);
        for j in (0..slice.yvals.len()).rev() {
            let mut line = format!("{:>7} |", map::row_label(&slice, j));
            for i in 0..slice.xvals.len() {
                let rep = slice.rep(slice.xvals[i], slice.yvals[j]);
                let c = count(&stored, rep);
                let l = count(&leg_stored, rep);
                let ch = match (c.len(), l.len()) {
                    (0, 0) => {
                        same += 1;
                        '.'
                    }
                    (0, _) => {
                        leg_only += 1;
                        s_leg += 1;
                        'L'
                    }
                    (_, 0) => {
                        cur_only += 1;
                        s_cur += 1;
                        'C'
                    }
                    (a, b) if a != b => {
                        count_diff += 1;
                        s_x += 1;
                        'X'
                    }
                    _ => {
                        let d = (c[0].rom - l[0].rom).abs();
                        if d > worst_rom {
                            worst_rom = d;
                        }
                        if d > ROM_TOL {
                            rom_diff += 1;
                            s_rom += 1;
                            '~'
                        } else {
                            same += 1;
                            '='
                        }
                    }
                };
                line.push(ch);
            }
            println!("{line}");
        }
        println!("  this slice: leg-only {s_leg}, current-only {s_cur}, count {s_x}, ROM {s_rom}");
    }

    println!(
        "\ncells: {} agree, {} leg-only, {} current-only, {} count differs, {} ROM differs",
        same, leg_only, cur_only, count_diff, rom_diff
    );
    println!("worst ROM disagreement on a shared cell: {worst_rom:.3} m");
}
