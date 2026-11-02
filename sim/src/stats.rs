//! A coverage report for the detector — phases 1, 3 and 4 in one pass.
//!
//! A coverage report answers "how much of the program did the tests reach" by counting what it
//! touched. This answers the same question about the detector's parameter space: how much of the
//! rep grid comes back as the one rep it asked for, how much of the non-rep grid comes back as
//! nothing, and how many sets come back whole. The analogy stops at the number — **the grid is
//! ours**, so a percentage moves when the grid does, and only two runs over the same grid compare.
//!
//! What does not need a denominator is the run: the contiguous stretch of one-rep cells through a
//! class's nominal, printed in the knob's own units (0.10 .. 0.29 m, not 71 % of cells). That is the
//! boundary the maps are drawn to find, and it is a number a tuning change can move in a direction.
//!
//! The cells come from [`crate::map`] and [`crate::false_positive`], the sets from
//! [`crate::motion::probe_sets`], so the report cannot measure a grid the maps do not draw.

use calypso_core::pb::msg;

use crate::false_positive;
use crate::fixtures::detect_and_trace;
use crate::map::{profiles, Counts, Knob, Outcome, Slice};
use crate::motion::{probe_sets, TRACE_POINTS};

/// A contiguous run of one-rep cells, in the knob's own units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Interval {
    pub lo: f32,
    pub hi: f32,
    /// The run reaches the first (or the last) grid point, so the boundary lies outside the sweep
    /// and the number is a lower bound on how far the behaviour holds.
    pub open_lo: bool,
    pub open_hi: bool,
    pub points: usize,
}

impl Interval {
    /// `[0.10 .. 0.29] m`: square brackets where the sweep ended before the detector did, round
    /// where the next point out is not one rep.
    pub fn label(&self, knob: &Knob) -> String {
        format!(
            "{}{} .. {}{} {}",
            if self.open_lo { '[' } else { '(' },
            knob.format(self.lo),
            knob.format(self.hi),
            if self.open_hi { ']' } else { ')' },
            knob.unit
        )
    }
}

/// How far a stored ROM drifted from the travel the motion asked for, as a fraction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Error {
    pub mean: f32,
    /// The reading furthest from the travel, signed so the direction shows.
    pub worst: f32,
}

impl Error {
    /// `+0.4% / -8.6%`: the mean and the worst.
    pub fn label(&self) -> String {
        format!("{:+.1}% / {:+.1}%", self.mean * 100.0, self.worst * 100.0)
    }
}

/// The contiguous run of one-rep cells through `(i, j)` along the x knob. `None` if that cell is
/// not one rep: the run is the class's own behaviour, and a nominal that misses has none.
pub fn run_along_x(slice: &Slice, i: usize, j: usize) -> Option<Interval> {
    run(
        &slice.x.values,
        |k| slice.at(k, j).outcome == Outcome::One,
        i,
    )
}

/// The same along the y knob, at column `i`.
pub fn run_along_y(slice: &Slice, i: usize, j: usize) -> Option<Interval> {
    run(
        &slice.y.values,
        |k| slice.at(i, k).outcome == Outcome::One,
        j,
    )
}

fn run(values: &[f32], one: impl Fn(usize) -> bool, start: usize) -> Option<Interval> {
    if !one(start) {
        return None;
    }
    let mut lo = start;
    while lo > 0 && one(lo - 1) {
        lo -= 1;
    }
    let mut hi = start;
    while hi + 1 < values.len() && one(hi + 1) {
        hi += 1;
    }
    Some(Interval {
        lo: values[lo],
        hi: values[hi],
        open_lo: lo == 0,
        open_hi: hi + 1 == values.len(),
        points: hi - lo + 1,
    })
}

/// Mean and worst ROM error over the readings that have one, relative to the travel asked for.
pub fn error(pairs: impl Iterator<Item = (f32, f32)>) -> Option<Error> {
    let mut n = 0usize;
    let mut sum = 0.0f32;
    let mut worst = 0.0f32;
    for (rom, asked) in pairs {
        if asked == 0.0 {
            continue;
        }
        let e = (rom - asked) / asked;
        n += 1;
        sum += e;
        if e.abs() > worst.abs() {
            worst = e;
        }
    }
    (n > 0).then(|| Error {
        mean: sum / n as f32,
        worst,
    })
}

/// The smallest travel at which the motion was stored, and the range of the other knob there.
fn first_stored(slice: &Slice) -> Option<(f32, f32, f32)> {
    let stored: Vec<_> = slice
        .cells
        .iter()
        .filter(|c| c.outcome != Outcome::None)
        .collect();
    let x = stored.iter().map(|c| c.x).fold(f32::INFINITY, f32::min);
    let ys: Vec<f32> = stored.iter().filter(|c| c.x == x).map(|c| c.y).collect();
    let lo = ys.iter().fold(f32::INFINITY, |lo, y| lo.min(*y));
    let hi = ys.iter().fold(f32::NEG_INFINITY, |hi, y| hi.max(*y));
    (!ys.is_empty()).then_some((x, lo, hi))
}

/// Print every phase's numbers, one section each.
pub fn print_report() {
    println!("detector report — synthetic motion through the real pipeline");
    println!();
    println!(
        "  phase 1  detectability    does a synthetic rep come back as the one rep it asked for?"
    );
    println!("  phase 3  false positives  is motion that is not a rep stored anyway?");
    println!("  phase 4  sets            does a set come back as a set?");
    println!();
    println!(
        "A count is over the grid printed with it, so a percentage moves when the grid does and"
    );
    println!(
        "only two runs over the same grid compare. A run is in the knob's own units, so it does"
    );
    println!(
        "not: `[`/`]` means the sweep ended before the detector did, `(`/`)` that the next point"
    );
    println!("out is not a rep.");

    detectability();
    false_positives();
    sets();

    println!();
    println!("The same cells are drawn by --example detector_map -- <bench|deadlift|squat> and");
    println!("--example false_positive_map; --example set_probe prints each stored rep of a set.");
}

fn detectability() {
    println!("\n## Detectability");

    let mut all = Counts::default();
    for p in profiles() {
        let r = p.rep;
        let slices = p.slices();
        println!(
            "\n{}  nominal travel {:.2} m, speed in {:.2} m/s, speed out {:.2} m/s, pauses {} / {} ms",
            p.name, r.travel, r.speed_in, r.speed_out, r.pause_top_ms, r.pause_bottom_ms
        );
        println!(
            "  swept   {} · {} · {}",
            label(&slices[0].x),
            label(&slices[0].y),
            label(&slices[1].y)
        );
        println!(
            "          {} · {}",
            label(&slices[2].y),
            label(&slices[3].y)
        );
        println!(
            "  {:<20} {:>5} {:>5} {:>6} {:>5}   rom mean / worst",
            "slice", "cells", "one", "split", "none"
        );

        let mut class = Counts::default();
        for slice in &slices {
            let counts = slice.counts();
            class.add_counts(counts);
            let rom = error(
                slice
                    .cells
                    .iter()
                    .filter_map(|c| c.rom.map(|rom| (rom, c.asked))),
            )
            .map(|e| e.label())
            .unwrap_or_else(|| "—".to_string());
            println!(
                "  {:<20} {:>5} {:>5} {:>6} {:>5}   {}",
                format!("{} × {}", slice.x.name, slice.y.name),
                counts.cells(),
                counts.one,
                counts.split,
                counts.none,
                rom
            );

            let (i, j) = match slice.nominal {
                Some(n) => n,
                None => continue,
            };
            for (knob, run) in [
                (&slice.x, run_along_x(slice, i, j)),
                (&slice.y, run_along_y(slice, i, j)),
            ] {
                let text = match run {
                    Some(run) => format!(
                        "{:<26} {} of {} points",
                        run.label(knob),
                        run.points,
                        knob.values.len()
                    ),
                    None => "not one rep at the nominal".to_string(),
                };
                println!("      {:<12} {}", knob.name, text);
            }
        }
        println!("  {}", totals("all slices", class));
        all.add_counts(class);
    }
    println!("\n  {}", totals("all classes", all));
}

fn false_positives() {
    println!("\n## False positives");

    let slices = [
        ("push", false_positive::push_slice()),
        ("wobble", false_positive::wobble_slice()),
    ];
    for (name, slice) in &slices {
        println!(
            "  swept   {:<6} {} × {}",
            name,
            label(&slice.x),
            label(&slice.y)
        );
    }
    println!(
        "\n  {:<12} {:>5} {:>6} {:>6}   stored only from",
        "motion", "cells", "stored", "split"
    );

    let mut all = Counts::default();
    for (name, slice) in &slices {
        let counts = slice.counts();
        all.add_counts(counts);
        let from = match first_stored(slice) {
            Some((x, lo, hi)) => format!(
                "{} {} at {} {} .. {} {}",
                slice.x.name,
                slice.x.format(x),
                slice.y.name,
                slice.y.format(lo),
                slice.y.format(hi),
                slice.y.unit
            ),
            None => "never stored".to_string(),
        };
        println!(
            "  {:<12} {:>5} {:>6} {:>6}   {}",
            name,
            counts.cells(),
            counts.stored(),
            counts.split,
            from
        );
    }
    let cells = all.cells().max(1);
    println!(
        "\n  all non-reps {} cells   stored {:.1}% (of which {:.1}% split)   rejected {:.1}%",
        all.cells(),
        all.stored() as f32 * 100.0 / cells as f32,
        all.split as f32 * 100.0 / all.stored().max(1) as f32,
        all.none as f32 * 100.0 / cells as f32
    );
}

fn sets() {
    println!("\n## Sets");
    println!(
        "  {:<36} {:>3} {:>6} {:>7}   {:<17} verdict",
        "set", "in", "spans", "stored", "rom mean / worst"
    );

    let mut asked = 0usize;
    let mut stored_total = 0usize;
    for (name, set) in probe_sets() {
        let found = detect_and_trace(&set.samples(), TRACE_POINTS);
        let stored: Vec<&msg::RepSummary> = found
            .iter()
            .map(|(summary, _)| summary)
            .filter(|summary| summary.is_a_rep())
            .collect();
        let expected = set.reps as usize;
        asked += expected;
        stored_total += stored.len();

        let rom = error(stored.iter().map(|s| (s.rom, set.rep.travel)))
            .map(|e| e.label())
            .unwrap_or_else(|| "—".to_string());
        let in_order = stored
            .windows(2)
            .all(|w| w[0].t_start_ms <= w[1].t_start_ms);
        let verdict = match stored.len().cmp(&expected) {
            std::cmp::Ordering::Equal => "ok".to_string(),
            std::cmp::Ordering::Less => format!("lost {}", expected - stored.len()),
            std::cmp::Ordering::Greater => format!("extra {}", stored.len() - expected),
        };
        println!(
            "  {:<36} {:>3} {:>6} {:>7}   {:<17} {}{}",
            name,
            expected,
            found.len(),
            stored.len(),
            rom,
            verdict,
            if in_order { "" } else { ", out of order" }
        );
    }
    println!("\n  {asked} reps asked for, {stored_total} stored.");
}

fn label(knob: &Knob) -> String {
    format!("{} {}", knob.name, knob.extent())
}

fn totals(name: &str, counts: Counts) -> String {
    let cells = counts.cells().max(1);
    format!(
        "{name:<12} {cells:>5} cells   one {:.1}%   split {:.1}%   none {:.1}%",
        counts.one as f32 * 100.0 / cells as f32,
        counts.split as f32 * 100.0 / cells as f32,
        counts.none as f32 * 100.0 / cells as f32,
    )
}
