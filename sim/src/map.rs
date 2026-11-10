//! Detectability map.
//!
//! For one lift class, draw where a synthetic rep is stored and where it is not, as 2-D ASCII slices. A cell is one of three outcomes: `.` nothing stored, `#` exactly one rep stored, `X` more than one — the detector splitting one rep into two, which a boolean could not show. `*` marks the nominal. The map is a boundary, not a score — the interesting part is how far each nominal sits inside it, and the map is expected to show that some of the pause range stores nothing (that is a result, not a failure).
//!
//! Five knobs (travel, speed in, speed out, pause top, pause bottom) do not plot, so the map is four slices. Travel is the x axis on every one of them: it is the dimension a lifter actually varies, and keeping it fixed to the columns makes the slices read the same way across.
//!
//!   1. travel × speed in
//!   2. travel × speed out
//!   3. travel × pause top
//!   4. travel × pause bottom
//!
//! A slice is computed once, into [`Cell`]s by [`Slice::scan`], and the drawing here and the numbers in [`crate::stats`] are two views of those cells rather than two sweeps of the detector.
//! The grid machinery is shared: [`crate::false_positive`] draws its slices with the same types, so both maps and the report read at the same resolution.

use crate::motion::{stored, Motion, Rep};

/// One knob's sweep: the values it is drawn at, `steps` of them from `lo` to `hi` inclusive.
///
/// A class carries these rather than the renderer imposing them, because how far from the nominal a rep still reads as a rep is a property of the lift: a bench's travel range is not a deadlift's, and a squat's speeds are not a press's. A class that wants a different grid for one knob writes its own (see [`profiles`]).
#[derive(Debug, Clone, Copy)]
pub struct Sweep {
    pub lo: f32,
    pub hi: f32,
    pub steps: usize,
}

impl Sweep {
    pub const fn new(lo: f32, hi: f32, steps: usize) -> Self {
        Sweep { lo, hi, steps }
    }

    /// The values, low to high, both ends included.
    pub fn values(&self) -> Vec<f32> {
        (0..self.steps)
            .map(|i| self.lo + (self.hi - self.lo) * i as f32 / (self.steps - 1) as f32)
            .collect()
    }

    /// `lo..hi (n pts)`, for the header that says what a map was drawn over, printed to the same precision as the axis so the header cannot name a range the columns do not show.
    fn label(&self) -> String {
        let d = decimals_for(&self.values());
        format!(
            "{}..{} ({} pts)",
            fmt(self.lo, d),
            fmt(self.hi, d),
            self.steps
        )
    }
}

/// One axis of a slice: what the knob is called, the unit it is measured in, and the values it was drawn at, low to high.
#[derive(Debug, Clone)]
pub struct Knob {
    pub name: &'static str,
    pub unit: &'static str,
    pub values: Vec<f32>,
}

impl Knob {
    pub fn new(name: &'static str, unit: &'static str, sweep: Sweep) -> Self {
        Knob {
            name,
            unit,
            values: sweep.values(),
        }
    }

    /// Decimals needed to tell one value from its neighbour, so a run's ends do not all print as the same number.
    pub fn decimals(&self) -> usize {
        decimals_for(&self.values)
    }

    pub fn format(&self, v: f32) -> String {
        fmt(v, self.decimals())
    }

    /// `lo .. hi unit`, the extent the knob was drawn over — the denominator of any count taken over it.
    pub fn extent(&self) -> String {
        let (first, last) = match (self.values.first(), self.values.last()) {
            (Some(f), Some(l)) => (f, l),
            _ => return String::new(),
        };
        format!(
            "{} .. {} {}",
            self.format(*first),
            self.format(*last),
            self.unit
        )
    }
}

/// What the detector did with one grid point's motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    /// Nothing stored: a miss on a rep, or the right answer for motion that is not one.
    None,
    /// Exactly one rep — the question the grid asks.
    One,
    /// More than one: one motion split into several reps.
    Split,
}

impl Outcome {
    fn of(stored: usize) -> Self {
        match stored {
            0 => Outcome::None,
            1 => Outcome::One,
            _ => Outcome::Split,
        }
    }

    /// The index into a renderer's symbol table.
    pub const fn index(self) -> usize {
        self as usize
    }
}

/// How many cells came back each way.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counts {
    pub none: usize,
    pub one: usize,
    pub split: usize,
}

impl Counts {
    pub fn of(slice: &Slice) -> Self {
        let mut counts = Counts::default();
        for cell in &slice.cells {
            counts.add(cell.outcome);
        }
        counts
    }

    pub fn add(&mut self, outcome: Outcome) {
        match outcome {
            Outcome::None => self.none += 1,
            Outcome::One => self.one += 1,
            Outcome::Split => self.split += 1,
        }
    }

    pub fn add_counts(&mut self, other: Counts) {
        self.none += other.none;
        self.one += other.one;
        self.split += other.split;
    }

    pub fn cells(&self) -> usize {
        self.none + self.one + self.split
    }

    /// Cells that stored at least one rep.
    pub fn stored(&self) -> usize {
        self.one + self.split
    }
}

/// One grid point: the two knob values, the travel the motion asked for, and what came back.
#[derive(Debug, Clone, Copy)]
pub struct Cell {
    pub x: f32,
    pub y: f32,
    /// The travel the motion asked for, so a stored ROM can be read against it.
    pub asked: f32,
    pub outcome: Outcome,
    /// The stored rep's ROM. `None` unless exactly one was stored: no reps and several reps both leave no single number to read.
    pub rom: Option<f32>,
}

/// A drawn slice: the x columns, the y rows from low to high, and a [`Cell`] at every crossing.
#[derive(Debug, Clone)]
pub struct Slice {
    pub x: Knob,
    pub y: Knob,
    pub cells: Vec<Cell>,
    /// The grid point a class's own numbers land on, marked in the drawing.
    pub nominal: Option<(usize, usize)>,
}

impl Slice {
    /// Run every crossing of `x` and `y`: `build` turns the two values into the motion to test.
    pub fn scan(x: Knob, y: Knob, build: &dyn Fn(f32, f32) -> Motion) -> Self {
        let mut cells = Vec::with_capacity(x.values.len() * y.values.len());
        for &yv in &y.values {
            for &xv in &x.values {
                let motion = build(xv, yv);
                let reps = stored(&motion);
                cells.push(Cell {
                    x: xv,
                    y: yv,
                    asked: motion.travel(),
                    outcome: Outcome::of(reps.len()),
                    rom: match reps.as_slice() {
                        [one] => Some(one.rom),
                        _ => None,
                    },
                });
            }
        }
        Slice {
            x,
            y,
            cells,
            nominal: None,
        }
    }

    /// The cell at column `i`, row `j`.
    pub fn at(&self, i: usize, j: usize) -> &Cell {
        &self.cells[j * self.x.values.len() + i]
    }

    /// Mark the cell nearest `(x, y)`, the class's nominal, so the drawing can point at it and the report can measure the run through it.
    pub fn mark(mut self, x: f32, y: f32) -> Self {
        self.nominal = Some((
            nearest_index(&self.x.values, x),
            nearest_index(&self.y.values, y),
        ));
        self
    }

    pub fn counts(&self) -> Counts {
        Counts::of(self)
    }
}

/// Half to one and a half times the nominal, so the map reaches well past where a class's own reps live in both directions: the boundary is what is being looked for, and its position is only meaningful if the map goes on past it.
fn travel_sweep(nominal: f32) -> Sweep {
    Sweep::new(nominal * 0.5, nominal * 1.5, 21)
}

/// Both strokes, 0.1 to 2.0 m/s: from barely moving to the fastest a bar is pulled through a rep.
/// Shared by every class — what a lift can be driven at is a property of the lift, but what the detector can resolve is a property of the detector, and that is what the sweep is asking.
const SPEEDS: Sweep = Sweep::new(0.1, 2.0, 21);

/// Stillness at a turnaround, 0 to 1 s: no pause at all, to longer than any lifter holds at a turn.
const PAUSES: Sweep = Sweep::new(0.0, 1000.0, 21);

pub struct Profile {
    pub name: &'static str,
    pub rep: Rep,
    pub travel: Sweep,
    pub speed_in: Sweep,
    pub speed_out: Sweep,
    pub pause_top: Sweep,
    pub pause_bottom: Sweep,
}

impl Profile {
    /// The four slices, each marked at the class's nominal.
    pub fn slices(&self) -> [Slice; 4] {
        let r = self.rep;
        [
            Slice::scan(
                Knob::new("travel", "m", self.travel),
                Knob::new("speed_in", "m/s", self.speed_in),
                &|tr, s| {
                    Motion::Rep(Rep {
                        travel: tr,
                        speed_in: s,
                        ..r
                    })
                },
            )
            .mark(r.travel, r.speed_in),
            Slice::scan(
                Knob::new("travel", "m", self.travel),
                Knob::new("speed_out", "m/s", self.speed_out),
                &|tr, s| {
                    Motion::Rep(Rep {
                        travel: tr,
                        speed_out: s,
                        ..r
                    })
                },
            )
            .mark(r.travel, r.speed_out),
            Slice::scan(
                Knob::new("travel", "m", self.travel),
                Knob::new("pause_top", "ms", self.pause_top),
                &|tr, p| {
                    Motion::Rep(Rep {
                        travel: tr,
                        pause_top_ms: p as u32,
                        ..r
                    })
                },
            )
            .mark(r.travel, r.pause_top_ms as f32),
            Slice::scan(
                Knob::new("travel", "m", self.travel),
                Knob::new("pause_bottom", "ms", self.pause_bottom),
                &|tr, p| {
                    Motion::Rep(Rep {
                        travel: tr,
                        pause_bottom_ms: p as u32,
                        ..r
                    })
                },
            )
            .mark(r.travel, r.pause_bottom_ms as f32),
        ]
    }
}

/// A class with the sweeps every class starts from: travel ±50% of its own nominal, the speeds and pauses over [`SPEEDS`] and [`PAUSES`]. Override a field to give a class its own grid for that knob.
fn profile_of(name: &'static str, rep: Rep) -> Profile {
    Profile {
        name,
        rep,
        travel: travel_sweep(rep.travel),
        speed_in: SPEEDS,
        speed_out: SPEEDS,
        pause_top: PAUSES,
        pause_bottom: PAUSES,
    }
}

pub fn profiles() -> [Profile; 3] {
    [
        profile_of("bench", Rep::bench()),
        profile_of("deadlift", Rep::deadlift()),
        // Provisional: no squat rep has ever been captured.
        profile_of("squat", Rep::squat()),
    ]
}

/// Decimals needed to tell one axis value from its neighbour, so travel columns do not all print as the same two-decimal number.
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

/// The grid index closest to `target`. The nominal travel lands exactly on its grid; a knob whose range does not put the nominal on a grid point gets the nearest cell marked instead.
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

/// The profile a `-- bench|deadlift|squat` argument names.
pub fn profile(name: &str) -> Profile {
    profiles()
        .into_iter()
        .find(|p| p.name == name)
        .unwrap_or_else(|| panic!("unknown profile '{name}' (try bench|deadlift|squat)"))
}

/// Draw the whole map for one profile.
pub fn render_profile(name: &str) {
    let p = profile(name);
    let r = p.rep;
    println!(
        "detectability map: {}  travel {:.2} m, speed in {:.2} m/s, speed out {:.2} m/s, pauses {:?} / {:?} ms",
        p.name, r.travel, r.speed_in, r.speed_out, r.pause_top_ms, r.pause_bottom_ms
    );
    println!(
        "  swept: travel {} m, speed in {} m/s, speed out {} m/s, pause top {} ms, pause bottom {} ms",
        p.travel.label(),
        p.speed_in.label(),
        p.speed_out.label(),
        p.pause_top.label(),
        p.pause_bottom.label()
    );
    for slice in p.slices() {
        draw(
            &slice,
            ['.', '#', 'X'],
            "'#' = one rep, 'X' = more than one, '.' = none",
        );
    }
}

/// Draw one slice, `symbols` indexed by [`Outcome::index`], with the nominal marked `*`.
pub(crate) fn draw(slice: &Slice, symbols: [char; 3], legend: &str) {
    let xd = slice.x.decimals();
    let yd = slice.y.decimals();
    println!(
        "\n=== {} (rows) × {} (columns) ===",
        slice.y.name, slice.x.name
    );
    println!(
        "{} columns: {}",
        slice.x.name,
        slice
            .x
            .values
            .iter()
            .map(|x| fmt(*x, xd))
            .collect::<Vec<_>>()
            .join(" ")
    );
    for j in (0..slice.y.values.len()).rev() {
        let mut line = format!("{:>7} |", fmt(slice.y.values[j], yd));
        for i in 0..slice.x.values.len() {
            let symbol = match slice.nominal {
                Some((ni, nj)) if (ni, nj) == (i, j) => '*',
                _ => symbols[slice.at(i, j).outcome.index()],
            };
            line.push(symbol);
        }
        println!("{line}");
    }
    let nominal = if slice.nominal.is_some() {
        ", '*' = nominal"
    } else {
        ""
    };
    println!(
        "row order top→bottom: {} {} → {}   ({legend}{nominal})",
        slice.y.name,
        fmt(*slice.y.values.last().unwrap(), yd),
        fmt(slice.y.values[0], yd)
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The sweeps are the class's, so a class that needs its own grid gets one without the renderer knowing: this is the property the fields exist for.
    #[test]
    fn each_class_sweeps_its_own_grid() {
        let all = profiles();
        let bench = &all[0];
        let deadlift = &all[1];

        // Where the classes must differ: their nominal travels are 36 cm apart, and the grid follows the nominal, so a shared travel sweep would have one of them off its own map.
        assert_ne!(bench.travel.lo, deadlift.travel.lo);
        assert_ne!(bench.travel.hi, deadlift.travel.hi);

        for p in &all {
            for (knob, nominal, sweep) in [
                ("travel", p.rep.travel, p.travel),
                ("speed in", p.rep.speed_in, p.speed_in),
                ("speed out", p.rep.speed_out, p.speed_out),
            ] {
                let values = sweep.values();
                assert_eq!(values.len(), sweep.steps, "{}: {knob} steps", p.name);
                assert!(
                    values[0] <= nominal && nominal <= *values.last().unwrap(),
                    "{}: {knob} sweep {:?} does not reach its nominal {nominal}",
                    p.name,
                    (sweep.lo, sweep.hi)
                );
                assert!(
                    values.windows(2).all(|w| w[1] > w[0]),
                    "{}: {knob} sweep must ascend",
                    p.name
                );
            }
        }
    }

    /// A pause sweep starts at no pause at all: the no-rest edge is the one the detector is measured against, so no class may quietly stop drawing it.
    #[test]
    fn every_pause_sweep_reaches_no_pause() {
        for p in &profiles() {
            for sweep in [p.pause_top, p.pause_bottom] {
                assert_eq!(sweep.values()[0], 0.0, "{}: pauses start at 0", p.name);
            }
        }
    }

    /// The four slices are the four knobs each crossed with travel, and a slice is drawn on its class's own sweeps — so the map cannot plot a knob over another knob's range, and the report's run is over the grid the drawing shows.
    #[test]
    fn the_slices_are_the_four_knobs_against_travel() {
        for p in profiles() {
            let slices = p.slices();
            let names: Vec<&str> = slices.iter().map(|s| s.y.name).collect();
            assert_eq!(
                names,
                ["speed_in", "speed_out", "pause_top", "pause_bottom"],
                "{}: the knobs that do not plot",
                p.name
            );

            let sweeps = [p.speed_in, p.speed_out, p.pause_top, p.pause_bottom];
            let nominals = [
                p.rep.speed_in,
                p.rep.speed_out,
                p.rep.pause_top_ms as f32,
                p.rep.pause_bottom_ms as f32,
            ];
            for (slice, (sweep, nominal)) in slices.iter().zip(sweeps.iter().zip(nominals)) {
                assert_eq!(slice.x.values, p.travel.values(), "{}: travel on x", p.name);
                assert_eq!(
                    slice.y.values,
                    sweep.values(),
                    "{}: {} on y",
                    p.name,
                    slice.y.name
                );
                assert_eq!(
                    slice.cells.len(),
                    slice.x.values.len() * slice.y.values.len()
                );

                // The star is the nearest grid point, so a class whose nominal is off-grid (the bench's 0.55 m/s between 0.48 and 0.575) is marked within half a step of it.
                let (i, j) = slice.nominal.expect("every class marks its nominal");
                let half_x = (slice.x.values[1] - slice.x.values[0]) / 2.0;
                let half_y = (slice.y.values[1] - slice.y.values[0]) / 2.0;
                assert!((slice.x.values[i] - p.rep.travel).abs() <= half_x + 1e-6);
                assert!((slice.y.values[j] - nominal).abs() <= half_y + 1e-6);
                assert_eq!(
                    slice.at(i, j).outcome,
                    Outcome::One,
                    "{}: {} at the nominal",
                    p.name,
                    slice.y.name
                );
            }
        }
    }
}
