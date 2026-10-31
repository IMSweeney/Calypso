//! The detectability map — phase 1 of `docs/DETECTOR_TESTS.md`.
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
//! Which detector fills the cells is the caller's choice ([`store`]), so the same sweep can be
//! drawn for the firmware's detector and for a prototype and the two compared cell by cell.

use calypso_core::pb::msg;

use crate::motion::{Motion, Rep};

/// The summaries a detector stores for one motion — one entry per rep, in order.
pub type Store<'a> = &'a dyn Fn(&Motion) -> Vec<msg::RepSummary>;

pub struct Profile {
    pub name: &'static str,
    pub rep: Rep,
    /// The half-width the travel knob is swept over (from the plan's class table).
    pub travel_range: f32,
}

pub fn profiles() -> [Profile; 3] {
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

pub fn profile(name: &str) -> Profile {
    profiles()
        .into_iter()
        .find(|p| p.name == name)
        .unwrap_or_else(|| panic!("unknown profile '{name}' (try bench|deadlift|squat)"))
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

/// One slice of the map: two swept knobs and a way to build the rep they name.
pub struct Slice {
    pub xlabel: &'static str,
    pub xvals: Vec<f32>,
    pub ylabel: &'static str,
    pub yvals: Vec<f32>,
    pub nominal_x: f32,
    pub nominal_y: f32,
    build: Box<dyn Fn(f32, f32) -> Rep>,
}

/// The four slices of one profile, in the order the map prints them.
pub fn slices(p: &Profile) -> Vec<Slice> {
    let r = p.rep;
    let travel = linspace(r.travel - p.travel_range, r.travel + p.travel_range, 21);
    let speed_in = linspace(r.speed_in * 0.4, r.speed_in * 1.1, 21);
    let speed_out = linspace(r.speed_out * 0.4, r.speed_out * 1.1, 21);
    let pauses = pause_sweep();

    let mut out = Vec::new();
    let mut push =
        |ylabel: &'static str, yvals: Vec<f32>, nominal_y: f32, f: Box<dyn Fn(f32, f32) -> Rep>| {
            out.push(Slice {
                xlabel: "travel",
                xvals: travel.clone(),
                ylabel,
                yvals,
                nominal_x: r.travel,
                nominal_y,
                build: f,
            });
        };
    let rr = r;
    push(
        "speed_in",
        speed_in,
        r.speed_in,
        Box::new(move |tr, sin| Rep {
            travel: tr,
            speed_in: sin,
            ..rr
        }),
    );
    push(
        "speed_out",
        speed_out,
        r.speed_out,
        Box::new(move |tr, sout| Rep {
            travel: tr,
            speed_out: sout,
            ..rr
        }),
    );
    push(
        "pause_top",
        pauses.clone(),
        r.pause_top_ms as f32,
        Box::new(move |tr, ptop| Rep {
            travel: tr,
            pause_top_ms: ptop as u32,
            ..rr
        }),
    );
    push(
        "pause_bottom",
        pauses,
        r.pause_bottom_ms as f32,
        Box::new(move |tr, pbot| Rep {
            travel: tr,
            pause_bottom_ms: pbot as u32,
            ..rr
        }),
    );
    out
}

/// Draw one slice, reading each cell through `store`.
pub fn render(slice: &Slice, store: Store<'_>) {
    let xd = decimals_for(&slice.xvals);
    let yd = decimals_for(&slice.yvals);
    let xi = nearest_index(&slice.xvals, slice.nominal_x);
    let yi = nearest_index(&slice.yvals, slice.nominal_y);

    println!(
        "\n=== {} (rows) × {} (columns) ===",
        slice.ylabel, slice.xlabel
    );
    println!(
        "{} columns: {}",
        slice.xlabel,
        slice
            .xvals
            .iter()
            .map(|x| fmt(*x, xd))
            .collect::<Vec<_>>()
            .join(" ")
    );
    for (j, y) in slice.yvals.iter().enumerate().rev() {
        let mut line = format!("{:>7} |", fmt(*y, yd));
        for (i, x) in slice.xvals.iter().enumerate() {
            let symbol = if i == xi && j == yi {
                '*'
            } else {
                match store(&Motion::Rep((slice.build)(*x, *y))).len() {
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
        "row order top→bottom: {} {} → {}   ('#' = one rep, 'X' = more than one, '.' = none, '*' = nominal)",
        slice.ylabel,
        fmt(*slice.yvals.last().unwrap(), yd),
        fmt(*slice.yvals.first().unwrap(), yd)
    );
}

/// Draw the whole map for one profile.
pub fn render_profile(name: &str, store: Store<'_>) {
    let p = profile(name);
    let r = p.rep;
    println!(
        "detectability map: {}  travel {:.2} ± {:.2} m, speed in {:.2} m/s, speed out {:.2} m/s, pauses {:?} / {:?} ms",
        p.name, r.travel, p.travel_range, r.speed_in, r.speed_out, r.pause_top_ms, r.pause_bottom_ms
    );
    for slice in slices(&p) {
        render(&slice, store);
    }
}

impl Slice {
    /// The rep this cell names.
    pub fn rep(&self, x: f32, y: f32) -> Rep {
        (self.build)(x, y)
    }

    /// The index of the cell carrying the `*`.
    pub fn nominal_cell(&self) -> (usize, usize) {
        (
            nearest_index(&self.xvals, self.nominal_x),
            nearest_index(&self.yvals, self.nominal_y),
        )
    }
}

/// The column labels, formatted the way the map prints them.
pub fn column_labels(slice: &Slice) -> String {
    let xd = decimals_for(&slice.xvals);
    slice
        .xvals
        .iter()
        .map(|x| fmt(*x, xd))
        .collect::<Vec<_>>()
        .join(" ")
}

/// One row label, formatted the way the map prints it.
pub fn row_label(slice: &Slice, j: usize) -> String {
    let yd = decimals_for(&slice.yvals);
    fmt(slice.yvals[j], yd)
}
