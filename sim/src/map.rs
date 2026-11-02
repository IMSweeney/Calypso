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

use crate::motion::{stored, Motion, Rep};

/// One knob's sweep: the values it is drawn at, `steps` of them from `lo` to `hi` inclusive.
///
/// A class carries these rather than the renderer imposing them, because how far from the nominal
/// a rep still reads as a rep is a property of the lift: a bench's travel range is not a deadlift's,
/// and a squat's speeds are not a press's. A class that wants a different grid for one knob writes
/// its own (see [`profiles`]).
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

    /// `lo..hi (n pts)`, for the header that says what a map was drawn over, printed to the same
    /// precision as the axis so the header cannot name a range the columns do not show.
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

/// Half to one and a half times the nominal, so the map reaches well past where a class's own reps
/// live in both directions: the boundary is what is being looked for, and its position is only
/// meaningful if the map goes on past it.
fn travel_sweep(nominal: f32) -> Sweep {
    Sweep::new(nominal * 0.5, nominal * 1.5, 21)
}

/// Both strokes, 0.1 to 2.0 m/s: from barely moving to the fastest a bar is pulled through a rep.
/// Shared by every class — what a lift can be driven at is a property of the lift, but what the
/// detector can resolve is a property of the detector, and that is what the sweep is asking.
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

/// A class with the sweeps every class starts from: travel ±50% of its own nominal, the speeds and
/// pauses over [`SPEEDS`] and [`PAUSES`]. Override a field to give a class its own grid for that
/// knob.
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

    // Travel on the x axis for every slice; only the y knob changes.
    let travel = p.travel.values();
    let speed_in = p.speed_in.values();
    let speed_out = p.speed_out.values();

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
        &p.pause_top.values(),
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
        &p.pause_bottom.values(),
        r.travel,
        r.pause_bottom_ms as f32,
        &|tr, pbot| Rep {
            travel: tr,
            pause_bottom_ms: pbot as u32,
            ..r
        },
    );
}

/// One cell's worth of rep count, for a caller that wants a number rather than a drawing.
pub fn count(rep: Rep) -> usize {
    stored(&Motion::Rep(rep)).len()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The sweeps are the class's, so a class that needs its own grid gets one without the
    /// renderer knowing: this is the property the fields exist for.
    #[test]
    fn each_class_sweeps_its_own_grid() {
        let all = profiles();
        let bench = &all[0];
        let deadlift = &all[1];

        // Where the classes must differ: their nominal travels are 36 cm apart, and the grid
        // follows the nominal, so a shared travel sweep would have one of them off its own map.
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

    /// A pause sweep starts at no pause at all: the no-rest edge is the one the detector is
    /// measured against, so no class may quietly stop drawing it.
    #[test]
    fn every_pause_sweep_reaches_no_pause() {
        for p in &profiles() {
            for sweep in [p.pause_top, p.pause_bottom] {
                assert_eq!(sweep.values()[0], 0.0, "{}: pauses start at 0", p.name);
            }
        }
    }
}
