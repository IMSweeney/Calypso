//! Pins for the report's arithmetic: the run and the ROM error are the two numbers it adds up, and both are read off a grid rather than out of the detector, so they are tested on a grid somebody wrote down here instead of on motion the detector might change under.

use calypso_sim::map::{Cell, Knob, Outcome, Slice};
use calypso_sim::stats::{error, run_along_x, run_along_y};

/// One row of five travel values, each cell given the outcome in `outcomes`.
fn row(outcomes: &[Outcome]) -> Slice {
    let x = Knob {
        name: "travel",
        unit: "m",
        values: (0..outcomes.len()).map(|i| i as f32).collect(),
    };
    let y = Knob {
        name: "speed_in",
        unit: "m/s",
        values: vec![1.0],
    };
    let cells = outcomes
        .iter()
        .zip(&x.values)
        .map(|(&outcome, &xv)| Cell {
            x: xv,
            y: 1.0,
            asked: xv,
            outcome,
            rom: None,
        })
        .collect();
    Slice {
        x,
        y,
        cells,
        nominal: Some((2, 0)),
    }
}

/// The run is the stretch the detector holds, not the whole row it was drawn on: a miss ends it in both directions, and the ends are reported in the knob's own values.
#[test]
fn a_run_stops_at_the_first_cell_that_is_not_one_rep() {
    let slice = row(&[
        Outcome::None,
        Outcome::One,
        Outcome::One,
        Outcome::One,
        Outcome::None,
    ]);
    let run = run_along_x(&slice, 2, 0).expect("the nominal is a rep");
    assert_eq!((run.lo, run.hi), (1.0, 3.0));
    assert_eq!(run.points, 3);
    assert!(!run.open_lo && !run.open_hi, "both ends were measured");
    assert_eq!(run.label(&slice.x), "(1 .. 3) m");
}

/// A run that reaches the end of the sweep is a lower bound, and says so: the boundary is somewhere the grid did not go.
#[test]
fn a_run_that_reaches_the_sweep_edge_is_open_there() {
    let slice = row(&[Outcome::One; 5]);
    let run = run_along_x(&slice, 2, 0).expect("the nominal is a rep");
    assert!(run.open_lo && run.open_hi);
    assert_eq!(run.points, 5);
    assert_eq!(run.label(&slice.x), "[0 .. 4] m");
}

/// A single rep with misses either side is still a run, of one point — the nominal inside a split's neighbourhood is a different reading from the nominal inside a region.
#[test]
fn an_isolated_one_rep_is_a_run_of_one_point() {
    let slice = row(&[
        Outcome::None,
        Outcome::None,
        Outcome::One,
        Outcome::None,
        Outcome::None,
    ]);
    let run = run_along_x(&slice, 2, 0).expect("the nominal is a rep");
    assert_eq!(run.points, 1);
    assert_eq!((run.lo, run.hi), (2.0, 2.0));
}

/// A nominal that is not a rep has no run to report: the run is the class's own behaviour, and the report says so rather than measuring a neighbourhood the class never reaches.
#[test]
fn a_nominal_that_is_not_one_rep_has_no_run() {
    let split = row(&[
        Outcome::One,
        Outcome::One,
        Outcome::Split,
        Outcome::One,
        Outcome::One,
    ]);
    assert!(run_along_x(&split, 2, 0).is_none());

    let miss = row(&[
        Outcome::One,
        Outcome::None,
        Outcome::None,
        Outcome::One,
        Outcome::One,
    ]);
    assert!(run_along_x(&miss, 2, 0).is_none());
}

/// The same along the other knob, so a slice's y axis is read the same way as its x.
#[test]
fn the_run_along_the_other_knob_reads_the_column() {
    let mut slice = row(&[Outcome::One; 5]);
    slice.y = Knob {
        name: "pause_top",
        unit: "ms",
        values: vec![0.0, 250.0, 500.0, 750.0],
    };
    slice.cells = (0..20)
        .map(|n| Cell {
            x: slice.x.values[n % 5],
            y: slice.y.values[n / 5],
            asked: slice.x.values[n % 5],
            outcome: if n / 5 == 3 {
                Outcome::None
            } else {
                Outcome::One
            },
            rom: None,
        })
        .collect();
    slice.nominal = Some((2, 2));

    let run = run_along_y(&slice, 2, 2).expect("the nominal is a rep");
    assert_eq!((run.lo, run.hi), (0.0, 500.0), "the top row is not a rep");
    assert_eq!(run.points, 3);
    assert!(run.open_lo && !run.open_hi);
}

/// ROM error is relative to the travel the motion asked for, so the same absolute drift is a bigger error on a short rep — which is the comparison a reader of the map wants, since short reps are where the detector has least to measure.
#[test]
fn rom_error_is_relative_to_the_travel_asked_for() {
    let shorts = error([(0.12, 0.10)].into_iter()).unwrap();
    let longs = error([(0.57, 0.55)].into_iter()).unwrap();
    assert!((shorts.mean - 0.20).abs() < 1e-6);
    assert!((longs.mean - 0.20 / 5.5).abs() < 1e-6);

    let mixed = error([(0.22, 0.20), (0.19, 0.20)].into_iter()).unwrap();
    assert!(
        (mixed.mean - 0.025).abs() < 1e-6,
        "+10% and -5% average to +2.5%"
    );
    assert!(
        (mixed.worst - 0.10).abs() < 1e-6,
        "the worst is the +10% one"
    );
}
