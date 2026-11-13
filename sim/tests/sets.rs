//! Pins for the `Set` generator: a set is the unrack, several reps, and the re-rack assembled into one stream, so these are the invariants about how several motions *coexist* — the thing a single-rep scenario cannot express.

use calypso_core::motion::fusion::GRAVITY;
use calypso_core::sampler::Sample;
use calypso_sim::fixtures::{detect_and_trace, LEAD_IN_MS, SAMPLE_DT_MS};
use calypso_sim::motion::{stored_set, Push, Rep, Set, TRACE_POINTS};

/// A comfortable set (rest between every rep) comes back as its reps, in time order, with each ROM reading its travel.
#[test]
fn a_set_of_five_comes_back_as_five_in_order() {
    let set = Set {
        rep: Rep::bench(),
        reps: 5,
        inter_rep_rest_ms: 3000,
        unrack: None,
        rerack: None,
    };
    let reps = stored_set(&set);
    assert_eq!(reps.len(), 5, "five reps in, five reps out");

    let t: Vec<u32> = reps.iter().map(|s| s.t_start_ms).collect();
    let mut sorted = t.clone();
    sorted.sort_unstable();
    assert_eq!(t, sorted, "reps should be emitted in time order");

    for s in &reps {
        assert!(
            (s.rom - set.rep.travel).abs() < set.rep.travel * 0.10,
            "rom {:.4} for travel {:.3}",
            s.rom,
            set.rep.travel
        );
    }
}

/// A set nobody pauses through still splits into its reps. The reversal-based close is what makes this possible — a detector that waited for rest would merge the whole set into one span and throw it away. This is the count that matters.
///
/// Note the one thing *not* pinned here: the later reps' ROMs read high (~9% over travel), because the velocity has not settled to zero between reps. That is a fidelity issue, not a detection one, and is left as a number for the maps to show rather than pinned here.
#[test]
fn a_no_rest_set_still_splits_into_five() {
    let mut rep = Rep::bench();
    rep.pause_top_ms = 0;
    let set = Set {
        rep,
        reps: 5,
        inter_rep_rest_ms: 0,
        unrack: None,
        rerack: None,
    };
    let reps = stored_set(&set);
    assert_eq!(
        reps.len(),
        5,
        "a continuous set should split into five reps"
    );
}

/// The motion that is not a rep — the unrack and the re-rack — is not stored, even when it sits right next to a rep. Only the rep comes back.
#[test]
fn the_unrack_and_rerack_are_not_stored() {
    let push = Push {
        travel: 0.06,
        speed: 0.8,
        hold_ms: 500,
    };
    let set = Set {
        rep: Rep::bench(),
        reps: 1,
        inter_rep_rest_ms: 1000,
        unrack: Some(push),
        rerack: Some(push),
    };
    let reps = stored_set(&set);
    assert_eq!(reps.len(), 1, "the unrack and re-rack should not be stored");
    assert!((reps[0].rom - set.rep.travel).abs() < set.rep.travel * 0.10);
}

/// A set with no reps in it — just the unrack — records nothing.
#[test]
fn a_lone_unrack_is_not_a_rep() {
    let set = Set {
        rep: Rep::bench(),
        reps: 0,
        inter_rep_rest_ms: 0,
        unrack: Some(Push {
            travel: 0.06,
            speed: 0.8,
            hold_ms: 500,
        }),
        rerack: None,
    };
    assert!(stored_set(&set).is_empty());
}

/// An unrack big enough to open a span must not merge into the first rep. The push in `the_unrack_and_rerack_are_not_stored` is too small to open one; a real lift-off does, and a detector that waited for its rest would run it into the first drive and store a bogus heavier rep instead of the rep.
#[test]
fn a_substantial_unrack_is_not_merged_into_the_first_rep() {
    for travel in [0.15, 0.20] {
        let set = Set {
            rep: Rep::bench(),
            reps: 1,
            inter_rep_rest_ms: 1000,
            unrack: Some(Push {
                travel,
                speed: 0.6,
                hold_ms: 500,
            }),
            rerack: None,
        };
        let reps = stored_set(&set);
        assert_eq!(
            reps.len(),
            1,
            "unrack {travel} m: exactly the rep, and not the unrack"
        );
        assert!(
            (reps[0].rom - set.rep.travel).abs() < set.rep.travel * 0.10,
            "unrack {travel} m: rom {:.4} for travel {:.3}",
            reps[0].rom,
            set.rep.travel
        );
    }
}

/// Still samples at 1 g, appended from `t`: a bar on the rack, or the lead-in before a set.
fn still(out: &mut Vec<Sample>, t: &mut u32, ms: u32) {
    for _ in 0..(ms / SAMPLE_DT_MS) {
        out.push(Sample::from_f32(*t, [0.0, 0.0, GRAVITY], [0.0; 3]));
        *t += SAMPLE_DT_MS;
    }
}

/// A motion's own stream, moved to the end of what has been built so far.
fn append(out: &mut Vec<Sample>, t: &mut u32, samples: Vec<Sample>) {
    for mut s in samples {
        s.t_ms += *t;
        out.push(s);
    }
    *t = out.last().map_or(*t, |s| s.t_ms + SAMPLE_DT_MS);
}

/// A set after a long rest counts from its own first stroke.
///
/// The bar on the rack is not still in the stroke that carried it there: the turn the next stroke leaves is in the rest, not back at the previous set's last turn. A detector that keeps the old turn starts the first leg of the set that far back, has it refused as a rep's half, and pairs every stroke after it one out of phase — five presses counted as four.
#[test]
fn a_set_after_a_long_rest_counts_all_five() {
    let set = Set {
        rep: Rep::bench(),
        reps: 5,
        inter_rep_rest_ms: 3000,
        unrack: None,
        rerack: None,
    };

    let mut samples = Vec::new();
    let mut t = 0u32;
    still(&mut samples, &mut t, LEAD_IN_MS);
    append(&mut samples, &mut t, Rep::bench().samples());
    still(&mut samples, &mut t, 20_000);
    append(&mut samples, &mut t, set.samples());

    let reps = detect_and_trace(&samples, TRACE_POINTS)
        .into_iter()
        .map(|(summary, _)| summary)
        .filter(|s| s.is_a_rep())
        .count();
    assert_eq!(reps, 6, "one rep, the bar put down, then five more");
}
