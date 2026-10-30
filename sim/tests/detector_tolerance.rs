//! Pins for the synthetic-motion generators: the few deterministic points that must stay true
//! across every detector change, so a tuning edit is measured rather than assumed.
//!
//! These are the complements to `real_capture.rs` (the captured reps and non-reps, checked in) and
//! the sweep examples (the maps). The maps are broad and read by eye; these are the one-sentence
//! invariants that must not regress.

use calypso_sim::motion::{stored, Motion, Push, Rep, Rng};

/// Each nominal scenario is exactly one stored rep, and its ROM tracks the asked-for travel.
///
/// The detector closes a nominal rep via the two velocity reversals (out and back), so the
/// velocity detrend in `compute_rep` sees a full excursion and the ROM reads the travel, not a
/// halved bump. The ROM runs a hair under travel (the half-cosine is truncated to samples and the
/// detrend trims the endpoints), so the pin is relative and loose enough not to flake on a
/// rounding change but tight enough to catch a rep that splits into two rejected halves.
#[test]
fn each_nominal_is_exactly_one_stored_rep() {
    for (name, rep) in [
        ("bench", Rep::bench()),
        ("deadlift", Rep::deadlift()),
        ("squat", Rep::squat()),
    ] {
        let summaries = stored(&Motion::Rep(rep));
        assert_eq!(
            summaries.len(),
            1,
            "{name}: expected exactly one stored rep, got {}",
            summaries.len()
        );
        let rom = summaries[0].rom;
        assert!(
            (rom - rep.travel).abs() < rep.travel * 0.10,
            "{name}: rom {rom:.4} for travel {:.3} drifted past 10%",
            rep.travel
        );
    }
}

/// The generated non-reps — a one-way push and an oscillation — stay rejected. The captured
/// wobble is pinned separately in `real_capture.rs`; these are the synthetic versions. The push
/// is under [`calypso_core::rep::MIN_ROM_M`] outright; the oscillation never even closes a span.
/// The harder cases — a full-height push (whose ROM the velocity detrend halves) or an oscillation
/// big enough to matter — are what the false-positive map sweeps, not these two points.
#[test]
fn generated_non_reps_stay_rejected() {
    for motion in [
        Motion::Push(Push {
            travel: 0.04,
            speed: 1.0,
            hold_ms: 500,
        }),
        Motion::Wobble {
            travel: 0.03,
            hz: 3.0,
            ms: 3000,
        },
    ] {
        assert!(
            stored(&motion).is_empty(),
            "{motion:?} should not store a rep"
        );
    }
}

/// The noise-map generator is deterministic: one seed draws the same sequence every time, and a
/// different seed draws a different one. This is what makes a noise-map run repeatable by seed.
#[test]
fn rng_is_deterministic() {
    let mut a = Rng::new(0x1234_5678);
    let mut b = Rng::new(0x1234_5678);
    let mut c = Rng::new(0x8765_4321);
    let (mut same, mut differs) = (true, false);
    for _ in 0..64 {
        let x = a.gauss();
        same &= x == b.gauss();
        differs |= x != c.gauss();
    }
    assert!(same, "the same seed drew different gauss() values");
    assert!(differs, "two seeds drew identical gauss() values");
}
