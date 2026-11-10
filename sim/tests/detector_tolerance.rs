//! Pins for the synthetic-motion generators: the few deterministic points that must stay true across every detector change, so a tuning edit is measured rather than assumed.
//!
//! These are the complements to the sweep examples (the maps). The maps are broad and read by eye;
//! these are the one-sentence invariants that must not regress. Nothing here is a captured rep: a stored window has no rest at either end, so a detector cannot be judged on one.
//! `real_capture.rs` is where real motion is pinned.

use calypso_sim::motion::{stored, Motion, Push, Rep, Rng, Stall};

/// Each nominal scenario is exactly one stored rep, and its ROM tracks the asked-for travel.
///
/// The detector closes a nominal rep via the two velocity reversals (out and back), so the trace `compute_rep` integrates sees the full excursion and the ROM reads the travel, not a halved bump. It runs a hair over — about 4%, from what the integration and the anchor leave at the ends — so the pin is relative and loose enough not to flake on a rounding change but tight enough to catch a rep that splits into two rejected halves.
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

/// The generated non-reps — a one-way push and an oscillation — stay rejected. The push is under [`calypso_core::rep::MIN_ROM_M`] outright; the oscillation never even closes a span.
/// The harder cases — a full-height push (which pairs no leg, so its travel is never measured) or an oscillation big enough to matter — are what the false-positive map sweeps, not these two points.
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

/// The noise-map generator is deterministic: one seed draws the same sequence every time, and a different seed draws a different one. This is what makes a noise-map run repeatable by seed.
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

/// A rep held at the top is one rep, not a drive and a lowering. Before the detector closed on the return, a lockout pause past `T_REST_MS` split the rep into two spans — one at the edge of `MIN_ROM_M`, one under it — so the real rep was lost. The return ends it now.
#[test]
fn a_rep_paused_at_the_top_is_one_rep() {
    for (name, mut rep) in [("bench", Rep::bench()), ("deadlift", Rep::deadlift())] {
        rep.pause_top_ms = 500;
        let summaries = stored(&Motion::Rep(rep));
        assert_eq!(
            summaries.len(),
            1,
            "{name}: a paused rep should be exactly one rep"
        );
        let rom = summaries[0].rom;
        assert!(
            (rom - rep.travel).abs() < rep.travel * 0.10,
            "{name}: rom {rom:.4} for travel {:.3}",
            rep.travel
        );
    }
}

/// A drive that stalls partway is one rep carrying its whole travel, not two half-reps.
///
/// The detector this one replaced keyed on momentum: it opened a span when the bar moved and shut it when the bar returned, so a stall inside the drive closed the span early. Measured on these three shapes it read 0.122, lost the rep entirely, and read 0.152. The turning-point detector fuses the two same-direction halves of a stalled stroke, because a stall without a rest is a grind and a stall after a rest is a hold.
#[test]
fn a_stall_mid_stroke_is_one_rep_with_its_travel() {
    let bench = Stall::bench();
    for (first, second, stall_ms) in [(0.10, 0.09, 150), (0.15, 0.04, 150), (0.05, 0.14, 200)] {
        let stall = Stall {
            first,
            second,
            stall_ms,
            ..bench
        };
        let summaries = stored(&Motion::Stall(stall));
        assert_eq!(
            summaries.len(),
            1,
            "{first}+{second} @{stall_ms} ms should be one rep, got {}",
            summaries.len()
        );
        // Loose on purpose: the ROM of a *stalled* stroke is a fidelity question these three shapes do not agree on yet (0.05 + 0.14 reads ~20% low, in this detector and the one before it). What must not regress is that the rep exists and is not a half.
        let travel = stall.first + stall.second;
        let rom = summaries[0].rom;
        assert!(
            rom > travel * 0.75 && rom < travel * 1.15,
            "{first}+{second} @{stall_ms} ms: rom {rom:.4} for travel {travel:.3}"
        );
    }
}

/// **The known blind spot, pinned so it is a decision rather than a surprise.**
///
/// A leg ends where the speed falls below `VEL_MIN`, so a return gentle enough never to reach
/// 0.07 m/s never turns, and the rep is never stored. This is the cost of asking the question with velocity, and it is accepted: for a bench the boundary sits between 0.12 and 0.14 m/s, where the filter that now feeds the detector moved it up from 0.14–0.16 — the same blind spot, a little further out. A fix would close a leg on bounded travel as well as on a turn, at which point this test is the one to delete, not to relax.
///
/// `VEL_MIN` and `T_TURN_MS` move together: dropping the speed bar alone made fast turnarounds
/// *harder* to close, because the bar then spends less time below it than the debounce asks for.
#[test]
fn a_gentle_return_is_not_detected_yet() {
    for speed_out in [0.10, 0.12] {
        let rep = Rep {
            speed_out,
            ..Rep::bench()
        };
        assert!(
            stored(&Motion::Rep(rep)).is_empty(),
            "speed_out {speed_out} is now detected: the blind spot has been fixed, so delete this test"
        );
    }
}
