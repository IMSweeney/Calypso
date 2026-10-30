//! Set probe — what a set really comes back as. Prints the detected stored reps for a few
//! canonical sets, so the measurement (not the fix) is what is on show.

use calypso_sim::fixtures::detect_and_trace;
use calypso_sim::motion::{Push, Rep, Set, TRACE_POINTS};

fn probe(name: &str, set: Set) {
    let reps = detect_and_trace(&set.samples(), TRACE_POINTS);
    let stored: Vec<_> = reps.iter().filter(|(s, _)| s.is_a_rep()).collect();
    println!(
        "{name}: raw spans={}  stored reps={}",
        reps.len(),
        stored.len()
    );
    for (s, _) in &stored {
        println!(
            "    rom={:.4} dur={}ms t0={}ms",
            s.rom, s.duration_ms, s.t_start_ms
        );
    }
}

fn main() {
    let push = Push {
        travel: 0.06,
        speed: 0.8,
        hold_ms: 500,
    };

    probe(
        "bench x5, 3s rest",
        Set {
            rep: Rep::bench(),
            reps: 5,
            inter_rep_rest_ms: 3000,
            unrack: None,
            rerack: None,
        },
    );

    let mut no_rest = Rep::bench();
    no_rest.pause_top_ms = 0;
    probe(
        "bench x5, no rest",
        Set {
            rep: no_rest,
            reps: 5,
            inter_rep_rest_ms: 0,
            unrack: None,
            rerack: None,
        },
    );

    probe(
        "unrack + bench x1 + rerack (1s gaps)",
        Set {
            rep: Rep::bench(),
            reps: 1,
            inter_rep_rest_ms: 1000,
            unrack: Some(push),
            rerack: Some(push),
        },
    );

    probe(
        "unrack only",
        Set {
            rep: Rep::bench(),
            reps: 0,
            inter_rep_rest_ms: 0,
            unrack: Some(push),
            rerack: None,
        },
    );
}
