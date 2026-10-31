//! Side-by-side probe: the firmware's momentum detector vs the prototype leg detector.
//!
//! Runs each scenario through both and prints the stored reps (count + ROMs), so a divergence
//! shows up as a number rather than an opinion.

use calypso_core::pb::msg;
use calypso_core::rep::TracePoint;
use calypso_core::sampler::Sample;
use calypso_sim::fixtures::detect_and_trace;
use calypso_sim::leg::leg_detect_and_trace;
use calypso_sim::motion::{Motion, Push, Rep, Set, TRACE_POINTS};

const PI: f32 = core::f32::consts::PI;

fn sample(t_ms: u32, accel: [f32; 3]) -> Sample {
    Sample::from_f32(t_ms, accel, [1.0, 0.0, 0.0, 0.0])
}

fn rest(out: &mut Vec<Sample>, from: u32, ms: u32) -> u32 {
    let mut t = from;
    for _ in 0..(ms / 10) {
        out.push(sample(t, [0.0; 3]));
        t += 10;
    }
    t
}

fn stroke(out: &mut Vec<Sample>, from: u32, travel: f32, speed: f32, dir: f32) -> u32 {
    let dt = 0.01;
    let duration = PI * travel / (2.0 * speed);
    let steps = ((duration / dt).round() as u32).max(1);
    let t_len = steps as f32 * dt;
    let peak = 2.0 * speed * speed / travel;
    let mut t = from;
    for i in 0..=steps {
        let tau = i as f32 * dt;
        let accel = dir * peak * libm::cosf(PI * tau / t_len);
        out.push(sample(t, [0.0, 0.0, accel]));
        t += 10;
    }
    t
}

/// A rep whose drive stalls partway: move `a`, stop for `stall_ms`, move `b` more, then
/// return `a + b`. The stall is what the sweep could not generate and what the leg model
/// is supposed to absorb.
fn stall_rep(a: f32, b: f32, stall_ms: u32) -> Vec<Sample> {
    let mut out = Vec::new();
    let mut t = rest(&mut out, 0, 300);
    t = stroke(&mut out, t, a, 0.55, 1.0);
    t = rest(&mut out, t, stall_ms);
    t = stroke(&mut out, t, b, 0.55, 1.0);
    t = stroke(&mut out, t, a + b, 0.35, -1.0);
    rest(&mut out, t, 300);
    out
}

type Out = Vec<(msg::RepSummary, Vec<TracePoint>)>;

fn roms(out: &Out) -> Vec<f32> {
    out.iter()
        .filter(|(s, _)| s.is_a_rep())
        .map(|(s, _)| (s.rom * 1000.0).round() / 1000.0)
        .collect()
}

fn compare(label: &str, samples: &[Sample]) {
    let cur = detect_and_trace(samples, TRACE_POINTS);
    let leg = leg_detect_and_trace(samples, TRACE_POINTS);
    let (c, l) = (roms(&cur), roms(&leg));
    let mark = if c.len() == l.len() { " " } else { "!" };
    println!(
        "{mark} {label}: current {} {:?} | leg {} {:?}",
        c.len(),
        c,
        l.len(),
        l
    );
}

fn main() {
    println!("== single reps ==");
    compare("bench nominal", &Motion::Rep(Rep::bench()).samples());
    let mut r = Rep::bench();
    r.pause_top_ms = 500;
    compare("bench top-pause 500", &Motion::Rep(r).samples());
    compare("deadlift nominal", &Motion::Rep(Rep::deadlift()).samples());
    let mut r = Rep::deadlift();
    r.pause_top_ms = 500;
    compare("deadlift top-pause 500", &Motion::Rep(r).samples());

    println!("== sets ==");
    let bench = Rep::bench();
    compare(
        "bench x5, 3s rest",
        &Set {
            rep: bench,
            reps: 5,
            inter_rep_rest_ms: 3000,
            unrack: None,
            rerack: None,
        }
        .samples(),
    );
    let mut no_pause = bench;
    no_pause.pause_top_ms = 0;
    compare(
        "bench x5, no rest",
        &Set {
            rep: no_pause,
            reps: 5,
            inter_rep_rest_ms: 0,
            unrack: None,
            rerack: None,
        }
        .samples(),
    );

    println!("== unrack ==");
    let push = |t: f32| Push {
        travel: t,
        speed: 0.8,
        hold_ms: 500,
    };
    compare(
        "unrack 0.06 + bench + rerack",
        &Set {
            rep: bench,
            reps: 1,
            inter_rep_rest_ms: 1000,
            unrack: Some(push(0.06)),
            rerack: Some(push(0.06)),
        }
        .samples(),
    );
    compare(
        "unrack 0.15 + bench",
        &Set {
            rep: bench,
            reps: 1,
            inter_rep_rest_ms: 1000,
            unrack: Some(push(0.15)),
            rerack: None,
        }
        .samples(),
    );
    compare(
        "unrack 0.15 only",
        &Set {
            rep: bench,
            reps: 0,
            inter_rep_rest_ms: 0,
            unrack: Some(push(0.15)),
            rerack: None,
        }
        .samples(),
    );

    println!("== mid-drive stall (travel 0.19, split a+b) ==");
    for (a, b, stall) in [(0.10, 0.09, 150), (0.15, 0.04, 150), (0.05, 0.14, 200)] {
        compare(
            &format!("stall {a:.2}+{b:.2} @ {stall}ms"),
            &stall_rep(a, b, stall),
        );
    }
}
