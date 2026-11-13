//! Run a stored window's raw samples back through the filter and the detector, twice: once from the state the device stored, and once from that state with the velocity zeroed.
//!
//! The two runs answer different questions and the pair is what names the fault. A window replayed from its own state is what the summary's `rom` already is — `compute_rep` integrates forward from it, so a state that arrives carrying speed produces travel the bar never did. The same samples replayed from rest are what the detector would have made of the window if the filter had known where the bar was: spans, legs, and a `rom` measured against the motion rather than the drift.
//!
//! Nothing here changes what the device stores; it is the instrument that says whether a stored window is real motion or an estimate's error.
//!
//! Usage: `cargo run --release --example replay_capture -- <dump.pblog> [window-prefix] [--series|--spans]`, where the dump came from `cargo run --example device_log -- <image.bin> --dump <dump.pblog>`.

use std::env;
use std::fs;

use calypso_core::motion::fusion::{magnitude, GRAVITY};
use calypso_core::motion::pipeline::Pipeline;
use calypso_core::motion::rep::RepSeed;
use calypso_sim::fixtures::{capture_windows, Window};
use calypso_sim::replay::{self, TRACE_CAPACITY};
use libm::atan2f;

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    let Some(path) = args.first() else {
        eprintln!("usage: replay_capture <dump.pblog> [window-prefix] [--series|--spans]");
        std::process::exit(2);
    };
    let series = args.iter().any(|a| a == "--series");
    let spans = args.iter().any(|a| a == "--spans");
    let only = args
        .iter()
        .skip(1)
        .find(|a| !a.starts_with("--"))
        .map(|p| p.to_lowercase());
    let bytes = fs::read(path).expect("the dump reads");
    let (windows, truncated) = capture_windows(&bytes);
    if truncated {
        eprintln!("{path}: torn at the end, the last record is dropped");
    }

    println!("{path}: {} windows with a raw trace", windows.len());
    for (i, w) in windows.iter().enumerate() {
        let wanted = match &only {
            Some(prefix) => w.uuid.starts_with(prefix.as_str()),
            None => true,
        };
        if !wanted {
            continue;
        }
        if series {
            print_series(w);
            continue;
        }
        let v0 = magnitude(w.state.velocity);
        println!(
            "\nwindow {} {}  {} samples  rom {:.2} m  v0 {:.1} m/s",
            i + 1,
            w.uuid,
            w.samples.len(),
            w.rom,
            v0
        );
        report("as stored  ", run_window(w, w.state, spans));
        let mut from_rest = w.state;
        from_rest.velocity = [0.0; 3];
        report("from rest  ", run_window(w, from_rest, spans));
    }
}

/// The vertical acceleration and velocity the detector would see, one line a sample.
///
/// This is the instrument a turn rule is designed against: it prints the two signals the detector gates on, so whether a turnaround shows up in them is a question with an answer.
fn print_series(w: &Window) {
    let mut motion = Pipeline::from_seed(w.state);
    println!("{}\tt_ms\ta_z\tv_z\tv_h\ta_x\ta_y\ttilt\tstill", w.uuid);
    for s in &w.samples {
        let step = motion.update(s, false);
        let f = &step.filtered;
        let v = step.integrated.velocity;
        // Where the world frame thinks gravity is, in degrees off vertical: zero if the attitude is right, and at a moment the bar is not accelerating, the attitude's own error.
        let a = f.linear_accel;
        let tilt = atan2f((a[0] * a[0] + a[1] * a[1]).sqrt(), a[2] + GRAVITY).to_degrees();
        println!(
            "{}\t{}\t{:+.2}\t{:+.2}\t{:.2}\t{:+.2}\t{:+.2}\t{:+.1}\t{}",
            w.uuid,
            s.t_ms - w.samples[0].t_ms,
            f.linear_accel[2],
            v[2],
            (v[0] * v[0] + v[1] * v[1]).sqrt(),
            a[0],
            a[1],
            tilt,
            if f.resting { 1 } else { 0 }
        );
    }
}

fn report(label: &str, r: replay::Replay) {
    let roms: Vec<String> = r
        .summaries
        .iter()
        .map(|s| format!("{:.2}", s.rom))
        .collect();
    println!(
        "  {label} halves {}  spans {}  rom [{}]  peak |v| {:.2} m/s (horiz {:.2})  resting {} samples",
        r.halves,
        r.summaries.len(),
        roms.join(", "),
        r.peak_speed,
        r.peak_horizontal,
        r.resting
    );
}

/// Feed one window through a filter seeded at `from`, and close the reps the detector finds.
///
/// `spans` prints where each rep's travel came from: the trace's vertical range, the velocity the replay started on, and the trace itself, coarse. A `rom` that grows between two ends sitting at zero metres was paid for inside the window; one that starts high was the state's.
fn run_window(w: &Window, from: RepSeed, spans: bool) -> replay::Replay {
    let out = replay::run(
        &w.samples,
        Pipeline::from_seed(from),
        TRACE_CAPACITY,
        &mut |_, _, _, _| {},
    );
    if spans {
        for ((rep, summary), trace) in out.reps.iter().zip(&out.summaries).zip(&out.traces) {
            println!(
                "  span {}..{}  rom {:.2} m  v_z {:+.2} m/s at the window's edge",
                rep.span.start_ms, rep.span.end_ms, summary.rom, rep.state.velocity[2]
            );
            let every = trace.len().div_ceil(16).max(1);
            let z: Vec<String> = trace
                .iter()
                .step_by(every)
                .map(|p| format!("{:+.2}", p.pos[2]))
                .collect();
            println!("    z {}", z.join(" "));
        }
    }
    out
}
