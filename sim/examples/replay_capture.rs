//! Run a stored window's raw samples back through the filter and the detector, twice: once from the
//! state the device stored, and once from that state with the velocity zeroed.
//!
//! The two runs answer different questions and the pair is what names the fault. A window replayed
//! from its own state is what the summary's `rom` already is — `compute_rep` integrates forward from
//! it, so a state that arrives carrying speed produces travel the bar never did. The same samples
//! replayed from rest are what the detector would have made of the window if the filter had known
//! where the bar was: spans, legs, and a `rom` measured against the motion rather than the drift.
//!
//! Nothing here changes what the device stores; it is the instrument that says whether a stored
//! window is real motion or an estimate's error.
//!
//! Usage: `cargo run --release --example replay_capture -- <dump.pblog> [window-prefix] [--series]`,
//! where the dump came from `cargo run --example device_log -- <image.bin> --dump <dump.pblog>`.

use std::env;
use std::fs;

use calypso_core::fusion::{magnitude, Fusion, FusionState};
use calypso_core::rep::{compute_rep, Detector, RepSpan, TraceBuf, TracePoint, MARGIN_MS};
use calypso_core::sampler::{Sample, SampleRing};
use calypso_sim::fixtures::{capture_windows, Window};

/// Long enough for any window the device stores, with the margins the ring has to hold.
const RING: usize = 2048;
const WINDOW_SAMPLES: usize = 1200;
const TRACE_POINTS: usize = 256;

/// What one replay produced.
struct Replay {
    spans: Vec<(RepSpan, f32)>,
    legs: u32,
    still: usize,
    /// The largest speed reached, and the largest of it that was horizontal.
    ///
    /// Splitting the two is what tells a bar that moved from a gravity that leaked: a tilt puts speed into
    /// the horizontal plane, where nothing a rep does ought to put metres per second.
    peak_v: f32,
    peak_h: f32,
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    let Some(path) = args.first() else {
        eprintln!("usage: replay_capture <dump.pblog> [window-prefix] [--series]");
        std::process::exit(2);
    };
    let series = args.iter().any(|a| a == "--series");
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
        report("as stored  ", replay(w, w.state));
        let mut from_rest = w.state;
        from_rest.velocity = [0.0; 3];
        report("from rest  ", replay(w, from_rest));
    }
}

/// The vertical acceleration and velocity the detector would see, one line a sample.
///
/// This is the instrument a turn rule is designed against: it prints the two signals the detector
/// gates on, so whether a turnaround shows up in them is a question with an answer.
fn print_series(w: &Window) {
    let mut fusion = Fusion::from_state(w.state);
    println!("{}\tt_ms\ta_z\tv_z\tv_h\tstill", w.uuid);
    for s in &w.samples {
        let f = fusion.update(s);
        let v = f.state.velocity;
        println!(
            "{}\t{}\t{:+.2}\t{:+.2}\t{:.2}\t{}",
            w.uuid,
            s.t_ms - w.samples[0].t_ms,
            f.linear_accel[2],
            v[2],
            (v[0] * v[0] + v[1] * v[1]).sqrt(),
            if f.still { 1 } else { 0 }
        );
    }
}

fn report(label: &str, r: Replay) {
    let roms: Vec<String> = r.spans.iter().map(|(_, rom)| format!("{rom:.2}")).collect();
    println!(
        "  {label} legs {}  spans {}  rom [{}]  peak |v| {:.2} m/s (horiz {:.2})  still {} samples",
        r.legs,
        r.spans.len(),
        roms.join(", "),
        r.peak_v,
        r.peak_h,
        r.still
    );
}

/// Feed one window through a filter seeded at `from`, and close the reps the detector finds.
fn replay(w: &Window, from: FusionState) -> Replay {
    let mut ring_buf = vec![Sample::ZERO; RING];
    let mut ring = SampleRing::new(&mut ring_buf);
    let mut fusion = Fusion::from_state(from);
    let mut det = Detector::new();
    let mut window = vec![Sample::ZERO; WINDOW_SAMPLES];
    let mut storage = vec![TracePoint::ZERO; TRACE_POINTS];
    let mut out = Replay {
        spans: Vec::new(),
        legs: 0,
        still: 0,
        peak_v: 0.0,
        peak_h: 0.0,
    };

    for s in &w.samples {
        ring.push(*s);
        let filtered = fusion.update(s);
        if filtered.still {
            out.still += 1;
        }
        let v = filtered.state.velocity;
        out.peak_v = out.peak_v.max(magnitude(v));
        out.peak_h = out.peak_h.max((v[0] * v[0] + v[1] * v[1]).sqrt());
        if let Some(span) = det.update(&filtered) {
            let (from_ms, state) = det.rep_state();
            let n = ring.copy_window(from_ms, span.end_ms, 0, MARGIN_MS, &mut window);
            if n == 0 {
                continue;
            }
            let mut trace = TraceBuf::new(&mut storage);
            let summary = compute_rep(&window[..n], &state, &mut trace);
            out.spans.push((span, summary.rom));
        }
    }
    out.legs = det.legs();
    out
}
