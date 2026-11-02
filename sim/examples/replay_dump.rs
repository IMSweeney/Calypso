//! Replay a rep log dump through the detector, to see what today's detector makes of motion an older one recorded.
//!
//! Usage: `cargo run --example replay_dump -- <path.pblog>`
//!
//! **What a dump can and cannot answer.** A `.pblog` — a fixture, or a copy of the app's archive — holds the records
//! the device stored: one summary per rep it closed, and that rep's trace. The trace is a window around the rep
//! (the motion span plus `MARGIN_MS` either side), decimated to the trace buffer, and *the time between two reps is
//! not in the file at all*. So this can say what the detector does with the motion that was kept, and it cannot say
//! anything about a rep the old detector never closed: there is no sample of it here to replay.
//!
//! Each window is replayed on its own, followed by stillness built from its own last sample, because that is what a
//! stream looks like after a rep that ended at rest. A span is counted only if it starts inside the window.

use std::env;
use std::fs;

use calypso_core::pb::{decode_record, Record};
use calypso_core::rep::{
    compute_rep, Detector, TraceBuf, TracePoint, MARGIN_MS, MIN_ROM_M, REP_FLAG_DECIMATED,
};
use calypso_core::sampler::Sample;
use calypso_sim::fixtures::read_dump;

/// How much stillness to append after a window, so a rep whose end is the window's edge can still close.
const TAIL_MS: u32 = 2_000;
/// The widest gap between a window's own samples that is still a timeline rather than a hole.
///
/// The device samples at 100 Hz and a decimated trace stores every other one, so 20 ms is normal.
const MAX_STEP_MS: u32 = 50;
/// The narrowest step that is still a sample rather than a catch-up burst.
const MIN_STEP_MS: u32 = 4;

/// Whether a window's own sample times can be integrated at all.
///
/// A stored trace is not a stream: the tracer copies samples out of the ring, and a sampler that
/// stalled leaves a jump in the times with a burst of catch-up samples after it. A window either
/// has a timeline or it does not, and this says which — so a replay can report the windows it can
/// judge instead of averaging a detector's behaviour over motion it was never shown.
enum Timeline {
    /// Monotonic, 4–50 ms between samples.
    Usable,
    /// A jump in the times: the window is missing the motion it claims to describe.
    Hole(u32),
    /// Samples closer together than the device can take them.
    Burst(u32),
}

impl Timeline {
    fn of(samples: &[Sample]) -> Timeline {
        let mut widest = 0u32;
        let mut narrowest = u32::MAX;
        let mut backwards = false;
        for pair in samples.windows(2) {
            let step = pair[1].t_ms;
            let prev = pair[0].t_ms;
            if step < prev {
                backwards = true;
            } else {
                widest = widest.max(step - prev);
                narrowest = narrowest.min(step - prev);
            }
        }
        if widest > MAX_STEP_MS {
            return Timeline::Hole(widest);
        }
        if narrowest < MIN_STEP_MS || backwards {
            return Timeline::Burst(narrowest);
        }
        Timeline::Usable
    }

    fn label(&self) -> String {
        match self {
            Timeline::Usable => String::new(),
            Timeline::Hole(ms) => format!("hole {ms} ms"),
            Timeline::Burst(ms) => format!("burst {ms} ms"),
        }
    }

    fn usable(&self) -> bool {
        matches!(self, Timeline::Usable)
    }
}

struct Window {
    uuid: Vec<u8>,
    duration_ms: u32,
    t_unix_ms: u64,
    flags: u32,
    chunks: Vec<(u32, Vec<Sample>)>,
}

fn main() {
    let path = match env::args().nth(1) {
        Some(path) => path,
        None => {
            eprintln!("usage: replay_dump <path.pblog>");
            std::process::exit(2);
        }
    };
    let bytes = fs::read(&path).expect("the dump reads");
    let (records, truncated) = read_dump(&bytes);
    println!(
        "{}: {} records{}",
        path,
        records.len(),
        if truncated { ", torn at the end" } else { "" }
    );

    let mut windows: Vec<Window> = Vec::new();
    // Unix milliseconds at the uptime zero of the boot these records are in, from the newest clock before them. A
    // stored summary carries its uptime and not its time, so a dump's dates come from here
    // (`docs/FIXTURES.md`), and a clock at 0 is a boot opening rather than a time.
    let mut epoch_ms: Option<u64> = None;
    for record in &records {
        let record = decode_record(record).expect("every record in the dump decodes");
        match record {
            Record::Clock(c) => match c.epoch_s {
                0 => epoch_ms = None,
                s => epoch_ms = Some(u64::from(s) * 1000),
            },
            Record::Summary(s) => windows.push(Window {
                uuid: s.rep_uuid.to_vec(),
                duration_ms: s.duration_ms,
                // A summary streamed to the app carries its own time; one read out of the log does not.
                t_unix_ms: match (s.t_unix_ms, epoch_ms) {
                    (0, Some(epoch)) => epoch + u64::from(s.t_start_ms),
                    (own, _) => own,
                },
                flags: s.flags,
                chunks: Vec::new(),
            }),
            Record::Trace(c) => {
                if let Some(w) = windows.iter_mut().find(|w| w.uuid == c.rep_uuid.to_vec()) {
                    let samples = c
                        .samples
                        .iter()
                        .map(|p| Sample {
                            t_ms: p.t_offset_ms,
                            accel: [p.ax as i16, p.ay as i16, p.az as i16],
                            quat: [p.qw as i16, p.qx as i16, p.qy as i16, p.qz as i16],
                        })
                        .collect();
                    w.chunks.push((c.index, samples));
                }
            }
        }
    }

    println!(
        "{} reps over {} chunks\n",
        windows.len(),
        windows.iter().map(|w| w.chunks.len()).sum::<usize>()
    );
    println!(
        "{:>3} {:>9} {:>7} {:>6} {:>7} {:>5} {:>4} {:>12}  spans",
        "#", "rep", "dur ms", "points", "peak a", "n", "dec", "timeline"
    );

    let mut stored = 0;
    let mut rejected = 0;
    let mut one_span = 0;
    let mut usable_windows = 0;
    let mut usable_spans = 0;
    let mut unusable_spans = 0;
    let mut unusable: Vec<usize> = Vec::new();
    let mut still = Vec::new();
    let mut split = Vec::new();
    let mut traceless = Vec::new();
    for (i, w) in windows.iter_mut().enumerate() {
        w.chunks.sort_by_key(|(index, _)| *index);
        let mut samples: Vec<Sample> = w.chunks.iter().flat_map(|(_, s)| s.clone()).collect();
        let Some(last) = samples.last().copied() else {
            traceless.push(i);
            continue;
        };
        let window_end = last.t_ms;
        // Judged on the window's own samples, before the tail this harness appends is folded in.
        let timeline = Timeline::of(&samples);
        if timeline.usable() {
            usable_windows += 1;
        } else {
            unusable.push(i + 1);
        }
        // Stillness at the bias the window ended on: a rep that finished at rest, and the stream goes on.
        for n in 1..=(TAIL_MS / 10) {
            samples.push(Sample {
                t_ms: window_end + n * 10,
                ..last
            });
        }

        let peak = samples
            .iter()
            .map(|s| {
                let [x, y, z] = s.accel_f32();
                (x * x + y * y + z * z).sqrt()
            })
            .fold(0.0f32, f32::max);

        let mut detector = Detector::new();
        let mut spans = Vec::new();
        for s in &samples {
            if let Some(span) = detector.update(s) {
                if span.start_ms <= window_end {
                    spans.push(span);
                }
            }
        }
        if timeline.usable() {
            usable_spans += spans.len();
            match spans.len() {
                1 => one_span += 1,
                0 => still.push((i, peak)),
                n => split.push((i, n)),
            }
        } else {
            unusable_spans += spans.len();
        }

        // What the device would do with each of those: a span it closed is only stored if the sensor actually moved.
        let mut kept = Vec::new();
        let mut storage = vec![TracePoint::ZERO; 512];
        let mut window_buf = vec![Sample::ZERO; 900];
        for span in &spans {
            let Some(first) = samples
                .iter()
                .position(|s| s.t_ms >= span.start_ms.saturating_sub(MARGIN_MS))
            else {
                continue;
            };
            let mut n = 0;
            for s in &samples[first..] {
                if s.t_ms > span.end_ms + MARGIN_MS || n == window_buf.len() {
                    break;
                }
                window_buf[n] = *s;
                n += 1;
            }
            let mut trace = TraceBuf::new(&mut storage);
            let sum = compute_rep(&window_buf[..n], [0.0; 3], &mut trace);
            let accepted = sum.is_a_rep() && timeline.usable();
            kept.push(format!(
                "{}+{}ms rom {:.2}m{}",
                span.start_ms,
                span.duration_ms(),
                sum.rom,
                if accepted {
                    ""
                } else if !sum.is_a_rep() {
                    " REJECTED"
                } else {
                    " (window not replayable)"
                }
            ));
            if accepted {
                stored += 1;
            } else if timeline.usable() && !sum.is_a_rep() {
                rejected += 1;
            }
        }

        // A decimated window has a wider sample spacing than the device's, which is the first thing to suspect when it replays badly.
        let decimated = w.flags & REP_FLAG_DECIMATED != 0;
        println!(
            "{:>3} {:>9} {:>7} {:>6} {:>7.2} {:>5} {:>4} {:>12}  {}",
            i + 1,
            short(&w.uuid),
            w.duration_ms,
            samples.len() - (TAIL_MS / 10) as usize,
            peak,
            spans.len(),
            if decimated { "yes" } else { "" },
            timeline.label(),
            kept.join(", ")
        );
    }

    let replayed = windows.len() - traceless.len();
    println!("\n{replayed} windows replayed");
    println!(
        "  {usable_windows} with a timeline, {usable_spans} spans; {} without, {unusable_spans} spans {unusable:?}",
        replayed - usable_windows
    );
    println!("\n  of the windows with a timeline:");
    println!("  one rep   {one_span}");
    println!("  stored    {stored}; rejected as under {MIN_ROM_M} m of travel {rejected}");
    if !still.is_empty() {
        // The peak is worth printing: a window that never closed a span because nothing moved past the threshold
        // and one that was loud but never turned both land here, and only the number tells them apart.
        println!(
            "  no span   {} — nothing in them closed as a rep, peak |a| {}",
            still.len(),
            still
                .iter()
                .map(|(_, p)| format!("{p:.1}"))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    if !split.is_empty() {
        println!("  split     {} — {split:?}", split.len());
    }
    if !traceless.is_empty() {
        println!(
            "{} windows have no trace stored (nothing to replay)",
            traceless.len()
        );
    }
    let undated = windows.iter().filter(|w| w.t_unix_ms == 0).count();
    if undated > 0 {
        println!("{undated} of the reps carry no time (a boot with no clock)");
    }
}

fn short(uuid: &[u8]) -> String {
    uuid.iter()
        .take(4)
        .map(|b| format!("{b:02x}"))
        .collect::<String>()
}
