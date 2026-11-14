//! What a recorded window's raw samples say about the sensor and the room it is in.
//!
//! The noise map's two largest terms cannot be fitted from a datasheet — how much of a still window's spread is the part and how much is the platform, and what shape the second has — so they come off a device. One capture from a `--features record-mode` build answers all four questions the model needs, and this prints them.
//!
//! Everything is in the units the gate is written in: milligrammes of specific force, and milliseconds against the window and the run length `Fusion` is judged on. The constants come from `calypso_core::motion::fusion` rather than being copied here, so a report cannot describe a gate nothing uses.
//!
//! Usage: `cargo run --release --example raw_dump -- <dump.pblog>`, where the dump came from `cargo run --example device_log -- <image.bin> --dump <dump.pblog>` or from the app's history.

use std::cmp::Reverse;
use std::env;
use std::fs;

use calypso_core::motion::fusion::{
    GRAVITY, READY_S, STILL_GRAVITY, STILL_RATE_DPS, STILL_STRAY, STILL_WINDOW,
};
use calypso_core::pb::{self, decode_record, Record};
use calypso_core::sampler::{Sample, ACCEL_SCALE};
use calypso_sim::fixtures::read_dump;

/// One window's raw samples, with what its summary said about it.
struct Window {
    uuid: String,
    t_start_ms: u32,
    duration_ms: u32,
    rom: f32,
    samples: Vec<Sample>,
}

fn main() {
    let Some(path) = env::args().nth(1) else {
        eprintln!("usage: raw_dump <dump.pblog>");
        std::process::exit(2);
    };
    let bytes = fs::read(&path).expect("the dump reads");
    let (records, truncated) = read_dump(&bytes);
    if truncated {
        eprintln!("{path}: torn at the end, the last record is dropped");
    }

    let mut windows: Vec<Window> = Vec::new();
    for record in &records {
        match decode_record(record).expect("a record in the dump decodes") {
            Record::Summary(s) => {
                if s.raw_len == 0 {
                    println!(
                        "{} ({}): no raw trace, so nothing here can be measured",
                        hex(&s.rep_uuid),
                        if s.duration_ms == 0 {
                            "empty"
                        } else {
                            "window"
                        }
                    );
                    continue;
                }
                windows.push(Window {
                    uuid: hex(&s.rep_uuid),
                    t_start_ms: s.t_start_ms,
                    duration_ms: s.duration_ms,
                    rom: s.rom,
                    samples: Vec::with_capacity(s.raw_len as usize),
                });
            }
            Record::Raw(r) => {
                let Some(w) = windows.last_mut() else {
                    panic!("a raw chunk before any summary")
                };
                // A point's time is an offset from the rep's first sample, which is the summary's `t_start_ms` — the same origin the plotted trace counts from — so that plus the offset is the sample's uptime. The chunk's own `first_t_offset_ms` places the chunk in a stream and is not part of it.
                for p in &r.samples {
                    let mut s = pb::raw_point_from_message(p);
                    s.t_ms = s.t_ms.saturating_add(w.t_start_ms);
                    w.samples.push(s);
                }
            }
            _ => {}
        }
    }

    println!(
        "{path}: {} windows, {} raw samples",
        windows.len(),
        windows.iter().map(|w| w.samples.len()).sum::<usize>()
    );
    if windows.is_empty() {
        return;
    }

    for (i, w) in windows.iter().enumerate() {
        report(i + 1, w);
    }
    means(&windows);
}

/// Everything one window has to say, in the order the questions are asked.
fn report(index: usize, w: &Window) {
    let n = w.samples.len();
    if n < STILL_WINDOW + 2 {
        println!(
            "\nwindow {index} {} — {n} samples, too short to measure",
            w.uuid
        );
        return;
    }
    let accel: Vec<[f32; 3]> = w
        .samples
        .iter()
        .map(|s| {
            let a = s.specific_force_f32();
            [a[0], a[1], a[2]]
        })
        .collect();
    let gyro: Vec<[f32; 3]> = w
        .samples
        .iter()
        .map(|s| {
            let g = s.gyro_f32();
            [g[0], g[1], g[2]]
        })
        .collect();

    let (dt_ms, dt_min, dt_max, holes) = timeline(&w.samples);
    println!(
        "\nwindow {index} {}  t+{} ms, {} ms  rom {:.2} m",
        w.uuid, w.t_start_ms, w.duration_ms, w.rom
    );
    println!(
        "  timeline    {} samples, dt {} / {dt_min} / {dt_max} ms (median/min/max), {} holes",
        n, dt_ms, holes
    );
    if holes > 0 {
        // A hole is samples the device never took, so it is also a window the gate never saw end to end: where they are and how big they are is what says which of its paths is doing it.
        let mut gaps: Vec<(i64, u32)> = w
            .samples
            .windows(2)
            .filter_map(|pair| {
                let d = pair[1].t_ms as i64 - pair[0].t_ms as i64;
                (d > dt_ms * 3 / 2).then(|| (d, pair[1].t_ms))
            })
            .collect();
        gaps.sort_by_key(|gap| Reverse(gap.0));
        print!("    largest:");
        for (d, at) in gaps.iter().take(6) {
            print!(" {} ms at +{} ms", d, *at as i64 - w.t_start_ms as i64);
        }
        println!();
    }

    // Where gravity points is which way the sensor was bolted on, and nothing else: a still sensor reads 1 g up its own up axis, so this is the check that a window is the orientation it claims.
    let mean = mean3(&accel);
    println!(
        "  mean        x {:+.3}  y {:+.3}  z {:+.3} m/s²   |  {:.0} mg, and 1 g is {:.0} mg",
        mean[0],
        mean[1],
        mean[2],
        magnitude(mean) * 1000.0 / GRAVITY,
        1000.0
    );

    println!(
        "  peak-to-peak over the window   x {:.1}  y {:.1}  z {:.1} mg",
        spread(&accel, 0, n) * 1000.0 / GRAVITY,
        spread(&accel, 1, n) * 1000.0 / GRAVITY,
        spread(&accel, 2, n) * 1000.0 / GRAVITY
    );
    // The first sample after the chip is configured is worth seeing: a window that opens at boot starts on it, and a reading that is not yet a reading would explain a window that swings a g.
    let first = &w.samples[0];
    println!(
        "  first sample at +{} ms   x {:+.2}  y {:+.2}  z {:+.2} m/s²   gyro {:.2} {:.2} {:.2} dps",
        first.t_ms as i64 - w.t_start_ms as i64,
        first.specific_force_f32()[0],
        first.specific_force_f32()[1],
        first.specific_force_f32()[2],
        first.gyro_f32()[0],
        first.gyro_f32()[1],
        first.gyro_f32()[2]
    );
    // A window that swings a tenth of a g is not a still window however its p50 reads, and where the swing is says which it was: at the very beginning the chip was still coming up, later on the device was being handled.
    for axis in 0..3 {
        let (far, at) = (0..n)
            .map(|i| ((accel[i][axis] - mean[axis]).abs(), i))
            .fold((0.0f32, 0usize), |a, b| if b.0 > a.0 { b } else { a });
        if far * 1000.0 / GRAVITY > 100.0 {
            println!(
                "    not still: {} swings {:.0} mg from its mean, at +{} ms",
                "xyz".chars().nth(axis).unwrap(),
                far * 1000.0 / GRAVITY,
                w.samples[at].t_ms as i64 - w.t_start_ms as i64
            );
        }
    }

    // The gate's own statistics, over every `STILL_WINDOW` run that has no hole in it: how far the run's mean specific force sits from 1 g, and how far its worst sample does. The first is what a slow stroke cannot hold and the second what a fast one cannot avoid.
    // Windows with a hole in them are left out of the count, because the gate never saw one: it is handed every sample the loop took, and a gap is a window that closed on nine hundred ms.
    println!(
        "  stillness window {:.0} ms (p50 / p95 / max)",
        STILL_WINDOW as f32 * dt_ms as f32
    );
    let (means, strays) = window_gate(&accel, &w.samples, dt_ms);
    println!(
        "    mean from 1 g {:.1} / {:.1} / {:.1} mg   (STILL_GRAVITY {:.1} mg)",
        pct(&means, 0.50) * 1000.0 / GRAVITY,
        pct(&means, 0.95) * 1000.0 / GRAVITY,
        means.last().copied().unwrap_or(0.0) * 1000.0 / GRAVITY,
        STILL_GRAVITY * 1000.0 / GRAVITY
    );
    println!(
        "    worst sample  {:.1} / {:.1} / {:.1} mg   (STILL_STRAY {:.1} mg)",
        pct(&strays, 0.50) * 1000.0 / GRAVITY,
        pct(&strays, 0.95) * 1000.0 / GRAVITY,
        strays.last().copied().unwrap_or(0.0) * 1000.0 / GRAVITY,
        STILL_STRAY * 1000.0 / GRAVITY
    );
    // The rate the gate judges is the window's mean with the gyro's offset out, so the statistic is how far that mean wanders once the window's own reading of the offset is taken out.
    let rates = window_rates(&gyro, &w.samples, dt_ms);
    println!(
        "    mean rate     {:.2} / {:.2} / {:.2} dps  (STILL_RATE_DPS {:.0} dps)",
        pct(&rates, 0.50),
        pct(&rates, 0.95),
        rates.last().copied().unwrap_or(0.0),
        STILL_RATE_DPS
    );
    let (quiet_share, longest_ms) = quiet_runs(&accel, &w.samples, dt_ms);
    println!(
        "    worst sample under it: {:.0}% of samples, longest unbroken run {} ms (READY_S {:.0})",
        quiet_share * 100.0,
        longest_ms,
        READY_S * 1000.0
    );

    // The quantiser: how often two consecutive readings are the same value, and by how much they move when they are not. A part whose die is 3.9 mg a count at ±8 g shows steps of about four of the output's 1 mg counts; one that rescales internally shows steps of one.
    let mut steps = [0u32; 9];
    let mut pairs = 0u32;
    for (axis, name) in "xyz".chars().enumerate() {
        let mut repeats = 0u32;
        for i in 1..n {
            let a = (accel[i - 1][axis] * ACCEL_SCALE as f32).round() as i32;
            let b = (accel[i][axis] * ACCEL_SCALE as f32).round() as i32;
            let d = (b - a).unsigned_abs() as usize;
            steps[d.min(steps.len() - 1)] += 1;
            pairs += 1;
            if d == 0 {
                repeats += 1;
            }
        }
        print!(
            "  repeats {} {:>4.0}%  ",
            name,
            repeats as f32 * 100.0 / (n - 1) as f32
        );
    }
    println!();
    print!("  step (LSB, 1 = 0.01 m/s²)  ");
    for (d, count) in steps.iter().enumerate() {
        let label = if d == steps.len() - 1 {
            format!("{d}+")
        } else {
            d.to_string()
        };
        print!("{label}:{:.0}% ", *count as f32 * 100.0 / pairs as f32);
    }
    println!();

    // White noise is flat here and a bar in a rack is not, which is the distinction a single deviation cannot make.
    println!("  band power, share of variance in each band");
    for axis in 0..3 {
        let xs: Vec<f32> = accel.iter().map(|a| a[axis]).collect();
        let (shares, peak_hz, rms_1_10) = bands(&xs, dt_ms as f32 / 1000.0);
        println!(
            "    {}   0.5-1 {:>3.0}%  1-2 {:>3.0}%  2-5 {:>3.0}%  5-10 {:>3.0}%  10-50 {:>3.0}%   \
             peak {:.2} Hz, rms 1-10 Hz {:.2} mg",
            "xyz".chars().nth(axis).unwrap(),
            shares[0] * 100.0,
            shares[1] * 100.0,
            shares[2] * 100.0,
            shares[3] * 100.0,
            shares[4] * 100.0,
            peak_hz,
            rms_1_10 * 1000.0 / GRAVITY
        );
    }

    let gyro_mean = mean3(&gyro);
    println!(
        "  gyro mean   x {:+.2}  y {:+.2}  z {:+.2} dps   |  p-p {:.2} / {:.2} / {:.2} dps",
        gyro_mean[0],
        gyro_mean[1],
        gyro_mean[2],
        spread(&gyro, 0, n),
        spread(&gyro, 1, n),
        spread(&gyro, 2, n)
    );
}

/// The means side by side: the same orientation twice is the check that the run order was what the session said, and its two rows agreeing is the check that nothing drifted between them.
fn means(windows: &[Window]) {
    println!("\nwhat every window's mean was, in m/s² — the axis carrying 1 g is the mount's up");
    for (i, w) in windows.iter().enumerate() {
        let accel: Vec<[f32; 3]> = w.samples.iter().map(|s| s.specific_force_f32()).collect();
        if accel.len() < STILL_WINDOW {
            continue;
        }
        let m = mean3(&accel);
        let up = (0..3)
            .max_by(|a, b| m[*a].abs().partial_cmp(&m[*b].abs()).unwrap())
            .unwrap();
        println!(
            "  {}  {} {:+.3}  {:+.3}  {:+.3}   up on {}",
            i + 1,
            w.uuid,
            m[0],
            m[1],
            m[2],
            "xyz".chars().nth(up).unwrap()
        );
    }
}

/// Median, extremes and holes of the sample times.
fn timeline(samples: &[Sample]) -> (i64, i64, i64, u32) {
    let mut dt: Vec<i64> = samples
        .windows(2)
        .map(|w| w[1].t_ms as i64 - w[0].t_ms as i64)
        .collect();
    let holes = {
        let mut sorted = dt.clone();
        sorted.sort_unstable();
        let median = *sorted.get(sorted.len() / 2).unwrap_or(&0);
        dt.iter().filter(|d| **d > median * 3 / 2).count() as u32
    };
    dt.sort_unstable();
    let median = *dt.get(dt.len() / 2).unwrap_or(&0);
    let min = *dt.first().unwrap_or(&0);
    let max = *dt.last().unwrap_or(&0);
    (median, min, max, holes)
}

fn mean3(xs: &[[f32; 3]]) -> [f32; 3] {
    let mut sum = [0.0f32; 3];
    for x in xs {
        for k in 0..3 {
            sum[k] += x[k];
        }
    }
    let n = xs.len().max(1) as f32;
    [sum[0] / n, sum[1] / n, sum[2] / n]
}

fn magnitude(v: [f32; 3]) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Peak-to-peak of one axis over `samples[0..n]`.
fn spread(xs: &[[f32; 3]], axis: usize, n: usize) -> f32 {
    let mut lo = f32::MAX;
    let mut hi = f32::MIN;
    for x in &xs[..n] {
        lo = lo.min(x[axis]);
        hi = hi.max(x[axis]);
    }
    hi - lo
}

/// Both statistics the stillness gate takes from a window: how far the run's mean specific force sits from 1 g, and how far its worst sample does, each sorted for a percentile.
fn window_gate(xs: &[[f32; 3]], samples: &[Sample], dt_ms: i64) -> (Vec<f32>, Vec<f32>) {
    let (mut means, mut strays) = (Vec::new(), Vec::new());
    for start in 0..xs.len().saturating_sub(STILL_WINDOW - 1) {
        if !contiguous(samples, start, dt_ms) {
            continue;
        }
        let run = &xs[start..start + STILL_WINDOW];
        let mean = mean3(run);
        means.push((magnitude(mean) - GRAVITY).abs());
        strays.push(
            run.iter()
                .map(|x| (magnitude(*x) - GRAVITY).abs())
                .fold(0.0f32, f32::max),
        );
    }
    means.sort_by(|a, b| a.partial_cmp(b).unwrap());
    strays.sort_by(|a, b| a.partial_cmp(b).unwrap());
    (means, strays)
}

/// How far each window's mean angular rate sits from the rate the window itself has, which is what the gate compares with a bound: the offset is a constant and the filter takes it out, so what is left is how much the bar turned inside the run.
fn window_rates(xs: &[[f32; 3]], samples: &[Sample], dt_ms: i64) -> Vec<f32> {
    let mut out = Vec::new();
    let centre = mean3(xs);
    for start in 0..xs.len().saturating_sub(STILL_WINDOW - 1) {
        if !contiguous(samples, start, dt_ms) {
            continue;
        }
        let mean = mean3(&xs[start..start + STILL_WINDOW]);
        let d = [
            mean[0] - centre[0],
            mean[1] - centre[1],
            mean[2] - centre[2],
        ];
        out.push(magnitude(d));
    }
    out.sort_by(|a, b| a.partial_cmp(b).unwrap());
    out
}

/// Whether every sample in a run of `STILL_WINDOW` starts is contiguous — the gate sees samples, not positions, so a window the sampler gapped is one it never evaluated.
fn contiguous(samples: &[Sample], start: usize, dt_ms: i64) -> bool {
    samples[start..start + STILL_WINDOW - 1]
        .windows(2)
        .all(|p| (p[1].t_ms as i64 - p[0].t_ms as i64) <= dt_ms * 3 / 2)
}

/// The furthest any one sample in a run sits from 1 g (m/s²).
fn stray_slice(xs: &[[f32; 3]], start: usize, len: usize) -> f32 {
    xs[start..start + len]
        .iter()
        .map(|x| (magnitude(*x) - GRAVITY).abs())
        .fold(0.0f32, f32::max)
}

/// What share of samples would pass the gate's worst-sample test, and the longest unbroken run of them — the run `READY_S` counts toward.
fn quiet_runs(xs: &[[f32; 3]], samples: &[Sample], dt_ms: i64) -> (f32, i64) {
    let mut passed = 0usize;
    let mut total = 0usize;
    let mut run = 0i64;
    let mut longest = 0i64;
    for start in 0..xs.len().saturating_sub(STILL_WINDOW - 1) {
        // A gapped window ends the run as surely as a wide one: the filter is only asked about samples that arrived.
        if !contiguous(samples, start, dt_ms) {
            run = 0;
            continue;
        }
        total += 1;
        let quiet = stray_slice(xs, start, STILL_WINDOW) <= STILL_STRAY;
        if quiet {
            passed += 1;
            run += 1;
            longest = longest.max(run);
        } else {
            run = 0;
        }
    }
    let share = if total == 0 {
        0.0
    } else {
        passed as f32 / total as f32
    };
    (share, longest * dt_ms)
}

fn pct(sorted: &[f32], p: f32) -> f32 {
    if sorted.is_empty() {
        return 0.0;
    }
    let i = ((sorted.len() - 1) as f32 * p).round() as usize;
    sorted[i.min(sorted.len() - 1)]
}

/// Share of variance in five bands, the strongest frequency between 1 and 10 Hz, and the RMS of that band in m/s². A naive transform: a nine-second window is 450 bins, which is nothing.
fn bands(xs: &[f32], dt_s: f32) -> ([f32; 5], f32, f32) {
    let n = xs.len();
    let mean = xs.iter().sum::<f32>() / n as f32;
    let edges = [0.5f32, 1.0, 2.0, 5.0, 10.0, 50.0];
    let mut power = [0.0f32; 5];
    let mut total = 0.0f32;
    let mut peak_hz = 0.0f32;
    let mut peak_power = 0.0f32;
    for k in 1..=n / 2 {
        let hz = k as f32 / (n as f32 * dt_s);
        if hz > edges[5] {
            break;
        }
        let (mut re, mut im) = (0.0f32, 0.0f32);
        for (i, x) in xs.iter().enumerate() {
            let phase = 2.0 * std::f32::consts::PI * k as f32 * i as f32 / n as f32;
            re += (x - mean) * phase.cos();
            im -= (x - mean) * phase.sin();
        }
        let p = 2.0 * (re * re + im * im) / (n * n) as f32;
        total += p;
        for b in 0..5 {
            if hz >= edges[b] && hz < edges[b + 1] {
                power[b] += p;
            }
        }
        if (1.0..10.0).contains(&hz) && p > peak_power {
            peak_power = p;
            peak_hz = hz;
        }
    }
    let mut shares = [0.0f32; 5];
    if total > 0.0 {
        for b in 0..5 {
            shares[b] = power[b] / total;
        }
    }
    (shares, peak_hz, power[1].sqrt())
}

fn hex(uuid: &[u8]) -> String {
    uuid.iter().take(4).map(|b| format!("{b:02x}")).collect()
}
