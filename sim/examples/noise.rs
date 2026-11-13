//! What the sensors' noise actually is, measured off a recording, against the BNO055's datasheet.
//!
//! Every constant in the filter answers "how far does this reading move while nothing is happening", and a constant chosen without that number is a guess in either direction: too tight and a still bar never passes, too loose and a stroke passes for one.
//! This measures it where the device actually is — a bar at rest, on the mountings a session provides — and puts the answer beside the figure the chip's maker publishes.
//!
//! The rests are found without the filter's predicates: a stretch where the gyro stays under a degree per second and the specific force stays near 1 g.
//! Noise is pooled **within** each rest, because a sensor's orientation changes between them and pooling across would measure the mounting rather than the sensor.
//!
//! Usage: `cargo run --release --example noise -- <capture.pblog>...`, where each capture is a name in `captures/` or one the bucket holds.

use calypso_core::motion::fusion::{
    magnitude, GRAVITY, STILL_GRAVITY, STILL_RATE_DPS, STILL_STRAY, STILL_WINDOW,
};
use calypso_core::sampler::{Sample, ACCEL_SCALE, GYRO_SCALE, SAMPLE_RATE_HZ};
use calypso_sim::{captures, fixtures, stream};

/// The datasheet the figures below are quoted from: BST-BNO055-DS000-18.
///
/// The accelerometer's noise density is specified at `gFS2g`, and this device runs at ±8 g, so a reader comparing the two should know the ranges differ — the measurement is what the reading is at the range in use, whatever the scaling between them is.
const DATASHEET: &str = "BST-BNO055-DS000-18";

/// The accelerometer's output noise density in µg/√Hz, normal mode: typical, then maximum.
const ACCEL_DENSITY_UG: f32 = 150.0;
const ACCEL_DENSITY_UG_MAX: f32 = 190.0;
/// The gyroscope's output noise in °/s/√Hz at RFS2000, which is the range the chip's defaults put it in.
const GYRO_DENSITY_DPS: f32 = 0.014;

/// The bandwidths a density is integrated over to become an RMS: the accelerometer's chosen by the firmware, the gyroscope's the chip's default.
const ACCEL_BW_HZ: f32 = 62.5;
const GYRO_BW_HZ: f32 = 32.0;
/// A 2nd-order filter's noise-equivalent bandwidth, as a multiple of its −3 dB bandwidth.
const NOISE_BANDWIDTH: f32 = 1.11;

/// A rest is this long (s) with the gyro under `REST_RATE_DPS` and the specific force within `REST_FORCE` of 1 g.
const MIN_REST_S: f32 = 3.0;
const REST_RATE_DPS: f32 = 1.0;
const REST_FORCE: f32 = 0.5;

/// Samples between which the clock may not jump for a stretch to count as one rest, which is how a stored capture's windows stay separate.
const REST_MAX_STEP_MS: u32 = 200;

const RAD_PER_DPS: f32 = core::f32::consts::PI / 180.0;

fn main() {
    let names: Vec<String> = std::env::args().skip(1).collect();
    if names.is_empty() {
        eprintln!("usage: noise <capture.pblog>...");
        std::process::exit(2);
    }

    println!(
        "{DATASHEET}: accel {ACCEL_DENSITY_UG:.0} typ / {ACCEL_DENSITY_UG_MAX:.0} max µg/√Hz over {ACCEL_BW_HZ} Hz, gyro {GYRO_DENSITY_DPS} dps/√Hz over {GYRO_BW_HZ} Hz\n"
    );

    for name in &names {
        let samples = samples_of(&captures::bytes(name));
        let rests = rests(&samples);
        if rests.is_empty() {
            println!(
                "{name}: {} samples, no rest of {MIN_REST_S} s",
                samples.len()
            );
            continue;
        }

        let quiet: usize = rests.iter().map(|(a, b)| b - a).sum();
        println!(
            "{name}: {} samples, {:.1} s in {} rests",
            samples.len(),
            quiet as f32 / SAMPLE_RATE_HZ as f32,
            rests.len()
        );

        let force_x: Vec<f32> = samples.iter().map(|s| s.specific_force_f32()[0]).collect();
        let force_y: Vec<f32> = samples.iter().map(|s| s.specific_force_f32()[1]).collect();
        let force_z: Vec<f32> = samples.iter().map(|s| s.specific_force_f32()[2]).collect();
        let rate_x: Vec<f32> = samples.iter().map(|s| s.gyro_f32()[0]).collect();
        let rate_y: Vec<f32> = samples.iter().map(|s| s.gyro_f32()[1]).collect();
        let rate_z: Vec<f32> = samples.iter().map(|s| s.gyro_f32()[2]).collect();

        let (ax, ay, az) = (
            within_sigma(&force_x, &rests),
            within_sigma(&force_y, &rests),
            within_sigma(&force_z, &rests),
        );
        let (gx, gy, gz) = (
            within_sigma(&rate_x, &rests),
            within_sigma(&rate_y, &rests),
            within_sigma(&rate_z, &rests),
        );
        let accel_rms = ((ax * ax + ay * ay + az * az) / 3.0).sqrt();
        let gyro_rms = ((gx * gx + gy * gy + gz * gz) / 3.0).sqrt();

        println!(
            "  accel σ   x {ax:.4}   y {ay:.4}   z {az:.4} m/s²   → {:.0} µg/√Hz",
            accel_rms / (NOISE_BANDWIDTH * ACCEL_BW_HZ).sqrt() / GRAVITY * 1e6
        );
        println!(
            "  gyro  σ   x {gx:.4}   y {gy:.4}   z {gz:.4} dps    → {:.4} dps/√Hz",
            gyro_rms / (NOISE_BANDWIDTH * GYRO_BW_HZ).sqrt()
        );
        println!(
            "  resolution   accel {:.4} m/s², gyro {:.4} dps (a reading quantized to its own register cannot be quieter)",
            1.0 / ACCEL_SCALE as f32 / 12.0f32.sqrt(),
            1.0 / GYRO_SCALE as f32 / 12.0f32.sqrt()
        );

        let (mean_dev, stray, mean_rate) = window_stats(&samples, &rests);
        println!("  the stillness gates' statistics over a {STILL_WINDOW}-sample window, each rest's own mean removed:");
        report("mean |f| − g", &mean_dev, "m/s²", STILL_GRAVITY);
        report("stray", &stray, "m/s²", STILL_STRAY);
        report("mean |gyro|", &mean_rate, "dps", STILL_RATE_DPS);

        let rests_vec: Vec<([f32; 3], f32)> = rests
            .iter()
            .map(|&(a, b)| {
                let n = (b - a) as f32;
                let mut f = [0.0f32; 3];
                let mut g = [0.0f32; 3];
                for s in &samples[a..b] {
                    let sf = s.specific_force_f32();
                    let sg = s.gyro_f32();
                    for k in 0..3 {
                        f[k] += sf[k];
                        g[k] += sg[k];
                    }
                }
                (f.map(|v| v / n), magnitude(g.map(|v| v / n)))
            })
            .collect();
        let offsets: Vec<f32> = rests_vec
            .iter()
            .map(|(f, _)| (magnitude(*f) - GRAVITY) * 1000.0 / GRAVITY)
            .collect();
        let biases: Vec<f32> = rests_vec.iter().map(|(_, g)| *g).collect();
        println!(
            "  what is not noise, across the {} rests: |f| − 1 g {:.1}..{:.1} mg, |gyro| {:.3}..{:.3} dps",
            rests.len(),
            min(&offsets),
            max(&offsets),
            min(&biases),
            max(&biases)
        );

        let crossover = (accel_rms / GRAVITY) / (gyro_rms * RAD_PER_DPS);
        println!(
            "  crossover   σ_accel/g ÷ σ_gyro = {crossover:.2} s, which is the τ the accelerometer and the gyro balance at"
        );
        println!();
    }
}

/// Every sample a capture holds, whether it is a recording of the live stream or a device log's stored windows.
fn samples_of(dump: &[u8]) -> Vec<Sample> {
    let (windows, _) = fixtures::capture_windows(dump);
    if windows.iter().any(|w| !w.samples.is_empty()) {
        return windows.into_iter().flat_map(|w| w.samples).collect();
    }
    stream::samples(dump)
}

/// Every stretch that could be a bar at rest, as `[start, end)` sample indices.
fn rests(samples: &[Sample]) -> Vec<(usize, usize)> {
    let min = (MIN_REST_S * SAMPLE_RATE_HZ as f32) as usize;
    let mut out: Vec<(usize, usize)> = Vec::new();
    let mut start = 0;
    for i in 0..=samples.len() {
        let continuous = i == 0
            || i >= samples.len()
            || samples[i].t_ms.saturating_sub(samples[i - 1].t_ms) <= REST_MAX_STEP_MS;
        let holds = i < samples.len()
            && continuous
            && magnitude(samples[i].gyro_f32()) < REST_RATE_DPS
            && (magnitude(samples[i].specific_force_f32()) - GRAVITY).abs() < REST_FORCE;
        if holds {
            continue;
        }
        if i - start >= min {
            out.push((start, i));
        }
        start = i + 1;
    }
    out
}

/// The spread of one channel within the rests, pooled across them: each rest is measured against its own mean, so the mounting's offset and orientation drop out.
fn within_sigma(values: &[f32], rests: &[(usize, usize)]) -> f32 {
    let (mut sum, mut n) = (0.0f64, 0usize);
    for &(a, b) in rests {
        let slice = &values[a..b];
        if slice.len() < 2 {
            continue;
        }
        let mean = slice.iter().map(|v| *v as f64).sum::<f64>() / slice.len() as f64;
        sum += slice
            .iter()
            .map(|v| (*v as f64 - mean).powi(2))
            .sum::<f64>();
        n += slice.len();
    }
    if n == 0 {
        return 0.0;
    }
    (sum / n as f64).sqrt() as f32
}

/// The three statistics the stillness predicates are asked about, taken over every window a rest holds with that rest's own mean removed.
fn window_stats(samples: &[Sample], rests: &[(usize, usize)]) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
    let (mut mean_dev, mut stray, mut mean_rate) = (Vec::new(), Vec::new(), Vec::new());
    for &(a, b) in rests {
        if b - a < STILL_WINDOW {
            continue;
        }
        let force: Vec<[f32; 3]> = samples[a..b]
            .iter()
            .map(|s| s.specific_force_f32())
            .collect();
        let rate: Vec<[f32; 3]> = samples[a..b].iter().map(|s| s.gyro_f32()).collect();
        let rest_force = mean_of(&force);
        let rest_rate = mean_of(&rate);
        // The rest's mean is what the offset would be with gravity still in it, so gravity goes back: what is left is the reading minus the offset the boot would have learned, which is the quantity the gates are asked about.
        let magnitude_rest = magnitude(rest_force);
        let up = if magnitude_rest > 0.0 {
            rest_force.map(|v| v / magnitude_rest)
        } else {
            [0.0, 0.0, 1.0]
        };

        for w in 0..=force.len() - STILL_WINDOW {
            let (mut sum_f, mut sum_g, mut worst) = ([0.0f32; 3], [0.0f32; 3], 0.0f32);
            for i in w..w + STILL_WINDOW {
                let d = relative(force[i], rest_force);
                let f = [
                    d[0] + up[0] * GRAVITY,
                    d[1] + up[1] * GRAVITY,
                    d[2] + up[2] * GRAVITY,
                ];
                let g = relative(rate[i], rest_rate);
                for k in 0..3 {
                    sum_f[k] += f[k];
                    sum_g[k] += g[k];
                }
                worst = worst.max((magnitude(f) - GRAVITY).abs());
            }
            mean_dev.push((magnitude(sum_f.map(|v| v / STILL_WINDOW as f32)) - GRAVITY).abs());
            stray.push(worst);
            mean_rate.push(magnitude(sum_g.map(|v| v / STILL_WINDOW as f32)));
        }
    }
    (mean_dev, stray, mean_rate)
}

fn mean_of(v: &[[f32; 3]]) -> [f32; 3] {
    let n = v.len() as f32;
    let mut sum = [0.0f32; 3];
    for s in v {
        for k in 0..3 {
            sum[k] += s[k];
        }
    }
    sum.map(|x| x / n)
}

fn relative(v: [f32; 3], mean: [f32; 3]) -> [f32; 3] {
    [v[0] - mean[0], v[1] - mean[1], v[2] - mean[2]]
}

/// Print one measured statistic beside the constant that judges it, so the two are read together.
fn report(label: &str, values: &[f32], unit: &str, constant: f32) {
    if values.is_empty() {
        return;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let pick = |p: f32| sorted[((sorted.len() - 1) as f32 * p) as usize];
    println!(
        "    {label:<12} median {:.4}  p99 {:.4}  max {:.4} {unit:<4}  (the constant is {constant})",
        pick(0.5),
        pick(0.99),
        pick(1.0),
    );
}

fn min(v: &[f32]) -> f32 {
    v.iter().copied().reduce(f32::min).unwrap_or(0.0)
}

fn max(v: &[f32]) -> f32 {
    v.iter().copied().reduce(f32::max).unwrap_or(0.0)
}
