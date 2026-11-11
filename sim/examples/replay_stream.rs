//! Replay a phone's recording of the live stream through the filter and the detector, and print the
//! reps it finds.
//!
//! A `.pblog` the app writes while it records holds samples and nothing else, so `calypso_sim::stream`
//! puts them back through the firmware's own `Fusion` and `Detector`, and this prints what came out.
//! It is the instrument for one session; what a session is *supposed* to look like lives in
//! `captures/captures.json` and is checked by `tests/captures.rs`.
//!
//! Usage: `cargo run --release --example replay_stream -- <capture.pblog> [--series]`, where the
//! capture is a name in `captures/` or one the bucket holds.

use std::env;

use calypso_core::fusion::{Filtered, GRAVITY};
use calypso_core::sampler::{Sample, SAMPLE_RATE_HZ};
use calypso_sim::{captures, stream};
use libm::atan2f;

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    let Some(name) = args.first() else {
        eprintln!("usage: replay_stream <capture.pblog> [--series]");
        std::process::exit(2);
    };
    let series = args.iter().any(|a| a == "--series");

    let samples = stream::samples(&captures::bytes(name));
    let Some(first) = samples.first() else {
        println!("{name}: no samples");
        return;
    };
    let t0 = first.t_ms;
    let end = samples.last().map_or(t0, |s| s.t_ms);
    println!(
        "{name}: {} samples, {:.1} s",
        samples.len(),
        (end - t0) as f32 / 1000.0
    );
    holes(&samples, t0);

    // One line a sample: the acceleration, the filter's velocity, and the speed the detector counts on, which is what a threshold or a turn rule has to be read against; `tilt` is how far the world frame's gravity is off vertical, and a velocity that runs away is made of it.
    let mut each = |s: &Sample, f: &Filtered, counted: f32| {
        if !series {
            return;
        }
        let v = f.state.velocity;
        let a = f.linear_accel;
        let tilt = atan2f((a[0] * a[0] + a[1] * a[1]).sqrt(), a[2] + GRAVITY).to_degrees();
        println!(
            "{:.2}\t{:+.2}\t{:+.2}\t{:.2}\t{:+.2}\t{:+.2}\t{:+.1}\t{}\t{:+.2}",
            s.t_ms.saturating_sub(t0) as f32 / 1000.0,
            a[2],
            v[2],
            (v[0] * v[0] + v[1] * v[1]).sqrt(),
            a[0],
            a[1],
            tilt,
            if f.still { 1 } else { 0 },
            counted
        );
    };
    let out = stream::replay(&samples, &mut each);

    for (i, (rep, rom)) in out.reps.iter().zip(&out.roms).enumerate() {
        println!(
            "  rep {:>2}  {:.2} s .. {:.2} s  ({} ms)  rom {:.2} m",
            i + 1,
            rep.span.start_ms.saturating_sub(t0) as f32 / 1000.0,
            rep.span.end_ms.saturating_sub(t0) as f32 / 1000.0,
            rep.span.duration_ms(),
            rom
        );
    }

    let ready = out.ready_at_ms.map_or("-".to_string(), |t| {
        format!("{:.2} s", t.saturating_sub(t0) as f32 / 1000.0)
    });
    println!(
        "  filter      ready at {ready}, still {:.0}% of the stream, peak |v| {:.2} m/s (horiz {:.2})",
        out.still as f32 * 100.0 / samples.len() as f32,
        out.peak_speed,
        out.peak_horizontal
    );
    println!(
        "  detector    {} reps, {} halves",
        out.reps.len(),
        out.halves
    );
}

/// Where the stream has a gap in it, which is the sampler stalling rather than anything the filter did.
///
/// A hole matters here for the same reason it matters on the device: the detector's windows are cut
/// from the ring by time, so a stall is missing samples inside a rep, and a rep that reads short is
/// the stall's doing before it is the detector's.
fn holes(samples: &[Sample], t0: u32) {
    let expected = 1000 / SAMPLE_RATE_HZ;
    let mut found: Vec<(u32, u32)> = samples
        .windows(2)
        .filter_map(|pair| {
            let d = pair[1].t_ms.saturating_sub(pair[0].t_ms);
            (d > expected * 3 / 2).then(|| (pair[0].t_ms.saturating_sub(t0), d))
        })
        .collect();
    if found.is_empty() {
        println!("  timeline    no holes");
        return;
    }
    found.sort_by(|a, b| b.1.cmp(&a.1));
    print!("  timeline    {} holes, largest:", found.len());
    for (at, d) in found.iter().take(6) {
        print!(" {} ms at +{} ms", d, at);
    }
    println!();
}
