//! Every stage of the pipeline for one recording, as CSV a plotting script can read.
//!
//! The device only ever hands out the reps it closed, so what happened inside them — the filter's frame, the cues, the velocity the integrator carried — is not visible anywhere else. This writes one row per sample for the whole recording, plus the reps the detector closed and the trace each was given, and leaves the drawing to whoever reads it.
//!
//! Usage: `cargo run --release --example pipeline_dump -- <capture> [--out DIR]`. The capture is a name in `captures/` or one the bucket holds; the default output is the working directory.

use std::env;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::PathBuf;

use calypso_core::motion::integrate::Anchor;
use calypso_core::motion::pipeline::Pipeline;
use calypso_sim::{captures, replay, stream};

/// The gap that separates one set from the next: the rest between sets is minutes, and the pause at the top of a stroke is a couple of seconds.
const SET_GAP_MS: u32 = 30_000;

/// A turn as a cell: a rep that has no turn there — one stroke from landing to landing has nothing between its halves — writes nothing rather than a time it never had.
fn turn(anchor: Option<Anchor>) -> String {
    anchor.map_or_else(String::new, |a| a.t_ms.to_string())
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    let Some(name) = args.first() else {
        eprintln!("usage: pipeline_dump <capture> [--out DIR]");
        std::process::exit(2);
    };
    let out: PathBuf = args
        .iter()
        .position(|a| a == "--out")
        .and_then(|i| args.get(i + 1))
        .map_or_else(|| PathBuf::from("."), PathBuf::from);
    fs::create_dir_all(&out).unwrap_or_else(|e| panic!("{}: {e}", out.display()));

    let samples = stream::samples(&captures::bytes(name));
    let found = replay::run(
        &samples,
        Pipeline::new(),
        replay::TRACE_CAPACITY,
        &mut |_, _, _, _| {},
    );

    let mut reps = BufWriter::new(File::create(out.join("reps.csv")).expect("reps.csv"));
    writeln!(
        reps,
        "set,rep,t_start_ms,t_end_ms,turn0_ms,turn1_ms,turn2_ms,duration_ms,rom_m,back_m"
    )
    .unwrap();
    let mut traces = BufWriter::new(File::create(out.join("traces.csv")).expect("traces.csv"));
    writeln!(traces, "set,rep,t_ms,pos_x,pos_y,pos_z").unwrap();

    let mut set = 0;
    let mut prev_end: Option<u32> = None;
    for (i, rep) in found.reps.iter().enumerate() {
        if prev_end.is_some_and(|end| rep.span.start_ms.saturating_sub(end) > SET_GAP_MS) {
            set += 1;
        }
        prev_end = Some(rep.span.end_ms);

        let trace = &found.traces[i];
        let back = match (trace.first(), trace.last()) {
            (Some(first), Some(last)) => (last.pos[2] - first.pos[2]).abs(),
            _ => 0.0,
        };
        writeln!(
            reps,
            "{set},{i},{},{},{},{},{},{},{:.4},{back:.4}",
            rep.span.start_ms,
            rep.span.end_ms,
            turn(rep.turns.zeros[0]),
            turn(rep.turns.zeros[1]),
            turn(rep.turns.zeros[2]),
            rep.span.duration_ms(),
            found.summaries[i].rom,
        )
        .unwrap();
        for point in trace {
            writeln!(
                traces,
                "{set},{i},{},{:.4},{:.4},{:.4}",
                point.t_ms, point.pos[0], point.pos[1], point.pos[2]
            )
            .unwrap();
        }
    }

    let mut csv = BufWriter::new(File::create(out.join("samples.csv")).expect("samples.csv"));
    writeln!(
        csv,
        "t_ms,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,ax,ay,az,rx,ry,rz,rate_mag,tilt_deg,sign_vert,sign_change,resting,ready,vx,vy,vz,vcross_x,vcross_y,vcross_z,rep_open"
    )
    .unwrap();
    const DEG: f32 = 180.0 / core::f32::consts::PI;
    let mut pipeline = Pipeline::new();
    for s in &samples {
        let step = pipeline.update(s, true);
        let raw = s.specific_force_f32();
        let gyro = s.gyro_f32();
        let a = step.filtered.linear_accel;
        let r = step.filtered.rate;
        let v = step.integrated.velocity;
        let c = step.integrated.v_cross;
        writeln!(
            csv,
            "{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.4},{:.4},{:.4},{:.5},{:.5},{:.5},{:.4},{:.3},{},{},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{}",
            s.t_ms,
            raw[0], raw[1], raw[2],
            gyro[0], gyro[1], gyro[2],
            a[0], a[1], a[2],
            r[0], r[1], r[2],
            step.cues.rate_mag,
            step.cues.tilt * DEG,
            step.cues.sign_vert,
            i32::from(step.cues.sign_change),
            i32::from(step.filtered.resting),
            i32::from(step.filtered.ready),
            v[0], v[1], v[2],
            c[0], c[1], c[2],
            i32::from(pipeline.rep_open()),
        )
        .unwrap();
    }
    eprintln!(
        "{name}: {} samples, {} reps in {} sets -> {}",
        samples.len(),
        found.reps.len(),
        set + 1,
        out.display()
    );
}
