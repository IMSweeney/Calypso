//! Writes the client's test fixtures to `calypso-app/assets/fixtures/`.
//! Fixtures are `.pblog` log dumps from the firmware's own encoders (see `calypso_sim::fixtures`), so they cannot drift from the wire format.
//!
//! Run from this crate: `cargo run --example gen_fixtures`

use std::fs;
use std::path::{Path, PathBuf};

use calypso_core::motion::fusion::GRAVITY;
use calypso_core::motion::rep::RepSeed;
use calypso_core::motion::trace::{compute_rep, TraceBuf, TracePoint, TRACE_POINTS};
use calypso_core::pb::{self, msg};
use calypso_core::sampler::{quantize, Sample, ACCEL_SCALE};
use calypso_core::store::RECORD_BUF_BYTES;
use calypso_core::uuid::fixture_uuid;
use calypso_sim::fixtures::{self, push_record, push_rep, set_dump};
use calypso_sim::motion::{Rep, Set};

fn repo_root() -> PathBuf {
    // <repo>/sim/examples/gen_fixtures.rs -> <repo>
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("calypso-core lives in the repo root")
        .to_path_buf()
}

fn write(dir: &Path, name: &str, bytes: &[u8]) -> std::io::Result<()> {
    let path = dir.join(name);
    fs::write(&path, bytes)?;
    println!("{:<28} {:>7} bytes", name, bytes.len());
    Ok(())
}

/// Push the bar sideways in proportion to how hard it is accelerating vertically, so the path bows: the vertical travel is unchanged, so the detector and the summary are unaffected, but the trace visibly leaves the vertical plane.
///
/// Proportional and one-signed, so the bar drifts left through the whole rep rather than returning to centre — a bow you can see, not one straight line traversed twice.
/// Measured against gravity rather than against zero, because a still sensor reads 1 g and a constant sideways push at rest is not a bow.
fn bow(samples: &mut [Sample], lateral_fraction: f32) {
    let rest = quantize(GRAVITY, ACCEL_SCALE);
    for s in samples.iter_mut() {
        let motion = s.accel[2].abs().saturating_sub(rest);
        s.accel[0] = (f32::from(motion) * lateral_fraction) as i16;
    }
}

fn main() -> std::io::Result<()> {
    let dir = repo_root().join("calypso-app/assets/fixtures");
    fs::create_dir_all(&dir)?;
    println!("writing fixtures to {}", dir.display());

    let mut rec_buf = [0u8; RECORD_BUF_BYTES];

    // A squat set: 0.4 m of travel per rep, 1.5 s each, 0.8 s of rest between.
    let squat = Set {
        rep: Rep::cosine(0.4, 1500),
        reps: 3,
        inter_rep_rest_ms: 800,
        unrack: None,
        rerack: None,
    };

    // 1. A normal set: three squat reps, the happy path for the list and plot.
    write(&dir, "synthetic-squat.pblog", &set_dump(&squat, 512))?;

    // 2. A single-rep set whose trace is one point: the degenerate plot case (axis fitting, camera fit and the polyline must all cope).
    {
        let mut dump = fixtures::new_dump(&mut rec_buf);
        let trace = vec![TracePoint {
            t_ms: 1_000,
            pos: [0.0, 0.0, -0.01],
            quat: [16_384, 0, 0, 0],
        }];
        let mut summary = msg::RepSummary {
            rep_uuid: pb::uuid_to_field(&fixture_uuid(1)),
            t_start_ms: 900,
            t_end_ms: 1_100,
            duration_ms: 200,
            computed_len: 1,
            ..Default::default()
        };
        push_rep(&mut dump, &mut rec_buf, &mut summary, &trace, None).unwrap();
        write(&dir, "single-point.pblog", &dump)?;
    }

    // 3. A long rep as the device would store it: more samples than the plotted trace keeps, so the path the app draws is a reduced sample of the motion.
    //
    //    Built by running the trace computation directly rather than through the detector: a smooth 5 s squat has a peak acceleration of ~0.5 m/s^2, under the detector's 0.6 m/s^2 start threshold.
    //    Real long reps are jerky (that is why a long rep can overflow the device buffer at all), but a smooth synthetic one cannot trip the detector.
    {
        let samples = Rep::cosine(1.2, 10_000).samples();
        let lead_in = fixtures::LEAD_IN_MS as usize / fixtures::SAMPLE_DT_MS as usize;
        let motion = 10_000 / fixtures::SAMPLE_DT_MS as usize;
        // Half a second either side, so the filter sees the bar at rest at both ends and anchors the trace there.
        let margin = 50;
        let window = &samples[lead_in - margin..lead_in + motion + margin];

        let mut storage = vec![TracePoint::ZERO; TRACE_POINTS];
        let mut trace = TraceBuf::new(&mut storage);
        let mut summary = compute_rep(window, &RepSeed::LEVEL, None, &mut trace);
        assert!(
            summary.computed_len as usize <= TRACE_POINTS
                && summary.computed_len as usize > TRACE_POINTS / 2,
            "a 10 s rep should be reduced to the plot's budget, not to nothing: {} points",
            summary.computed_len
        );

        summary.rep_uuid = pb::uuid_to_field(&fixture_uuid(1));
        let mut dump = fixtures::new_dump(&mut rec_buf);
        push_rep(
            &mut dump,
            &mut rec_buf,
            &mut summary,
            trace.as_slice(),
            None,
        )
        .unwrap();
        write(&dir, "long-decimated.pblog", &dump)?;
    }

    // 4. A rep whose trace fell off the device's ring: the summary is there, its chunks are not. The app must say "trace unavailable", not crash.
    {
        let mut dump = fixtures::new_dump(&mut rec_buf);
        let summary = msg::RepSummary {
            rep_uuid: pb::uuid_to_field(&fixture_uuid(7)),
            t_start_ms: 5_000,
            t_end_ms: 6_400,
            duration_ms: 1_400,
            rom: 0.41,
            peak_velocity: 0.7,
            computed_len: 140,
            computed_chunks: 3,
            ..Default::default()
        };
        let n = pb::encode_summary(&summary, &mut rec_buf).unwrap();
        push_record(&mut dump, &rec_buf[..n]);
        write(&dir, "ring-dropped.pblog", &dump)?;
    }

    // 5. Extreme values: every field at its maximum encoding, to check the decoders and the plot's axis fitting on pathological input.
    {
        let mut dump = fixtures::new_dump(&mut rec_buf);
        let trace: Vec<TracePoint> = (0..4)
            .map(|i| TracePoint {
                t_ms: u32::MAX - 40 + i * 10,
                pos: [f32::MAX, f32::MIN, -1.5e30],
                quat: [i16::MIN, i16::MAX, -16_384, 16_383],
            })
            .collect();
        let mut summary = msg::RepSummary {
            rep_uuid: pb::uuid_to_field(&fixture_uuid(99)),
            t_start_ms: u32::MAX - 100,
            t_end_ms: u32::MAX,
            duration_ms: 100,
            rom: f32::MAX,
            peak_velocity: f32::MIN,
            roll_min_deg: f32::MIN,
            roll_max_deg: f32::MAX,
            pitch_min_deg: f32::MIN,
            pitch_max_deg: f32::MAX,
            computed_len: trace.len() as u32,
            ..Default::default()
        };
        push_rep(&mut dump, &mut rec_buf, &mut summary, &trace, None).unwrap();
        write(&dir, "extreme-values.pblog", &dump)?;
    }

    // 6. A truncated dump: the last record ends mid-message, as a torn write or a dropped packet would leave it. Decoding must report the truncation.
    {
        let one = Set { reps: 1, ..squat };
        let full = set_dump(&one, 512);
        let cut = full.len() - 40;
        write(&dir, "truncated.pblog", &full[..cut])?;
    }

    // 7. A squat where the bar dips to one side: the vertical travel is the same, but the path bows, which is what the 3D plot is for.
    {
        let mut samples = Set { reps: 2, ..squat }.samples();
        bow(&mut samples, 0.35);
        let reps = fixtures::detect_and_trace(&samples, 512);
        assert_eq!(reps.len(), 2, "the dip should not stop the detector");
        let mut dump = fixtures::new_dump(&mut rec_buf);
        for (i, (mut summary, trace)) in reps.into_iter().enumerate() {
            let xs: Vec<f32> = trace.iter().map(|p| p.pos[0]).collect();
            let range = xs.iter().cloned().fold(f32::MIN, f32::max)
                - xs.iter().cloned().fold(f32::MAX, f32::min);
            assert!(
                range > 0.05,
                "the dipped path should bow sideways: {range} m"
            );
            summary.rep_uuid = pb::uuid_to_field(&fixture_uuid((i + 1) as u32));
            push_rep(&mut dump, &mut rec_buf, &mut summary, &trace, None).unwrap();
        }
        write(&dir, "dip-left.pblog", &dump)?;
    }

    // 8. An empty dump: no reps at all (fresh device, or a wrap that erased everything). The list and plot must handle it.
    write(&dir, "empty.pblog", &[])?;

    Ok(())
}
