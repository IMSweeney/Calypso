//! Writes the client's test fixtures to `calypso-app/assets/fixtures/`.
//!
//! Fixtures are `.pblog` log dumps — the same records the device stores,
//! produced by the firmware's own encoders (see `calypso_core::fixtures`), so
//! they cannot drift from the wire format.
//!
//! Run from the repo root:
//!
//! ```text
//! cargo run -p calypso-core --example gen_fixtures
//! ```

use std::fs;
use std::path::{Path, PathBuf};

use calypso_core::fixtures::{self, push_rep, push_record, squat_set};
use calypso_core::pb::{self, msg};
use calypso_core::rep::{compute_rep, TraceBuf, TracePoint};
use calypso_core::store::RECORD_BUF_BYTES;
use calypso_core::uuid::fixture_uuid;

fn repo_root() -> PathBuf {
    // <repo>/calypso-core/examples/gen_fixtures.rs -> <repo>
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

fn main() -> std::io::Result<()> {
    let dir = repo_root().join("calypso-app/assets/fixtures");
    fs::create_dir_all(&dir)?;
    println!("writing fixtures to {}", dir.display());

    let mut rec_buf = [0u8; RECORD_BUF_BYTES];

    // 1. A normal set: three squat reps, the happy path for the list and plot.
    write(&dir, "synthetic-squat.pblog", &fixtures::squat_dump(3, 0.4))?;

    // 2. A single-rep set whose trace is one point: the degenerate plot case
    //    (axis fitting, camera fit and the polyline must all cope).
    {
        let mut dump = Vec::new();
        let trace = vec![TracePoint {
            t_ms: 1_000,
            pos: [0.0, 0.0, -0.01],
            accel: [0, 0, -100],
            quat: [16_384, 0, 0, 0],
        }];
        let mut summary = msg::RepSummary {
            rep_uuid: pb::uuid_to_field(&fixture_uuid(1)),
            t_start_ms: 900,
            t_end_ms: 1_100,
            duration_ms: 200,
            trace_len: 1,
            ..Default::default()
        };
        push_rep(&mut dump, &mut rec_buf, &mut summary, &trace).unwrap();
        write(&dir, "single-point.pblog", &dump)?;
    }

    // 3. A long rep as the device would store it: more trace points than the
    //    device's 512-point buffer, so the trace comes back decimated with
    //    `decimated = true`.
    //
    //    Built by running the trace computation directly rather than through
    //    the detector: a smooth 5 s squat has a peak acceleration of ~0.5 m/s^2,
    //    under the detector's 0.6 m/s^2 start threshold. Real long reps are
    //    jerky (that is why a long rep can overflow the device buffer at all),
    //    but a smooth synthetic one cannot trip the detector.
    {
        let (samples, _) = squat_set(1, 10_000, 1_000, 1.2);
        let lead_in = 300 / fixtures::SAMPLE_DT_MS as usize;
        let motion = 10_000 / fixtures::SAMPLE_DT_MS as usize;
        let margin = 12; // 120 ms either side -> 1024 samples, halved to 512
        let window = &samples[lead_in - margin..lead_in + motion + margin];

        let mut storage = vec![TracePoint::ZERO; 512];
        let mut trace = TraceBuf::new(&mut storage);
        let mut summary = compute_rep(window, &mut trace);
        assert!(summary.decimated, "a 10 s rep should exceed the 512-point buffer");
        assert_eq!(summary.trace_len, 512);

        summary.rep_uuid = pb::uuid_to_field(&fixture_uuid(1));
        let mut dump = Vec::new();
        push_rep(&mut dump, &mut rec_buf, &mut summary, trace.as_slice()).unwrap();
        write(&dir, "long-decimated.pblog", &dump)?;
    }

    // 4. A rep whose trace fell off the device's ring: the summary is there,
    //    its chunks are not. The app must say "trace unavailable", not crash.
    {
        let mut dump = Vec::new();
        let summary = msg::RepSummary {
            rep_uuid: pb::uuid_to_field(&fixture_uuid(7)),
            t_start_ms: 5_000,
            t_end_ms: 6_400,
            duration_ms: 1_400,
            rom: 0.41,
            peak_velocity: 0.7,
            trace_len: 140,
            chunk_count: 3,
            ..Default::default()
        };
        let n = pb::encode_summary(&summary, &mut rec_buf).unwrap();
        push_record(&mut dump, &rec_buf[..n]);
        write(&dir, "ring-dropped.pblog", &dump)?;
    }

    // 5. Extreme values: every field at its maximum encoding, to check the
    //    decoders and the plot's axis fitting on pathological input.
    {
        let mut dump = Vec::new();
        let trace: Vec<TracePoint> = (0..4)
            .map(|i| TracePoint {
                t_ms: u32::MAX - 40 + i * 10,
                pos: [f32::MAX, f32::MIN, -1.5e30],
                accel: [i16::MIN, i16::MAX, -32_767],
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
            mean_velocity: f32::MAX,
            peak_accel: f32::MAX,
            roll_min_deg: f32::MIN,
            roll_max_deg: f32::MAX,
            pitch_min_deg: f32::MIN,
            pitch_max_deg: f32::MAX,
            trace_len: trace.len() as u32,
            decimated: true,
            ..Default::default()
        };
        push_rep(&mut dump, &mut rec_buf, &mut summary, &trace).unwrap();
        write(&dir, "extreme-values.pblog", &dump)?;
    }

    // 6. A truncated dump: the last record ends mid-message, as a torn write or
    //    a dropped packet would leave it. Decoding must report the truncation.
    {
        let full = fixtures::squat_dump(1, 0.4);
        let cut = full.len() - 40;
        write(&dir, "truncated.pblog", &full[..cut])?;
    }

    // 7. A squat where the bar dips to one side: the vertical travel is the
    //    same, but the path bows, which is what the 3D plot is for.
    {
        let (samples, _) = fixtures::dipped_squat_set(2, 0.35);
        let reps = fixtures::detect_and_trace(&samples, 512);
        assert_eq!(reps.len(), 2, "the dip should not stop the detector");
        let mut dump = Vec::new();
        for (i, (mut summary, trace)) in reps.into_iter().enumerate() {
            let xs: Vec<f32> = trace.iter().map(|p| p.pos[0]).collect();
            let range = xs.iter().cloned().fold(f32::MIN, f32::max)
                - xs.iter().cloned().fold(f32::MAX, f32::min);
            assert!(range > 0.05, "the dipped path should bow sideways: {range} m");
            summary.rep_uuid = pb::uuid_to_field(&fixture_uuid((i + 1) as u32));
            push_rep(&mut dump, &mut rec_buf, &mut summary, &trace).unwrap();
        }
        write(&dir, "dip-left.pblog", &dump)?;
    }

    // 8. An empty dump: no reps at all (fresh device, or a wrap that erased
    //    everything). The list and plot must handle it.
    write(&dir, "empty.pblog", &[])?;

    Ok(())
}
