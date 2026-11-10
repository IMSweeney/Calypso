use calypso_core::pb::{self, msg, Record, TRACE_CHUNK_SAMPLES};
use calypso_core::rep::TracePoint;
use calypso_core::uuid::fixture_uuid;

fn sample_summary() -> msg::RepSummary {
    let mut summary = msg::RepSummary {
        t_start_ms: 12_345,
        t_end_ms: 13_800,
        duration_ms: 1455,
        rom: 0.412,
        peak_velocity: -0.71,
        roll_min_deg: -1.25,
        roll_max_deg: 2.4,
        pitch_min_deg: -0.8,
        pitch_max_deg: 3.1,
        computed_len: 145,
        raw_len: 1_455,
        ..Default::default()
    };
    summary.set_state(msg::RepState {
        vx: 0.12,
        vy: -0.05,
        vz: -0.31,
        qw: 0.9,
        qx: -0.1,
        qy: 0.2,
        qz: 0.3,
        accel_bias_x: 0.01,
        accel_bias_y: -0.02,
        accel_bias_z: 0.03,
        gyro_bias_x: 0.001,
        gyro_bias_y: -0.002,
        gyro_bias_z: 0.003,
        settled: true,
    });
    summary.set_config(msg::RepConfig {
        accel_range_g: 8,
        gyro_range_dps: 500,
        accel_bandwidth_hz: 62,
        gyro_bandwidth_hz: 32,
        sample_rate_hz: 100,
        detector_version: 1,
    });
    summary
}

fn sample_point(i: usize, t0: u32) -> TracePoint {
    TracePoint {
        t_ms: t0 + (i as u32) * 10,
        pos: [i as f32 * 0.001, -0.2, 0.35],
        quat: [16384, 0, -2048, 2048],
    }
}

#[test]
fn summary_round_trips() {
    let mut summary = sample_summary();
    summary.rep_uuid = pb::uuid_to_field(&fixture_uuid(7));
    summary.computed_chunks = 4;
    let mut buf = [0u8; 256];
    let n = pb::encode_summary(&summary, &mut buf).unwrap();
    assert!(n > 0 && n < buf.len());

    match pb::decode_record(&buf[..n]).unwrap() {
        Record::Summary(s) => {
            assert_eq!(pb::summary_uuid(&s), Some(fixture_uuid(7)));
            assert_eq!(s.t_start_ms, summary.t_start_ms);
            assert_eq!(s.t_end_ms, summary.t_end_ms);
            assert_eq!(s.duration_ms, summary.duration_ms);
            assert!((s.rom - summary.rom).abs() < 1e-6);
            assert!((s.peak_velocity - summary.peak_velocity).abs() < 1e-6);
            assert!((s.roll_min_deg - summary.roll_min_deg).abs() < 1e-6);
            assert!((s.pitch_max_deg - summary.pitch_max_deg).abs() < 1e-6);
            assert_eq!(s.computed_len, summary.computed_len);
            assert_eq!(s.raw_len, summary.raw_len, "the raw trace's size survives");
            let state = pb::state_from_message(&s.state);
            assert_eq!(state.velocity[0], 0.12);
            assert_eq!(state.attitude[3], 0.3);
            assert_eq!(state.accel_bias[2], 0.03);
            assert_eq!(state.gyro_bias[1], -0.002);
            assert!(state.settled, "whether the bar was at rest survives");
            assert_eq!(s.config.accel_range_g, 8);
            assert_eq!(s.config.detector_version, 1);
            assert_eq!(s.computed_chunks, 4);
        }
        other => panic!("expected a summary, got {other:?}"),
    }
}

#[test]
fn computed_chunk_round_trips() {
    let points: Vec<TracePoint> = (0..TRACE_CHUNK_SAMPLES)
        .map(|i| sample_point(i, 500))
        .collect();
    let mut buf = [0u8; 2048];
    let n = pb::encode_computed_chunk(fixture_uuid(7), 2, 500, 9, &points, &mut buf).unwrap();

    match pb::decode_record(&buf[..n]).unwrap() {
        Record::Computed(chunk) => {
            assert_eq!(pb::computed_uuid(&chunk), Some(fixture_uuid(7)));
            assert_eq!(chunk.index, 2);
            assert_eq!(chunk.first_t_offset_ms, 0);
            assert_eq!(chunk.samples.len(), TRACE_CHUNK_SAMPLES);
            assert_eq!(chunk.total_chunks, 9, "the stream total survives the wire");
            for (i, s) in chunk.samples.iter().enumerate() {
                let back = pb::computed_point_from_message(s);
                let want = &points[i];
                assert_eq!(back.t_ms, want.t_ms - 500, "t_offset at {i}");
                assert_eq!(back.quat, want.quat);
                assert!((back.pos[0] - want.pos[0]).abs() < 1e-9);
                assert!((back.pos[1] - want.pos[1]).abs() < 1e-9);
                assert!((back.pos[2] - want.pos[2]).abs() < 1e-9);
            }
        }
        other => panic!("expected a trace chunk, got {other:?}"),
    }
}

#[test]
fn chunk_count_covers_the_trace() {
    assert_eq!(pb::chunk_count_for(0), 0);
    assert_eq!(pb::chunk_count_for(1), 1);
    assert_eq!(pb::chunk_count_for(TRACE_CHUNK_SAMPLES), 1);
    assert_eq!(pb::chunk_count_for(TRACE_CHUNK_SAMPLES + 1), 2);
    assert_eq!(pb::chunk_count_for(512), 11);
}

#[test]
fn full_chunk_fits_one_flash_sector() {
    let worst = TracePoint {
        t_ms: u32::MAX,
        pos: [f32::MAX, f32::MIN, -1.5e30],
        quat: [i16::MIN, i16::MAX, -16384, 16383],
    };
    let points = vec![worst; TRACE_CHUNK_SAMPLES];
    let mut buf = [0u8; 4096];
    let n = pb::encode_computed_chunk(fixture_uuid(1), 0, 0, 0, &points, &mut buf).unwrap();
    assert!(
        n <= 4076,
        "chunk encoded to {n} bytes, over the record limit"
    );
    assert!(
        n <= calypso_core::store::RECORD_BUF_BYTES,
        "chunk encoded to {n} bytes, over RECORD_BUF_BYTES"
    );
}

#[test]
fn undersized_buffer_is_reported() {
    let points: Vec<TracePoint> = (0..TRACE_CHUNK_SAMPLES)
        .map(|i| sample_point(i, 0))
        .collect();
    let mut tiny = [0u8; 8];
    assert_eq!(
        pb::encode_computed_chunk(fixture_uuid(1), 0, 0, 0, &points, &mut tiny),
        Err(pb::Error::BufferTooSmall)
    );
}

#[test]
fn corrupt_bytes_do_not_decode() {
    assert_eq!(pb::decode_record(&[0x00]), Err(pb::Error::Decode));
    assert_eq!(pb::decode_record(&[0x0A, 0x7F]), Err(pb::Error::Decode));
    assert_eq!(pb::decode_record(&[]), Err(pb::Error::Empty));
}

#[test]
fn trace_request_round_trips_in_its_envelope() {
    let mut buf = [0u8; 32];
    let request = msg::TraceRequest {
        kind: msg::TraceKind::Computed,
        rep_uuid: pb::uuid_to_field(&fixture_uuid(42)),
    };
    let n = pb::encode_trace_request(&request, &mut buf).unwrap();

    let envelope = pb::decode_sync_request(&buf[..n]).unwrap();
    assert!(matches!(
        envelope.body,
        Some(msg::SyncRequest_::Body::Trace(_))
    ));

    let decoded = pb::decode_trace_request(&buf[..n]).unwrap();
    assert_eq!(
        pb::uuid_from_field(&decoded.rep_uuid),
        Some(fixture_uuid(42))
    );

    assert_eq!(pb::decode_trace_request(&[]), Err(pb::Error::Empty));
}

#[test]
fn sync_message_carries_a_summary() {
    let mut summary = sample_summary();
    summary.rep_uuid = pb::uuid_to_field(&fixture_uuid(3));
    summary.computed_chunks = 2;
    let mut buf = [0u8; 256];
    let n = pb::encode_sync(msg::SyncMessage_::Body::Summary(summary.clone()), &mut buf).unwrap();
    let decoded = pb::decode_sync(&buf[..n]).unwrap();
    match decoded.body {
        Some(msg::SyncMessage_::Body::Summary(s)) => assert_eq!(s, summary),
        other => panic!("expected a summary, got {other:?}"),
    }
}

#[test]
fn chunk_container_capacity_matches_the_constant() {
    // Both trace kinds carry the same bound: `build.rs` configures it per message and the log's record buffer is sized against it.
    let computed: heapless::Vec<msg::TraceComputedPoint, TRACE_CHUNK_SAMPLES> =
        msg::TraceChunk::default().samples;
    let raw: heapless::Vec<msg::TraceRawPoint, TRACE_CHUNK_SAMPLES> =
        msg::RawChunk::default().samples;
    assert_eq!(computed.capacity(), TRACE_CHUNK_SAMPLES);
    assert_eq!(raw.capacity(), TRACE_CHUNK_SAMPLES);
}

/// The raw trace is a replay's input, so its sample has to come back bit for bit: no rescaling, no loss.
#[test]
fn raw_chunk_round_trips_every_sample() {
    use calypso_core::sampler::{Sample, GYRO_SCALE};
    let samples: Vec<Sample> = (0..TRACE_CHUNK_SAMPLES)
        .map(|i| Sample {
            t_ms: 500 + i as u32 * 10,
            // The extremes an i16 can hold, because a raw sample is the sensor's register value.
            accel: [i16::MIN + i as i16, i16::MAX - i as i16, 12],
            gyro: [i as i16, -(i as i16), GYRO_SCALE as i16],
        })
        .collect();
    let mut buf = [0u8; 2048];
    let n = pb::encode_raw_chunk(fixture_uuid(7), 1, 500, 4, &samples, &mut buf).unwrap();

    match pb::decode_record(&buf[..n]).unwrap() {
        Record::Raw(chunk) => {
            assert_eq!(pb::raw_uuid(&chunk), Some(fixture_uuid(7)));
            assert_eq!(chunk.index, 1);
            assert_eq!(chunk.total_chunks, 4);
            for (i, s) in chunk.samples.iter().enumerate() {
                let back = pb::raw_point_from_message(s);
                assert_eq!(back.t_ms, samples[i].t_ms - 500, "t_offset at {i}");
                assert_eq!(back.accel, samples[i].accel);
                assert_eq!(back.gyro, samples[i].gyro);
            }
            let _ = GYRO_SCALE;
        }
        other => panic!("expected a raw chunk, got {other:?}"),
    }
}
