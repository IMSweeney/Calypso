use calypso_core::pb::{self, msg, Record, TRACE_CHUNK_SAMPLES};
use calypso_core::rep::TracePoint;
use calypso_core::uuid::fixture_uuid;

fn sample_summary() -> msg::RepSummary {
    msg::RepSummary {
        t_start_ms: 12_345,
        t_end_ms: 13_800,
        duration_ms: 1455,
        rom: 0.412,
        peak_velocity: -0.71,
        mean_velocity: 0.19,
        peak_accel: 2.61,
        roll_min_deg: -1.25,
        roll_max_deg: 2.4,
        pitch_min_deg: -0.8,
        pitch_max_deg: 3.1,
        trace_len: 145,
        decimated: false,
        ..Default::default()
    }
}

fn sample_point(i: usize, t0: u32) -> TracePoint {
    TracePoint {
        t_ms: t0 + (i as u32) * 10,
        pos: [i as f32 * 0.001, -0.2, 0.35],
        accel: [12, -981, 0],
        quat: [16384, 0, -2048, 2048],
    }
}

#[test]
fn summary_round_trips() {
    let mut summary = sample_summary();
    summary.rep_uuid = pb::uuid_to_field(&fixture_uuid(7));
    summary.chunk_count = 4;
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
            assert_eq!(s.trace_len, summary.trace_len);
            assert_eq!(s.chunk_count, 4);
            assert!(!s.decimated);
        }
        other => panic!("expected a summary, got {other:?}"),
    }
}

#[test]
fn trace_chunk_round_trips() {
    let points: Vec<TracePoint> = (0..TRACE_CHUNK_SAMPLES)
        .map(|i| sample_point(i, 500))
        .collect();
    let mut buf = [0u8; 2048];
    let n = pb::encode_trace_chunk(fixture_uuid(7), 2, 500, 9, &points, &mut buf).unwrap();

    match pb::decode_record(&buf[..n]).unwrap() {
        Record::Trace(chunk) => {
            assert_eq!(pb::chunk_uuid(&chunk), Some(fixture_uuid(7)));
            assert_eq!(chunk.index, 2);
            assert_eq!(chunk.first_t_offset_ms, 0);
            assert_eq!(chunk.samples.len(), TRACE_CHUNK_SAMPLES);
            assert_eq!(chunk.total_chunks, 9, "the stream total survives the wire");
            for (i, s) in chunk.samples.iter().enumerate() {
                let back = pb::trace_point_from_message(s);
                let want = &points[i];
                assert_eq!(back.t_ms, want.t_ms - 500, "t_offset at {i}");
                assert_eq!(back.accel, want.accel);
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
        accel: [i16::MIN, i16::MAX, -32767],
        quat: [i16::MIN, i16::MAX, -16384, 16383],
    };
    let points = vec![worst; TRACE_CHUNK_SAMPLES];
    let mut buf = [0u8; 4096];
    let n = pb::encode_trace_chunk(fixture_uuid(1), 0, 0, 0, &points, &mut buf).unwrap();
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
        pb::encode_trace_chunk(fixture_uuid(1), 0, 0, 0, &points, &mut tiny),
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
        rep_uuid: pb::uuid_to_field(&fixture_uuid(42)),
        all_chunks: true,
        chunk_index: 0,
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
    assert!(decoded.all_chunks);

    assert_eq!(pb::decode_trace_request(&[]), Err(pb::Error::Empty));
}

#[test]
fn sync_message_carries_a_summary() {
    let mut summary = sample_summary();
    summary.rep_uuid = pb::uuid_to_field(&fixture_uuid(3));
    summary.chunk_count = 2;
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
    let chunk = msg::TraceChunk::default();
    let samples: heapless::Vec<msg::TraceSample, TRACE_CHUNK_SAMPLES> = chunk.samples;
    assert_eq!(samples.capacity(), TRACE_CHUNK_SAMPLES);
}
