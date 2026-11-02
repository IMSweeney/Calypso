//! The sync path: what the device says, and the rules it applies to what the app asks for.
//!
//! Messages go out through the codec the pipe carries them with (`gatt::pipe_sync`), so "this fits a transfer" is checked here by the same code the device runs rather than by a number repeated in the test.
//! The calibration status is the one characteristic that carries a value rather than a message, and its ceiling is checked here for the same reason.
//! How a message travels — chunks, acknowledgements, retries — belongs to `tests/pipe.rs`.

use calypso_core::gatt::pipe_sync;
use calypso_core::pb::{self, msg};
use calypso_core::pipe::MESSAGE_MAX;
use calypso_core::sync::{
    dev_mode, summary_message, trace_request, RepIndex, RepPlace, SummaryCursor, REP_INDEX_LEN,
};
use calypso_core::uuid::fixture_uuid;

/// A message as the pipe carries it: the bytes the app reassembles, and their length.
///
/// The `expect` is the assertion these tests exist for — the message has to encode into the pipe's reassembly buffer, which is the buffer both ends were generated with (`pipe_msg_max`).
fn wire(message: &msg::SyncMessage) -> (Vec<u8>, usize) {
    let mut buf = [0u8; MESSAGE_MAX];
    let n = pipe_sync::encode_response(message, &mut buf).expect("the message fits a transfer");
    (buf[..n].to_vec(), n)
}

/// The trace request inside a written `SyncRequest`, validated the way the device does: bytes in, as a client sends them.
fn trace_request_of(bytes: &[u8]) -> Option<msg::TraceRequest> {
    trace_request(&pipe_sync::decode_request(bytes)?)
}

#[test]
fn worst_case_chunk_fits_a_transfer() {
    use calypso_core::rep::TracePoint;

    let worst = TracePoint {
        t_ms: u32::MAX,
        pos: [f32::MAX, f32::MIN, -1.5e30],
        quat: [i16::MIN, i16::MAX, -16384, 16383],
    };
    let points = vec![worst; pb::TRACE_CHUNK_SAMPLES];
    let chunk = pb::computed_chunk_message(fixture_uuid(1), 0, 0, 1, &points);
    let (_, n) = wire(&msg::SyncMessage {
        body: Some(msg::SyncMessage_::Body::Computed(chunk)),
    });
    // 2459 bytes worst case: the generated table's assertion is what makes this a build failure rather than a transfer that stalls part-way, and this says the number is what the wire really produces.
    assert!(
        n < MESSAGE_MAX,
        "worst-case chunk is {n} bytes, and the pipe carries {MESSAGE_MAX}"
    );
}

#[test]
fn worst_case_summary_fits_a_transfer() {
    let mut summary = msg::RepSummary {
        rep_uuid: pb::uuid_to_field(&fixture_uuid(3)),
        t_start_ms: u32::MAX,
        t_end_ms: u32::MAX,
        duration_ms: u32::MAX,
        rom: f32::MAX,
        peak_velocity: f32::MIN,
        mean_velocity: f32::MAX,
        peak_accel: f32::MIN,
        roll_min_deg: f32::MIN,
        pitch_min_deg: f32::MIN,
        pitch_max_deg: f32::MAX,
        computed_len: u32::MAX,
        computed_chunks: u32::MAX,
        raw_len: u32::MAX,
        raw_chunks: u32::MAX,
        t_unix_ms: u64::MAX,
        ..Default::default()
    };
    // Both headers are submessages, so their presence and every field cost wire bytes: put a worst case in each.
    summary.set_state(msg::RepState {
        vx: f32::MAX,
        vy: f32::MAX,
        vz: f32::MAX,
        qw: f32::MAX,
        qx: f32::MAX,
        qy: f32::MAX,
        qz: f32::MAX,
        accel_bias_x: f32::MAX,
        accel_bias_y: f32::MAX,
        accel_bias_z: f32::MAX,
        gyro_bias_x: f32::MAX,
        gyro_bias_y: f32::MAX,
        gyro_bias_z: f32::MAX,
        settled: true,
    });
    summary.set_config(msg::RepConfig {
        accel_range_g: u32::MAX,
        gyro_range_dps: u32::MAX,
        accel_bandwidth_hz: u32::MAX,
        gyro_bandwidth_hz: u32::MAX,
        sample_rate_hz: u32::MAX,
        detector_version: u32::MAX,
    });
    let (_, n) = wire(&summary_message(summary));
    assert!(n <= MESSAGE_MAX, "worst-case summary is {n} bytes");
}

#[test]
fn a_summary_survives_the_envelope() {
    let summary = msg::RepSummary {
        rep_uuid: pb::uuid_to_field(&fixture_uuid(5)),
        computed_chunks: 2,
        ..Default::default()
    };
    let (buf, n) = wire(&summary_message(summary));
    match pb::decode_sync(&buf[..n]).unwrap().body {
        Some(msg::SyncMessage_::Body::Summary(s)) => {
            assert_eq!(pb::summary_uuid(&s), Some(fixture_uuid(5)))
        }
        other => panic!("expected a summary, got {other:?}"),
    }
}

#[test]
fn the_cursor_clamps_to_the_log_it_is_reading() {
    let mut cursor = SummaryCursor::new();
    assert_eq!(cursor.next(), 0);
    assert_eq!(cursor.remaining(4), 4);
    cursor.advance(2);
    assert_eq!(cursor.remaining(4), 2);

    // A device that wrapped has fewer records than the cursor had got to, and one that gained some does not move it forwards.
    cursor.clamp_to(1);
    assert_eq!(cursor.next(), 1);
    cursor.clamp_to(9);
    assert_eq!(cursor.next(), 1);
}

#[test]
fn trace_requests_are_validated() {
    let mut buf = [0u8; 64];
    let mut encode = |rep_uuid, all_chunks, chunk_index, kind| {
        let n = pb::encode_trace_request(
            &msg::TraceRequest {
                kind,
                rep_uuid: pb::uuid_to_field(&rep_uuid),
                all_chunks,
                chunk_index,
            },
            &mut buf,
        )
        .unwrap();
        buf[..n].to_vec()
    };

    let want = fixture_uuid(12);
    let ok = trace_request_of(&encode(want, true, 0, msg::TraceKind::Computed)).unwrap();
    assert_eq!(pb::uuid_from_field(&ok.rep_uuid), Some(want));
    assert!(ok.all_chunks);

    // Both kinds are served: which traces a rep has is its summary's business, and one it does not have reads back empty.
    let raw = trace_request_of(&encode(want, true, 0, msg::TraceKind::Raw)).unwrap();
    assert_eq!(raw.kind, msg::TraceKind::Raw);

    let one = trace_request_of(&encode(
        fixture_uuid(12),
        false,
        5,
        msg::TraceKind::Computed,
    ))
    .unwrap();
    assert_eq!(one.chunk_index, 5);

    // Its own buffer: the closure above holds `buf`.
    let mut scratch = [0u8; 64];
    let too_short: heapless::Vec<u8, 16> = heapless::Vec::from_slice(&[1, 2, 3]).unwrap();
    let n = pb::encode_trace_request(
        &msg::TraceRequest {
            rep_uuid: too_short,
            all_chunks: true,
            chunk_index: 0,
            kind: msg::TraceKind::Computed,
        },
        &mut scratch,
    )
    .unwrap();
    assert!(trace_request_of(&scratch[..n]).is_none());
    assert!(trace_request_of(&encode(
        fixture_uuid(1),
        false,
        9999,
        msg::TraceKind::Computed
    ))
    .is_none());
    assert!(trace_request_of(&[0xFF, 0xFF, 0xFF, 0xFF]).is_none());
}

/// The dev switch is the request the device acts on without answering, so what it decodes to is the whole of it.
#[test]
fn the_dev_switch_arrives_with_the_way_it_sets_it() {
    let mut buf = [0u8; 32];
    for enabled in [true, false] {
        let n = pb::encode_dev_mode(enabled, &mut buf).unwrap();
        let request = pipe_sync::decode_request(&buf[..n]).expect("the request decodes");
        assert_eq!(dev_mode(&request), Some(enabled));
        // And it is not a trace request: the device reads one or the other, never both.
        assert!(trace_request(&request).is_none());
    }
}

/// Where a rep's records are, so a trace request is a walk of them rather than of the log.
///
/// The index is a cache and is allowed to forget: the test that matters is that a forgotten rep is a *miss* (which
/// the device answers by walking the log) rather than a wrong answer, which would send the app another rep's trace.
#[test]
fn the_device_remembers_where_a_reps_records_are() {
    let mut index = RepIndex::new();
    assert_eq!(index.find(&fixture_uuid(1)), None, "nothing yet");

    index.record(fixture_uuid(1), 42, 5, 0);
    assert_eq!(
        index.find(&fixture_uuid(1)),
        Some(RepPlace {
            rep_uuid: fixture_uuid(1),
            first_chunk_id: 42,
            computed_chunks: 5,
            raw_chunks: 0,
        })
    );

    // A full index forgets the oldest place first: the asks a batch causes come in the order it was sent.
    for i in 0..REP_INDEX_LEN as u32 {
        index.record(fixture_uuid(100 + i), 1000 + i, 1, 0);
    }
    assert_eq!(index.find(&fixture_uuid(1)), None, "the oldest is gone");
    assert!(
        index.find(&fixture_uuid(101)).is_some(),
        "and the ones after it are not"
    );
    // Re-recording a rep that is in the index moves nothing: same uuid, same place.
    index.record(fixture_uuid(101), 1001, 1, 0);
    assert_eq!(index.find(&fixture_uuid(101)).unwrap().first_chunk_id, 1001);
}
