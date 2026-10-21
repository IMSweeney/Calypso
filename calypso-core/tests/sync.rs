//! The sync path: what the device encodes, and the rules it applies to what the
//! app sends.
//!
//! Messages go out through the value type the generated table stores
//! ([`SyncMessagesValue`]), so "this fits the characteristic" is checked here by
//! the same code the device runs rather than by a number repeated in the test.

use calypso_core::calibration::Calibration;
use calypso_core::gatt::SyncMessagesValue;
use calypso_core::pb::{self, msg, Record};
use calypso_core::rep::TracePoint;
use calypso_core::sync::{
    calibration_message, summary_message, trace_request, SummaryCursor, TraceStreamer,
    BLE_CHUNK_SAMPLES, BLE_NOTIFY_BYTES, SYNC_CHUNK_MAX,
};
use calypso_core::uuid::fixture_uuid;

fn point(i: usize) -> TracePoint {
    TracePoint {
        t_ms: 1000 + (i as u32) * 10,
        pos: [0.01 * i as f32, 0.2, -0.3],
        accel: [0, -981, 12],
        quat: [16384, 0, 1024, -2048],
    }
}

fn trace(n: usize) -> Vec<TracePoint> {
    (0..n).map(point).collect()
}

/// A message as the notify characteristic carries it: the bytes the app receives,
/// and their length.
///
/// The `expect` is the assertion these tests exist for: encoding through the
/// value type the table stores fails when the message does not fit the
/// characteristic the schema declares.
fn wire(message: &msg::SyncMessage) -> (Vec<u8>, usize) {
    let value =
        SyncMessagesValue::encode(message).expect("the message fits the notify characteristic");
    (value.as_bytes().to_vec(), value.as_bytes().len())
}

/// The trace request inside a written `SyncRequest`, validated the way the device
/// does: bytes in, as a client sends them.
fn trace_request_of(bytes: &[u8]) -> Option<msg::TraceRequest> {
    trace_request(&pb::decode_sync_request(bytes).ok()?)
}

#[test]
fn trace_streams_in_ble_sized_chunks() {
    let points = trace(10);
    let mut streamer = TraceStreamer::all(fixture_uuid(3), points[0].t_ms, &points);

    let mut seen = Vec::new();
    let mut indices = Vec::new();
    while let Some(message) = streamer.next_message() {
        let (buf, n) = wire(&message);
        assert!(
            n <= BLE_NOTIFY_BYTES,
            "chunk message {n} bytes, over the MTU"
        );
        let msg = pb::decode_sync(&buf[..n]).unwrap();
        match msg.body {
            Some(msg::SyncMessage_::Body::Trace(chunk)) => {
                assert_eq!(pb::chunk_uuid(&chunk), Some(fixture_uuid(3)));
                indices.push(chunk.index);
                for s in chunk.samples.iter() {
                    seen.push(pb::trace_point_from_message(s));
                }
            }
            other => panic!("expected a trace chunk, got {other:?}"),
        }
    }

    let expected_chunks = 10usize.div_ceil(BLE_CHUNK_SAMPLES) as u32;
    assert_eq!(TraceStreamer::chunk_count(10), expected_chunks);
    assert_eq!(indices, (0..expected_chunks).collect::<Vec<_>>());
    assert_eq!(seen.len(), points.len());
    for (got, want) in seen.iter().zip(points.iter()) {
        assert_eq!(got.t_ms, want.t_ms - points[0].t_ms);
        assert_eq!(got.accel, want.accel);
        assert_eq!(got.quat, want.quat);
        assert!((got.pos[0] - want.pos[0]).abs() < 1e-6);
    }
}

#[test]
fn every_streamed_chunk_carries_the_stream_total() {
    let points = trace(10);
    let mut streamer = TraceStreamer::all(fixture_uuid(3), points[0].t_ms, &points);
    let mut seen = 0;
    while let Some(message) = streamer.next_message() {
        let (buf, n) = wire(&message);
        match pb::decode_sync(&buf[..n]).unwrap().body {
            Some(msg::SyncMessage_::Body::Trace(chunk)) => {
                assert_eq!(chunk.total_chunks, 4, "10 points / 3 per chunk");
                seen += 1;
            }
            other => panic!("expected a trace chunk, got {other:?}"),
        }
    }
    assert_eq!(seen, 4);
}

#[test]
fn stored_records_leave_the_stream_total_unset() {
    let points = trace(4);
    let mut rec_buf = [0u8; 512];
    let n = pb::encode_trace_chunk(fixture_uuid(1), 0, points[0].t_ms, 0, &points, &mut rec_buf)
        .unwrap();
    match pb::decode_record(&rec_buf[..n]).unwrap() {
        Record::Trace(chunk) => assert_eq!(chunk.total_chunks, 0),
        other => panic!("expected a trace chunk, got {other:?}"),
    }
}

#[test]
fn chunk_count_matches_the_stream() {
    for n in [0usize, 1, 4, 5, 8, 512] {
        let points = trace(n);
        let expected = n.div_ceil(BLE_CHUNK_SAMPLES) as u32;
        assert_eq!(TraceStreamer::chunk_count(n), expected, "n = {n}");

        let mut streamer = TraceStreamer::all(fixture_uuid(1), 1000, &points);
        let mut count = 0;
        while streamer.next_message().is_some() {
            count += 1;
        }
        assert_eq!(count, expected as usize, "n = {n}");
    }
}

#[test]
fn single_chunk_request_sends_exactly_one() {
    let points = trace(20);
    let mut streamer = TraceStreamer::one(fixture_uuid(7), points[0].t_ms, &points, 3);

    let message = streamer.next_message().unwrap();
    let (buf, n) = wire(&message);
    match pb::decode_sync(&buf[..n]).unwrap().body {
        Some(msg::SyncMessage_::Body::Trace(chunk)) => {
            assert_eq!(chunk.index, 3);
            assert_eq!(pb::chunk_uuid(&chunk), Some(fixture_uuid(7)));
            assert_eq!(chunk.samples.len(), BLE_CHUNK_SAMPLES);
        }
        other => panic!("expected a trace chunk, got {other:?}"),
    }
    assert!(streamer.next_message().is_none());
}

#[test]
fn out_of_range_single_chunk_yields_nothing() {
    let points = trace(8);
    let mut streamer = TraceStreamer::one(fixture_uuid(7), 1000, &points, 9);
    assert!(streamer.next_message().is_none());
}

#[test]
fn worst_case_chunk_fits_the_notification_budget() {
    let worst = TracePoint {
        t_ms: u32::MAX,
        pos: [f32::MAX, f32::MIN, -1.5e30],
        accel: [i16::MIN, i16::MAX, -32767],
        quat: [i16::MIN, i16::MAX, -16384, 16383],
    };
    let points = vec![worst; BLE_CHUNK_SAMPLES];
    let mut streamer = TraceStreamer::all(fixture_uuid(1), 0, &points);
    let message = streamer.next_message().unwrap();
    let (_, n) = wire(&message);
    assert!(
        n <= SYNC_CHUNK_MAX,
        "worst-case chunk is {n} bytes, over the {SYNC_CHUNK_MAX}-byte characteristic"
    );
    assert!(
        n <= BLE_NOTIFY_BYTES,
        "worst-case chunk is {n} bytes, over the {BLE_NOTIFY_BYTES}-byte budget"
    );
}

#[test]
fn worst_case_summary_fits_the_summary_characteristic() {
    let summary = msg::RepSummary {
        rep_uuid: pb::uuid_to_field(&fixture_uuid(3)),
        t_start_ms: u32::MAX,
        t_end_ms: u32::MAX,
        duration_ms: u32::MAX,
        rom: f32::MAX,
        peak_velocity: f32::MIN,
        mean_velocity: f32::MAX,
        peak_accel: f32::MIN,
        roll_min_deg: f32::MIN,
        roll_max_deg: f32::MAX,
        pitch_min_deg: f32::MIN,
        pitch_max_deg: f32::MAX,
        trace_len: u32::MAX,
        chunk_count: u32::MAX,
        decimated: true,
    };
    let (_, n) = wire(&summary_message(summary));
    assert!(
        n <= SYNC_CHUNK_MAX,
        "worst-case summary is {n} bytes, over the {SYNC_CHUNK_MAX}-byte characteristic"
    );

    let (_, n) = wire(&calibration_message(&Calibration::new(3, 3, 3, 0)));
    assert!(
        n <= SYNC_CHUNK_MAX,
        "worst-case calibration status is {n} bytes, over the {SYNC_CHUNK_MAX}-byte characteristic"
    );
}

#[test]
fn summary_messages_and_cursor() {
    let summary = msg::RepSummary {
        rep_uuid: pb::uuid_to_field(&fixture_uuid(5)),
        chunk_count: 2,
        ..Default::default()
    };
    let (buf, n) = wire(&summary_message(summary));
    match pb::decode_sync(&buf[..n]).unwrap().body {
        Some(msg::SyncMessage_::Body::Summary(s)) => {
            assert_eq!(pb::summary_uuid(&s), Some(fixture_uuid(5)))
        }
        other => panic!("expected a summary, got {other:?}"),
    }

    let mut cursor = SummaryCursor::new();
    assert_eq!(cursor.next(), 0);
    assert_eq!(cursor.remaining(4), 4);
    cursor.advance(2);
    assert_eq!(cursor.remaining(4), 2);
    cursor.clamp_to(1);
    assert_eq!(cursor.next(), 1);
    cursor.clamp_to(9);
    assert_eq!(cursor.next(), 1);
}

#[test]
fn calibration_status_carries_every_sensor() {
    let calibration = Calibration::new(3, 2, 1, 0);
    let (buf, n) = wire(&calibration_message(&calibration));
    assert!(n <= BLE_NOTIFY_BYTES);

    match pb::decode_sync(&buf[..n]).unwrap().body {
        Some(msg::SyncMessage_::Body::Calibration(c)) => {
            assert_eq!((c.sys, c.accel, c.gyro, c.mag), (3, 2, 1, 0));
            assert!(!c.ready, "gyro and accel are short of 3");
        }
        other => panic!("expected a calibration status, got {other:?}"),
    }

    let (buf, n) = wire(&calibration_message(&Calibration::new(3, 3, 3, 0)));
    match pb::decode_sync(&buf[..n]).unwrap().body {
        Some(msg::SyncMessage_::Body::Calibration(c)) => {
            assert!(
                c.ready,
                "IMU mode: the magnetometer is not part of readiness"
            );
            assert_eq!(c.mag, 0);
        }
        other => panic!("expected a calibration status, got {other:?}"),
    }

    let (buf, n) = wire(&calibration_message(&Calibration::new(0, 3, 3, 0)));
    match pb::decode_sync(&buf[..n]).unwrap().body {
        Some(msg::SyncMessage_::Body::Calibration(c)) => {
            assert!(c.ready, "accel and gyro are what readiness is");
            assert_eq!(c.sys, 0, "reported, and zero");
        }
        other => panic!("expected a calibration status, got {other:?}"),
    }
}

#[test]
fn trace_requests_are_validated() {
    let mut buf = [0u8; 32];
    let mut encode = |rep_uuid, all_chunks, chunk_index| {
        let n = pb::encode_trace_request(
            &msg::TraceRequest {
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
    let ok = trace_request_of(&encode(want, true, 0)).unwrap();
    assert_eq!(pb::uuid_from_field(&ok.rep_uuid), Some(want));
    assert!(ok.all_chunks);

    let one = trace_request_of(&encode(fixture_uuid(12), false, 5)).unwrap();
    assert_eq!(one.chunk_index, 5);

    // Its own buffer: the closure above holds `buf`.
    let mut scratch = [0u8; 32];
    let too_short: heapless::Vec<u8, 16> = heapless::Vec::from_slice(&[1, 2, 3]).unwrap();
    let n = pb::encode_trace_request(
        &msg::TraceRequest {
            rep_uuid: too_short,
            all_chunks: true,
            chunk_index: 0,
        },
        &mut scratch,
    )
    .unwrap();
    assert!(trace_request_of(&scratch[..n]).is_none());
    assert!(trace_request_of(&encode(fixture_uuid(1), false, 9999)).is_none());
    assert!(trace_request_of(&[0xFF, 0xFF, 0xFF, 0xFF]).is_none());
}
