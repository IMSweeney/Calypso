//! Host tests for the BLE sync streaming logic: BLE-sized trace chunks, the
//! summary cursor, and request decoding. No radio involved.

use calypso_core::pb::{self, msg, Record};
use calypso_core::uuid::fixture_uuid;
use calypso_core::rep::TracePoint;
use calypso_core::sync::{
    decode_trace_request, encode_summary_message, SummaryCursor, TraceStreamer, BLE_CHUNK_SAMPLES,
    BLE_NOTIFY_BYTES,
};

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

#[test]
fn trace_streams_in_ble_sized_chunks() {
    let points = trace(10);
    let mut streamer = TraceStreamer::all(fixture_uuid(3), points[0].t_ms, &points);
    let mut buf = [0u8; 512];

    let mut seen = Vec::new();
    let mut indices = Vec::new();
    while let Some(n) = streamer.next_message(&mut buf) {
        assert!(n <= BLE_NOTIFY_BYTES, "chunk message {n} bytes, over the MTU");
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

    // Derived from the constant, not hardcoded: this assertion used to spell out
    // four-sample arithmetic and broke the moment the chunk size changed.
    let expected_chunks = 10usize.div_ceil(BLE_CHUNK_SAMPLES) as u32;
    assert_eq!(TraceStreamer::chunk_count(10), expected_chunks);
    assert_eq!(indices, (0..expected_chunks).collect::<Vec<_>>());
    // Reassembled in index order, the trace is identical (timestamps are
    // relative to the rep's first point).
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
    // What the client uses to know it has the whole trace: it cannot derive the
    // count from the summary, because the device re-chunks to fit the MTU
    // (4 points here, 48 in the log).
    let points = trace(10);
    let mut streamer = TraceStreamer::all(fixture_uuid(3), points[0].t_ms, &points);
    let mut buf = [0u8; 512];
    let mut seen = 0;
    while let Some(n) = streamer.next_message(&mut buf) {
        let message = pb::decode_sync(&buf[..n]).unwrap();
        match message.body {
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
    // A record written to the log must not claim to be part of a stream: there,
    // the summary's chunk_count is what describes the trace.
    let points = trace(4);
    let mut rec_buf = [0u8; 512];
    let n = pb::encode_trace_chunk(fixture_uuid(1), 0, points[0].t_ms, 0, &points, &mut rec_buf).unwrap();
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
        let mut buf = [0u8; 512];
        let mut count = 0;
        while streamer.next_message(&mut buf).is_some() {
            count += 1;
        }
        assert_eq!(count, expected as usize, "n = {n}");
    }
}

#[test]
fn single_chunk_request_sends_exactly_one() {
    let points = trace(20); // 5 chunks
    let mut streamer = TraceStreamer::one(fixture_uuid(7), points[0].t_ms, &points, 3);
    let mut buf = [0u8; 512];

    let n = streamer.next_message(&mut buf).unwrap();
    let msg = pb::decode_sync(&buf[..n]).unwrap();
    match msg.body {
        Some(msg::SyncMessage_::Body::Trace(chunk)) => {
            assert_eq!(chunk.index, 3);
            assert_eq!(pb::chunk_uuid(&chunk), Some(fixture_uuid(7)));
            assert_eq!(chunk.samples.len(), BLE_CHUNK_SAMPLES);
        }
        other => panic!("expected a trace chunk, got {other:?}"),
    }
    // ...and then the stream is finished.
    assert!(streamer.next_message(&mut buf).is_none());
}

#[test]
fn out_of_range_single_chunk_yields_nothing() {
    let points = trace(8); // 2 chunks (0, 1)
    let mut streamer = TraceStreamer::one(fixture_uuid(7), 1000, &points, 9);
    let mut buf = [0u8; 512];
    assert!(streamer.next_message(&mut buf).is_none());
}

#[test]
fn worst_case_chunk_fits_the_notification_budget() {
    // Extreme values, so every field takes its maximum encoding.
    let worst = TracePoint {
        t_ms: u32::MAX,
        pos: [f32::MAX, f32::MIN, -1.5e30],
        accel: [i16::MIN, i16::MAX, -32767],
        quat: [i16::MIN, i16::MAX, -16384, 16383],
    };
    let points = vec![worst; BLE_CHUNK_SAMPLES];
    let mut streamer = TraceStreamer::all(fixture_uuid(1), 0, &points);
    let mut buf = [0u8; 512];
    let n = streamer.next_message(&mut buf).unwrap();
    assert!(
        n <= BLE_NOTIFY_BYTES,
        "worst-case chunk is {n} bytes, over the {BLE_NOTIFY_BYTES}-byte budget"
    );
}

#[test]
fn summary_messages_and_cursor() {
    let summary = pb::summary_message(fixture_uuid(5), &Default::default(), 2);
    let mut buf = [0u8; 128];
    let n = encode_summary_message(summary, &mut buf).unwrap();
    let msg = pb::decode_sync(&buf[..n]).unwrap();
    match msg.body {
        Some(msg::SyncMessage_::Body::Summary(s)) => {
            assert_eq!(pb::summary_uuid(&s), Some(fixture_uuid(5)))
        }
        other => panic!("expected a summary, got {other:?}"),
    }

    // The cursor tracks what the subscriber has already been sent.
    let mut cursor = SummaryCursor::new();
    assert_eq!(cursor.next(), 0);
    assert_eq!(cursor.remaining(4), 4);
    cursor.advance(2);
    assert_eq!(cursor.remaining(4), 2);
    // Re-subscribing after reps fell off the ring rewinds to what exists.
    cursor.clamp_to(1);
    assert_eq!(cursor.next(), 1);
    cursor.clamp_to(9);
    assert_eq!(cursor.next(), 1);
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
    let ok = decode_trace_request(&encode(want, true, 0)).unwrap();
    assert_eq!(pb::uuid_from_field(&ok.rep_uuid), Some(want));
    assert!(ok.all_chunks);

    let one = decode_trace_request(&encode(fixture_uuid(12), false, 5)).unwrap();
    assert_eq!(one.chunk_index, 5);

    // A short uuid is not an identity, and an absurd chunk index is rejected.
    // (Its own buffer: the closure above holds `buf`.)
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
    assert!(decode_trace_request(&scratch[..n]).is_none());
    assert!(decode_trace_request(&encode(fixture_uuid(1), false, 9999)).is_none());
    // Garbage bytes don't panic.
    assert!(decode_trace_request(&[0xFF, 0xFF, 0xFF, 0xFF]).is_none());
}
