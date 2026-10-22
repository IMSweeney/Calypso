//! The sync path: what the device says, and the rules it applies to what the app
//! asks for.
//!
//! Messages go out through the codec the pipe carries them with
//! (`gatt::pipe_sync`), so "this fits a transfer" is checked here by the same code
//! the device runs rather than by a number repeated in the test. How a message
//! travels — chunks, acknowledgements, retries — belongs to `tests/pipe.rs`.

use calypso_core::gatt::pipe_sync;
use calypso_core::pb::{self, msg};
use calypso_core::pipe::MESSAGE_MAX;
use calypso_core::sync::{summary_message, trace_request, SummaryCursor};
use calypso_core::uuid::fixture_uuid;

/// A message as the pipe carries it: the bytes the app reassembles, and their
/// length.
///
/// The `expect` is the assertion these tests exist for — the message has to
/// encode into the pipe's reassembly buffer, which is the buffer both ends were
/// generated with (`pipe_msg_max`).
fn wire(message: &msg::SyncMessage) -> (Vec<u8>, usize) {
    let mut buf = [0u8; MESSAGE_MAX];
    let n = pipe_sync::encode_response(message, &mut buf).expect("the message fits a transfer");
    (buf[..n].to_vec(), n)
}

/// The trace request inside a written `SyncRequest`, validated the way the device
/// does: bytes in, as a client sends them.
fn trace_request_of(bytes: &[u8]) -> Option<msg::TraceRequest> {
    trace_request(&pipe_sync::decode_request(bytes)?)
}

#[test]
fn worst_case_chunk_fits_a_transfer() {
    use calypso_core::rep::TracePoint;

    let worst = TracePoint {
        t_ms: u32::MAX,
        pos: [f32::MAX, f32::MIN, -1.5e30],
        accel: [i16::MIN, i16::MAX, -32767],
        quat: [i16::MIN, i16::MAX, -16384, 16383],
    };
    let points = vec![worst; pb::TRACE_CHUNK_SAMPLES];
    let chunk = pb::chunk_message(fixture_uuid(1), 0, 0, 1, &points);
    let (_, n) = wire(&msg::SyncMessage {
        body: Some(msg::SyncMessage_::Body::Trace(chunk)),
    });
    // 2459 bytes worst case: the generated table's assertion is what makes this a
    // build failure rather than a transfer that stalls part-way, and this says the
    // number is what the wire really produces.
    assert!(
        n < MESSAGE_MAX,
        "worst-case chunk is {n} bytes, and the pipe carries {MESSAGE_MAX}"
    );
    assert!(
        n > 193,
        "which is the whole point: it is more than one notification held"
    );
}

#[test]
fn worst_case_summary_fits_a_transfer() {
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
    assert!(n <= MESSAGE_MAX, "worst-case summary is {n} bytes");
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
fn trace_requests_are_validated() {
    let mut buf = [0u8; 64];
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
    let mut scratch = [0u8; 64];
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
