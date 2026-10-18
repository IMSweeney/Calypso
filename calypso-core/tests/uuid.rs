//! Rep identity: v4 UUIDs, minted where a rep is created.

use calypso_core::pb;
use calypso_core::uuid::{self, RepUuid, UUID_LEN};

/// A predictable source of randomness: a counter run through a mixing step.
fn counter_rand() -> impl FnMut() -> u32 {
    let mut n = 0u32;
    move || {
        n += 1;
        n.wrapping_mul(0x9e37_79b9)
    }
}

#[test]
fn a_minted_uuid_is_a_v4_uuid() {
    let uuid = uuid::new_uuid(&mut counter_rand());
    assert_eq!(uuid[6] >> 4, 4, "version nibble");
    assert_eq!(uuid[8] >> 6, 0b10, "variant bits");
}

#[test]
fn minting_consumes_the_randomness_it_is_given() {
    // Same seed, same id — so a test can reproduce one.
    let mut source = counter_rand();
    let first = uuid::new_uuid(&mut source);
    assert_eq!(first, uuid::new_uuid(&mut counter_rand()));

    // ...and the same source advances, so consecutive reps differ.
    let second = uuid::new_uuid(&mut source);
    assert_ne!(first, second, "the source advances between reps");

    // Sixteen bytes means four words: a source that never changes must not
    // collapse to a constant id, but it should still fill all four.
    let constant = uuid::new_uuid(&mut || 0);
    assert_eq!(&constant[..4], &[0, 0, 0, 0]);
    assert_ne!(constant, uuid::new_uuid(&mut || 0xdead_beef));
}

#[test]
fn fixture_uuids_are_stable_and_distinct() {
    // Fixtures are regenerated and compared, so these must not drift; and two
    // reps inside one fixture must not collide.
    assert_eq!(uuid::fixture_uuid(0), uuid::fixture_uuid(0));
    assert_eq!(uuid::fixture_uuid(0)[6] >> 4, 4, "still a v4 uuid");

    let mut seen = std::collections::HashSet::new();
    for index in 0..1000 {
        assert!(seen.insert(uuid::fixture_uuid(index)), "collision at {index}");
    }
}

#[test]
fn short_form_is_eight_hex_characters() {
    let uuid: RepUuid = [
        0x3f, 0x2a, 0x9c, 0x81, 0, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa,
        0xbb,
    ];
    assert_eq!(&uuid::short(&uuid), b"3f2a9c81");

    // Zero-padded, so a column of ids lines up.
    let leading_zero: RepUuid = [0, 0, 0x0a, 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    assert_eq!(&uuid::short(&leading_zero), b"00000aff");
}

#[test]
fn the_wire_form_is_sixteen_bytes_and_rejects_anything_else() {
    let uuid = uuid::fixture_uuid(7);
    let field = pb::uuid_to_field(&uuid);
    assert_eq!(field.len(), UUID_LEN);
    assert_eq!(pb::uuid_from_field(&field), Some(uuid));

    // A truncated, empty or oversized field is not an identity: this is what
    // keeps a half-written record from being mistaken for a rep.
    assert_eq!(pb::uuid_from_field(&field[..15]), None);
    assert_eq!(pb::uuid_from_field(&[]), None);
    assert_eq!(pb::uuid_from_field(&[1u8; 17]), None);
}

#[test]
fn a_uuid_survives_both_record_kinds() {
    // The identity has to come back out of a summary and out of a chunk: the
    // client keys its archive on it, and the device matches chunks by it.
    let uuid = uuid::fixture_uuid(3);
    let summary = pb::summary_message(uuid, &Default::default(), 0);
    assert_eq!(pb::summary_uuid(&summary), Some(uuid));

    let points = [calypso_core::rep::TracePoint::ZERO];
    let chunk = pb::chunk_message(uuid, 0, 0, 1, &points);
    assert_eq!(pb::chunk_uuid(&chunk), Some(uuid));

    let mut buf = [0u8; 4096];
    let n = pb::encode_trace_chunk(uuid, 0, 0, 0, &points, &mut buf).unwrap();
    match pb::decode_record(&buf[..n]).unwrap() {
        pb::Record::Trace(chunk) => assert_eq!(pb::chunk_uuid(&chunk), Some(uuid)),
        other => panic!("expected a trace chunk, got {other:?}"),
    }
}
