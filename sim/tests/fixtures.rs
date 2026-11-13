use calypso_core::motion::trace::TracePoint;
use calypso_core::pb::{self, msg, Record};
use calypso_core::store::RECORD_BUF_BYTES;
use calypso_core::uuid::fixture_uuid;
use calypso_sim::fixtures::{self, push_record, push_rep, read_dump, set_dump};
use calypso_sim::motion::{Rep, Set};

/// The set the squat fixtures are made of: 0.4 m of travel per rep, 1.5 s each, 0.8 s of rest between.
fn squat_set(reps: u32) -> Set {
    Set {
        rep: Rep::cosine(0.4, 1500),
        reps,
        inter_rep_rest_ms: 800,
        unrack: None,
        rerack: None,
    }
}

#[test]
fn a_squat_set_is_detected_and_traced() {
    let reps = fixtures::detect_and_trace(&squat_set(3).samples(), 512);
    assert_eq!(reps.len(), 3, "the detector should find three reps");
    for (summary, trace) in &reps {
        assert!(!trace.is_empty());
        assert_eq!(summary.computed_len, trace.len() as u32);
        assert_eq!(
            summary.computed_len as usize,
            trace.len(),
            "one chunk per whatever the budget holds"
        );
        assert!(
            (summary.rom - 0.4).abs() < 0.15,
            "rom {} for a 0.4 m squat",
            summary.rom
        );
    }
}

#[test]
fn dump_round_trips_every_record() {
    let dump = set_dump(&squat_set(2), 512);
    let (records, truncated) = read_dump(&dump);
    assert!(!truncated);

    let mut summaries = Vec::new();
    let mut chunks = Vec::new();
    let mut clocks = Vec::new();
    for record in &records {
        match pb::decode_record(record).expect("record should decode") {
            Record::Summary(s) => summaries.push(s),
            Record::Computed(t) => chunks.push(t),
            Record::Raw(_) => {}
            // A live chunk is the phone's record type: a device dump that holds one is not the dump this test wrote.
            Record::Imu(_) => panic!("a device dump cannot hold a live IMU chunk"),
            Record::Clock(c) => clocks.push(c),
        }
    }
    assert_eq!(
        clocks.len(),
        1,
        "a dump opens with the boot's clock, so its reps have a time"
    );
    assert_eq!(
        clocks[0].epoch_s,
        fixtures::FIXTURE_EPOCH_S,
        "the epoch survives the framing"
    );
    assert_eq!(summaries.len(), 2);
    assert_eq!(chunks.len(), records.len() - 3);

    for (i, summary) in summaries.iter().enumerate() {
        assert_eq!(pb::summary_uuid(summary), Some(fixture_uuid(i as u32)));
        let mine: Vec<&msg::TraceChunk> = chunks
            .iter()
            .filter(|c| pb::computed_uuid(c) == pb::summary_uuid(summary))
            .collect();
        assert_eq!(mine.len(), summary.computed_chunks as usize);
        let points: usize = mine.iter().map(|c| c.samples.len()).sum();
        assert_eq!(points as u32, summary.computed_len);
        for (index, chunk) in mine.iter().enumerate() {
            assert_eq!(chunk.index, index as u32);
        }
    }
}

#[test]
fn truncation_is_reported_not_ignored() {
    let dump = set_dump(&squat_set(1), 512);
    let (full_records, truncated) = read_dump(&dump);
    assert!(!truncated);
    assert!(full_records.len() > 1);

    let cut = &dump[..dump.len() - 40];
    let (records, truncated) = read_dump(cut);
    assert!(truncated);
    assert_eq!(records.len(), full_records.len() - 1);
}

#[test]
fn empty_and_single_point_dumps_are_valid() {
    let (records, truncated) = read_dump(&[]);
    assert!(records.is_empty());
    assert!(!truncated);

    let mut dump = Vec::new();
    let mut rec_buf = [0u8; RECORD_BUF_BYTES];
    let trace = vec![TracePoint {
        t_ms: 10,
        pos: [0.0, 0.0, -0.01],
        quat: [16_384, 0, 0, 0],
    }];
    let mut summary = msg::RepSummary {
        rep_uuid: pb::uuid_to_field(&fixture_uuid(1)),
        computed_len: 1,
        ..Default::default()
    };
    push_rep(&mut dump, &mut rec_buf, &mut summary, &trace, None).unwrap();

    let (records, truncated) = read_dump(&dump);
    assert!(!truncated);
    assert_eq!(records.len(), 2);
    match pb::decode_record(records[0]).unwrap() {
        Record::Summary(s) => {
            assert_eq!(s.computed_len, 1);
            assert_eq!(s.computed_chunks, 1);
        }
        other => panic!("expected a summary, got {other:?}"),
    }
}

#[test]
fn framing_is_u32_little_endian() {
    let mut dump = Vec::new();
    push_record(&mut dump, b"abc");
    assert_eq!(&dump[..4], &[3, 0, 0, 0]);
    assert_eq!(&dump[4..], b"abc");
}
