use calypso_core::fixtures::{self, push_record, push_rep, read_dump, squat_dump, squat_set};
use calypso_core::pb::{self, msg, Record};
use calypso_core::rep::{compute_rep, TraceBuf, TracePoint};
use calypso_core::store::RECORD_BUF_BYTES;
use calypso_core::uuid::fixture_uuid;

#[test]
fn squat_set_is_detected_and_traced() {
    let (samples, spans) = squat_set(3, 1500, 800, 0.4);
    assert_eq!(spans.len(), 3);

    let reps = fixtures::detect_and_trace(&samples, 512);
    assert_eq!(reps.len(), 3, "the detector should find three reps");
    for (summary, trace) in &reps {
        assert!(!trace.is_empty());
        assert_eq!(summary.trace_len, trace.len() as u32);
        assert!(!summary.decimated);
        assert!(
            (summary.rom - 0.4).abs() < 0.15,
            "rom {} for a 0.4 m squat",
            summary.rom
        );
    }
}

#[test]
fn dump_round_trips_every_record() {
    let dump = squat_dump(2, 0.4);
    let (records, truncated) = read_dump(&dump);
    assert!(!truncated);

    let mut summaries = Vec::new();
    let mut chunks = Vec::new();
    for record in &records {
        match pb::decode_record(record).expect("record should decode") {
            Record::Summary(s) => summaries.push(s),
            Record::Trace(t) => chunks.push(t),
        }
    }
    assert_eq!(summaries.len(), 2);
    assert_eq!(chunks.len(), records.len() - 2);

    for (i, summary) in summaries.iter().enumerate() {
        assert_eq!(pb::summary_uuid(summary), Some(fixture_uuid(i as u32)));
        let mine: Vec<&msg::TraceChunk> = chunks
            .iter()
            .filter(|c| pb::chunk_uuid(c) == pb::summary_uuid(summary))
            .collect();
        assert_eq!(mine.len(), summary.chunk_count as usize);
        let points: usize = mine.iter().map(|c| c.samples.len()).sum();
        assert_eq!(points as u32, summary.trace_len);
        for (index, chunk) in mine.iter().enumerate() {
            assert_eq!(chunk.index, index as u32);
        }
    }
}

#[test]
fn truncation_is_reported_not_ignored() {
    let dump = squat_dump(1, 0.4);
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
        accel: [0, 0, -100],
        quat: [16_384, 0, 0, 0],
    }];
    let mut summary = msg::RepSummary {
        rep_uuid: pb::uuid_to_field(&fixture_uuid(1)),
        trace_len: 1,
        ..Default::default()
    };
    push_rep(&mut dump, &mut rec_buf, &mut summary, &trace).unwrap();

    let (records, truncated) = read_dump(&dump);
    assert!(!truncated);
    assert_eq!(records.len(), 2);
    match pb::decode_record(records[0]).unwrap() {
        Record::Summary(s) => {
            assert_eq!(s.trace_len, 1);
            assert_eq!(s.chunk_count, 1);
        }
        other => panic!("expected a summary, got {other:?}"),
    }
}

#[test]
fn decimation_keeps_the_last_point() {
    let (samples, _) = squat_set(1, 10_000, 1_000, 1.2);
    let lead_in = 300 / fixtures::SAMPLE_DT_MS as usize;
    let window = &samples[lead_in - 12..lead_in + 1_000 + 12];

    let mut storage = vec![TracePoint::ZERO; 512];
    let mut trace = TraceBuf::new(&mut storage);
    let summary = compute_rep(window, &mut trace);
    assert!(summary.decimated);
    assert_eq!(summary.trace_len, 512);
    assert_eq!(
        trace.get(trace.len() - 1).unwrap().t_ms,
        window.last().unwrap().t_ms,
        "the last trace point should be the last sample"
    );
    assert!(trace.get(1).unwrap().t_ms > trace.get(0).unwrap().t_ms + 10);
}

#[test]
fn framing_is_u32_little_endian() {
    let mut dump = Vec::new();
    push_record(&mut dump, b"abc");
    assert_eq!(&dump[..4], &[3, 0, 0, 0]);
    assert_eq!(&dump[4..], b"abc");
}
