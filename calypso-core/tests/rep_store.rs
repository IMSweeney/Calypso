mod common;

use calypso_core::flashlog::FlashLog;
use calypso_core::motion::rep::{TraceBuf, TracePoint};
use calypso_core::pb::msg;
use calypso_core::sampler::{Sample, QUAT_SCALE};
use calypso_core::store::{self, RECORD_BUF_BYTES};
use calypso_core::uuid::{fixture_uuid, RepUuid};
use calypso_sim::fixtures::detect_and_trace;
use calypso_sim::motion::{Rep, Set};
use common::{erases, mount, reads, reset_erases, reset_reads, BigFlash, MockFlash, WideFlash};

/// A set of 0.4 m squat reps with rest between them, which is what the store tests replay.
fn squat_set(reps: u32) -> Set {
    Set {
        rep: Rep::cosine(0.4, 1500),
        reps,
        inter_rep_rest_ms: 800,
        unrack: None,
        rerack: None,
    }
}

fn pipeline(samples: &[Sample]) -> (Vec<Vec<TracePoint>>, Vec<msg::RepSummary>) {
    let reps = detect_and_trace(samples, 512);
    (
        reps.iter().map(|(_, trace)| trace.clone()).collect(),
        reps.iter().map(|(summary, _)| summary.clone()).collect(),
    )
}

fn append_set(
    log: &mut FlashLog<'_, WideFlash>,
    buf: &mut [u8],
    summary: &msg::RepSummary,
    traces: &[Vec<TracePoint>],
    set: u32,
) {
    for (i, trace) in traces.iter().enumerate() {
        let mut summary = summary.clone();
        summary.rep_uuid = calypso_core::pb::uuid_to_field(&fixture_uuid(set * 10 + i as u32));
        store::append_rep(log, buf, &mut summary, trace, None).unwrap();
    }
}

/// A set costs the sampler nothing but the write when the ring is erased ahead of it, and this is that said as the flash sees it: the same three reps, into the same wrapped ring, with and without a reserve.
#[test]
fn a_set_appended_into_a_reserve_erases_nothing() {
    let samples = squat_set(3).samples();
    let (traces, summaries) = pipeline(&samples);
    let mut flash = WideFlash::new();
    let mut buf = [0u8; RECORD_BUF_BYTES];

    // Round the ring, so that the sector it drops next holds a rep and a set landing in it has to erase. How many sets that takes is the record sizes' business, so the loop asks the ring rather than guessing: one sector ahead that reads erased means the ring has not been round yet.
    let mut log = mount(&mut flash);
    let mut set = 0;
    while log.prepared_ahead(1).unwrap() > 0 {
        append_set(&mut log, &mut buf, &summaries[0], &traces, set);
        set += 1;
        assert!(set < 500, "the ring never came round");
    }

    reset_erases();
    append_set(&mut log, &mut buf, &summaries[0], &traces, 100);
    assert!(
        erases() >= 1,
        "a wrapped ring should have to erase to make room"
    );

    for _ in 0..16 {
        log.prepare_one(16).unwrap();
    }
    reset_erases();
    append_set(&mut log, &mut buf, &summaries[0], &traces, 200);
    assert_eq!(erases(), 0, "a set appended into the reserve erased");
}

#[test]
fn rep_survives_the_round_trip_through_flash() {
    let samples = squat_set(3).samples();
    let (traces, summaries) = pipeline(&samples);
    assert_eq!(traces.len(), 3);
    assert_eq!(summaries.len(), 3);

    let mut flash = BigFlash::new();
    let mut buf = [0u8; RECORD_BUF_BYTES];
    let stored: Vec<(msg::RepSummary, Vec<TracePoint>)> = summaries
        .iter()
        .zip(traces.iter())
        .enumerate()
        .map(|(i, (sum, trace))| {
            let mut sum = sum.clone();
            sum.rep_uuid = calypso_core::pb::uuid_to_field(&fixture_uuid(i as u32));
            (sum, trace.clone())
        })
        .collect();

    {
        let mut log = mount(&mut flash);
        for (sum, trace) in &stored {
            let written =
                store::append_rep(&mut log, &mut buf, &mut sum.clone(), trace, None).unwrap();
            let expected_chunks = 1 + calypso_core::pb::chunk_count_for(trace.len());
            assert_eq!(written, expected_chunks);
        }
    }

    let mut log = mount(&mut flash);
    let mut rec = calypso_core::pb::msg::RepRecord::default();
    let mut summaries_back = Vec::new();
    store::for_each_summary(&mut log, &mut buf, &mut rec, |s| {
        summaries_back.push(s.clone())
    })
    .unwrap();
    assert_eq!(
        summaries_back.len(),
        3,
        "all three summaries should be in the log"
    );
    for (i, s) in summaries_back.iter().enumerate() {
        let (want, _) = &stored[i];
        assert_eq!(s.rep_uuid, want.rep_uuid);
        assert_eq!(s.t_start_ms, want.t_start_ms);
        assert_eq!(s.duration_ms, want.duration_ms);
        assert_eq!(s.computed_len, want.computed_len);
        assert!((s.rom - want.rom).abs() < 1e-6);
        assert!((s.peak_velocity - want.peak_velocity).abs() < 1e-6);
    }

    for (sum, want_trace) in &stored {
        let rep_uuid = calypso_core::pb::summary_uuid(sum).unwrap();
        let mut storage = vec![TracePoint::ZERO; 512];
        let mut out = TraceBuf::new(&mut storage);
        let n = store::read_trace(&mut log, &mut buf, &mut rec, rep_uuid, &mut out).unwrap();
        assert_eq!(n, want_trace.len(), "trace length for {rep_uuid:?}");
        for (got, want) in out.iter().zip(want_trace.iter()) {
            assert_eq!(got.t_ms, want.t_ms - want_trace[0].t_ms);
            assert!((got.pos[0] - want.pos[0]).abs() < 1e-6);
            assert!((got.pos[2] - want.pos[2]).abs() < 1e-6);
            assert_eq!(got.quat, want.quat);
        }
    }

    let mut storage = vec![TracePoint::ZERO; 512];
    let mut out = TraceBuf::new(&mut storage);
    assert_eq!(
        store::read_trace(&mut log, &mut buf, &mut rec, fixture_uuid(99), &mut out).unwrap(),
        0
    );
}

#[test]
fn oldest_reps_fall_off_when_the_ring_wraps() {
    let samples = squat_set(1).samples();
    let mut summary = pipeline(&samples).1.remove(0);
    let trace: Vec<TracePoint> = (0..4)
        .map(|i| TracePoint {
            t_ms: i * 10,
            pos: [0.0, 0.0, -0.1 * i as f32],
            quat: [QUAT_SCALE as i16, 0, 0, 0],
        })
        .collect();

    let mut flash = MockFlash::new();
    let mut buf = [0u8; RECORD_BUF_BYTES];
    {
        let mut log = mount(&mut flash);
        for i in 1..=20 {
            summary.rep_uuid = calypso_core::pb::uuid_to_field(&fixture_uuid(i));
            store::append_rep(&mut log, &mut buf, &mut summary, &trace, None).unwrap();
        }
    }

    let mut log = mount(&mut flash);
    let mut rec = calypso_core::pb::msg::RepRecord::default();
    let mut surviving = Vec::new();
    store::for_each_summary(&mut log, &mut buf, &mut rec, |s| {
        surviving.push(calypso_core::pb::summary_uuid(s).unwrap())
    })
    .unwrap();

    assert!(!surviving.is_empty());
    assert!(
        surviving.len() < 20,
        "the ring should have dropped some: {surviving:?}"
    );
    assert!(surviving.contains(&fixture_uuid(20)), "newest rep survives");
    let written: Vec<RepUuid> = (1..=20).map(fixture_uuid).collect();
    assert!(
        surviving.iter().all(|uuid| written.contains(uuid)),
        "only our reps"
    );

    let first_surviving = (1..=20)
        .find(|i| surviving.contains(&fixture_uuid(*i)))
        .expect("something survived");
    assert!(
        first_surviving > 1,
        "the oldest reps should have been overwritten"
    );
    for i in first_surviving..=20 {
        assert!(
            surviving.contains(&fixture_uuid(i)),
            "rep {i} should have survived"
        );
    }
    assert_eq!(surviving.len(), (20 - first_surviving + 1) as usize);
}

/// A trace read that is told where the rep is: the same points as the walk, and a fraction of the reads.
///
/// This is the path a trace request takes, and the cost is the thing being tested: the walk is what used to hold the sampler off its tick for seconds while the phone waited for a trace.
#[test]
fn a_trace_read_from_its_summary_walks_the_reps_records_only() {
    let samples = squat_set(1).samples();
    let (traces, summaries) = pipeline(&samples);
    let trace = &traces[0];
    let chunks = calypso_core::pb::chunk_count_for(trace.len());

    let mut flash = WideFlash::new();
    let mut buf = [0u8; RECORD_BUF_BYTES];
    let mut places = Vec::new();
    {
        let mut log = mount(&mut flash);
        store::append_clock(&mut log, &mut buf, 1_760_000_000).unwrap();
        for i in 1..=20u32 {
            let mut summary = summaries[0].clone();
            summary.rep_uuid = calypso_core::pb::uuid_to_field(&fixture_uuid(i));
            let summary_id = log.next_id();
            store::append_rep(&mut log, &mut buf, &mut summary, trace, None).unwrap();
            // The chunks are the records right after the summary, and the summary says how many there are.
            assert_eq!(summary.computed_chunks, chunks);
            places.push(summary_id + 1);
        }
    }

    let mut log = mount(&mut flash);
    let mut rec = msg::RepRecord::default();
    let mut storage = vec![TracePoint::ZERO; trace.len()];
    let mut walked = TraceBuf::new(&mut storage);
    reset_reads();
    let n = store::read_trace(&mut log, &mut buf, &mut rec, fixture_uuid(12), &mut walked).unwrap();
    let walk = reads();

    let mut storage = vec![TracePoint::ZERO; trace.len()];
    let mut from = TraceBuf::new(&mut storage);
    reset_reads();
    let m = store::read_trace_from(
        &mut log,
        &mut buf,
        &mut rec,
        places[11],
        chunks,
        fixture_uuid(12),
        &mut from,
    )
    .unwrap();
    let point = reads();

    assert_eq!(n, trace.len(), "the walk reads the whole trace");
    assert_eq!(m, n, "and so does the walk from the summary");
    for (a, b) in walked.iter().zip(from.iter()) {
        assert_eq!(a.t_ms, b.t_ms);
        assert!((a.pos[2] - b.pos[2]).abs() < 1e-6);
    }
    assert!(
        point * 4 < walk,
        "{point} bytes read for one trace, {walk} to walk the log for it"
    );
}

/// A rep a dev session recorded carries its window as well, and either trace reads back without disturbing the other:
/// the two runs are written together and read by kind, from a summary's record id and from a walk of the log.
#[test]
fn a_raw_trace_is_stored_after_the_plotted_one_and_reads_back_whole() {
    let samples = squat_set(1).samples();
    let (traces, summaries) = pipeline(&samples);
    let trace = &traces[0];
    let mut summary = summaries[0].clone();

    // The window the rep was measured from, at full rate: what a dev session stores beside the plotted trace.
    // It starts where the summary says it does, because that origin is what both traces count from.
    let from = samples
        .iter()
        .position(|s| s.t_ms >= summary.t_start_ms)
        .expect("a rep starts at one of the samples it was measured from");
    let window: Vec<Sample> = samples[from..].iter().take(300).copied().collect();
    assert_eq!(window[0].t_ms, summary.t_start_ms);
    let raw_chunks = calypso_core::pb::chunk_count_for(window.len());
    assert!(
        raw_chunks > calypso_core::pb::chunk_count_for(trace.len()),
        "the raw trace is the bigger one: full rate against a fixed budget"
    );

    let mut flash = BigFlash::new();
    let mut buf = [0u8; RECORD_BUF_BYTES];
    let want = fixture_uuid(7);
    summary.rep_uuid = calypso_core::pb::uuid_to_field(&want);
    let first_chunk_id = {
        let mut log = mount(&mut flash);
        let summary_id = log.next_id();
        store::append_rep(&mut log, &mut buf, &mut summary, trace, Some(&window)).unwrap();
        // What `store::append_rep` promises and the summary index relies on: the raw run starts after the plotted one.
        assert_eq!(summary.raw_len, window.len() as u32);
        assert_eq!(summary.raw_chunks, raw_chunks);
        assert_eq!(
            log.next_id(),
            summary_id + 1 + summary.computed_chunks + raw_chunks
        );
        summary_id + 1 + summary.computed_chunks
    };

    let mut log = mount(&mut flash);
    let mut rec = msg::RepRecord::default();

    // Read from the raw run's first record, which is what the summary stream records as a rep's place.
    let mut read = vec![Sample::ZERO; window.len()];
    let n = store::read_raw_from(
        &mut log,
        &mut buf,
        &mut rec,
        first_chunk_id,
        raw_chunks,
        want,
        &mut read,
    )
    .unwrap();
    assert_eq!(n, window.len(), "every sample of the window");
    assert_eq!(read, rebased(&window), "as the sensor read them");

    // And the same by walking the log, for a rep this connection never placed.
    let mut walked = vec![Sample::ZERO; window.len()];
    let m = store::read_raw(&mut log, &mut buf, &mut rec, want, &mut walked).unwrap();
    assert_eq!(m, n, "the walk and the placement find the same trace");
    assert_eq!(walked, read);

    // The plotted trace is still whole either way: the two runs do not eat into each other.
    let mut storage = vec![TracePoint::ZERO; trace.len()];
    let mut plotted = TraceBuf::new(&mut storage);
    let p = store::read_trace_from(
        &mut log,
        &mut buf,
        &mut rec,
        first_chunk_id - summary.computed_chunks,
        summary.computed_chunks,
        want,
        &mut plotted,
    )
    .unwrap();
    assert_eq!(p, trace.len());
    for (got, want) in plotted.iter().zip(trace.iter()) {
        assert_eq!(got.t_ms, want.t_ms - trace[0].t_ms);
        assert_eq!(got.quat, want.quat);
    }
}

/// What a stored sample reads back as: the wire counts from the rep's first sample, so an uptime arrives rebased.
fn rebased(window: &[Sample]) -> Vec<Sample> {
    let t0 = window.first().map_or(0, |s| s.t_ms);
    window
        .iter()
        .map(|s| Sample {
            t_ms: s.t_ms - t0,
            ..*s
        })
        .collect()
}

#[test]
fn summary_and_chunks_share_the_rep_uuid() {
    let samples = squat_set(1).samples();
    let (traces, summaries) = pipeline(&samples);
    let trace = &traces[0];
    let mut summary = summaries[0].clone();

    let mut flash = BigFlash::new();
    let mut buf = [0u8; RECORD_BUF_BYTES];
    let mut log = mount(&mut flash);
    let want = fixture_uuid(5);
    summary.rep_uuid = calypso_core::pb::uuid_to_field(&want);
    store::append_rep(&mut log, &mut buf, &mut summary, trace, None).unwrap();

    let mut records = Vec::new();
    log.for_each(&mut buf, |_, payload| {
        records.push(calypso_core::pb::decode_record(payload).unwrap())
    })
    .unwrap();

    let expected_chunks = calypso_core::pb::chunk_count_for(trace.len());
    assert_eq!(records.len(), 1 + expected_chunks as usize);
    match &records[0] {
        calypso_core::pb::Record::Summary(s) => {
            assert_eq!(calypso_core::pb::summary_uuid(s), Some(want));
            assert_eq!(s.computed_chunks, expected_chunks);
        }
        other => panic!("the rep starts with its summary, not {other:?}"),
    }
    for (i, record) in records[1..].iter().enumerate() {
        match record {
            calypso_core::pb::Record::Computed(t) => {
                assert_eq!(calypso_core::pb::computed_uuid(t), Some(want), "chunk {i}");
                assert_eq!(t.index, i as u32, "chunks are stored in index order");
            }
            other => panic!("expected a trace chunk, got {other:?}"),
        }
    }
}

#[test]
fn clock_records_sit_among_the_reps_without_disturbing_them() {
    let samples = squat_set(1).samples();
    let (traces, summaries) = pipeline(&samples);
    assert_eq!(traces.len(), 1);
    let mut summary = summaries[0].clone();
    summary.rep_uuid = calypso_core::pb::uuid_to_field(&fixture_uuid(1));
    let chunks = calypso_core::pb::chunk_count_for(traces[0].len());

    let mut flash = BigFlash::new();
    let mut buf = [0u8; RECORD_BUF_BYTES];
    {
        let mut log = mount(&mut flash);
        assert_eq!(log.next_id(), 1, "an empty log starts numbering at 1");
        // Every boot opens with its clock, and the client's write replaces it later in the same boot.
        assert_eq!(store::append_clock(&mut log, &mut buf, 0).unwrap(), 1);
        store::append_rep(&mut log, &mut buf, &mut summary, &traces[0], None).unwrap();
        assert_eq!(
            store::append_clock(&mut log, &mut buf, 1_760_000_000).unwrap(),
            3 + chunks,
            "the client's write lands after the summary and chunks of the rep it dates"
        );
    }

    // Reading it back: the clocks are not reps, and the rep is whole.
    let mut log = mount(&mut flash);
    let mut rec = msg::RepRecord::default();
    let mut back = Vec::new();
    store::for_each_summary(&mut log, &mut buf, &mut rec, |s| back.push(s.clone())).unwrap();
    assert_eq!(back.len(), 1);
    assert_eq!(back[0].computed_len, traces[0].len() as u32);

    let mut storage = vec![calypso_core::motion::rep::TracePoint::ZERO; traces[0].len()];
    let mut trace = TraceBuf::new(&mut storage);
    let points =
        store::read_trace(&mut log, &mut buf, &mut rec, fixture_uuid(1), &mut trace).unwrap();
    assert_eq!(points, traces[0].len());
}

#[test]
#[should_panic(expected = "is not a RepRecord")]
fn a_crc_valid_record_that_is_not_a_rep_record_panics() {
    let mut flash = MockFlash::new();
    let mut buf = [0u8; RECORD_BUF_BYTES];
    let mut rec = calypso_core::pb::msg::RepRecord::default();
    let mut log = mount(&mut flash);
    // The log CRCs whatever it is handed, so this passes the log's own check and is still not a RepRecord.
    log.append(b"not a RepRecord").unwrap();

    store::for_each_summary(&mut log, &mut buf, &mut rec, |_| {}).unwrap();
}
