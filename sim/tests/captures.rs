//! What a session did, against what the detector makes of it.
//!
//! `captures/captures.json` says how many reps each set of a session had, written down by the person
//! who performed them rather than read off the code under test. This runs the firmware's own filter
//! and detector over the recording and asserts they found those sets and no others, which is the only
//! check that says a detector change helps a real session and not just a fixture.
//!
//! The recordings are recorded data and live in the bucket, so these tests are `#[ignore]`d: run them
//! with `cargo test -- --ignored` once `CAPTURES_REMOTE` names the bucket, or after pulling the
//! captures into `captures/`.

use calypso_core::rep::DetectedRep;
use calypso_sim::captures::{self, Capture};
use calypso_sim::stream;

/// The gap that separates one set from the next.
///
/// The rest between sets is minutes; the longest gap between two reps of one set is the pause at the
/// top of a stroke, which is a couple of seconds.
const SET_GAP_MS: u32 = 30_000;

#[test]
#[ignore = "reads the captures from the bucket"]
fn a_session_is_the_sets_it_was() {
    let described: Vec<Capture> = captures::manifest()
        .into_iter()
        .filter(|c| c.expected.is_some())
        .collect();
    assert!(
        !described.is_empty(),
        "no capture in the manifest says what its session did"
    );

    let mut wrong: Vec<String> = Vec::new();
    for capture in &described {
        let expected = capture.expected.as_ref().expect("filtered for it");
        assert_eq!(
            capture.kind, "live-stream",
            "{}: only a recording of the stream can be replayed as a session",
            capture.name
        );
        let found = reps_per_set(&capture.name);
        let want: Vec<u32> = expected.sets.iter().map(|s| s.reps).collect();
        if found != want {
            wrong.push(format!(
                "{} ({}): the detector closed sets of {found:?}, and the session was {want:?}",
                capture.name, capture.recorded
            ));
        }
    }
    assert!(wrong.is_empty(), "\n{}", wrong.join("\n"));
}

/// The reps the detector closes in one recording, counted per set.
fn reps_per_set(name: &str) -> Vec<u32> {
    let samples = stream::samples(&captures::bytes(name));
    let out = stream::replay(&samples, &mut |_, _| {});
    sets(&out.reps)
        .iter()
        .map(|set| u32::try_from(set.len()).expect("a set of reps"))
        .collect()
}

/// The reps grouped back into the sets they were performed in, in order.
fn sets(reps: &[DetectedRep]) -> Vec<Vec<DetectedRep>> {
    let mut out: Vec<Vec<DetectedRep>> = Vec::new();
    for &rep in reps {
        let continues = out.last().is_some_and(|set| {
            let prev = set.last().expect("a set holds a rep");
            rep.span.start_ms.saturating_sub(prev.span.end_ms) <= SET_GAP_MS
        });
        if continues {
            out.last_mut().expect("just checked").push(rep);
        } else {
            out.push(vec![rep]);
        }
    }
    out
}
