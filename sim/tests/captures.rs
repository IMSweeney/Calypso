//! What a session did, against what the detector makes of it.
//!
//! `captures/captures.json` says how many reps each set of a session had, written down by the person who performed them rather than read off the code under test. This runs the firmware's own filter and detector over the recording and asserts they found those sets and no others, which is the only check that says a detector change helps a real session and not just a fixture.
//!
//! The recordings are recorded data and live in the bucket, so these tests are `#[ignore]`d: run them with `cargo test -- --ignored` once `CAPTURES_REMOTE` names the bucket, or after pulling the captures into `captures/`.

use calypso_core::rep::{DetectedRep, TracePoint, Zero};
use calypso_sim::captures::{self, Capture};
use calypso_sim::stream;

/// The gap that separates one set from the next.
///
/// The rest between sets is minutes; the longest gap between two reps of one set is the pause at the top of a stroke, which is a couple of seconds.
const SET_GAP_MS: u32 = 30_000;

/// How far a stored rep's bar path may end from where it began, as a fraction of the travel it made.
///
/// A rep is out and back, so its path closes at the turn it ends on. What is left over is the turn itself — a sample or two either side of the bar really stopping — and the velocity the window was seeded with, which is the device's rather than the true one. Half a stroke out of phase is nothing like that small.
const CLOSED_SLACK: f32 = 0.15;

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

/// Every stored rep's bar path closes: it begins at a turn, and comes back to the same height, with the turn between its halves as far from both as the rep's own travel.
///
/// This is the property the app needs — one bar path per rep is a rep that starts and ends at the same place — and it is the one a count cannot see. A span half a stroke out of phase is still one rep per stroke, and the trace it carries is an hourglass rather than a stroke.
#[test]
#[ignore = "reads the captures from the bucket"]
fn a_stored_rep_begins_and_ends_at_the_same_height() {
    let mut wrong: Vec<String> = Vec::new();
    for capture in captures::manifest()
        .into_iter()
        .filter(|c| c.expected.is_some())
    {
        let samples = stream::samples(&captures::bytes(&capture.name));
        let out = stream::replay(&samples, &mut |_, _, _| {});
        for ((rep, rom), trace) in out.reps.iter().zip(&out.roms).zip(&out.traces) {
            let Some((start, middle, end)) = shape(rep, trace) else {
                continue;
            };
            let back = (end - start).abs();
            let out = (middle - start).abs();
            if back > CLOSED_SLACK * rom || out < (1.0 - CLOSED_SLACK) * rom {
                wrong.push(format!(
                    "{} ({}): {}..{} ms ends {back:.3} m from where it began and is {out:.3} m deep, on a rom of {rom:.3} m",
                    capture.name, capture.recorded, rep.span.start_ms, rep.span.end_ms
                ));
            }
        }
    }
    assert!(wrong.is_empty(), "\n{}", wrong.join("\n"));
}

/// The height a rep's trace begins at, is at the turn between its halves, and ends at.
///
/// The end is read at the rep's own last turn rather than at the window's last sample, because the window plots a margin past the rep and what the bar did in it is not what is being measured.
fn shape(rep: &DetectedRep, trace: &[TracePoint]) -> Option<(f32, f32, f32)> {
    let middle = rep.turns.zeros[1].map_or(rep.span.end_ms, |zero| match zero {
        Zero::Rest(t_ms) | Zero::Turn(t_ms) => t_ms,
    });
    Some((
        trace.first()?.pos[2],
        height_at(trace, middle)?,
        height_at(trace, rep.span.end_ms)?,
    ))
}

fn height_at(trace: &[TracePoint], t_ms: u32) -> Option<f32> {
    trace
        .iter()
        .min_by_key(|p| p.t_ms.abs_diff(t_ms))
        .map(|p| p.pos[2])
}

/// The reps the detector closes in one recording, counted per set.
fn reps_per_set(name: &str) -> Vec<u32> {
    let samples = stream::samples(&captures::bytes(name));
    let out = stream::replay(&samples, &mut |_, _, _| {});
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
