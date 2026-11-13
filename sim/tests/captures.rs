//! What a session did, against what the detector makes of it.
//!
//! `captures/captures.json` says what each set of a session was — how many reps it had, and when the note recording it was written — taken from the person who performed the work rather than read off the code under test. This runs the firmware's own filter and detector over the recording and asserts they found those sets, in the places those sets happened, which is the only check that says a detector change helps a real session and not just a fixture.
//!
//! The recordings are recorded data and live in the bucket, so a run needs them pulled into `captures/` or `CAPTURES_REMOTE` naming the bucket; without either they fail rather than read nothing.

use calypso_core::motion::fusion::Fusion;
use calypso_core::motion::rep::{DetectedRep, TracePoint};
use calypso_sim::captures::{self, Capture, Set};
use calypso_sim::{replay, stream};

/// A recording's samples through the firmware's filter and detector, started from a boot.
fn replay_recording(pblog: &[u8]) -> replay::Replay {
    replay::run(
        &stream::samples(pblog),
        Fusion::new(),
        [0.0; 3],
        replay::TRACE_CAPACITY,
        &mut |_, _, _, _| {},
    )
}

/// The gap that separates one set from the next.
///
/// The rest between sets is minutes; the longest gap between two reps of one set is the pause at the top of a stroke, which is a couple of seconds.
const SET_GAP_MS: u32 = 30_000;

/// How far a stored rep's bar path may end from where it began, as a fraction of the travel it made.
///
/// A rep is out and back, so its path closes at the turn it ends on. What is left over is the turn itself — a sample or two either side of the bar really stopping — and the velocity the window was seeded with, which is the device's rather than the true one. Half a stroke out of phase is nothing like that small.
const CLOSED_SLACK: f32 = 0.15;

#[test]
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

/// A set is over before the note that records it was written.
///
/// A note is typed once the set is done, so its time is an upper bound on when the set's last rep happened, and it is a bound the performance set rather than the recording. A rep the detector closes after it is made of the rest, not of the set.
///
/// Where the detector closed a different number of sets than the session had, there is nothing to pair them on, and `a_session_is_the_sets_it_was` has that.
#[test]
fn a_set_is_over_before_its_note_is_written() {
    let mut wrong: Vec<String> = Vec::new();
    for capture in captures::manifest()
        .into_iter()
        .filter(|c| c.expected.is_some())
    {
        let expected = capture.expected.as_ref().expect("filtered for it");
        let pblog = captures::bytes(&capture.name);
        let found = found_sets(&pblog);
        if found.len() != expected.sets.len() {
            continue;
        }
        let epoch_ms = epoch_ms(&capture.name, &pblog);
        for (n, (set, want)) in found.iter().zip(&expected.sets).enumerate() {
            let end_ms = set.last().expect("a set holds a rep").span.end_ms;
            let late_ms = epoch_ms + i64::from(end_ms) - want.noted_at.0;
            if late_ms > 0 {
                wrong.push(format!(
                    "{} ({}): set {}, {}, closed its last rep {:.1} s after the note that says it was over",
                    capture.name,
                    capture.recorded,
                    n + 1,
                    describe(want),
                    late_ms as f64 / 1000.0
                ));
            }
        }
    }
    assert!(wrong.is_empty(), "\n{}", wrong.join("\n"));
}

/// Nothing the detector closes comes after the session's last note.
///
/// That note is written between the last set and the recording being stopped, so a rep closed after it is the bar going back in the rack or the phone being picked up, and never a rep.
/// It is also the one placement check that needs no pairing, so it still says something on a session whose sets the detector got wrong.
#[test]
fn nothing_is_closed_after_the_session_was_noted_over() {
    let mut wrong: Vec<String> = Vec::new();
    for capture in captures::manifest()
        .into_iter()
        .filter(|c| c.expected.is_some())
    {
        let expected = capture.expected.as_ref().expect("filtered for it");
        let over_at = expected
            .sets
            .last()
            .expect("a session performed a set")
            .noted_at
            .0;
        let pblog = captures::bytes(&capture.name);
        let epoch_ms = epoch_ms(&capture.name, &pblog);
        let out = replay_recording(&pblog);
        for rep in &out.reps {
            let late_ms = epoch_ms + i64::from(rep.span.end_ms) - over_at;
            if late_ms > 0 {
                wrong.push(format!(
                    "{} ({}): a rep closed {}..{} ms, {:.1} s after the note that ended the session",
                    capture.name,
                    capture.recorded,
                    rep.span.start_ms,
                    rep.span.end_ms,
                    late_ms as f64 / 1000.0
                ));
            }
        }
    }
    assert!(wrong.is_empty(), "\n{}", wrong.join("\n"));
}

/// Every stored rep's bar path closes: it begins at a turn, and comes back to the same height, with the turn between its halves as far from both as the rep's own travel.
///
/// This is the property the app needs — one bar path per rep is a rep that starts and ends at the same place — and it is the one a count cannot see. A span half a stroke out of phase is still one rep per stroke, and the trace it carries is an hourglass rather than a stroke.
#[test]
fn a_stored_rep_begins_and_ends_at_the_same_height() {
    let mut wrong: Vec<String> = Vec::new();
    for capture in captures::manifest()
        .into_iter()
        .filter(|c| c.expected.is_some())
    {
        let out = replay_recording(&captures::bytes(&capture.name));
        for ((rep, summary), trace) in out.reps.iter().zip(&out.summaries).zip(&out.traces) {
            let rom = summary.rom;
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
    let middle = rep.turns.zeros[1].map_or(rep.span.end_ms, |anchor| anchor.t_ms);
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
    found_sets(&captures::bytes(name))
        .iter()
        .map(|set| u32::try_from(set.len()).expect("a set of reps"))
        .collect()
}

/// The sets the detector closes in one recording, in the order it closed them.
fn found_sets(pblog: &[u8]) -> Vec<Vec<DetectedRep>> {
    sets(&replay_recording(pblog).reps)
}

/// The wall clock the recording's uptime counts from, in unix milliseconds.
fn epoch_ms(name: &str, pblog: &[u8]) -> i64 {
    let epoch_s = stream::epoch_s(pblog).unwrap_or_else(|| {
        panic!("{name}: the recording holds no clock, so nothing on its timeline can be dated")
    });
    i64::from(epoch_s) * 1000
}

/// A set as the note describes it, for a reader of a failure.
fn describe(set: &Set) -> String {
    match set.load_lb {
        Some(lb) => format!("{} reps at {lb} lb", set.reps),
        None => format!("{} reps bar only", set.reps),
    }
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
