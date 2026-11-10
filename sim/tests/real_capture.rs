//! What a real set looks like to the detector: two hand-held sets off one device, each labelled by the person who performed it.
//!
//! This is the one place a stored window is judged, and it is here because a synthetic stream cannot make the motion that broke the old turn rule. A stroke is measured through an attitude the filter can only correct while the bar is still, so a set with almost no rest carries a tilt that the integrated velocity turns into metres per second — and a turn read off a speed that never comes back to zero is a turn that never happens. On this capture it stored nothing at all from four reps, each time.
//!
//! The windows are `captures/sets-2026-11-04.pblog`: three still ones and these two.

use calypso_core::fusion::{Fusion, FusionState};
use calypso_core::pb::msg;
use calypso_core::rep::{compute_rep, Detector, TraceBuf, TracePoint, MARGIN_MS};
use calypso_core::sampler::{Sample, SampleRing};
use calypso_sim::fixtures::{capture_windows, Window};

const DUMP: &str = "../captures/sets-2026-11-04.pblog";
const RING: usize = 2048;
const WINDOW_SAMPLES: usize = 1200;
const TRACE_POINTS: usize = 256;

fn windows() -> Vec<Window> {
    let bytes = std::fs::read(DUMP).expect("the capture reads");
    let (windows, truncated) = capture_windows(&bytes);
    assert!(!truncated, "{DUMP}: torn at the end");
    windows
}

/// The reps a window holds, replayed from `from`.
fn reps(w: &Window, from: FusionState) -> Vec<msg::RepSummary> {
    let mut ring_buf = vec![Sample::ZERO; RING];
    let mut ring = SampleRing::new(&mut ring_buf);
    let mut fusion = Fusion::from_state(from);
    let mut det = Detector::new();
    let mut window = vec![Sample::ZERO; WINDOW_SAMPLES];
    let mut storage = vec![TracePoint::ZERO; TRACE_POINTS];
    let mut out = Vec::new();
    for s in &w.samples {
        ring.push(*s);
        let filtered = fusion.update(s);
        if let Some(rep) = det.update(&filtered) {
            let n = ring.copy_window(rep.from_ms, rep.span.end_ms, 0, MARGIN_MS, &mut window);
            if n == 0 {
                continue;
            }
            let mut trace = TraceBuf::new(&mut storage);
            out.push(compute_rep(
                &window[..n],
                &rep.state,
                Some(rep.turns),
                &mut trace,
            ));
        }
    }
    out
}

/// Four reps were performed, and four come back — whichever velocity the replay starts from.
///
/// The second half is the point of the rule: the capture's own state carries 8.1 and 15.0 m/s, which is what the un-corrected attitude integrated over the set before it. A detector that needs that number to be right to find a turn is a detector that only works on a bar that never moves for long, so the same window replayed from rest has to answer the same way.
#[test]
fn a_hand_held_set_comes_back_as_the_reps_it_was() {
    let windows = windows();
    for (prefix, v0) in [("bb872722", 8.1f32), ("ebb17161", 15.0)] {
        let w = windows
            .iter()
            .find(|w| w.uuid.starts_with(prefix))
            .unwrap_or_else(|| panic!("{prefix} is in {DUMP}"));
        // The filter's own velocity when the window opened, which is what the summary stores.
        let carried = (w.state.velocity[0].powi(2)
            + w.state.velocity[1].powi(2)
            + w.state.velocity[2].powi(2))
        .sqrt();
        assert!(
            (carried - v0).abs() < 0.1,
            "{prefix}: the window was stored carrying {carried:.1} m/s, not {v0}"
        );

        let as_stored = reps(w, w.state);
        assert_eq!(
            as_stored.len(),
            4,
            "{prefix}: four reps were performed, replayed from {carried:.1} m/s"
        );

        let mut from_rest = w.state;
        from_rest.velocity = [0.0; 3];
        let from_rest = reps(w, from_rest);
        assert_eq!(from_rest.len(), 4, "{prefix}: the same window from rest");

        // **The travel a rep reports no longer depends on the speed the window carried.** Both replays anchor at the same turns, and the turn is the one instant in a stroke where the bar's own speed is zero, so the two can only differ where a cue could not name that instant and the carried velocity ran on. Before the anchoring these read 9× and 24× the from-rest travel for the two windows' worst reps; the pin is that neither column can be more than twice the other.
        for (k, (stored, rest)) in as_stored.iter().zip(&from_rest).enumerate() {
            let ratio = stored.rom / rest.rom;
            assert!(
                (0.5..=2.0).contains(&ratio),
                "{prefix} rep {}: {:.2} m from the state the window carried, {:.2} m from rest",
                k + 1,
                stored.rom,
                rest.rom
            );
        }
    }
}

/// A still window holds no reps, and that has to stay true at any starting velocity too.
///
/// The three still windows of the same session are the control: the filter never ran away on them, and nothing in them moves enough to turn.
#[test]
fn a_still_window_still_holds_nothing() {
    let windows = windows();
    for prefix in ["8c862e0c", "accb9c08", "9ed28b16"] {
        let w = windows
            .iter()
            .find(|w| w.uuid.starts_with(prefix))
            .unwrap_or_else(|| panic!("{prefix} is in {DUMP}"));
        assert!(
            reps(w, w.state).is_empty(),
            "{prefix}: a still window stored a rep"
        );
    }
}
