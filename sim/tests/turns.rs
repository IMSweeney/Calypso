//! What a rep is delimited by: the vertical motion, and only the vertical motion.
//!
//! A rep is counted on the **vertical** speed — the bar setting off one way and coming back the other — and not on the whole velocity. The two are not the same question on a real device: the frame has only the accelerometer to hold it level, and the residue of a couple of degrees sits in the horizontal, where it accumulates through a stroke. The bar turns; the horizontal never does.
//!
//! The other half of the same question is what happens when the bar gives a few centimetres back at the turnaround. That motion is opposite to the drive, and it must not be allowed to split or consume the rep it belongs to.

use calypso_core::sampler::Sample;
use calypso_sim::fixtures::{detect_and_trace, LEAD_IN_MS, SAMPLE_DT_MS};
use calypso_sim::motion::{Rep, TRACE_POINTS};

fn stored(samples: &[Sample]) -> Vec<calypso_core::pb::msg::RepSummary> {
    detect_and_trace(samples, TRACE_POINTS)
        .into_iter()
        .map(|(summary, _)| summary)
        .filter(|s| s.is_a_rep())
        .collect()
}

/// A bar that settles back at the top before the return is the same rep it would have been.
///
/// The settle is real motion — a bar caught at lockout gives a few centimetres back — and it is far too slow to clear the speed a rep is counted by, so it is not half of one and cannot split the rep it belongs to.
#[test]
fn a_settle_at_the_turnaround_is_still_one_rep() {
    let base = Rep::bench();
    for (settle_m, settle_speed) in [(0.03f32, 0.15f32), (0.05, 0.15), (0.06, 0.2)] {
        let rep = Rep {
            settle_m,
            settle_speed,
            ..base
        };
        let reps = stored(&rep.samples());
        assert_eq!(
            reps.len(),
            1,
            "a {settle_m} m settle at {settle_speed} m/s: {} reps stored",
            reps.len()
        );
        let rom = reps[0].rom;
        assert!(
            (rom - base.travel).abs() < base.travel * 0.10,
            "a {settle_m} m settle: rom {rom:.4} for {:.3} m of travel",
            base.travel
        );
    }
}

/// A gyroscope whose offset moves after the boot does not stop the bar from turning.
///
/// The filter learns the offset at rest and cannot learn it through a stroke, so a degree per second that arrives with the first rep is a degree per second of tilt through it: `g·sin` of that is a few tenths of a metre per second squared in the horizontal, which over a stroke is a speed of its own. The vertical is unaffected, and the vertical is what the rep is counted on.
#[test]
fn a_gyro_offset_that_arrives_with_a_rep_does_not_hide_its_turn() {
    let base = Rep::bench();
    let samples = base.samples();
    let mut out = Vec::new();
    for s in &samples {
        let mut s = *s;
        // Nothing while the bar is still, then 1 dps about X from the moment it is not: the offset a gyro's temperature drift leaves when it moves between the rack and the set.
        if s.t_ms > LEAD_IN_MS - SAMPLE_DT_MS {
            s.gyro[0] += calypso_core::sampler::quantize(1.0, calypso_core::sampler::GYRO_SCALE);
        }
        out.push(s);
    }
    let reps = stored(&out);
    assert_eq!(reps.len(), 1, "{} reps stored", reps.len());
    let rom = reps[0].rom;
    assert!(
        (rom - base.travel).abs() < base.travel * 0.10,
        "rom {rom:.4} for {:.3} m of travel",
        base.travel
    );
}
