//! One pass through the pipeline the firmware runs: samples in, the reps the detector closes out.
//!
//! Every replay in this crate — synthetic sets, a recording of the live stream, one stored window — is this loop with a different starting filter state, so the instruments and the tests cannot disagree about what the device would have done.

use calypso_core::motion::fusion::{magnitude, Filtered, Fusion};
use calypso_core::motion::integrate::{Integrated, Integration};
use calypso_core::motion::rep::{
    compute_rep, DetectedRep, Detector, TraceBuf, TracePoint, MARGIN_MS,
};
use calypso_core::pb::msg;
use calypso_core::sampler::{Sample, SampleRing};

/// Long enough for any window the device stores, with the margins the ring has to hold.
const RING: usize = 4096;
const WINDOW_SAMPLES: usize = 1200;

/// A trace budget for an instrument: longer than the device stores, so a replay is not reading its own decimation.
pub const TRACE_CAPACITY: usize = 256;

/// What one pass found.
pub struct Replay {
    /// Every rep the detector closed, in the order it closed them.
    pub reps: Vec<DetectedRep>,
    /// Each rep's summary, in the same order: in particular the travel its own window reports.
    pub summaries: Vec<msg::RepSummary>,
    /// Each rep's plotted trace, in the same order: what the app would draw for it.
    pub traces: Vec<Vec<TracePoint>>,
    /// Where the filter first knew which way is down, on the device's clock.
    pub ready_at_ms: Option<u32>,
    /// How many samples it called resting.
    pub resting: usize,
    /// The largest speed the filter reached, and the largest of it that was horizontal.
    ///
    /// Splitting the two is what tells a bar that moved from a gravity that leaked: a tilt puts speed into the horizontal plane, where nothing a rep does ought to put metres per second.
    pub peak_speed: f32,
    pub peak_horizontal: f32,
    pub halves: u32,
}

/// Feed the whole stream through one filter and one detector, and close the reps it finds.
///
/// `fusion` is where the replay starts: [`Fusion::new`] for a stream whose beginning is a boot, [`Fusion::from_state`] for a window that says the state it was cut from.
/// `each` sees every sample, where the filter stood after it, and the speed the detector counted on it, which is what a report on the signals the detections are made of is; a caller that only wants the reps passes a no-op.
pub fn run(
    samples: &[Sample],
    mut fusion: Fusion,
    velocity: [f32; 3],
    trace_capacity: usize,
    each: &mut dyn FnMut(&Sample, &Filtered, &Integrated, f32),
) -> Replay {
    let mut ring_buf = vec![Sample::ZERO; RING];
    let mut ring = SampleRing::new(&mut ring_buf);
    let mut integration = Integration::from_velocity(velocity);
    let mut det = Detector::new();
    let mut window = vec![Sample::ZERO; WINDOW_SAMPLES];
    let mut storage = vec![TracePoint::ZERO; trace_capacity];
    let mut out = Replay {
        reps: Vec::new(),
        summaries: Vec::new(),
        traces: Vec::new(),
        ready_at_ms: None,
        resting: 0,
        peak_speed: 0.0,
        peak_horizontal: 0.0,
        halves: 0,
    };

    for s in samples {
        ring.push(*s);
        let filtered = fusion.update(s);
        let integrated = integration.step(
            filtered.t_ms,
            filtered.linear_accel,
            filtered.resting,
            filtered.ready,
            &[],
        );
        if filtered.resting {
            out.resting += 1;
        }
        let v = integrated.velocity;
        out.peak_speed = out.peak_speed.max(magnitude(v));
        out.peak_horizontal = out.peak_horizontal.max((v[0] * v[0] + v[1] * v[1]).sqrt());
        if out.ready_at_ms.is_none() && fusion.ready() {
            out.ready_at_ms = Some(s.t_ms);
        }
        // The device gates the detector on the filter knowing which way is down, and so does this: before that there is no vertical for a rep to be measured against.
        if fusion.ready() {
            if let Some(rep) = det.update(&filtered, &integrated) {
                let n = ring.copy_window(rep.from_ms, rep.span.end_ms, 0, MARGIN_MS, &mut window);
                if n > 0 {
                    let mut trace = TraceBuf::new(&mut storage);
                    let summary =
                        compute_rep(&window[..n], &rep.state, Some(rep.turns), &mut trace);
                    out.reps.push(rep);
                    out.summaries.push(summary);
                    out.traces.push(trace.as_slice().to_vec());
                }
            }
        }
        each(s, &filtered, &integrated, det.vertical());
    }
    out.halves = det.halves();
    out
}
