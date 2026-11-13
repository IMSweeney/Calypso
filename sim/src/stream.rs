//! What the detector makes of a recorded stream, which is the only way to ask what it made of a real session.
//!
//! A phone's recording holds samples and nothing else: the device was streaming for the whole session and stored no window, no summary and no state. So the samples go back through the same `Fusion` and `Detector` the firmware runs, from the state a device that has just powered on starts in, and what comes out is what the device would have stored had it been detecting.
//!
//! One thing a replay cannot recover is the stream's own beginning. A recording starts wherever the record button was pressed, so the filter spends its first `READY_S` of quiet earning a gate the device had already earned before the recording began — a rep inside that stretch is unmeasurable here and was measurable on the device.

use calypso_core::fusion::{magnitude, Filtered, Fusion};
use calypso_core::pb::{decode_record, raw_point_from_message, Record};
use calypso_core::rep::{compute_rep, DetectedRep, Detector, TraceBuf, TracePoint, MARGIN_MS};
use calypso_core::sampler::{Sample, SampleRing};

use crate::fixtures::read_dump;

/// Long enough for any window the device stores, with the margins the ring has to hold.
const RING: usize = 2048;
const WINDOW_SAMPLES: usize = 1200;
const TRACE_POINTS: usize = 256;

/// One recording, replayed.
pub struct Replay {
    /// Every rep the detector closed, in the order it closed them.
    pub reps: Vec<DetectedRep>,
    /// The travel each rep's own window reports, in the same order.
    pub roms: Vec<f32>,
    /// Each rep's plotted trace, in the same order: what the app would draw for it.
    pub traces: Vec<Vec<TracePoint>>,
    /// Where the filter first knew which way is down, on the device's clock.
    pub ready_at_ms: Option<u32>,
    /// How many samples it called still.
    pub still: usize,
    /// The largest speed the filter reached, and the largest of it that was horizontal.
    ///
    /// Splitting the two is what tells a bar that moved from a gravity that leaked: a tilt puts speed into the horizontal plane, where nothing a rep does ought to put metres per second.
    pub peak_speed: f32,
    pub peak_horizontal: f32,
    pub halves: u32,
}

/// The samples one recording holds, on the device's own clock.
///
/// A point's time is an offset from its chunk's `first_t_ms`, which is the uptime of the chunk's first sample: adding it back is what puts the whole session on the clock the detector steps in.
pub fn samples(pblog: &[u8]) -> Vec<Sample> {
    let (records, truncated) = read_dump(pblog);
    assert!(!truncated, "the recording is torn at the end");
    let mut samples: Vec<Sample> = Vec::new();
    for record in &records {
        match decode_record(record).expect("a record in the recording decodes") {
            Record::Imu(c) => {
                for p in c.samples.iter() {
                    let mut s = raw_point_from_message(p);
                    s.t_ms = c.first_t_ms.saturating_add(s.t_ms);
                    samples.push(s);
                }
            }
            // The boot's epoch, for dating the stream on a wall clock. Nothing here reads a calendar.
            Record::Clock(_) => {}
            other => panic!("a live stream recording cannot hold a {other:?}"),
        }
    }
    samples
}

/// Feed the whole stream through one filter and one detector, and close the reps it finds.
///
/// `each` sees every sample, where the filter stood after it, and the speed the detector counted on it, which is what a report on the signals the detections are made of is; a caller that only wants the reps passes a no-op.
pub fn replay(samples: &[Sample], each: &mut dyn FnMut(&Sample, &Filtered, f32)) -> Replay {
    let mut ring_buf = vec![Sample::ZERO; RING];
    let mut ring = SampleRing::new(&mut ring_buf);
    let mut fusion = Fusion::new();
    let mut det = Detector::new();
    let mut window = vec![Sample::ZERO; WINDOW_SAMPLES];
    let mut storage = vec![TracePoint::ZERO; TRACE_POINTS];
    let mut out = Replay {
        reps: Vec::new(),
        roms: Vec::new(),
        traces: Vec::new(),
        ready_at_ms: None,
        still: 0,
        peak_speed: 0.0,
        peak_horizontal: 0.0,
        halves: 0,
    };

    for s in samples {
        ring.push(*s);
        let f = fusion.update(s);
        if f.still {
            out.still += 1;
        }
        let v = f.state.velocity;
        out.peak_speed = out.peak_speed.max(magnitude(v));
        out.peak_horizontal = out.peak_horizontal.max((v[0] * v[0] + v[1] * v[1]).sqrt());
        if out.ready_at_ms.is_none() && fusion.ready() {
            out.ready_at_ms = Some(s.t_ms);
        }
        // The device gates the detector on the filter knowing which way is down, and so does this:
        // before that there is no vertical for a rep to be measured against.
        if fusion.ready() {
            if let Some(rep) = det.update(&f) {
                let n = ring.copy_window(rep.from_ms, rep.span.end_ms, 0, MARGIN_MS, &mut window);
                if n > 0 {
                    let mut trace = TraceBuf::new(&mut storage);
                    let summary =
                        compute_rep(&window[..n], &rep.state, Some(rep.turns), &mut trace);
                    out.reps.push(rep);
                    out.roms.push(summary.rom);
                    out.traces.push(trace.as_slice().to_vec());
                }
            }
        }
        each(s, &f, det.vertical());
    }
    out.halves = det.halves();
    out
}
