//! A rep's replay: the plotted trace it stores, and the summary that goes with it.

use crate::motion::fusion::{magnitude, roll_pitch_deg, Fusion};
use crate::motion::integrate::{Anchor, Integration};
use crate::motion::rep::{RepSeed, RepTurns};
use crate::pb::msg;
use crate::sampler::{quantize, Sample, QUAT_SCALE};

/// How many points a rep's **plotted** trace keeps, however long the rep is.
pub const TRACE_POINTS: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TracePoint {
    pub t_ms: u32,
    /// ZUPT-anchored displacement from the rep start (m).
    pub pos: [f32; 3],
    /// The attitude the filter computed at this point `[w, x, y, z]` (`sampler::QUAT_SCALE` LSB).
    pub quat: [i16; 4],
}

impl TracePoint {
    pub const ZERO: TracePoint = TracePoint {
        t_ms: 0,
        pos: [0.0; 3],
        quat: [0; 4],
    };
}

pub struct TraceBuf<'a> {
    points: &'a mut [TracePoint],
    len: usize,
}

impl<'a> TraceBuf<'a> {
    pub fn new(points: &'a mut [TracePoint]) -> Self {
        TraceBuf { points, len: 0 }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn capacity(&self) -> usize {
        self.points.len()
    }

    pub fn get(&self, i: usize) -> Option<&TracePoint> {
        if i < self.len {
            self.points.get(i)
        } else {
            None
        }
    }

    pub fn iter(&self) -> core::slice::Iter<'_, TracePoint> {
        self.points[..self.len].iter()
    }

    pub fn as_slice(&self) -> &[TracePoint] {
        &self.points[..self.len]
    }

    pub fn push(&mut self, p: TracePoint) {
        if self.len < self.points.len() {
            self.points[self.len] = p;
            self.len += 1;
        }
    }

    fn clear(&mut self) {
        self.len = 0;
    }
}

#[derive(Debug, Clone, Copy)]
struct Replay {
    /// Velocity and position at the rep's own end.
    v_end: [f32; 3],
    p_end: [f32; 3],
    v_max: f32,
    lo: [f32; 3],
    hi: [f32; 3],
    roll_min: f32,
    roll_max: f32,
    pitch_min: f32,
    pitch_max: f32,
}

fn replay(
    samples: &[Sample],
    from: &RepSeed,
    turns: Option<RepTurns>,
    out: &mut TraceBuf<'_>,
    velocity_shift: Option<[f32; 3]>,
) -> Replay {
    out.clear();
    let n = samples.len();
    let capacity = out.capacity();
    let stride = if capacity == 0 {
        n.max(1)
    } else {
        n.div_ceil(capacity).max(1)
    };
    let first = if capacity == 0 { 0 } else { (n - 1) % stride };

    let mut fusion = Fusion::from_state(from.filter);
    let mut integration = Integration::from_velocity(from.velocity);
    let mut p = [0.0f32; 3];
    let mut v_prev = from.velocity;
    let mut anchors = [Anchor::rest(0); 3];
    let mut anchor_len = 0usize;
    if let Some(t) = turns {
        for anchor in t.zeros.into_iter().flatten() {
            anchors[anchor_len] = anchor;
            anchor_len += 1;
        }
    }
    for anchor in &mut anchors[..anchor_len] {
        if let Some(sample) = samples.iter().find(|s| s.t_ms >= anchor.t_ms) {
            anchor.t_ms = sample.t_ms;
        }
    }
    let anchors = &anchors[..anchor_len];
    let (span_start, span_end) = turns.map_or((u32::MIN, u32::MAX), |t| (t.start_ms, t.end_ms));
    let span_ms = span_end.saturating_sub(span_start).max(1);

    let mut pass = Replay {
        v_end: [0.0; 3],
        p_end: [0.0; 3],
        v_max: 0.0,
        lo: [f32::MAX; 3],
        hi: [f32::MIN; 3],
        roll_min: f32::MAX,
        roll_max: f32::MIN,
        pitch_min: f32::MAX,
        pitch_max: f32::MIN,
    };
    let mut roll_offset = 0.0f32;
    let mut roll_prev = 0.0f32;
    let mut roll_seen = false;

    for i in 0..n {
        let sample = &samples[i];
        let filtered = fusion.update(sample);
        integration.step(
            sample.t_ms,
            filtered.linear_accel,
            filtered.resting,
            filtered.ready,
            anchors,
        );
        let mut v = integration.velocity();
        if let Some(shift) = velocity_shift {
            if sample.t_ms <= span_end {
                let k = (sample.t_ms.saturating_sub(span_start) as f32 / span_ms as f32)
                    .clamp(0.0, 1.0);
                for (v, shift) in v.iter_mut().zip(shift) {
                    *v -= shift * k;
                }
            }
        }
        if i > 0 {
            let dt = (sample.t_ms - samples[i - 1].t_ms) as f32 / 1000.0;
            for (p, (v_prev, v)) in p.iter_mut().zip(v_prev.iter().zip(v.iter())) {
                *p += 0.5 * (v_prev + v) * dt;
            }
        }
        v_prev = v;

        let speed = magnitude(v);
        if speed > pass.v_max {
            pass.v_max = speed;
        }
        if sample.t_ms >= span_start && sample.t_ms <= span_end {
            for (p, (lo, hi)) in p.iter().zip(pass.lo.iter_mut().zip(pass.hi.iter_mut())) {
                *lo = lo.min(*p);
                *hi = hi.max(*p);
            }
            pass.v_end = v;
            pass.p_end = p;
        }

        let (roll, pitch) = roll_pitch_deg(filtered.state.attitude);
        if roll_seen {
            let step = roll - roll_prev;
            if step > 180.0 {
                roll_offset -= 360.0;
            } else if step < -180.0 {
                roll_offset += 360.0;
            }
        }
        roll_seen = true;
        roll_prev = roll;
        let roll = roll + roll_offset;
        if roll < pass.roll_min {
            pass.roll_min = roll;
        }
        if roll > pass.roll_max {
            pass.roll_max = roll;
        }
        if pitch < pass.pitch_min {
            pass.pitch_min = pitch;
        }
        if pitch > pass.pitch_max {
            pass.pitch_max = pitch;
        }

        if i >= first && (i - first) % stride == 0 {
            out.push(TracePoint {
                t_ms: sample.t_ms,
                pos: p,
                quat: filtered.state.attitude.map(|q| quantize(q, QUAT_SCALE)),
            });
        }
    }
    pass
}

/// Remove the height the rep did not end on by taking it out of the trace evenly from its start.
fn detrend_position(out: &mut TraceBuf<'_>, residual: f32, turns: RepTurns, pass: &mut Replay) {
    let span_ms = turns.end_ms.saturating_sub(turns.start_ms);
    if span_ms == 0 || out.is_empty() {
        return;
    }
    let mut lo = f32::MAX;
    let mut hi = f32::MIN;
    let mut seen = false;
    let len = out.len();
    for point in out.points[..len].iter_mut() {
        if point.t_ms <= turns.end_ms {
            let k =
                (point.t_ms.saturating_sub(turns.start_ms) as f32 / span_ms as f32).clamp(0.0, 1.0);
            point.pos[2] -= residual * k;
            if point.t_ms >= turns.start_ms {
                lo = lo.min(point.pos[2]);
                hi = hi.max(point.pos[2]);
                seen = true;
            }
        } else {
            point.pos[2] -= residual;
        }
    }
    if seen {
        pass.lo[2] = lo;
        pass.hi[2] = hi;
    }
}

/// Replay a window: anchor its turns, take the residual velocity and height out, and decimate it into `out`.
pub fn compute_rep(
    samples: &[Sample],
    from: &RepSeed,
    turns: Option<RepTurns>,
    out: &mut TraceBuf<'_>,
) -> msg::RepSummary {
    out.clear();
    let n = samples.len();
    if n == 0 {
        return msg::RepSummary {
            roll_min_deg: 0.0,
            roll_max_deg: 0.0,
            pitch_min_deg: 0.0,
            pitch_max_deg: 0.0,
            ..Default::default()
        };
    }

    let shift = turns.map(|_| {
        let mut no_points: [TracePoint; 0] = [];
        replay(
            samples,
            from,
            turns,
            &mut TraceBuf::new(&mut no_points),
            None,
        )
        .v_end
    });
    let mut pass = replay(samples, from, turns, out, shift);
    if let Some(turns) = turns {
        // A rep begins and ends at a turn, where the bar is still, so it began and ended at the same height: whatever height the trace has left is drift to take out.
        detrend_position(out, pass.p_end[2], turns, &mut pass);
    }

    let mut summary = msg::RepSummary {
        roll_min_deg: pass.roll_min,
        roll_max_deg: pass.roll_max,
        pitch_min_deg: pass.pitch_min,
        pitch_max_deg: pass.pitch_max,
        rom: if pass.hi[2] >= pass.lo[2] {
            pass.hi[2] - pass.lo[2]
        } else {
            0.0
        },
        t_start_ms: samples[0].t_ms,
        t_end_ms: samples[n - 1].t_ms,
        duration_ms: samples[n - 1].t_ms - samples[0].t_ms,
        peak_velocity: pass.v_max,
        computed_len: out.len() as u32,
        ..Default::default()
    };
    summary.set_state(msg::RepState {
        vx: from.velocity[0],
        vy: from.velocity[1],
        vz: from.velocity[2],
        qw: from.filter.attitude[0],
        qx: from.filter.attitude[1],
        qy: from.filter.attitude[2],
        qz: from.filter.attitude[3],
        accel_bias_x: from.filter.accel_bias[0],
        accel_bias_y: from.filter.accel_bias[1],
        accel_bias_z: from.filter.accel_bias[2],
        gyro_bias_x: from.filter.gyro_bias[0],
        gyro_bias_y: from.filter.gyro_bias[1],
        gyro_bias_z: from.filter.gyro_bias[2],
        settled: from.filter.resting,
    });
    summary
}
