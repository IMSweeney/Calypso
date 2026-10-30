//! Synthetic motion: a rep, or motion that is not a rep, at the device's sample rate.
//!
//! The generators are code, not files: a scenario is built where it is needed, so a tuning
//! change is measured against the same motion instead of a stale `.pblog`. Everything is
//! replayed through the real pipeline (`Detector`, `compute_rep`) inside [`detect_and_trace`],
//! so a scenario is the summary the device would have produced.
//!
//! The one physical fact behind all of it: a stroke is half a cosine at its peak speed, so
//! its peak acceleration is `2v²/A` and its duration `πA/(2v)`, and its acceleration integral
//! is zero — the bar starts and ends the stroke at rest, which is what ZUPT anchoring assumes.

use calypso_core::pb::msg;
use calypso_core::sampler::{quantize, Sample, ACCEL_SCALE, QUAT_SCALE};

use crate::fixtures::{detect_and_trace, SAMPLE_DT_MS};

/// The trace capacity a scenario replays through, matching a device's stored trace.
pub const TRACE_POINTS: usize = 512;
/// Milliseconds still before the motion begins, so the detector starts settled.
const LEAD_IN_MS: u32 = 300;

const PI: f32 = core::f32::consts::PI;

/// A sample with the identity orientation: synthetic motion is level by construction.
fn at(t_ms: u32, accel: [f32; 3]) -> Sample {
    Sample {
        t_ms,
        accel: accel.map(|a| quantize(a, ACCEL_SCALE)),
        quat: [QUAT_SCALE as i16, 0, 0, 0],
    }
}

/// `ms` milliseconds of rest, appended after `from_ms`. Returns the next timestamp.
fn push_rest(out: &mut Vec<Sample>, from_ms: u32, ms: u32) -> u32 {
    let mut t_ms = from_ms;
    for _ in 0..(ms / SAMPLE_DT_MS) {
        out.push(at(t_ms, [0.0, 0.0, 0.0]));
        t_ms += SAMPLE_DT_MS;
    }
    t_ms
}

/// One half-cosine stroke moving `travel` metres at a peak `speed`, `dir` being `1` for the
/// drive and `-1` for the return. The acceleration peaks at `2·speed²/travel` and its integral
/// over the stroke is zero, so the velocity returns to exactly zero at the far end. Returns
/// the timestamp after the stroke.
fn stroke(out: &mut Vec<Sample>, from_ms: u32, travel: f32, speed: f32, dir: f32) -> u32 {
    let dt = SAMPLE_DT_MS as f32 / 1000.0;
    let duration = PI * travel / (2.0 * speed);
    let steps = ((duration / dt).round() as u32).max(1);
    let t = steps as f32 * dt;
    let peak = 2.0 * speed * speed / travel;
    let mut t_ms = from_ms;
    for i in 0..=steps {
        let tau = i as f32 * dt;
        let accel = dir * peak * libm::cosf(PI * tau / t);
        out.push(at(t_ms, [0.0, 0.0, accel]));
        t_ms += SAMPLE_DT_MS;
    }
    t_ms
}

/// A press: a drive at `speed_in`, a pause at the top, a return at `speed_out`, a pause at the
/// bottom. The drive and the return are the same half-cosine, mirrored.
#[derive(Debug, Clone, Copy)]
pub struct Rep {
    pub travel: f32,
    pub speed_in: f32,
    pub speed_out: f32,
    pub pause_top_ms: u32,
    pub pause_bottom_ms: u32,
}

impl Rep {
    /// The bench nominal: each such scenario is exactly one stored rep.
    pub const fn bench() -> Self {
        Rep {
            travel: 0.19,
            speed_in: 0.55,
            speed_out: 0.35,
            pause_top_ms: 150,
            pause_bottom_ms: 300,
        }
    }

    pub const fn deadlift() -> Self {
        Rep {
            travel: 0.55,
            speed_in: 1.00,
            speed_out: 0.70,
            pause_top_ms: 150,
            pause_bottom_ms: 300,
        }
    }

    /// Provisional: no squat rep has ever been captured.
    pub const fn squat() -> Self {
        Rep {
            travel: 0.70,
            speed_in: 1.10,
            speed_out: 0.80,
            pause_top_ms: 150,
            pause_bottom_ms: 300,
        }
    }

    pub fn samples(&self) -> Vec<Sample> {
        let mut out = Vec::new();
        let mut t = push_rest(&mut out, 0, LEAD_IN_MS);
        t = stroke(&mut out, t, self.travel, self.speed_in, 1.0);
        t = push_rest(&mut out, t, self.pause_top_ms);
        t = stroke(&mut out, t, self.travel, self.speed_out, -1.0);
        push_rest(&mut out, t, self.pause_bottom_ms);
        out
    }
}

/// Motion that is, or is not, a rep.
#[derive(Debug, Clone, Copy)]
pub enum Motion {
    Rep(Rep),
    /// The unrack, a re-rack, a bump: one push that never comes back down.
    Push {
        travel: f32,
        speed: f32,
        hold_ms: u32,
    },
    /// Oscillation with no net travel: a hand on the rack.
    Wobble {
        travel: f32,
        hz: f32,
        ms: u32,
    },
}

impl Motion {
    pub fn samples(&self) -> Vec<Sample> {
        match *self {
            Motion::Rep(r) => r.samples(),
            Motion::Push {
                travel,
                speed,
                hold_ms,
            } => {
                let mut out = Vec::new();
                let mut t = push_rest(&mut out, 0, LEAD_IN_MS);
                t = stroke(&mut out, t, travel, speed, 1.0);
                push_rest(&mut out, t, hold_ms);
                out
            }
            Motion::Wobble { travel, hz, ms } => {
                // z = (travel/2)·sin(2πft), so accel = -(travel/2)·(2πf)²·sin(2πft): oscillation
                // of full swing `travel` with no net displacement.
                let amp = travel / 2.0;
                let peak = amp * (2.0 * PI * hz) * (2.0 * PI * hz);
                let mut out = Vec::new();
                let mut t = push_rest(&mut out, 0, LEAD_IN_MS);
                for i in 0..(ms / SAMPLE_DT_MS) {
                    let tau = i as f32 * SAMPLE_DT_MS as f32 / 1000.0;
                    let accel = -peak * libm::sinf(2.0 * PI * hz * tau);
                    out.push(at(t, [0.0, 0.0, accel]));
                    t += SAMPLE_DT_MS;
                }
                out
            }
        }
    }

    /// The vertical excursion the motion asks for, as the map's truth column.
    pub fn travel(&self) -> f32 {
        match *self {
            Motion::Rep(r) => r.travel,
            Motion::Push { travel, .. } | Motion::Wobble { travel, .. } => travel,
        }
    }
}

/// The summaries the device would store for this motion — the ones that clear [`MIN_ROM_M`].
///
/// The detector closes a span for a wobble or a bump as readily as for a press; what it cannot
/// see is how far anything moved. `stored` is the answer after that check, so a cell is one
/// bit: did this motion produce a rep, or not.
pub fn stored(motion: &Motion) -> Vec<msg::RepSummary> {
    detect_and_trace(&motion.samples(), TRACE_POINTS)
        .into_iter()
        .map(|(summary, _)| summary)
        .filter(|s| s.is_a_rep())
        .collect()
}

/// A deterministic generator for the noise map: the same seed draws the same motion.
#[derive(Debug, Clone, Copy)]
pub struct Rng(u64);

impl Rng {
    pub const fn new(seed: u64) -> Self {
        Rng(seed ^ 0x9e37_79b9_7f4a_7c15)
    }

    fn next_u64(&mut self) -> u64 {
        let mut z = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        self.0 = z;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    fn uniform(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }

    /// A standard normal, Box–Muller from two uniforms.
    pub fn gauss(&mut self) -> f32 {
        let u1 = self.uniform().max(f32::EPSILON);
        let u2 = self.uniform();
        libm::sqrtf(-2.0 * libm::logf(u1)) * libm::cosf(2.0 * PI * u2)
    }
}

/// The impairments the capture showed, added to synthetic motion — never to a captured rep,
/// which already carries the device's own error. Every field is zero by default (a no-op), so
/// a loss can be attributed to the one error that was turned up.
#[derive(Debug, Clone, Copy)]
pub struct Impairments {
    /// Per-sample white acceleration noise (m/s²).
    pub noise_sigma: f32,
    /// Bias that ramps through the rep (m/s² per second).
    pub bias_drift: f32,
    /// Attitude offset the fusion settles on (degrees).
    pub attitude_offset_deg: f32,
    /// Attitude drift while the bar moves (degrees per second).
    pub attitude_drift_dps: f32,
    /// A constant body bias (m/s²), removed exactly by the velocity detrend.
    pub constant_bias: f32,
}

impl Default for Impairments {
    fn default() -> Self {
        Impairments {
            noise_sigma: 0.0,
            bias_drift: 0.0,
            attitude_offset_deg: 0.0,
            attitude_drift_dps: 0.0,
            constant_bias: 0.0,
        }
    }
}

impl Impairments {
    /// Add the impairments to generated samples, in place.
    ///
    /// The tilt turns the bar's own vertical acceleration `a` into `a·sin θ` of sideways span
    /// and `a·cos θ` of vertical — the ROM that survives to the summary is the bar's travel
    /// scaled by `cos θ`. The constant bias is left to the detrend, which removes it exactly.
    pub fn apply(&self, samples: &mut [Sample], rng: &mut Rng) {
        let t0 = samples.first().map(|s| s.t_ms).unwrap_or(0);
        for s in samples.iter_mut() {
            let az = s.accel_f32()[2];
            let t_s = (s.t_ms - t0) as f32 / 1000.0;
            let theta = (self.attitude_offset_deg + self.attitude_drift_dps * t_s).to_radians();
            let ax = rng.gauss() * self.noise_sigma;
            let ay = az * libm::sinf(theta) + rng.gauss() * self.noise_sigma;
            let az = az * libm::cosf(theta)
                + self.constant_bias
                + self.bias_drift * t_s
                + rng.gauss() * self.noise_sigma;
            s.accel = [
                quantize(ax, ACCEL_SCALE),
                quantize(ay, ACCEL_SCALE),
                quantize(az, ACCEL_SCALE),
            ];
        }
    }
}
