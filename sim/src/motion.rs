//! Synthetic motion: a rep, or motion that is not a rep, at the device's sample rate.
//!
//! The generators are code, not files: a scenario is built where it is needed, so a tuning change is measured against the same motion instead of a stale `.pblog`. Everything is replayed through the real pipeline (`Fusion`, `Detector`, `compute_rep`) inside [`detect_and_trace`], so a scenario is the summary the device would have produced.
//!
//! The one physical fact behind all of it: a stroke is half a cosine at its peak speed, so its peak acceleration is `2v²/A` and its duration `πA/(2v)`, and its acceleration integral is zero — the bar starts and ends the stroke at rest, which is what ZUPT anchoring assumes.
//!
//! A generator states the bar's own acceleration; [`at`] adds gravity and hands back what a sensor bolted level to the bar would measure. Everything below therefore reads as motion, and only [`Impairments`] speaks in sensor readings.

use calypso_core::motion::fusion::{conjugate, quat_from_rotvec, rotate_by_quat};
use calypso_core::pb::msg;
use calypso_core::sampler::{quantize, Sample, ACCEL_SCALE, GYRO_SCALE};

use crate::fixtures::{at, detect_and_trace, LEAD_IN_MS, SAMPLE_DT_MS};

/// Rest appended after a motion, because a recording does not end where the bar does: the last stroke needs its quiet to be read as a turn, and the device is still sampling while the bar is put down.
const LEAD_OUT_MS: u32 = 1000;

/// The trace capacity a scenario replays through, which is the plotted budget a device stores.
pub use calypso_core::motion::trace::TRACE_POINTS;

const PI: f32 = core::f32::consts::PI;

/// `ms` milliseconds of rest, appended after `from_ms`. Returns the next timestamp.
fn push_rest(out: &mut Vec<Sample>, from_ms: u32, ms: u32) -> u32 {
    let mut t_ms = from_ms;
    for _ in 0..(ms / SAMPLE_DT_MS) {
        out.push(at(t_ms, [0.0, 0.0, 0.0]));
        t_ms += SAMPLE_DT_MS;
    }
    t_ms
}

/// One half-cosine stroke moving `travel` metres at a peak `speed`, `dir` being `1` for the drive and `-1` for the return. The acceleration peaks at `2·speed²/travel` and its integral over the stroke is zero, so the velocity returns to exactly zero at the far end. Returns the timestamp after the stroke.
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

/// A press: a drive at `speed_in`, a pause at the top, a return at `speed_out`, a pause at the bottom. The drive and the return are the same half-cosine, mirrored.
#[derive(Debug, Clone, Copy)]
pub struct Rep {
    pub travel: f32,
    pub speed_in: f32,
    pub speed_out: f32,
    pub pause_top_ms: u32,
    pub pause_bottom_ms: u32,
    /// How far the bar settles back at the top before the return, and the peak speed it does it at.
    ///
    /// A bar does not stop where it is caught: at lockout it gives a few centimetres back and comes again, which is motion in the opposite direction to the drive and short of half a rep.
    pub settle_m: f32,
    pub settle_speed: f32,
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
            settle_m: 0.0,
            settle_speed: 0.0,
        }
    }

    pub const fn deadlift() -> Self {
        Rep {
            travel: 0.55,
            speed_in: 1.00,
            speed_out: 0.70,
            pause_top_ms: 150,
            pause_bottom_ms: 300,
            settle_m: 0.0,
            settle_speed: 0.0,
        }
    }

    /// A rep whose drive and return are one cosine of `travel` in `duration_ms`: the shape the synthetic sets and the app's fixtures are made of.
    pub fn cosine(travel: f32, duration_ms: u32) -> Self {
        let speed = core::f32::consts::PI * travel / (duration_ms as f32 / 1000.0);
        Rep {
            travel,
            speed_in: speed,
            speed_out: speed,
            pause_top_ms: 0,
            pause_bottom_ms: 0,
            settle_m: 0.0,
            settle_speed: 0.0,
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
            settle_m: 0.0,
            settle_speed: 0.0,
        }
    }

    /// The rep's motion only — drive, top pause, return, bottom pause — appended after `from_ms`, leaving the lead-in rest to the caller. Returns the timestamp after the rep. A set builds several of these end to end with a single lead-in; [`Rep::samples`] is this plus that.
    pub fn strokes(&self, out: &mut Vec<Sample>, from_ms: u32) -> u32 {
        let mut t = stroke(out, from_ms, self.travel, self.speed_in, 1.0);
        t = push_rest(out, t, self.pause_top_ms);
        if self.settle_m > 0.0 {
            t = stroke(out, t, self.settle_m, self.settle_speed, -1.0);
            t = stroke(out, t, self.settle_m, self.settle_speed, 1.0);
        }
        t = stroke(out, t, self.travel, self.speed_out, -1.0);
        push_rest(out, t, self.pause_bottom_ms)
    }

    pub fn samples(&self) -> Vec<Sample> {
        let mut out = Vec::new();
        let t = push_rest(&mut out, 0, LEAD_IN_MS);
        let t = self.strokes(&mut out, t);
        push_rest(&mut out, t, LEAD_OUT_MS);
        out
    }
}

/// A one-way push that never comes back down: the unrack, a re-rack, a bump.
#[derive(Debug, Clone, Copy)]
pub struct Push {
    pub travel: f32,
    pub speed: f32,
    pub hold_ms: u32,
}

impl Push {
    /// The push's motion only, appended after `from_ms`: one stroke, then the hold at the top.
    /// Returns the timestamp after the hold.
    pub fn strokes(&self, out: &mut Vec<Sample>, from_ms: u32) -> u32 {
        let t = stroke(out, from_ms, self.travel, self.speed, 1.0);
        push_rest(out, t, self.hold_ms)
    }

    pub fn samples(&self) -> Vec<Sample> {
        let mut out = Vec::new();
        let t = push_rest(&mut out, 0, LEAD_IN_MS);
        self.strokes(&mut out, t);
        out
    }
}

/// A rep whose drive stalls partway: `first` out, a hold, `second` more out, then the whole way back. What a grind looks like to the sensor — the bar stops without coming down, and a detector that only knows "the bar is moving" sees two strokes where the lifter made one.
#[derive(Debug, Clone, Copy)]
pub struct Stall {
    pub first: f32,
    pub second: f32,
    /// Peak speed of each drive half and of the return.
    pub speed_in: f32,
    pub speed_out: f32,
    pub stall_ms: u32,
    /// The pause at the bottom, so a set can chain these the way [`Rep`] does.
    pub pause_bottom_ms: u32,
}

impl Stall {
    /// The shape the turning-point detector was measured on: a bench's travel driven in two halves.
    pub const fn bench() -> Self {
        Stall {
            first: 0.10,
            second: 0.09,
            speed_in: 0.55,
            speed_out: 0.35,
            stall_ms: 150,
            pause_bottom_ms: 300,
        }
    }

    /// The rep's motion only, appended after `from_ms`. Returns the timestamp after it.
    pub fn strokes(&self, out: &mut Vec<Sample>, from_ms: u32) -> u32 {
        let mut t = stroke(out, from_ms, self.first, self.speed_in, 1.0);
        t = push_rest(out, t, self.stall_ms);
        t = stroke(out, t, self.second, self.speed_in, 1.0);
        t = stroke(out, t, self.first + self.second, self.speed_out, -1.0);
        push_rest(out, t, self.pause_bottom_ms)
    }

    pub fn samples(&self) -> Vec<Sample> {
        let mut out = Vec::new();
        let t = push_rest(&mut out, 0, LEAD_IN_MS);
        let t = self.strokes(&mut out, t);
        push_rest(&mut out, t, LEAD_OUT_MS);
        out
    }
}

/// Motion that is, or is not, a rep.
#[derive(Debug, Clone, Copy)]
pub enum Motion {
    Rep(Rep),
    /// The same rep, ground out in two halves.
    Stall(Stall),
    /// The unrack, a re-rack, a bump: one push that never comes back down.
    Push(Push),
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
            Motion::Stall(s) => s.samples(),
            Motion::Push(p) => p.samples(),
            Motion::Wobble { travel, hz, ms } => {
                // z = (travel/2)·(1 − cos(2πft)), so accel = +(travel/2)·(2πf)²·cos(2πft): an oscillation of full swing `travel` that starts at rest and ends where it began.
                // The cosine phase is what matters: a sine at phase zero already moves at `amp·ω` the moment it starts, and integrating that from rest leaves a velocity that never sits still, so the displacement drifts instead of oscillating about a centre.
                let amp = travel / 2.0;
                let peak = amp * (2.0 * PI * hz) * (2.0 * PI * hz);
                let mut out = Vec::new();
                let mut t = push_rest(&mut out, 0, LEAD_IN_MS);
                for i in 0..(ms / SAMPLE_DT_MS) {
                    let tau = i as f32 * SAMPLE_DT_MS as f32 / 1000.0;
                    let accel = peak * libm::cosf(2.0 * PI * hz * tau);
                    out.push(at(t, [0.0, 0.0, accel]));
                    t += SAMPLE_DT_MS;
                }
                push_rest(&mut out, t, LEAD_OUT_MS);
                out
            }
        }
    }

    /// The vertical excursion the motion asks for, as the map's truth column.
    pub fn travel(&self) -> f32 {
        match *self {
            Motion::Rep(r) => r.travel,
            Motion::Stall(s) => s.first + s.second,
            Motion::Push(p) => p.travel,
            Motion::Wobble { travel, .. } => travel,
        }
    }
}

/// A set: the unrack, several reps separated by the set's rest, and the re-rack, assembled into one timestamp-continuous stream. This is the thing the device actually records, so it is the only place one rep's motion can merge with, or hide, its neighbours.
#[derive(Debug, Clone, Copy)]
pub struct Set {
    /// The per-rep template (travel, speeds, top pause).
    pub rep: Rep,
    /// How many reps in the set.
    pub reps: u32,
    /// The rest between reps, applied as each rep's bottom pause; it overrides the template's `pause_bottom_ms`.
    pub inter_rep_rest_ms: u32,
    /// The bar lifted off the rack, before the first rep.
    pub unrack: Option<Push>,
    /// The bar put back on the rack, after the last rep.
    pub rerack: Option<Push>,
}

impl Set {
    pub fn samples(&self) -> Vec<Sample> {
        let mut out = Vec::new();
        let mut t = push_rest(&mut out, 0, LEAD_IN_MS);
        if let Some(p) = self.unrack {
            t = p.strokes(&mut out, t);
        }
        let mut rep = self.rep;
        rep.pause_bottom_ms = self.inter_rep_rest_ms;
        for _ in 0..self.reps {
            t = rep.strokes(&mut out, t);
        }
        if let Some(p) = self.rerack {
            t = p.strokes(&mut out, t);
        }
        push_rest(&mut out, t, LEAD_OUT_MS);
        out
    }
}

/// The canonical sets, the ones that ask different things of the detector: rest between reps, no rest, a push either side of a rep, and a push on its own.
pub fn probe_sets() -> Vec<(&'static str, Set)> {
    let push = Push {
        travel: 0.06,
        speed: 0.8,
        hold_ms: 500,
    };

    let mut no_rest = Rep::bench();
    no_rest.pause_top_ms = 0;

    vec![
        (
            "bench x5, 3 s rest",
            Set {
                rep: Rep::bench(),
                reps: 5,
                inter_rep_rest_ms: 3000,
                unrack: None,
                rerack: None,
            },
        ),
        (
            "bench x5, no rest",
            Set {
                rep: no_rest,
                reps: 5,
                inter_rep_rest_ms: 0,
                unrack: None,
                rerack: None,
            },
        ),
        (
            "unrack + bench x1 + rerack (1 s)",
            Set {
                rep: Rep::bench(),
                reps: 1,
                inter_rep_rest_ms: 1000,
                unrack: Some(push),
                rerack: Some(push),
            },
        ),
        (
            "unrack only",
            Set {
                rep: Rep::bench(),
                reps: 0,
                inter_rep_rest_ms: 0,
                unrack: Some(push),
                rerack: None,
            },
        ),
    ]
}

/// A shortcut: [`detect_and_trace`] a set's samples and keep only the summaries that clear [`MIN_ROM_M`], the same "would the device store it" question [`stored`] asks of a single motion.
pub fn stored_set(set: &Set) -> Vec<msg::RepSummary> {
    detect_and_trace(&set.samples(), TRACE_POINTS)
        .into_iter()
        .map(|(summary, _)| summary)
        .filter(|s| s.is_a_rep())
        .collect()
}

/// The summaries the device would store for this motion — the ones that clear [`MIN_ROM_M`].
///
/// The detector closes a span for a wobble or a bump as readily as for a press; what it cannot see is how far anything moved. `stored` is the answer after that check, so a cell is one bit: did this motion produce a rep, or not.
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

/// The ways a sensor reading is not the motion, added to synthetic motion — never to a captured rep, which already carries the device's own error. Every field is zero by default (a no-op), so a loss can be attributed to the one error that was turned up.
///
/// These are the sensor's faults, not the filter's: the mount is real and the offsets are real, so a scenario built from them is motion the device could actually have seen. What is *not* injectable any more is a wrong attitude — there is no estimate here to bias.
#[derive(Debug, Clone, Copy)]
pub struct Impairments {
    /// Per-sample white noise on the accelerometer (m/s²).
    pub noise_sigma: f32,
    /// The accelerometer's offset, in the sensor's own frame (m/s²).
    pub accel_bias: [f32; 3],
    /// The gyroscope's offset, in the sensor's own frame (degrees per second).
    pub gyro_bias_dps: [f32; 3],
    /// The angle the sensor is bolted to the bar at, as a rotation vector in degrees `(x, y, z)`.
    ///
    /// A real mount is never square to the bar. It rotates every reading into another frame, and the filter is supposed to be indifferent to it: what a tilt costs is a different attitude, not a different travel.
    pub tilt_deg: [f32; 3],
}

impl Default for Impairments {
    fn default() -> Self {
        Impairments {
            noise_sigma: 0.0,
            accel_bias: [0.0; 3],
            gyro_bias_dps: [0.0; 3],
            tilt_deg: [0.0; 3],
        }
    }
}

impl Impairments {
    /// Add the impairments to generated samples, in place.
    ///
    /// The generators emit specific force in the world frame with the sensor level, so this is where a sample becomes a *sensor* reading: rotated into the mount's frame, then given the accelerometer's offset and the platform noise.
    pub fn apply(&self, samples: &mut [Sample], rng: &mut Rng) {
        let mount = quat_from_rotvec(self.tilt_deg.map(|d| d.to_radians()));
        for s in samples.iter_mut() {
            let body = rotate_by_quat(conjugate(mount), s.specific_force_f32());
            s.accel = [0, 1, 2].map(|k| {
                quantize(
                    body[k] + self.accel_bias[k] + rng.gauss() * self.noise_sigma,
                    ACCEL_SCALE,
                )
            });
            s.gyro = self.gyro_bias_dps.map(|d| quantize(d, GYRO_SCALE));
        }
    }
}
