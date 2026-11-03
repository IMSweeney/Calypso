//! The attitude filter: the sensor's accelerometer and gyroscope in, a linear acceleration and an
//! attitude out.
//!
//! The accelerometer measures **specific force** — gravity included — and the gyroscope measures
//! rotation. Neither is an estimate, so there is nothing here that can be quietly wrong about the
//! frame: the attitude is integrated from the gyro and pulled toward the direction gravity actually
//! points whenever the bar is still, and the specific force minus the gravity the attitude implies
//! is the bar's acceleration.
//!
//! **Why it is not the chip's job.** A fused orientation hands back "linear acceleration", which is
//! gravity removed *by definition*: at rest it reads zero whatever the truth is. That leaves no way
//! to ask whether the reading is right, and a residue the fusion left behind is then indistinguishable
//! from the bar moving. Raw, a still sensor reads 1 g, and that is a physical fact an estimate cannot
//! drift out of.
//!
//! **Two questions, and they are not the same one.** "Has the reading stopped changing" is what
//! the offsets are learned from, because an offset is constant and so cannot move a window's width.
//! "Is the bar at rest" is what the attitude, the velocity and everything downstream hang off, and
//! it is a stricter thing: the reading has to be 1 g, and it has to have been 1 g for long enough
//! that a slow stroke cannot be what is being seen. A lift guarantees it at both ends of every
//! stroke, which is what bounds the drift to within one rep.
//!
//! **Two things are unobservable and are left alone.** Yaw, because there is no magnetometer — a
//! gyro bias leaves a slow heading drift, which is the mount's frame and not the room's. And an
//! accelerometer offset is indistinguishable from a tilt, so an offset of `b` leaves the vertical
//! off by about `b/g` radians; at a fraction of a metre per second squared that is a couple of
//! degrees, and it cancels out of the linear acceleration exactly.

use crate::sampler::Sample;

/// Standard gravity (m/s²): the specific force a sensor at rest reads up the world's up axis.
pub const GRAVITY: f32 = 9.80665;

/// Samples in the stillness window: 400 ms at the device's 100 Hz.
const STILL_WINDOW: usize = 40;
/// Per-axis spread of the specific force (m/s²) under which the reading has stopped changing.
///
/// A **spread**, not a magnitude, because it is the one test a sensor's own offset cannot fool: an
/// offset is constant, so it moves the window's centre and not its width. A still bar varies by a
/// hundredth of a metre per second squared; one being lifted varies by whole ones.
const STILL_SPREAD: f32 = 0.15;
/// How far the specific force may sit from 1 g, with the offset taken out, while the bar rests
/// (m/s²).
///
/// This is the term that tells a slow stroke from a still bar. A long gentle lift changes the
/// sensor's reading by a fraction of a metre per second squared across 400 ms, which a spread
/// cannot see, but it changes its *magnitude*, which this can. A tenth of a metre per second
/// squared is where the detector has always drawn this line, and the map agrees it is the right
/// place: below it, a stroke is slower than any rep is made of.
const STILL_GRAVITY: f32 = 0.12;
/// Angular rate (rad/s) under which the sensor is not turning: 2 dps.
const STILL_RATE_DPS: f32 = 2.0;
/// The same while the gyroscope's offset is still unknown, in dps.
///
/// A gyroscope that is resting reads its own offset, so the tight bound above cannot be the test
/// that decides whether it has stopped: a sensor three degrees per second out would never read
/// quiet, and would therefore never learn the three degrees per second. Wide enough to admit any
/// offset a gyroscope ships with, narrow enough that a bar being swung through a stroke is not it.
const STILL_RATE_BOOT_DPS: f32 = 5.0;
/// How long the reading must be quiet before the attitude is taken to know which way is down.
///
/// A second, because that is what it takes for the accelerometer's offset to come out of the same
/// window: an offset left half-learned is a metre per second squared of phantom acceleration
/// waiting at the first rep.
const READY_S: f32 = 1.0;
/// How long the reading must hold at 1 g before the bar counts as **at rest**.
///
/// Not the same question as quiet. A slow stroke spends a third of a second crossing zero
/// acceleration at its fastest point, where it reads 1 g and barely changes — local measures cannot
/// tell it from a bar on the rack, and the only thing that can is that a rest goes on. Long enough
/// to see past that shoulder, short enough that a bar set down is at rest before the lifter has
/// gripped it again.
const SETTLE_S: f32 = 0.4;
/// Time constant of the attitude's pull toward measured gravity, once it knows which way is down.
const ATT_TAU_S: f32 = 1.0;
/// Time constant of both offsets while the filter is still booting.
///
/// Short, because slow is not free: an offset still half learned when the first rep starts is an
/// acceleration that was never there, integrated for the length of the rep. A tenth of a degree per
/// second left in the gyro is a degree of tilt over a rep, which is a fifth of a metre per second
/// squared of gravity in the horizontal, which is more than the speed a turn is closed on.
const BOOT_TAU_S: f32 = 0.25;
/// The same once the offsets are known, and much slower: temperature moves a sensor's offset over
/// minutes, while a slow stroke's shoulder looks like a constant reading for a third of a second.
/// The second is the one to filter out, and it is the reason this is not the fast one.
const TRACK_TAU_S: f32 = 2.0;
/// Time constant over which velocity is pulled back to zero while the bar is still — the velocity
/// form of "it is not moving, so it is not going anywhere".
const V_TAU_S: f32 = 0.5;
/// Longest integration step honoured, so a hole in the samples cannot become a jump in velocity.
const MAX_DT_S: f32 = 0.1;

const RAD_PER_DPS: f32 = core::f32::consts::PI / 180.0;

/// Everything the filter remembers, which is also everything a replay needs to start where it did.
///
/// The filter is deterministic from this state and the samples that follow it, so a trace plus this
/// is the summary it produced.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FusionState {
    /// World-frame velocity (m/s).
    pub velocity: [f32; 3],
    /// Attitude, body to world, as `[w, x, y, z]`.
    pub attitude: [f32; 4],
    /// The accelerometer's offset in the sensor's own frame (m/s²).
    pub accel_bias: [f32; 3],
    /// The gyroscope's offset in the sensor's own frame (rad/s).
    pub gyro_bias: [f32; 3],
    /// Whether the bar was already at rest when this was taken.
    ///
    /// Part of the state, not a convenience: "at rest" is a fact about the last 400 ms, so a filter
    /// started without it has to earn it again and spends its first samples integrating a drift the
    /// filter it replaced had already taken out.
    pub settled: bool,
}

impl FusionState {
    /// Level, still, and nothing learned: where a boot starts.
    pub const LEVEL: FusionState = FusionState {
        velocity: [0.0; 3],
        attitude: [1.0, 0.0, 0.0, 0.0],
        accel_bias: [0.0; 3],
        gyro_bias: [0.0; 3],
        settled: false,
    };
}

impl Default for FusionState {
    fn default() -> Self {
        Self::LEVEL
    }
}

/// What the filter says about one sample.
#[derive(Debug, Clone, Copy)]
pub struct Filtered {
    /// The sample's own timestamp, so whatever consumes this never needs the raw sample back.
    pub t_ms: u32,
    /// Specific force with gravity and the sensor's offset taken out, in the world frame (m/s²).
    pub linear_accel: [f32; 3],
    /// Whether the bar has been still across the last window.
    pub still: bool,
    /// Where the filter stands after this sample.
    pub state: FusionState,
}

/// The filter. One of these runs for as long as the device is up, and another replays a stored
/// window from its recorded state.
#[derive(Debug, Clone)]
pub struct Fusion {
    state: FusionState,
    /// `None` until the first sample, because the step into the first one is not a step.
    prev_t_ms: Option<u32>,
    /// The specific force with the offset taken out, over the stillness window.
    still_buf: [[f32; 3]; STILL_WINDOW],
    still_head: usize,
    still_len: usize,
    /// Seconds of unbroken quiet since the attitude could last have been aligned.
    still_s: f32,
    /// Seconds the reading has held at 1 g, which is what makes the bar at rest rather than merely
    /// slow.
    settled_s: f32,
    /// Whether the attitude has been aligned to gravity at least once. Sticky: once gravity is
    /// known it stays known, and a boot's worth of motion before it is not recordable.
    ready: bool,
}

impl Fusion {
    /// A filter at a boot: level, still, nothing learned, and not ready until it has been still.
    pub fn new() -> Self {
        Fusion {
            state: FusionState::LEVEL,
            prev_t_ms: None,
            still_buf: [[0.0; 3]; STILL_WINDOW],
            still_head: 0,
            still_len: 0,
            still_s: 0.0,
            settled_s: 0.0,
            ready: false,
        }
    }

    /// A filter starting where another one was, which is how a stored window is replayed.
    ///
    /// The state it is given is taken as learned: a replay is not a boot, so it does not have to
    /// earn its attitude again.
    pub fn from_state(state: FusionState) -> Self {
        Fusion {
            state,
            ready: true,
            settled_s: if state.settled { SETTLE_S } else { 0.0 },
            ..Fusion::new()
        }
    }

    pub fn state(&self) -> FusionState {
        self.state
    }

    /// How long the bar has held still, in seconds: the number readiness is counted toward.
    ///
    /// For the log, so a device that never becomes ready can say whether it is being held or whether the sensor is
    /// giving it nothing it can call still.
    pub fn still_for_s(&self) -> f32 {
        self.still_s
    }

    /// Whether the attitude knows which way is down, which is the gate on recording anything.
    pub fn ready(&self) -> bool {
        self.ready
    }

    /// Feed one sample and say what it means.
    pub fn update(&mut self, s: &Sample) -> Filtered {
        let dt = match self.prev_t_ms {
            None => 0.0,
            Some(prev) => (s.t_ms.saturating_sub(prev) as f32 / 1000.0).min(MAX_DT_S),
        };
        self.prev_t_ms = Some(s.t_ms);

        let rate = sub(
            s.gyro_f32().map(|dps| dps * RAD_PER_DPS),
            self.state.gyro_bias,
        );
        let raw = s.specific_force_f32();

        // The gyro carries the attitude through a stroke, where the accelerometer is measuring the
        // stroke itself. Applied in the body frame: `R' = R · δ`.
        if dt > 0.0 {
            let turn = quat_from_rotvec(scale(rate, dt));
            self.state.attitude = normalize(quat_mul(self.state.attitude, turn));
        }

        let quiet = self.is_quiet(&raw, &rate);
        let f = sub(raw, self.state.accel_bias);
        // Nothing is left after gravity, which is the one thing an offset cannot fake once it has
        // been learned, and nothing is turning. That says "not accelerating", not "at rest": a
        // stroke passing its own peak speed reads the same. The wait below is what tells them apart.
        let resting = absf(magnitude(f) - GRAVITY) <= STILL_GRAVITY
            && magnitude(rate) < STILL_RATE_DPS * RAD_PER_DPS;
        self.settled_s = if resting { self.settled_s + dt } else { 0.0 };
        let still = resting && self.settled_s >= SETTLE_S;
        self.state.settled = still;

        // The attitude is only corrected from a bar known to be at rest — plus `quiet` at a boot,
        // where there is no offset to test a magnitude against yet. A stroke whose acceleration
        // barely changes is quiet for a third of a second, and correcting the attitude toward *its*
        // direction tilts the ground by the angle `a/g` — three degrees at half a metre per second
        // squared, every stroke, until the vertical is wherever the last rep left it.
        //
        // The offsets are a different question: a window where the reading is steady and nothing is
        // turning says what the sensor reads when it should read nothing, whatever else is going on.
        // They are learned from `quiet` always, slowly once they are known, so an offset that moves
        // with temperature is found again instead of leaving the bar permanently not-at-rest.
        let learn = quiet;
        let align = if self.ready { still } else { quiet };
        if learn {
            let tau = if self.ready { TRACK_TAU_S } else { BOOT_TAU_S };
            self.align_and_learn(&raw, &rate, dt, align, tau);
            self.still_s += dt;
            if self.still_s >= READY_S {
                self.ready = true;
            }
        } else {
            self.still_s = 0.0;
        }
        let f_world = rotate_by_quat(self.state.attitude, f);
        let linear_accel = [f_world[0], f_world[1], f_world[2] - GRAVITY];
        // Before gravity's direction is known there is no frame to integrate in, so there is no
        // velocity to report: a boot that starts moving is not recordable until the first rest.
        if dt > 0.0 && self.ready {
            for k in 0..3 {
                self.state.velocity[k] += linear_accel[k] * dt;
            }
            if still {
                for k in 0..3 {
                    self.state.velocity[k] += (0.0 - self.state.velocity[k]) * (dt / V_TAU_S);
                }
            }
        }

        Filtered {
            t_ms: s.t_ms,
            linear_accel,
            still,
            state: self.state,
        }
    }

    /// Whether the sensor's reading has stopped changing and it has stopped turning.
    ///
    /// **Quiet, not still**, and only asked before the filter knows which way is down. This is
    /// what the offsets and the attitude are bootstrapped from, so it cannot ask whether they are
    /// right: a still sensor carrying an offset reads something other than 1 g, and asking would
    /// refuse to be still until it had learned the very thing stillness is needed to learn. What is
    /// left is what an offset cannot fake — the reading's width over the window, and whether
    /// anything is turning.
    ///
    /// The width is of the **raw** reading for the same reason: measuring it after the offset is
    /// taken out would put the estimate inside its own input, and while that estimate was still
    /// moving a still sensor's corrected reading would move with it.
    fn is_quiet(&mut self, raw: &[f32; 3], rate: &[f32; 3]) -> bool {
        self.still_buf[self.still_head] = *raw;
        self.still_head = (self.still_head + 1) % STILL_WINDOW;
        self.still_len = (self.still_len + 1).min(STILL_WINDOW);
        if self.still_len < STILL_WINDOW || magnitude(*rate) >= STILL_RATE_BOOT_DPS * RAD_PER_DPS {
            return false;
        }
        for k in 0..3 {
            let (mut lo, mut hi) = (f32::MAX, f32::MIN);
            for v in &self.still_buf {
                if v[k] < lo {
                    lo = v[k];
                }
                if v[k] > hi {
                    hi = v[k];
                }
            }
            if hi - lo > STILL_SPREAD {
                return false;
            }
        }
        true
    }

    /// Put the attitude on the measured gravity and learn both offsets from what is left.
    ///
    /// The attitude follows the **raw** reading, so it points where the accelerometer says gravity
    /// is, and the offset is what gravity cannot explain from there. At one orientation a sideways
    /// offset and a tilt are the same measurement, so the sideways part stays in the attitude as a
    /// couple of degrees and cancels out of the acceleration — which is the honest outcome, rather
    /// than an estimate that would drift for as long as the bar is still.
    fn align_and_learn(&mut self, raw: &[f32; 3], rate: &[f32; 3], dt: f32, align: bool, tau: f32) {
        let up = rotate_by_quat(conjugate(self.state.attitude), [0.0, 0.0, 1.0]);
        let measured = unit(*raw);
        // The rotation that would turn what the sensor measured into what the attitude says is up.
        let err = cross(measured, up);

        if !self.ready {
            // There is no attitude worth protecting at a boot, so it is *set* rather than filtered:
            // one exact rotation onto the measured gravity. A filter would leave degrees of error,
            // and a degree of tilt is a sixth of a metre per second squared of gravity left in the
            // horizontal, integrated for the whole of the first rep.
            let angle = atan2f(magnitude(err), dot(measured, up));
            if angle > 1.0e-6 {
                // At a half turn the cross product vanishes and any axis across `up` is as good as
                // another: the sensor is upside down, and which way it faces after that is yaw.
                let axis = if magnitude(err) > 1.0e-3 {
                    unit(err)
                } else {
                    across(up)
                };
                let turn = quat_from_rotvec(scale(axis, angle));
                self.state.attitude = normalize(quat_mul(self.state.attitude, turn));
            }
        } else if align && dt > 0.0 {
            // Once it is set the gyro carries it, and the accelerometer only corrects what changes
            // slowly: an offset, or the platform itself.
            let turn = quat_from_rotvec(scale(err, dt / ATT_TAU_S));
            self.state.attitude = normalize(quat_mul(self.state.attitude, turn));
        }

        // Gravity's direction is the attitude's, now that it has been set: whatever specific force
        // is left over is the sensor's own offset, in its own frame.
        let up = rotate_by_quat(conjugate(self.state.attitude), [0.0, 0.0, 1.0]);
        let expected = scale(up, GRAVITY);
        if dt > 0.0 {
            for k in 0..3 {
                let offset = raw[k] - expected[k];
                self.state.accel_bias[k] += (offset - self.state.accel_bias[k]) * (dt / tau);
                self.state.gyro_bias[k] += rate[k] * (dt / tau);
            }
        }
    }
}

impl Default for Fusion {
    fn default() -> Self {
        Self::new()
    }
}

/// Length of a vector.
pub fn magnitude(v: [f32; 3]) -> f32 {
    sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

/// Rotate a body-frame vector into the world frame by a unit quaternion `[w, x, y, z]`: `v' = q ⊗ v ⊗ q*`.
pub fn rotate_by_quat(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    // t = 2 * (q.xyz × v)
    let tx = 2.0 * (y * v[2] - z * v[1]);
    let ty = 2.0 * (z * v[0] - x * v[2]);
    let tz = 2.0 * (x * v[1] - y * v[0]);
    // v + w*t + (q.xyz × t)
    [
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    ]
}

/// Roll and pitch (degrees) from a unit quaternion `[w, x, y, z]`.
/// Yaw is deliberately not reported: it drifts without a magnetometer.
pub fn roll_pitch_deg(q: [f32; 4]) -> (f32, f32) {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let roll = atan2f(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let pitch_arg = clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    let pitch = asinf(pitch_arg);
    (roll.to_degrees(), pitch.to_degrees())
}

/// The quaternion conjugate, which rotates the other way: for a unit quaternion, the inverse.
pub fn conjugate(q: [f32; 4]) -> [f32; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// `a ⊗ b`: the rotation `b` followed by the rotation `a`.
fn quat_mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// The quaternion of a rotation vector: its direction is the axis and its length the angle.
pub fn quat_from_rotvec(v: [f32; 3]) -> [f32; 4] {
    let angle = magnitude(v);
    // The first term of the series, because the axis is not defined at an angle of zero.
    if angle < 1.0e-6 {
        return [1.0, 0.5 * v[0], 0.5 * v[1], 0.5 * v[2]];
    }
    let s = sinf(0.5 * angle) / angle;
    [cosf(0.5 * angle), s * v[0], s * v[1], s * v[2]]
}

fn normalize(q: [f32; 4]) -> [f32; 4] {
    let n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if n == 0.0 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

fn unit(v: [f32; 3]) -> [f32; 3] {
    let n = magnitude(v);
    if n == 0.0 {
        return v;
    }
    scale(v, 1.0 / n)
}

/// Any unit vector at right angles to `v`.
fn across(v: [f32; 3]) -> [f32; 3] {
    let least = if absf(v[0]) <= absf(v[1]) && absf(v[0]) <= absf(v[2]) {
        [1.0, 0.0, 0.0]
    } else if absf(v[1]) <= absf(v[2]) {
        [0.0, 1.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    unit(cross(v, least))
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(v: [f32; 3], k: f32) -> [f32; 3] {
    [v[0] * k, v[1] * k, v[2] * k]
}

// --- small no_std helpers -------------------------------------------------

fn sqrtf(x: f32) -> f32 {
    libm::sqrtf(x)
}

fn sinf(x: f32) -> f32 {
    libm::sinf(x)
}

fn cosf(x: f32) -> f32 {
    libm::cosf(x)
}

fn atan2f(y: f32, x: f32) -> f32 {
    libm::atan2f(y, x)
}

fn asinf(x: f32) -> f32 {
    libm::asinf(x)
}

fn absf(x: f32) -> f32 {
    if x < 0.0 {
        -x
    } else {
        x
    }
}

fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}
