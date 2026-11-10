//! The attitude filter: the sensor's accelerometer and gyroscope in, a linear acceleration and an attitude out.
//!
//! The accelerometer measures **specific force**: gravity plus whatever the sensor is doing. Its
//! *direction* is gravity whenever the bar's own acceleration is small compared with `g`, which is true for a barbell even mid-stroke: a vertical stroke's acceleration is along gravity and does not tilt the measured direction at all.
//!
//! So the attitude is a complementary filter. The gyro carries it through rotation, and the accelerometer pulls it toward the measured gravity at all times — fast when the bar is still, slowly while it moves. Pulling only while still is what a first version did, and a gyro bias of a degree per second then walks the vertical several degrees across a set. A tilted vertical leaks `g·sin(tilt)` into the horizontal, and integrating that is a bar that appears to travel metres sideways.
//!
//! **Stillness is a separate question.** The accelerometer cannot tell a still bar from one crossing zero acceleration at its fastest point, so "has stopped changing" is not enough.
//! Still means the reading has held at 1 g, with the offset taken out, for long enough that a slow stroke cannot be what is being seen. That is the gate for the accelerometer's offset and the zero-velocity update; it is not the gate for believing gravity.
//!
//! **The gyro's offset is a question for the accelerometer.** A bias turns the attitude while gravity stays put in the sensor's frame; a real rotation moves gravity through it. That difference is what lets a set learn the bias between reps, where it is never still long enough to be called at rest.
//!
//! **What is not observable.** Yaw, because there is no magnetometer; it drifts and nothing here can stop it. An accelerometer offset is indistinguishable from a tilt, so its sideways part stays in the attitude as a couple of degrees and cancels out of the linear acceleration exactly.
//!
//! **Velocity drifts; stillness bounds it.** Integrating acceleration cannot know its own error, so velocity is only trustworthy between two still moments. Every rest sets it to zero, which is what a bar on a rack or at the bottom of a set is doing anyway.

use crate::sampler::Sample;

/// Standard gravity (m/s²): the specific force a sensor at rest reads up the world's up axis.
pub const GRAVITY: f32 = 9.80665;

/// Samples in the stillness window: 400 ms at the device's 100 Hz.
pub const STILL_WINDOW: usize = 40;
/// Per-axis spread of the specific force (m/s²) under which the reading has stopped changing.
///
/// A **spread**, not a magnitude, because a sensor's constant offset cannot widen it.
pub const STILL_SPREAD: f32 = 0.15;
/// How far the specific force may sit from 1 g, with the offset taken out, while the bar rests.
///
/// This is what tells a slow stroke from a still bar: a long gentle lift changes the reading's magnitude where its spread cannot see it.
const STILL_GRAVITY: f32 = 0.12;
/// Angular rate (rad/s) under which the sensor is not turning: 2 dps.
const STILL_RATE_DPS: f32 = 2.0;
/// The same while the gyro's offset is still unknown, in dps.
///
/// A resting gyro reads its own offset, so the tight bound above cannot decide whether it has stopped before that offset has been learned.
const STILL_RATE_BOOT_DPS: f32 = 5.0;

/// Per-axis spread of the measured gravity *direction* under which the sensor is not turning.
///
/// A gyro bias and a vertical stroke both leave gravity pointing the same way in the sensor's frame, so the bias can be learned during a set; a real rotation moves that direction, which is what keeps a roll from being mistaken for a bias.
const DIRECTION_SPREAD: f32 = 0.01;

/// How long the reading must be quiet before the attitude is taken to know which way is down.
pub const READY_S: f32 = 1.0;

/// How long the reading must hold at 1 g before the bar counts as at rest.
///
/// A slow stroke spends a third of a second crossing zero acceleration at its fastest point, where it reads 1 g and barely changes; only a rest that goes on can be told from that.
const SETTLE_S: f32 = 0.4;

/// Time constant of the attitude's pull toward measured gravity while the bar is still.
const ATT_TAU_STILL_S: f32 = 1.0;
/// The same while the bar is moving.
///
/// Slow enough that a stroke's own horizontal acceleration — a bench's bar path, a swing — is averaged rather than followed, fast enough that a changing gyro bias cannot walk the vertical.
const ATT_TAU_MOVING_S: f32 = 10.0;
/// Time constant of the accelerometer's offset while the filter is still booting.
const BOOT_TAU_S: f32 = 0.25;
/// The same once it is known: temperature moves it over minutes, while a slow stroke's shoulder looks like a constant reading for a third of a second.
const TRACK_TAU_S: f32 = 2.0;
/// Time constant of the gyro's offset, learned whenever gravity is not moving in the sensor's frame.
const GYRO_TAU_S: f32 = 0.25;
/// Time constant over which velocity is pulled back to zero while the bar is still.
const V_TAU_S: f32 = 0.5;
/// Longest integration step honoured, so a hole in the samples cannot become a jump in velocity.
const MAX_DT_S: f32 = 0.1;

const RAD_PER_DPS: f32 = core::f32::consts::PI / 180.0;

/// Everything the filter remembers, which is also everything a replay needs to start where it did.
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
    /// The sample's own timestamp.
    pub t_ms: u32,
    /// Specific force with gravity and the sensor's offset taken out, in the world frame (m/s²).
    pub linear_accel: [f32; 3],
    /// Whether the bar has been at rest across the last window.
    pub still: bool,
    /// Where the filter stands after this sample.
    pub state: FusionState,
}

/// The filter. One of these runs for as long as the device is up, and another replays a stored window from its recorded state.
#[derive(Debug, Clone)]
pub struct Fusion {
    state: FusionState,
    /// `None` until the first sample, because the step into the first one is not a step.
    prev_t_ms: Option<u32>,
    still_buf: [[f32; 3]; STILL_WINDOW],
    still_head: usize,
    still_len: usize,

    /// Seconds of unbroken quiet, which is what readiness is counted toward.
    quiet_s: f32,
    /// Seconds the reading has held at 1 g.
    settled_s: f32,
    /// Whether the attitude has been aligned to gravity at least once.
    ready: bool,
}

impl Fusion {
    pub fn new() -> Self {
        Fusion {
            state: FusionState::LEVEL,
            prev_t_ms: None,
            still_buf: [[0.0; 3]; STILL_WINDOW],
            still_head: 0,
            still_len: 0,

            quiet_s: 0.0,
            settled_s: 0.0,
            ready: false,
        }
    }

    /// A filter starting where another one was, which is how a stored window is replayed.
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

    /// Put the velocity back to zero, which is what a replay does at every turn it is told about.
    pub fn zero_velocity(&mut self) {
        self.state.velocity = [0.0; 3];
    }

    /// How long the reading has been quiet, in seconds.
    pub fn still_for_s(&self) -> f32 {
        self.quiet_s
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

        // The gyro carries the attitude whenever it is turning. Applied in the body frame.
        if dt > 0.0 {
            let turn = quat_from_rotvec(scale(rate, dt));
            self.state.attitude = normalize(quat_mul(self.state.attitude, turn));
        }

        let quiet = self.is_quiet(&raw, &rate);
        let f = sub(raw, self.state.accel_bias);
        let resting = absf(magnitude(f) - GRAVITY) <= STILL_GRAVITY
            && magnitude(rate) < STILL_RATE_DPS * RAD_PER_DPS;
        self.settled_s = if resting { self.settled_s + dt } else { 0.0 };
        let still = resting && self.settled_s >= SETTLE_S;
        self.state.settled = still;

        self.quiet_s = if quiet { self.quiet_s + dt } else { 0.0 };
        if !self.ready {
            // There is no attitude worth protecting at a boot, so it is set rather than filtered:
            // one exact rotation onto measured gravity. A filter would leave degrees of error.
            if quiet {
                self.set_attitude_from(&raw);
            }
            if self.quiet_s >= READY_S {
                self.ready = true;
            }
        } else {
            // From then on the gyro carries it and the accelerometer only pulls: hard while the bar is still, gently while it moves. Gently is enough, because gravity's direction is not corrupted by a vertical stroke.
            let tau = if still {
                ATT_TAU_STILL_S
            } else {
                ATT_TAU_MOVING_S
            };
            self.pull_toward_gravity(&raw, dt / tau);
        }

        // The accelerometer's offset is learned from a reading that has stopped changing, since that is the one thing an offset cannot fake. The gyro's is learned whenever gravity is not moving in the sensor's frame: a bias and a vertical stroke both leave it still, and a real rotation moves it, so a set teaches the gyro without ever being called at rest.
        let tau = if self.ready { TRACK_TAU_S } else { BOOT_TAU_S };
        if dt > 0.0 && quiet {
            self.learn_accel_offset(&raw, dt, tau);
        }
        if dt > 0.0 && self.direction_is_quiet() {
            for (bias, r) in self.state.gyro_bias.iter_mut().zip(rate.iter()) {
                *bias += r * (dt / GYRO_TAU_S);
            }
        }

        let f = sub(raw, self.state.accel_bias);
        let f_world = rotate_by_quat(self.state.attitude, f);
        let linear_accel = [f_world[0], f_world[1], f_world[2] - GRAVITY];

        // Before gravity's direction is known there is no frame to integrate in. After it, the velocity is the integral, and a bar at rest is where that integral is put back to zero.
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

    /// Whether the reading has stopped changing and the sensor has stopped turning.
    ///
    /// The width is of the **raw** reading, because measuring it after the offset is taken out would put the estimate inside its own input.
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
                lo = lo.min(v[k]);
                hi = hi.max(v[k]);
            }
            if hi - lo > STILL_SPREAD {
                return false;
            }
        }
        true
    }

    /// Turn the attitude so the measured direction maps exactly onto the world's up.
    fn set_attitude_from(&mut self, raw: &[f32; 3]) {
        let measured = unit(*raw);
        if magnitude(measured) == 0.0 {
            return;
        }
        let up = rotate_by_quat(conjugate(self.state.attitude), [0.0, 0.0, 1.0]);
        let err = cross(measured, up);
        let angle = atan2f(magnitude(err), dot(measured, up));
        if angle <= 1.0e-6 {
            return;
        }
        // At a half turn the cross product vanishes and any axis across `up` is as good as another: the sensor is upside down, and which way it faces after that is yaw.
        let axis = if magnitude(err) > 1.0e-3 {
            unit(err)
        } else {
            across(up)
        };
        let turn = quat_from_rotvec(scale(axis, angle));
        self.state.attitude = normalize(quat_mul(self.state.attitude, turn));
    }

    /// Rotate a fraction `k` of the way from the attitude's up toward measured gravity.
    fn pull_toward_gravity(&mut self, raw: &[f32; 3], k: f32) {
        if k <= 0.0 {
            return;
        }
        let up = rotate_by_quat(conjugate(self.state.attitude), [0.0, 0.0, 1.0]);
        let err = cross(unit(*raw), up);
        let turn = quat_from_rotvec(scale(err, k));
        self.state.attitude = normalize(quat_mul(self.state.attitude, turn));
    }

    /// Whether gravity has stopped moving in the sensor's frame, which is what the gyro's own offset is learned from.
    fn direction_is_quiet(&self) -> bool {
        if self.still_len < STILL_WINDOW {
            return false;
        }
        for k in 0..3 {
            let (mut lo, mut hi) = (f32::MAX, f32::MIN);
            for v in &self.still_buf {
                let d = unit(*v)[k];
                lo = lo.min(d);
                hi = hi.max(d);
            }
            if hi - lo > DIRECTION_SPREAD {
                return false;
            }
        }
        true
    }

    /// Learn the accelerometer's offset from what gravity does not explain.
    fn learn_accel_offset(&mut self, raw: &[f32; 3], dt: f32, tau: f32) {
        let up = rotate_by_quat(conjugate(self.state.attitude), [0.0, 0.0, 1.0]);
        let expected = scale(up, GRAVITY);
        for k in 0..3 {
            let offset = raw[k] - expected[k];
            self.state.accel_bias[k] += (offset - self.state.accel_bias[k]) * (dt / tau);
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

/// Rotate a body-frame vector into the world frame by a unit quaternion `[w, x, y, z]`.
pub fn rotate_by_quat(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let tx = 2.0 * (y * v[2] - z * v[1]);
    let ty = 2.0 * (z * v[0] - x * v[2]);
    let tz = 2.0 * (x * v[1] - y * v[0]);
    [
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    ]
}

/// Roll and pitch (degrees) from a unit quaternion. Yaw is not reported: it drifts without a magnetometer.
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
