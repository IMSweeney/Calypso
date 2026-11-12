//! The attitude filter: the sensor's accelerometer and gyroscope in, a linear acceleration and an attitude out.
//!
//! The accelerometer measures **specific force**: gravity plus whatever the sensor is doing. Its
//! *direction* is gravity whenever the bar's own acceleration is small compared with `g`, which is true for a barbell even mid-stroke: a vertical stroke's acceleration is along gravity and does not tilt the measured direction at all.
//!
//! So the attitude is a complementary filter. The gyro carries it through rotation, and the accelerometer pulls it toward the measured gravity at all times — fast when the bar is still, slowly while it moves. Pulling only while still is what a first version did, and a gyro bias of a degree per second then walks the vertical several degrees across a set. A tilted vertical leaks `g·sin(tilt)` into the horizontal, and integrating that is a bar that appears to travel metres sideways.
//!
//! **Stillness is a separate question.** The accelerometer cannot tell a still bar from one crossing zero acceleration at its fastest point, so "has stopped changing" is not enough.
//! Stillness is asked of the **window**, not of the reading: a resting bar's specific force swings by a fraction of a m/s² from sample to sample, so no single reading and no spread of them can say the bar is not moving, while a stroke's acceleration changes sign inside a window and averages to nothing.
//! Both halves of the question are needed. The window's mean is 1 g with the offset out, which a slow stroke cannot hold; and nothing in the window strays far from it, which a fast stroke cannot avoid.
//! That is the gate for the accelerometer's offset and the zero-velocity update; it is not the gate for believing gravity.
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
/// How far the window's **mean** specific force may sit from 1 g, with the offset taken out, while the bar rests (m/s²).
///
/// The mean and not the spread: a resting bar's reading strays by a few tenths from sample to sample, so a bound on the spread is a bound on the platform's vibration.
/// What a slow stroke cannot do is hold this mean for a window, because a stroke that did would be going nowhere.
pub const STILL_GRAVITY: f32 = 0.12;
/// How far any one sample in the window may sit from 1 g, with the offset taken out (m/s²).
///
/// The other half of the same question: a stroke's acceleration turns over inside a window and its mean comes back to 1 g while the bar is plainly moving.
/// Loose enough for a resting bar to swing under it and tight enough that a stroke cannot hide in it.
pub const STILL_STRAY: f32 = 0.8;
/// The window's mean angular rate (dps) under which the sensor is not turning.
pub const STILL_RATE_DPS: f32 = 2.0;

/// Per-axis spread of the measured gravity *direction* under which the sensor is not turning.
///
/// A gyro bias and a vertical stroke both leave gravity pointing the same way in the sensor's frame, so the bias can be learned during a set; a real rotation moves that direction, which is what keeps a roll from being mistaken for a bias.
const DIRECTION_SPREAD: f32 = 0.01;

/// How long the reading must be quiet before the attitude is taken to know which way is down.
pub const READY_S: f32 = 1.0;

/// Time constant of the attitude's pull toward measured gravity while the bar is still.
const ATT_TAU_STILL_S: f32 = 1.0;
/// The same while the bar is moving.
///
/// Slow enough that a stroke's own horizontal acceleration — a bench's bar path, a swing — is averaged rather than followed, fast enough that a changing gyro bias cannot walk the vertical.
const ATT_TAU_MOVING_S: f32 = 10.0;
/// Time constant of the accelerometer's offset, learned from a resting window.
///
/// Slow, because temperature moves it over minutes and a rest is the only reading that can be believed about it.
const TRACK_TAU_S: f32 = 2.0;
/// Time constant of the gyro's offset, learned whenever gravity is not moving in the sensor's frame.
const GYRO_TAU_S: f32 = 0.25;
/// Time constant over which velocity is pulled back to zero while the bar is still.
const V_TAU_S: f32 = 0.5;
/// Longest integration step honoured, so a hole in the samples cannot become a jump in velocity.
const MAX_DT_S: f32 = 0.1;
/// Largest specific force (m/s²) the sensor can have read: its configured clip.
///
/// A reading past the clip is not a bar: the device's stream carries a handful of corrupt samples a session, and each one is metres per second of velocity and a window of attitude that were never measured.
const MAX_FORCE: f32 = crate::sampler::ACCEL_RANGE_G as f32 * GRAVITY;

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

/// What the last [`STILL_WINDOW`] samples say about the bar.
#[derive(Debug, Clone, Copy)]
struct Window {
    /// Mean specific force, as the sensor read it (m/s²).
    mean_force: [f32; 3],
    /// Mean angular rate with the gyro's offset out (rad/s).
    mean_rate: [f32; 3],
    /// The furthest any one sample sat from 1 g, with the accelerometer's offset out (m/s²).
    stray: f32,
}

/// The filter. One of these runs for as long as the device is up, and another replays a stored window from its recorded state.
#[derive(Debug, Clone)]
pub struct Fusion {
    state: FusionState,
    /// `None` until the first sample, because the step into the first one is not a step.
    prev_t_ms: Option<u32>,
    /// The last [`STILL_WINDOW`] samples: the specific force as it was read, then the angular rate with the gyro's offset out, over storage that wraps.
    ///
    /// The specific force is kept **raw**, because the offset is what is learned from it, and judging it once the offset is out would put the estimate inside its own input.
    win: [[f32; 6]; STILL_WINDOW],
    win_head: usize,
    win_len: usize,

    /// Seconds of unbroken quiet, which is what readiness is counted toward.
    quiet_s: f32,
    /// Whether the attitude has been aligned to gravity at least once.
    ready: bool,
}

impl Fusion {
    pub fn new() -> Self {
        Fusion {
            state: FusionState::LEVEL,
            prev_t_ms: None,
            win: [[0.0; 6]; STILL_WINDOW],
            win_head: 0,
            win_len: 0,

            quiet_s: 0.0,
            ready: false,
        }
    }

    /// A filter starting where another one was, which is how a stored window is replayed.
    pub fn from_state(state: FusionState) -> Self {
        let mut fusion = Fusion {
            state,
            ready: true,
            ..Fusion::new()
        };
        if state.settled {
            // The state was taken in a rest, so the window the replay starts from is that rest: a filter seeded with one sample and then told the bar is still would have to earn the same window again, and the samples to earn it with are behind it.
            let up = rotate_by_quat(conjugate(state.attitude), [0.0, 0.0, 1.0]);
            let resting = [
                up[0] * GRAVITY + state.accel_bias[0],
                up[1] * GRAVITY + state.accel_bias[1],
                up[2] * GRAVITY + state.accel_bias[2],
            ];
            for _ in 0..STILL_WINDOW {
                fusion.record(&resting, &[0.0; 3]);
            }
        }
        fusion
    }

    pub fn state(&self) -> FusionState {
        self.state
    }

    /// Put the velocity back to zero, which is what a replay does at every turn it is told about.
    pub fn zero_velocity(&mut self) {
        self.state.velocity = [0.0; 3];
    }

    /// How long the bar could have been at rest, in seconds — what readiness is counted toward.
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
        // A sample the sensor cannot have read is not a measurement of the bar, so it moves the clock and nothing else.
        if magnitude(raw) > MAX_FORCE {
            return Filtered {
                t_ms: s.t_ms,
                linear_accel: [0.0; 3],
                still: self.state.settled,
                state: self.state,
            };
        }

        // The gyro carries the attitude whenever it is turning. Applied in the body frame.
        if dt > 0.0 {
            let turn = quat_from_rotvec(scale(rate, dt));
            self.state.attitude = normalize(quat_mul(self.state.attitude, turn));
        }

        self.record(&raw, &rate);
        let window = self.window();
        let full = self.win_len >= STILL_WINDOW;
        // A window the bar could have been resting in: nothing in it strayed far from 1 g and it was not turning.
        // What is left to ask is whether `1 g` is where its mean sits, which is a question about the offset rather than about the motion.
        let shaped = full && window.stray <= STILL_STRAY;
        let resting = shaped && magnitude(window.mean_rate) < STILL_RATE_DPS * RAD_PER_DPS;
        let offset_out = sub(window.mean_force, self.state.accel_bias);
        let still = resting && absf(magnitude(offset_out) - GRAVITY) <= STILL_GRAVITY;
        self.state.settled = still;

        self.quiet_s = if shaped { self.quiet_s + dt } else { 0.0 };
        if !self.ready {
            // There is no attitude worth protecting at a boot, so the state is set rather than filtered: one exact rotation onto measured gravity, and both offsets set with it.
            // A reading cannot say how much of itself is the bar's motion and how much is the sensor's error, and a filter would leave the difference as degrees and tenths of a m/s² for the rest of the session.
            if shaped {
                self.set_attitude_from(&window.mean_force);
                self.set_accel_offset(&window.mean_force);
                self.set_gyro_offset(&window.mean_rate);
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
            self.pull_toward_gravity(&window.mean_force, dt / tau);
        }

        // The accelerometer's offset is learned from a window the bar could have been resting in, and from nothing else. A stroke's shoulder holds one reading about as long as a rest does, so learning it from "the reading has stopped changing" teaches the offset the bar's own acceleration, in as many directions as the bar strokes — a bias that follows the motion, largest where the bar is fastest.
        if dt > 0.0 && resting {
            self.learn_accel_offset(&window.mean_force, dt, TRACK_TAU_S);
        }
        // The gyro's offset is learned whenever gravity is not moving in the sensor's frame: a bias and a vertical stroke both leave it still, and a real rotation moves it, so a set teaches the gyro without ever being called at rest.
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

    /// Take one sample into the window, oldest dropping off.
    fn record(&mut self, raw: &[f32; 3], rate: &[f32; 3]) {
        self.win[self.win_head] = [raw[0], raw[1], raw[2], rate[0], rate[1], rate[2]];
        self.win_head = (self.win_head + 1) % STILL_WINDOW;
        self.win_len = (self.win_len + 1).min(STILL_WINDOW);
    }

    /// What the window holds: the mean specific force and angular rate, and the furthest any one sample sat from 1 g.
    fn window(&self) -> Window {
        let n = self.win_len.max(1) as f32;
        let (mut force, mut rate, mut stray) = ([0.0f32; 3], [0.0f32; 3], 0.0f32);
        for w in &self.win {
            let raw = [w[0], w[1], w[2]];
            for k in 0..3 {
                force[k] += raw[k];
                rate[k] += w[3 + k];
            }
            let f = sub(raw, self.state.accel_bias);
            let off = absf(magnitude(f) - GRAVITY);
            if off > stray {
                stray = off;
            }
        }
        Window {
            mean_force: scale(force, 1.0 / n),
            mean_rate: scale(rate, 1.0 / n),
            stray,
        }
    }

    /// Put the accelerometer's offset where the mean reading is exactly 1 g, which is what a boot has to do before any window can be judged.
    fn set_accel_offset(&mut self, raw: &[f32; 3]) {
        let up = rotate_by_quat(conjugate(self.state.attitude), [0.0, 0.0, 1.0]);
        for k in 0..3 {
            self.state.accel_bias[k] = raw[k] - up[k] * GRAVITY;
        }
    }

    /// Put the gyroscope's offset where the mean rate is zero, which is the same boot question asked of the other sensor.
    fn set_gyro_offset(&mut self, rate: &[f32; 3]) {
        self.state.gyro_bias = *rate;
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
        if self.win_len < STILL_WINDOW {
            return false;
        }
        for k in 0..3 {
            let (mut lo, mut hi) = (f32::MAX, f32::MIN);
            for w in &self.win {
                let d = unit([w[0], w[1], w[2]])[k];
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
