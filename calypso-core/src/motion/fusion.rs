//! The attitude filter: the sensor's accelerometer and gyroscope in, a linear acceleration and an attitude out.

use crate::motion::MAX_DT_S;
use crate::sampler::Sample;

/// Standard gravity (m/s²): the specific force a sensor at rest reads up the world's up axis.
pub const GRAVITY: f32 = 9.80665;

/// Samples in the stillness window: 400 ms at the device's 100 Hz.
pub const STILL_WINDOW: usize = 40;
/// How far the window's **mean** specific force may sit from 1 g, with the offset taken out, while the bar rests (m/s²).
pub const STILL_GRAVITY: f32 = 0.12;
/// How far any one sample in the window may sit from 1 g, with the offset taken out (m/s²).
pub const STILL_STRAY: f32 = 0.8;
/// The window's mean angular rate (dps) under which the sensor is not turning.
pub const STILL_RATE_DPS: f32 = 2.0;

/// Per-axis spread of the measured gravity *direction* under which the sensor is not turning.
const DIRECTION_SPREAD: f32 = 0.01;

/// How long the reading must be quiet before the attitude is taken to know which way is down.
pub const READY_S: f32 = 1.0;

/// Time constant of the attitude's pull toward measured gravity while the bar is still.
const ATT_TAU_STILL_S: f32 = 1.0;
/// The same while the bar is moving.
const ATT_TAU_MOVING_S: f32 = 10.0;
/// Time constant of the accelerometer's offset, learned from a resting window.
const TRACK_TAU_S: f32 = 2.0;
/// Time constant of the gyro's offset, learned whenever gravity is not moving in the sensor's frame.
const GYRO_TAU_S: f32 = 0.25;
/// Largest specific force (m/s²) the sensor can have read: its configured clip.
const MAX_FORCE: f32 = crate::sampler::ACCEL_RANGE_G as f32 * GRAVITY;

const RAD_PER_DPS: f32 = core::f32::consts::PI / 180.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FusionState {
    /// Attitude, body to world, as `[w, x, y, z]`.
    pub attitude: [f32; 4],
    /// The accelerometer's offset in the sensor's own frame (m/s²).
    pub accel_bias: [f32; 3],
    /// The gyroscope's offset in the sensor's own frame (rad/s).
    pub gyro_bias: [f32; 3],
    /// Whether the bar was at rest when this was taken, which is what seeds the stillness window.
    pub resting: bool,
    /// Whether the filter has seen gravity at least once.
    pub ready: bool,
}

impl FusionState {
    pub const LEVEL: FusionState = FusionState {
        attitude: [1.0, 0.0, 0.0, 0.0],
        accel_bias: [0.0; 3],
        gyro_bias: [0.0; 3],
        resting: false,
        ready: false,
    };

    /// Level, with the frame already found: what a stored window starts from.
    pub const LEVEL_READY: FusionState = FusionState {
        attitude: [1.0, 0.0, 0.0, 0.0],
        accel_bias: [0.0; 3],
        gyro_bias: [0.0; 3],
        resting: false,
        ready: true,
    };
}

impl Default for FusionState {
    fn default() -> Self {
        Self::LEVEL
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Filtered {
    /// The sample's own timestamp.
    pub t_ms: u32,
    /// Specific force with gravity and the sensor's offset taken out, in the world frame (m/s²).
    pub linear_accel: [f32; 3],
    /// Bias-corrected angular rate in the sensor's own frame (rad/s).
    pub rate: [f32; 3],
    /// Whether the bar has been at rest across the last window.
    pub resting: bool,
    /// Whether the filter has seen gravity at least once.
    pub ready: bool,
    /// Where the filter stands after this sample.
    pub state: FusionState,
}

#[derive(Debug, Clone, Copy)]
struct Window {
    /// Mean specific force, as the sensor read it (m/s²).
    mean_force: [f32; 3],
    /// Mean angular rate as the gyroscope read it (rad/s).
    mean_rate: [f32; 3],
    /// The furthest any one sample sat from 1 g, with the accelerometer's offset out (m/s²).
    stray: f32,
}

#[derive(Debug, Clone)]
pub struct Fusion {
    state: FusionState,
    prev_t_ms: Option<u32>,

    win: [[f32; 6]; STILL_WINDOW],
    win_head: usize,
    win_len: usize,

    /// Seconds of unbroken quiet, which is what readiness is counted toward.
    quiet_s: f32,
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
        }
    }

    pub fn from_state(state: FusionState) -> Self {
        let mut fusion = Fusion {
            state,
            ..Fusion::new()
        };
        if state.resting {
            let up = rotate_by_quat(conjugate(state.attitude), [0.0, 0.0, 1.0]);
            let resting = [
                up[0] * GRAVITY + state.accel_bias[0],
                up[1] * GRAVITY + state.accel_bias[1],
                up[2] * GRAVITY + state.accel_bias[2],
            ];
            for _ in 0..STILL_WINDOW {
                fusion.record(&resting, &state.gyro_bias);
            }
        }
        fusion
    }

    /// A replay entry point: a window the detector confirmed at rest fixes the frame and both offsets.
    ///
    /// The last [`STILL_WINDOW`] samples are used, so a longer window adds nothing, and the step after it is measured from its last sample.
    pub fn from_rest(samples: &[Sample]) -> Option<Self> {
        let tail = samples.get(samples.len().saturating_sub(STILL_WINDOW)..)?;
        let last = tail.last()?;
        let mut fusion = Fusion {
            state: FusionState {
                resting: true,
                ready: true,
                ..FusionState::LEVEL
            },
            ..Fusion::new()
        };
        for s in tail {
            let rate = s.gyro_f32().map(|dps| dps * RAD_PER_DPS);
            fusion.record(&s.specific_force_f32(), &rate);
        }
        let window = fusion.window();
        if magnitude(window.mean_force) < 0.5 * GRAVITY {
            return None;
        }
        fusion.set_attitude_from(&window.mean_force);
        fusion.set_accel_offset(&window.mean_force);
        fusion.set_gyro_offset(&window.mean_rate);
        fusion.prev_t_ms = Some(last.t_ms);
        fusion.quiet_s = READY_S;
        Some(fusion)
    }

    pub fn state(&self) -> FusionState {
        self.state
    }

    pub fn ready(&self) -> bool {
        self.state.ready
    }

    pub fn quiet_for_s(&self) -> f32 {
        self.quiet_s
    }

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
        if magnitude(raw) > MAX_FORCE {
            return Filtered {
                t_ms: s.t_ms,
                linear_accel: [0.0; 3],
                rate,
                resting: self.state.resting,
                ready: self.state.ready,
                state: self.state,
            };
        }

        if dt > 0.0 {
            let turn = quat_from_rotvec(scale(rate, dt));
            self.state.attitude = normalize(quat_mul(self.state.attitude, turn));
        }

        self.record(&raw, &s.gyro_f32().map(|dps| dps * RAD_PER_DPS));
        let window = self.window();
        let full = self.win_len >= STILL_WINDOW;
        let shaped = full && window.stray <= STILL_STRAY;
        let mean_rate = sub(window.mean_rate, self.state.gyro_bias);
        let quiet = shaped && magnitude(mean_rate) < STILL_RATE_DPS * RAD_PER_DPS;
        let offset_out = sub(window.mean_force, self.state.accel_bias);
        let resting = quiet && absf(magnitude(offset_out) - GRAVITY) <= STILL_GRAVITY;
        self.state.resting = resting;

        self.quiet_s = if shaped { self.quiet_s + dt } else { 0.0 };
        if !self.state.ready {
            if shaped {
                self.set_attitude_from(&window.mean_force);
                self.set_accel_offset(&window.mean_force);
                if self.direction_is_quiet() {
                    self.set_gyro_offset(&window.mean_rate);
                }
            }
            if self.quiet_s >= READY_S {
                self.state.ready = true;
            }
        } else {
            let tau = if resting || self.direction_is_quiet() {
                ATT_TAU_STILL_S
            } else {
                ATT_TAU_MOVING_S
            };
            self.pull_toward_gravity(&window.mean_force, dt / tau);
        }

        if dt > 0.0 && quiet {
            self.learn_accel_offset(&window.mean_force, dt, TRACK_TAU_S);
        }
        if dt > 0.0 && self.direction_is_quiet() {
            for (bias, r) in self.state.gyro_bias.iter_mut().zip(rate.iter()) {
                *bias += r * (dt / GYRO_TAU_S);
            }
        }

        let f = sub(raw, self.state.accel_bias);
        let f_world = rotate_by_quat(self.state.attitude, f);
        let linear_accel = [f_world[0], f_world[1], f_world[2] - GRAVITY];

        Filtered {
            t_ms: s.t_ms,
            linear_accel,
            rate,
            resting,
            ready: self.state.ready,
            state: self.state,
        }
    }

    fn record(&mut self, raw: &[f32; 3], rate: &[f32; 3]) {
        self.win[self.win_head] = [raw[0], raw[1], raw[2], rate[0], rate[1], rate[2]];
        self.win_head = (self.win_head + 1) % STILL_WINDOW;
        self.win_len = (self.win_len + 1).min(STILL_WINDOW);
    }

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

    fn set_accel_offset(&mut self, raw: &[f32; 3]) {
        let up = rotate_by_quat(conjugate(self.state.attitude), [0.0, 0.0, 1.0]);
        for k in 0..3 {
            self.state.accel_bias[k] = raw[k] - up[k] * GRAVITY;
        }
    }

    fn set_gyro_offset(&mut self, rate: &[f32; 3]) {
        self.state.gyro_bias = *rate;
    }

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
        let axis = if magnitude(err) > 1.0e-3 {
            unit(err)
        } else {
            across(up)
        };
        let turn = quat_from_rotvec(scale(axis, angle));
        self.state.attitude = normalize(quat_mul(self.state.attitude, turn));
    }

    fn pull_toward_gravity(&mut self, raw: &[f32; 3], k: f32) {
        if k <= 0.0 {
            return;
        }
        let up = rotate_by_quat(conjugate(self.state.attitude), [0.0, 0.0, 1.0]);
        let err = cross(unit(*raw), up);
        let turn = quat_from_rotvec(scale(err, k));
        self.state.attitude = normalize(quat_mul(self.state.attitude, turn));
    }

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

    fn learn_accel_offset(&mut self, raw: &[f32; 3], dt: f32, tau: f32) {
        let expected = scale(unit(*raw), GRAVITY);
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

pub fn magnitude(v: [f32; 3]) -> f32 {
    sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

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

///Yaw is not reported: it drifts without a magnetometer.
pub fn roll_pitch_deg(q: [f32; 4]) -> (f32, f32) {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let roll = atan2f(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let pitch_arg = clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    let pitch = asinf(pitch_arg);
    (roll.to_degrees(), pitch.to_degrees())
}

pub fn conjugate(q: [f32; 4]) -> [f32; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

fn quat_mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

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
