//! Drift-resistant evidence: what the filter says about the bar without passing through an integrator.

use crate::motion::fusion::{conjugate, magnitude, rotate_by_quat, Filtered};

/// How far vertical acceleration must clear zero before it has a sign (m/s²).
const SIGN_EPS_MPS2: f32 = 0.05;

/// What the filter's own output says, with no integration behind it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cues {
    /// Which way the bar is accelerating vertically: -1, 0, or 1.
    pub sign_vert: i8,
    /// The bias-corrected angular rate's magnitude (rad/s).
    pub rate_mag: f32,
    /// Angle between the bar's own up axis and gravity (rad).
    pub tilt: f32,
    /// Whether this sample is the first with the opposite sign to the last one that had a sign.
    pub sign_change: bool,
}

/// Turns a filtered stream into cues, remembering only the last vertical sign.
#[derive(Debug, Clone, Copy)]
pub struct CueReader {
    prev_sign: i8,
}

impl CueReader {
    pub const fn new() -> Self {
        CueReader { prev_sign: 0 }
    }

    pub fn update(&mut self, f: &Filtered) -> Cues {
        let sign_vert = if f.linear_accel[2] > SIGN_EPS_MPS2 {
            1
        } else if f.linear_accel[2] < -SIGN_EPS_MPS2 {
            -1
        } else {
            0
        };
        let sign_change = sign_vert != 0 && self.prev_sign != 0 && sign_vert != self.prev_sign;
        if sign_vert != 0 {
            self.prev_sign = sign_vert;
        }

        let up = rotate_by_quat(conjugate(f.state.attitude), [0.0, 0.0, 1.0]);
        Cues {
            sign_vert,
            rate_mag: magnitude(f.rate),
            tilt: libm::acosf(up[2].clamp(-1.0, 1.0)),
            sign_change,
        }
    }
}

impl Default for CueReader {
    fn default() -> Self {
        Self::new()
    }
}
