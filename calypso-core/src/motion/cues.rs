//! Drift-resistant evidence: what the filter says about the bar without passing through an integrator.

use crate::motion::fusion::{conjugate, magnitude, rotate_by_quat, Filtered};

/// How far vertical acceleration must clear zero before it has a sign (m/s²).
const SIGN_EPS_MPS2: f32 = 0.05;

/// Vertical acceleration (m/s²) up that the bar only reaches when something stops it.
///
/// A drive peaks a few m/s² up; the landings in the recorded deadlifts read 6–32.
const CONTACT_MPS2: f32 = 6.0;
/// The rise (m/s²) across one sample that makes an excursion a landing rather than a stroke.
///
/// A bar under its own load changes vertical acceleration by a couple of m/s² per sample; a bar put down changes it by tens.
const CONTACT_RISE_MPS2: f32 = 5.0;
/// The fall (m/s²) back off that excursion which says it was a shock and not a ride.
const CONTACT_FALL_MPS2: f32 = 8.0;
/// Samples after the peak within which that fall may arrive: a landing reads as a pair of opposed spikes.
const CONTACT_SETTLE: usize = 3;
/// How long one landing holds off the next, so the bounce after it is not a second contact.
const CONTACT_HOLD_MS: u32 = 250;
/// Longest step a rise or fall is measured across: a gap in the stream is not a shock.
const CONTACT_STEP_MS: u32 = 100;

/// Samples of vertical acceleration the reader keeps, which is what a shock is recognised in: the peak, the two either side of it, and the one it rose from.
const VERT_HIST: usize = CONTACT_SETTLE + 3;

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
    /// When the bar was stopped by what it landed on, which is the sample its acceleration peaked at.
    ///
    /// It is named a sample or two after the fact, because a shock is only a shock once its acceleration has come back off.
    pub landing: Option<u32>,
}

/// Turns a filtered stream into cues, remembering the last vertical sign and the last few samples' worth of it.
#[derive(Debug, Clone, Copy)]
pub struct CueReader {
    prev_sign: i8,
    verts: [(u32, f32); VERT_HIST],
    /// Where the newest sample is written next.
    head: usize,
    len: usize,
    last_landing_ms: Option<u32>,
}

impl CueReader {
    pub const fn new() -> Self {
        CueReader {
            prev_sign: 0,
            verts: [(0, 0.0); VERT_HIST],
            head: 0,
            len: 0,
            last_landing_ms: None,
        }
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

        self.record(f.t_ms, f.linear_accel[2]);
        let landing = if f.ready { self.landing() } else { None };

        let up = rotate_by_quat(conjugate(f.state.attitude), [0.0, 0.0, 1.0]);
        Cues {
            sign_vert,
            rate_mag: magnitude(f.rate),
            tilt: libm::acosf(up[2].clamp(-1.0, 1.0)),
            sign_change,
            landing,
        }
    }

    fn record(&mut self, t_ms: u32, vert: f32) {
        self.verts[self.head] = (t_ms, vert);
        self.head = (self.head + 1) % VERT_HIST;
        self.len = (self.len + 1).min(VERT_HIST);
    }

    /// The sample `back` places before the newest, 0 being the newest itself.
    fn at(&self, back: usize) -> Option<(u32, f32)> {
        if back >= self.len {
            return None;
        }
        Some(self.verts[(self.head + VERT_HIST - 1 - back) % VERT_HIST])
    }

    /// When the bar was stopped: the newest of the recent samples that spiked up and fell back.
    fn landing(&mut self) -> Option<u32> {
        let (now_ms, now) = self.at(0)?;
        for back in 1..=CONTACT_SETTLE {
            let (peak_ms, peak) = self.at(back)?;
            let (before_ms, before) = self.at(back + 1)?;
            if now_ms.saturating_sub(peak_ms) > CONTACT_STEP_MS * CONTACT_SETTLE as u32 {
                return None;
            }
            if peak_ms.saturating_sub(before_ms) > CONTACT_STEP_MS {
                return None;
            }
            if peak >= CONTACT_MPS2
                && peak - before >= CONTACT_RISE_MPS2
                && peak - now >= CONTACT_FALL_MPS2
            {
                if self
                    .last_landing_ms
                    .is_some_and(|last| peak_ms.saturating_sub(last) < CONTACT_HOLD_MS)
                {
                    return None;
                }
                self.last_landing_ms = Some(peak_ms);
                return Some(peak_ms);
            }
        }
        None
    }
}

impl Default for CueReader {
    fn default() -> Self {
        Self::new()
    }
}
