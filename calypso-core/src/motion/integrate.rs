//! World-frame velocity from the filter's linear acceleration, with zero-velocity constraints at anchors.

use crate::motion::MAX_DT_S;

/// Time constant (s) over which velocity is pulled back toward zero while the bar is at rest.
pub const REST_TAU_S: f32 = 0.5;
/// Time constant (s) over which the crossing speed is pulled back toward zero.
pub const CROSS_TAU_S: f32 = 1.0;

/// What an anchor says about the bar.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnchorKind {
    /// The bar was already at rest when the step began: an onset, or a hold.
    Rest,
    /// The bar's speed reached zero at this sample: it turned.
    Turn,
}

/// Which side of the integration step an anchor applies to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Apply {
    /// Zero before the step, so the motion that follows the instant is integrated from rest.
    Before,
    /// Zero after the step, so the step carries the deceleration that led to the turn.
    After,
}

/// A zero-velocity instant detection has found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Anchor {
    pub t_ms: u32,
    pub kind: AnchorKind,
    pub apply: Apply,
}

impl Anchor {
    /// An anchor at `t_ms` of `kind`: a rest is zeroed before its step, a turn after it.
    pub const fn at(t_ms: u32, kind: AnchorKind) -> Self {
        Anchor {
            t_ms,
            kind,
            apply: match kind {
                AnchorKind::Rest => Apply::Before,
                AnchorKind::Turn => Apply::After,
            },
        }
    }

    pub const fn rest(t_ms: u32) -> Self {
        Anchor::at(t_ms, AnchorKind::Rest)
    }

    pub const fn turn(t_ms: u32) -> Self {
        Anchor::at(t_ms, AnchorKind::Turn)
    }
}

/// The motion after one sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Integrated {
    pub t_ms: u32,
    /// World-frame velocity (m/s).
    pub velocity: [f32; 3],
    /// The same acceleration through a leak, for crossings only.
    pub v_cross: [f32; 3],
}

impl Integrated {
    pub const ZERO: Integrated = Integrated {
        t_ms: 0,
        velocity: [0.0; 3],
        v_cross: [0.0; 3],
    };
}

#[derive(Debug, Clone)]
pub struct Integration {
    velocity: [f32; 3],
    v_cross: [f32; 3],
    prev_t_ms: Option<u32>,
}

impl Integration {
    pub const fn new() -> Self {
        Integration {
            velocity: [0.0; 3],
            v_cross: [0.0; 3],
            prev_t_ms: None,
        }
    }

    pub const fn from_velocity(velocity: [f32; 3]) -> Self {
        Integration {
            velocity,
            v_cross: [0.0; 3],
            prev_t_ms: None,
        }
    }

    pub fn velocity(&self) -> [f32; 3] {
        self.velocity
    }

    /// Start again from rest at `t_ms`, so a pause is not a step.
    pub fn restart(&mut self, t_ms: u32) {
        self.velocity = [0.0; 3];
        self.v_cross = [0.0; 3];
        self.prev_t_ms = Some(t_ms);
    }

    fn zero_velocity(&mut self) {
        self.velocity = [0.0; 3];
    }

    pub fn step(
        &mut self,
        t_ms: u32,
        a_world: [f32; 3],
        resting: bool,
        ready: bool,
        anchors: &[Anchor],
    ) -> Integrated {
        let dt = match self.prev_t_ms {
            None => 0.0,
            Some(prev) => (t_ms.saturating_sub(prev) as f32 / 1000.0).min(MAX_DT_S),
        };
        self.prev_t_ms = Some(t_ms);

        for anchor in anchors {
            if anchor.t_ms == t_ms && anchor.apply == Apply::Before {
                self.zero_velocity();
            }
        }

        if dt > 0.0 && ready {
            for (v, a) in self.velocity.iter_mut().zip(a_world) {
                *v += a * dt;
            }
            if resting {
                for v in self.velocity.iter_mut() {
                    *v -= *v * (dt / REST_TAU_S);
                }
            }
            for (v, a) in self.v_cross.iter_mut().zip(a_world) {
                *v += a * dt - *v * (dt / CROSS_TAU_S);
            }
        }

        for anchor in anchors {
            if anchor.t_ms == t_ms && anchor.apply == Apply::After {
                self.zero_velocity();
            }
        }

        Integrated {
            t_ms,
            velocity: self.velocity,
            v_cross: self.v_cross,
        }
    }
}

impl Default for Integration {
    fn default() -> Self {
        Self::new()
    }
}
