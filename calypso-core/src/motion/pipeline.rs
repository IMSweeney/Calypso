//! The live pipeline in one place: filter, cues, integrator, detector.

use crate::motion::cues::{CueReader, Cues};
use crate::motion::fusion::{magnitude, Filtered, Fusion, FusionState};
use crate::motion::integrate::{Integrated, Integration};
use crate::motion::rep::{DetectedRep, Detector, RepSeed};
use crate::sampler::Sample;

/// What one sample produced.
#[derive(Debug, Clone, Copy)]
pub struct Step {
    pub filtered: Filtered,
    pub integrated: Integrated,
    pub cues: Cues,
    /// The rep this sample closed, if detection was asked and a turn was confirmed.
    pub rep: Option<DetectedRep>,
}

/// The live pipeline, from a raw sample to a closed rep.
pub struct Pipeline {
    fusion: Fusion,
    cues: CueReader,
    integration: Integration,
    detector: Detector,
}

impl Pipeline {
    pub fn new() -> Self {
        Pipeline {
            fusion: Fusion::new(),
            cues: CueReader::new(),
            integration: Integration::new(),
            detector: Detector::new(),
        }
    }

    /// A replay that starts where a stored window was cut: its filter snapshot and the speed the bar carried.
    pub fn from_seed(seed: RepSeed) -> Self {
        Pipeline {
            fusion: Fusion::from_state(seed.filter),
            cues: CueReader::new(),
            integration: Integration::from_velocity(seed.velocity),
            detector: Detector::new(),
        }
    }

    /// Feed one sample. Detection runs only when asked and only once the filter knows which way is down.
    pub fn update(&mut self, sample: &Sample, detect: bool) -> Step {
        let filtered = self.fusion.update(sample);
        let integrated = self.integration.step(
            filtered.t_ms,
            filtered.linear_accel,
            filtered.resting,
            filtered.ready,
            &[],
        );
        let cues = self.cues.update(&filtered);
        let rep = if detect && filtered.ready {
            self.detector.update(&filtered, &integrated, &cues)
        } else {
            None
        };
        Step {
            filtered,
            integrated,
            cues,
            rep,
        }
    }

    /// Drop the detection and integration state across a pause, keeping what the filter learned.
    pub fn restart(&mut self, t_ms: u32) {
        self.integration.restart(t_ms);
        self.detector.restart();
        self.cues = CueReader::new();
    }

    pub fn ready(&self) -> bool {
        self.fusion.ready()
    }

    pub fn quiet_for_s(&self) -> f32 {
        self.fusion.quiet_for_s()
    }

    pub fn state(&self) -> FusionState {
        self.fusion.state()
    }

    /// The state a stored window starts from.
    pub fn seed(&self) -> RepSeed {
        RepSeed {
            filter: self.fusion.state(),
            velocity: self.integration.velocity(),
        }
    }

    pub fn speed(&self) -> f32 {
        magnitude(self.integration.velocity())
    }

    pub fn vertical(&self) -> f32 {
        self.detector.vertical()
    }

    pub fn halves(&self) -> u32 {
        self.detector.halves()
    }

    pub fn rep_open(&self) -> bool {
        self.detector.rep_open()
    }
}

impl Default for Pipeline {
    fn default() -> Self {
        Self::new()
    }
}
