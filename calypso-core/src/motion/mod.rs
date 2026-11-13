//! The bar's motion: the attitude filter, the integrator, the detector that finds a rep, and the replay that stores it.

pub mod cues;
pub mod fusion;
pub mod integrate;
pub mod rep;
pub mod trace;

/// Longest integration step honoured, so a hole in the samples cannot become a jump.
pub const MAX_DT_S: f32 = 0.1;
