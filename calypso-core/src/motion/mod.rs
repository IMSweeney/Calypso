//! The bar's motion: the attitude filter, the integrator, and the detector that reads a rep out of what they produce.

pub mod fusion;
pub mod integrate;
pub mod rep;

/// Longest integration step honoured, so a hole in the samples cannot become a jump.
pub const MAX_DT_S: f32 = 0.1;
