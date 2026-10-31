//! Simulation and fixture tooling for Calypso.
//!
//! A crate of its own so the firmware's library carries no generator it never calls: `calypso-core` depends on nothing here, and this depends on all of it.
//! What it produces goes to the app — `.pblog` fixtures the client is developed against — and to the detector's tests.

pub mod fixtures;
pub mod leg;
pub mod map;
pub mod motion;
