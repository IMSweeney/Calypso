#![allow(dead_code)] // not every test target uses every helper

//! Loading a fixture, for the tests that check what the generator makes of it.
//!
//! Fixtures are proto files under `tests/fixtures/`. They import
//! `gatt_options.proto`, which the crate ships, so nothing here reaches outside
//! the crate.

use std::path::PathBuf;

use gatt_gen::{Gatt, Input};

pub fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

/// An input for one fixture. The descriptor set names files relative to an
/// include directory, so the fixture's own directory is one; the vocabulary's
/// comes from the crate itself.
pub fn fixture_input(name: &str) -> Input {
    let file = fixture_path(name);
    Input {
        protos: vec![file.clone()],
        includes: vec![file.parent().expect("a file in a directory").to_path_buf()],
    }
}

/// Load one fixture, whatever the outcome.
pub fn load(name: &str) -> Result<(Gatt, Vec<String>), gatt_gen::Error> {
    gatt_gen::load(&fixture_input(name))
}

/// The errors from a fixture that is expected to fail, or a panic.
pub fn errors(name: &str) -> Vec<String> {
    match load(name) {
        Err(gatt_gen::Error::Invalid(problems)) => problems,
        Err(other) => panic!("{name}: expected schema errors, got {other}"),
        Ok((_, warnings)) => panic!("{name}: expected schema errors, got warnings {warnings:?}"),
    }
}
