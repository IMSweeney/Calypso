//! The captures: recorded data that lives in an object store rather than in the repository.
//!
//! A recording is far too big to be source, and it never changes, so the bytes belong in a bucket and what belongs here is the description of them — [`manifest`], read out of `captures/captures.json`.
//! Nothing in this module names a bucket: the store is whoever's account is holding it, so an `mc` alias and bucket arrive as `CAPTURES_REMOTE` when a capture is not already on disk.
//!
//! Both readers here start from `captures/` beside the crate, which is where `tools/captures.sh` puts what it pulls.

use std::path::PathBuf;
use std::process::Command;

use serde::Deserialize;

/// Where the manifest and any pulled capture are, from a test or an example: both run with the crate root as their working directory.
const DIR: &str = "../captures";

/// One capture as the manifest describes it.
#[derive(Debug, Deserialize)]
pub struct Capture {
    /// The file name in the bucket, which is also the name in `captures/`.
    pub name: String,
    /// The day it was recorded, `YYYY-MM-DD`.
    pub recorded: String,
    /// What produced the bytes: `live-stream` for the app's record button, `device-log` for a `record-mode` build's flash, `device-archive` for a phone's own history.
    pub kind: String,
    /// What was lifted, where the file is a workout.
    #[serde(default)]
    pub exercise: Option<String>,
    /// The note beside it that says what happened, if there is one.
    #[serde(default)]
    pub notes: Option<String>,
    /// Prose for a reader who wants to know what the file is before pulling it.
    pub about: String,
    /// What the detector should make of it: the reps per set, in the order they were performed.
    ///
    /// The value is the session's own — written down by the person who did the work — and not what the code under test currently answers, which is the whole reason it is here.
    #[serde(default)]
    pub expected: Option<Expected>,
}

/// What a session did, for the captures where that is known.
#[derive(Debug, Deserialize)]
pub struct Expected {
    pub sets: Vec<Set>,
}

/// One set of a session.
#[derive(Debug, Deserialize)]
pub struct Set {
    pub reps: u32,
    /// The bar's load, or nothing for a warmup at bar weight.
    #[serde(default)]
    pub load_lb: Option<u32>,
}

/// The manifest, in the order it lists them.
pub fn manifest() -> Vec<Capture> {
    let path = format!("{DIR}/captures.json");
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{path}: {e}, and the captures it describes need it"));
    #[derive(Deserialize)]
    struct Manifest {
        captures: Vec<Capture>,
    }
    serde_json::from_str::<Manifest>(&text)
        .unwrap_or_else(|e| panic!("{path}: {e}"))
        .captures
}

/// The bytes of one capture, from `captures/` if it has been pulled and out of the bucket if not.
///
/// A missing bucket name or a failed transfer is a panic rather than an empty document: a test that read no records would pass without having read anything.
pub fn bytes(name: &str) -> Vec<u8> {
    let local = local_path(name);
    if local.exists() {
        return std::fs::read(&local).unwrap_or_else(|e| panic!("{}: {e}", local.display()));
    }
    let remote = std::env::var("CAPTURES_REMOTE").unwrap_or_else(|_| {
        panic!(
            "{name} is not in {DIR}, and CAPTURES_REMOTE is not set to say where it is: either set it to an `mc` alias and bucket, or run `tools/captures.sh pull {name}`"
        )
    });
    let source = format!("{}/{name}", remote.trim_end_matches('/'));
    let out = Command::new("mc")
        .args(["cat", &source])
        .output()
        .unwrap_or_else(|e| panic!("mc cat {source}: {e}"));
    if !out.status.success() {
        panic!(
            "mc cat {source}: {}",
            String::from_utf8_lossy(&out.stderr).trim()
        );
    }
    out.stdout
}

fn local_path(name: &str) -> PathBuf {
    PathBuf::from(DIR).join(name)
}
