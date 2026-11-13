//! The captures: recorded data that lives in an object store rather than in the repository.
//!
//! A recording is far too big to be source, and it never changes, so the bytes belong in a bucket and what belongs here is the description of them — [`manifest`], read out of `captures/captures.json`.
//! Nothing in this module names a bucket: the store is whoever's account is holding it, so an `mc` alias and bucket arrive as `CAPTURES_REMOTE` when a capture is not already on disk.
//!
//! Both readers here start from `captures/` beside the crate, which is where `tools/captures.sh` puts what it pulls.

use std::path::PathBuf;
use std::process::Command;

use serde::de::Deserializer;
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
    /// When the note that records this set was written, which is always after the set it describes.
    pub noted_at: NotedAt,
}

/// A wall-clock time from a session's notes, as unix milliseconds.
///
/// The notes are local time to the second, because that is what a person types, so the reading carries its own offset rather than borrowing the zone the session happened to be in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NotedAt(pub i64);

impl<'de> Deserialize<'de> for NotedAt {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let text = String::deserialize(d)?;
        let ms = unix_ms(&text).ok_or_else(|| {
            serde::de::Error::custom(format!(
                "{text:?}: a note's time is a local time with its offset, like 2026-11-10T15:27:33-08:00"
            ))
        })?;
        Ok(NotedAt(ms))
    }
}

/// Unix milliseconds for a `YYYY-MM-DDTHH:MM:SS±HH:MM` reading, or `None` for anything else.
fn unix_ms(text: &str) -> Option<i64> {
    let (date, rest) = text.split_once('T')?;
    let (time, offset) = rest.split_at(rest.find(['+', '-'])?);
    let [year, month, day] = numbers(date, '-')?;
    let [hour, minute, second] = numbers(time, ':')?;
    let [offset_hour, offset_minute] = numbers(&offset[1..], ':')?;
    if !(1..=12).contains(&month)
        || !(1..=31).contains(&day)
        || hour > 23
        || minute > 59
        || second > 59
        || offset_hour > 23
        || offset_minute > 59
    {
        return None;
    }
    let east = if offset.starts_with('-') { -1 } else { 1 };
    let offset_s = east * (offset_hour * 3600 + offset_minute * 60);
    let s = days_from_epoch(year, month, day) * 86_400 + hour * 3600 + minute * 60 + second;
    Some((s - offset_s) * 1000)
}

/// The integers separated by `sep`, or `None` unless there is exactly one for each of `N`.
fn numbers<const N: usize>(text: &str, sep: char) -> Option<[i64; N]> {
    let mut out = [0; N];
    let mut parts = text.split(sep);
    for slot in &mut out {
        *slot = parts.next()?.parse().ok()?;
    }
    parts.next().is_none().then_some(out)
}

/// Days from 1970-01-01 to a civil date, by the Gregorian calendar's own rule for leap years.
fn days_from_epoch(year: i64, month: i64, day: i64) -> i64 {
    let year = if month <= 2 { year - 1 } else { year };
    let era = year.div_euclid(400);
    let year_of_era = year - era * 400;
    let day_of_year = (153 * (if month > 2 { month - 3 } else { month + 9 }) + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A note's time is a local reading, so its offset is the whole of what makes it a moment: the same clock face with a different offset is a different instant, and a reading without one is not a moment at all.
    #[test]
    fn a_note_time_is_read_in_the_offset_it_was_taken_in() {
        assert_eq!(unix_ms("1970-01-01T00:00:00+00:00"), Some(0));
        assert_eq!(
            unix_ms("2024-02-29T00:00:00+00:00"),
            Some(1_709_164_800_000)
        );
        assert_eq!(
            unix_ms("2026-11-10T15:27:33-08:00"),
            Some(1_794_353_253_000)
        );
        assert_eq!(
            unix_ms("2026-11-10T23:27:33+00:00"),
            Some(1_794_353_253_000)
        );
        assert_eq!(unix_ms("2026-11-10T15:27:33"), None);
        assert_eq!(unix_ms("2026-13-10T15:27:33-08:00"), None);
        assert_eq!(unix_ms("2026-11-10T15:27:33-08:00:00"), None);
    }
}
