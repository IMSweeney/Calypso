---
name: workout-capture
display-name: Workout Capture
description: "Use when recording a workout from the Calypso sensor: take set notes as they are typed, then save, describe and upload the capture."
---

# Workout capture

A session goes: the sensor is mounted on the bar, the user presses record in the app, lifts, and
types a terse note after each set. Your job is to stay out of the way while that happens, then turn
the finished recording into a described capture that is in the bucket and in the manifest.

All paths are from the Calypso repository root.

## 1. While the user is lifting

They open with something like "just connected, recording now", or "going to use this chat to record
some timestamped notes, no need to respond".

- Acknowledge in a single short line — "Noted." — and nothing else. No tools, no analysis, no
  commentary on their lifting.
- Each following note is one set, written however they type it: `warm up 5x 135`, `5x 185`,
  `10 reps bar only`.
- Never question a note, never suggest anything, never re-read files. The notes exist to become the
  rep counts at the end, and nothing else.
- If they correct something later ("it was bench btw"), fix it in the write-up rather than arguing.
- Do not run tools during this phase. It costs nothing to say "Noted." and it reads as attention.

## 2. When they say the recording is saved

Work through these in order. Ask before guessing: a wrong `load_lb` in the manifest is worse than
an absent one, and a rep count is the whole point of the session.

### a. Find the capture

The app writes `imu-YYYYMMDD-HHMMSS-mmm.pblog` into its imu folder:

```
~/Library/Containers/com.calypso.calypso/Data/Library/Application Support/com.calypso.calypso/imu/
```

Take the **newest** `.pblog` there, and check it really is this session's before copying: the file
name carries its start time, and its last write is when the recording was stopped. Older recordings
stay in that folder indefinitely; taking the wrong one is silent and expensive.

### b. Name it and bring it into the repo

`captures/<exercise>-<YYYY-MM-DD>.pblog` — `bench`, `deadlift`, `squat`. Copy it rather than moving
it: the app's copy is the app's. The local copy in `captures/` is the cache that keeps replays and
tests from re-downloading, and `.gitignore` keeps it out of git.

### c. Recover the note times

The session transcript holds the time each note was sent. Read it with the introspection tool:

```
bionic_tool(name="introspection.read_session", args=["--project", "self", "--session", "self",
  "--type", "user", "--limit", "30", "--page", "1"])
```

The newest entry carries an absolute time and every other carries an offset from the one above it, so
anchor on the newest and walk backwards. Then cross-check both ends against the file: its name gives
the start, its modification time the end. Those offsets are rounded to about two figures, so the
recovered times are good to roughly half a minute — say so in the write-up instead of implying the
transcript is a stopwatch. Each one goes into the manifest as its set's `noted_at`.

### d. Decode it and replay it

```sh
cargo run --release --manifest-path sim/Cargo.toml --example replay_stream -- captures/<name>.pblog
```

That prints the sample count and duration, any hole in the timeline, every span the detector closed
with its travel, and how the filter behaved. Write down what it says, not what you expect: the rate,
whether `seq` was unbroken, the spans against the reps that actually happened.

Add `--series` when a question needs the signals the detector gates on; it is a per-sample table.

### e. Write the session note

`captures/session-<YYYY-MM-DD>-<exercise>.md`, beside the capture:

- a table of the timestamped notes, in the user's words;
- the session's shape — sets × reps × load, and anything they said about how it felt;
- what the recording shows: rate, gaps, bytes per hour;
- what the detector makes of it: the sets it found, the reps it closed against the reps performed.

Numbers that move belong here, in the note. They do not belong in `docs/`.

### f. Add the manifest entry

`captures/captures.json`, appended to `captures`:

```json
{
  "name": "deadlift-2026-11-10.pblog",
  "recorded": "2026-11-10",
  "kind": "live-stream",
  "exercise": "deadlift",
  "notes": "session-2026-11-10-deadlift.md",
  "about": "One sentence on what the session was and why it was recorded.",
  "expected": {
    "sets": [
      { "reps": 5, "load_lb": 135, "noted_at": "2026-11-10T15:51:28-08:00" },
      { "reps": 5, "load_lb": null, "noted_at": "2026-11-10T15:54:34-08:00" }
    ]
  }
}
```

`kind` is `live-stream` for the app's record button, `device-log` for a flash dump, `device-archive`
for a phone's own history. `expected` is the **ground truth from the user's notes**, in the order
the sets were performed, never what the detector currently closes: `sim/tests/captures.rs` is
supposed to be able to fail, and it does. Bar-only sets are `"load_lb": null`.

`noted_at` is that set's time out of the notes, to the second, with the offset they were taken in
— `-08:00` or whatever the session ran in, never a bare local time and never an epoch number. It
is the time the note was typed, so it is after the set: the tests treat it as an upper bound on
when the set ended, and nothing else.

### g. Upload it, and prove it landed

```sh
export CAPTURES_REMOTE=r2/calypso-captures
tools/captures.sh push <name>
shasum -a 256 captures/<name>
mc cat r2/calypso-captures/<name> | shasum -a 256
```

The two hashes have to match; a push that reports success is not the same as bytes that arrived. If
the alias is missing, ask the user to set one with `mc alias set` — do not put credentials in the
repository or in a file.

### h. Say where the detector stands

```sh
CAPTURES_REMOTE=r2/calypso-captures cargo test --release --manifest-path sim/Cargo.toml \
  --test captures
```

It reads the captures from the bucket, replays them, and compares the sets it closes with the
manifest. It fails today, because a real rep is being closed as two strokes. Report the counts; do
not "fix" the expectations to match the code, and do not quietly delete the test.

## 3. What not to do

- Do not commit unless asked. The capture, the note and the manifest change are left uncommitted.
- Do not delete the app's copy, and do not delete the local one either: it is the cache.
- Do not invent a load, a rep, or a set that the user did not say.
- A second recording on the same connection does not restart `seq` and the device clock keeps
  running, so its chunks do not begin at 0 and the two sessions sit on one timeline. Do not read
  either as a gap.
- The detector's spans are places, not measurements: `rom` on real data runs to metres when the bar
  moved less than one. Report it as the detector's own error, not as what the bar did.
