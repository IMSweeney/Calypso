---
name: agents-rules-pass
display-name: AGENTS.md Rules Pass
description: Use when doing a pass over the Calypso repo to apply AGENTS.md — finding features to cut, rule violations to fix, and code to simplify.
---

# AGENTS.md rules pass

One sweep over the repo where every change is justified by a rule in `AGENTS.md`: features nothing
reads are deleted, comments and docs follow the rules, and the diff is smaller at the end than at the
start. The work is mostly judgement about *what to remove*, so the user decides; you bring evidence
and do the surgery.

Paths are from the Calypso repository root.

## 1. Before touching anything

- Read `AGENTS.md` itself, not a paraphrase. Every section has a rule that will come up (Code, Tests,
  Docs), and the doc rules are the ones most often skipped.
- `git status` and `git worktree list`. **Never revert a change you did not make** — the tree is
  often dirty with someone's capture or a half-finished edit. If it is dirty, say so and ask where
  to work; the user may want a worktree, and they may not.
- Take a snapshot before any mechanical rewrite:

  ```sh
  tar czf <scratchpad>/pre-pass.tgz --exclude target --exclude .dart_tool --exclude build \
    calypso-core/src calypso-core/tests calypso-core/build.rs calypso-core/tool \
    calypso-app/lib calypso-app/test calypso-app/tool calypso-esp32/src sim tools proto
  ```

  Excluding `target` matters: without it the tarball is hundreds of megabytes. This is the only way
  back from a rewrite that went wrong, because `git checkout` would take your own edits with it.

## 2. Survey, then ask once

Do not dribble questions. Build the whole list, then ask in one go, numbered, with a recommendation
per item so the answer can be terse ("A all yes, 6 keep, 7 drop the BLE side").

Group the findings:

- **A — dead.** Zero consumers anywhere outside generated code. Recommend deleting, with the
  evidence (`0 references to ReportKind::CalibrationProfile`).
- **B — live but questionable.** Real features that may have outlived their use. One decision each,
  and say what keeping it costs.
- **C — rule violations.** Mechanical, no judgement needed. State them and proceed unless the user
  objects; flag anything with a large blast radius (a repo-wide comment rewrite is a big diff).
- **D — small simplifications.** Duplicated example binaries, unused constants, orphan files.

Where to look:

- **Proto fields and enum values**: `grep -rn` each name outside `*/protocol/*` and the generated
  GATT files. A message that exists but is never sent (`SyncStatus`) is dead even though it compiles.
- **The app**: Dart does not warn about unused public extension members. Grep each getter by name;
  `RepSummaryX.origin` and the whole `TraceRawPointX` extension were dead and silent.
- **Rust**: `cargo clippy --all-targets` finds dead code; `cargo doc --no-deps` finds doc links that
  were already broken.
- **Features**: enumerate from `proto/*.proto`, the module list, and `cfg!(feature = "…")`. Ask about
  each one that has no consumer — a build flag with no client, a characteristic nothing reads.
- **Orphans**: files nothing references. `grep -rn "<name>" --include=*.md --include=*.rs --include=*.dart`
  across the repo, not just the folder they sit in.
- **Rule violations**:
  - code comments citing `docs/` (`grep -rn "docs/" --include=*.rs --include=*.dart --include=*.proto`)
  - comment lines wrapped mid-sentence (see §4)
  - `ROADMAP.md`'s Done section: "features only", so measurement arguments belong in `docs/`
  - byte counts, test counts and line counts in docs that move with every build
  - a test that restates what the code declares: an enum listed out to loop over it, a constant
    asserted against itself, a test for a process step

## 3. Execute in batches, green after each

Order matters: the schema is the source of truth both ends read, so it goes first and the rest
follows.

1. **`proto/`** — remove the field, **reserve its number and its name**, with a one-line reason:

   ```proto
   reserved 7, 8;  // was `mean_velocity` and `peak_accel`: nothing read them
   reserved "mean_velocity", "peak_accel";
   ```

   `reserved` is not allowed inside a `oneof`; put it in the message body.
2. **Regenerate** everything derived from it (see §3a).
3. **`calypso-core`** — then `cargo test`. Fix the tests the schema change broke rather than keeping
   fields alive for them.
4. **`calypso-esp32`** — `cargo check` with the ESP toolchain; the trace request, the characteristic
   table and the log lines all land here.
5. **`calypso-app`** — `flutter analyze`, then `flutter test`. Deleting a read characteristic also
   deletes `read()` from the generated client interface and its implementation, so look for
   `override_on_non_overriding_member`.
6. **`sim`** — fixtures encode summaries, so a removed field breaks `examples/gen_fixtures.rs`.
7. **Belt and braces**: a deletion has knock-ons in the log line that printed the field, in
   `docs/`, in `README`s, and in the tests that named the deleted file. Grep the name again at the
   end; the last one is always in a doc.

### 3a. Regenerating

Nothing checks the generated files against the schema, so they have to be regenerated by hand:

```sh
./calypso-core/tool/gen_gatt.sh
./calypso-app/tool/gen_gatt.sh

cd calypso-app
export PATH="$HOME/.pub-cache/bin:$PATH"
export PROTOC=$(find ~/.cargo/registry/src -name protoc -type f -perm +111 \
  -path '*protoc-bin-vendored-macos*' | head -1)
./tool/gen_proto.sh
```

`protoc` is often not on `PATH`; the vendored copy that `gatt-gen` already pulls in is a valid one.
Re-run the generators at the end and confirm they produce no diff — that is the only evidence the
committed output matches the schema.

## 4. Unwrapping wrapped comments

`AGENTS.md` asks for one thought per line. `unwrap_comments.py` in this folder merges a comment
paragraph back into sentences:

```sh
python3 .agents/skills/agents-rules-pass/unwrap_comments.py --check <paths>   # count, no writes
python3 .agents/skills/agents-rules-pass/unwrap_comments.py <paths>           # apply
```

It joins a line into the one above while the one above does not end a sentence, and leaves code
fences, list markers, tables, numbered lists, blank comment lines and `#` comments (in `.sh` only)
alone. Two hazards matter:

- **Multi-line string literals.** A generator writing `"…\n\` continuations has lines that *look*
  like comments. The script skips any line that follows, or ends with, a trailing `\`, and refuses
  to touch `#` outside shell scripts so Rust attributes survive.
- **Scope.** Run it on the whole tree in one go rather than per file, then verify that only comment
  lines changed: strip comment lines from the snapshot and from the tree and diff them. Formatting
  differences are expected; anything else is a bug.

The script is idempotent: a second `--check` must report `0 files would change`.

It is a heuristic, and the code drifts as soon as someone writes a new wrapped comment: run
`--check` over the tree at the start of a pass to see what has crept back in since the last one.

## 5. Docs

- `ROADMAP.md` Done: a bold name, one sentence, one link. Measurements and arguments are what
  `docs/GOTCHAS.md` and the subject docs are for.
- An Open item whose subject was just deleted goes; one half-resolved by a deletion says so.
- `docs/GOTCHAS.md`: keep the symptom → cause → fix and the shape of the thing; drop the exact byte,
  frame or heap figures, which move with every build. Keep numbers that are configuration constants
  or behaviour ("under about 8 KB free is a device that will not accept a connection").
- Anything that named a removed feature is wrong the moment you delete it: the characteristic table,
  the request fields, the report list, the sample log line in a README.

## 6. Verify

```sh
(cd calypso-core && cargo test)
(cd sim && cargo test)
(cd tools/gatt-pipe && cargo test)
(cd tools/gatt-gen && cargo test)
(cd calypso-esp32 && source ~/export-esp.sh && cargo check)
(cd calypso-app && flutter analyze && flutter test)
```

Then `cargo fmt --check` in each Rust crate and `dart format --set-exit-if-changed lib test`, because
a commit hook will check exactly that.

**If a check fails, find out whether you caused it** before "fixing" it: make a throwaway worktree at
`HEAD` (`git worktree add /tmp/check HEAD`), run the same command there, and remove it. A pre-existing
failure is something to report, not to bury under an unrelated change.

## 7. Commit

- `git add -u` — tracked changes only. Untracked captures in `captures/` belong to the user.
- Subject in the repo's voice: what the change does, not "chore" or "refactor".
- Body: the removals with their evidence, then the rule fixes, then what was run. The reserved proto
  tags read as deliberate rather than as gaps because the body says why they went.
- Do not push.
