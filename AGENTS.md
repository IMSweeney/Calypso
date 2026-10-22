# Working on Calypso

Rules for changes here.

## Code

- **Comments only where the code cannot say it.** When in doubt, delete.
- **One thought per line.** Never wrap a sentence across two lines. the width differs in every editor.
- **No pointers to `docs`.** "See [doc]" tells the reader nothing here and is likely to become stale
- **Fail fast.** An unexpected error crashes in dev, an expected one warns and retries, and nothing is ever swallowed silently.
- **In async, throw.** The code that owns the future decides whether to retry or rethrow.
- **One source of schema.** Anything the device and the app both read goes in `proto/`, and the generated types are used rather than converted (unless performance dictates otherwise).

## Tests

- Brief, one behaviour each.
- Test an interface against a recording of real or synthetic data where you can.

## Docs

- **`ROADMAP.md` is scope and status.** `Done` holds features only. A learnings are a `GOTCHAS.md`. `Open` holds only what is not done.
- **READMEs and `docs/*` describe the system**: the interface, the formats, the commands. Symptom → cause → fix belongs in `docs/GOTCHAS.md`.
- **No volatile facts**: test counts, line counts, sizes that move. Say what it is, not how many.
