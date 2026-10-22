# Working on Calypso

Rules for changes here.

## Code

- **Comments only where the code cannot say it.** When in doubt, delete.
- **One thought per line.** Never wrap a sentence across two lines. The width differs in every editor.
- **Comments should not point to `docs`.** "See [doc]" tells the reader nothing here and is likely to become stale.
- **Fail fast.** An unexpected error crashes in dev, an expected one warns and retries, and nothing is ever swallowed silently.
- **In async, throw.** The code that owns the future decides whether to retry or rethrow.
- **One source of schema.** Anything the device and the app both read goes in `proto/`, and the generated types are used rather than converted (unless performance dictates otherwise).

## Tests

- **The expected value has to come from outside the code under test.** 
  - **round trip**: encode with the real encoder, read back with the real reader.
  - **recorded or synthetic data through the real algorithm**: asserting what the data says, with a tolerance rather than a stored output blob.
  - **an algorithm's boundaries and promises**: e.g. `decimation_keeps_the_last_point`.
  - **a value the outside world defines**, asserted by number, because the test is the only place it is pinned. e.g. the BLE AD types and flag bits in `advertises_the_service_and_the_name_together`.
- **A test cannot restate what the code already declares.**
- **No tests for what the product cannot do.**
- **A test is not where a process step gets enforced.**: e.g. test that generated files were generated
- **Another crate's behaviour should live in another crate's test.**

## Docs

- **`ROADMAP.md` is scope and status.** `Done` holds features only. Learnings are a `GOTCHAS.md`. `Open` holds only what is not done.
- **READMEs and `docs/*` describe the system**: the interface, the formats, the commands. Symptom → cause → fix belongs in `docs/GOTCHAS.md`.
- **No volatile facts**: test counts, line counts, sizes that move. Say what it is, not how many.
