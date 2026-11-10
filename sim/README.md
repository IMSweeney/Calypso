# calypso-sim

Simulation and fixture tooling: synthetic barbell motion through the real detector,
and the `.pblog` fixtures the app is developed and tested against.

It depends on `calypso-core` and nothing in the firmware depends on it, so the device's
library carries no generator it never calls. The pipeline it drives is the real one —
the same `Detector`, `compute_rep` and record encoders the firmware runs — so a fixture
cannot describe motion the device would produce differently.

```sh
cargo test
cargo run --example gen_fixtures     # writes calypso-app/assets/fixtures/
cargo run --example replay_dump -- <path.pblog>
cargo run --example detector_report # every phase, counted
cargo run --example detector_map -- bench        # draw where a synthetic rep is stored
cargo run --example false_positive_map           # draw where motion that is not a rep is stored
```

`detector_report` is the one command that runs all of it: the same cells the maps draw, counted
per slice and per motion, and the canonical sets measured. The maps remain the drawing; the report
is the same grid read as numbers, so the two cannot disagree about what the detector did.

[`docs/FIXTURES.md`](../docs/FIXTURES.md) is what a fixture is for and how the set is
added to; [`docs/DETECTOR_TESTS.md`](../docs/DETECTOR_TESTS.md) is the detector's
tolerance suite.
