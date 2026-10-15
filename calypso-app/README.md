# calypso-app

The client side: an iOS app that syncs reps from the device over BLE, lists rep
summaries, and plots a single rep in 3D.

**Status: planning.** No code yet — the shell/rendering decision is still open,
and the M0 spike in the plan is what settles it.

See [`docs/app/README.md`](../docs/app/README.md) for the plan: goals, the
client shell options and recommendation, the planned crate layout, the fixture
format that makes the app testable without hardware, features, testing strategy,
milestones, and the open questions for review.

Once the spike lands, this directory gets:

- `app-core/` — Rust, no UI, no BLE; host-tested (`cargo test`)
- `app-ble/` — `RepTransport` implementations (btleplug + fixture/replay)
- `app-ios/` — uniffi surface (staticlib) + Xcode project
- `app-desktop/` — host binary for fast iteration (fixtures; real BLE on macOS)
- `fixtures/` — checked-in sample logs
