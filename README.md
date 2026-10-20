# Project Calypso

Calypso records barbell reps on a small ESP32 sensor and syncs them to a phone
app that plots the bar path in 3D — so a squat that dipped to one side, or a
lifter who leaned forward, is visible afterwards.

| Component | What it is |
|---|---|
| `calypso-esp32/` | Firmware for the original ESP32 (Xtensa). Samples the IMU at 100 Hz, detects reps, stores them in flash, serves them over BLE. |
| `calypso-core/` | All the platform-independent logic: sampling, rep detection, trace maths, wire format, flash log, GATT table. No embedded dependencies, so it compiles and is unit-tested on the host. |
| `calypso-app/` | The Flutter client: syncs reps over BLE, lists them, plots one in 3D. |
| `proto/` | `calypso.proto` — the single source of truth for the wire format. The firmware generates Rust from it, the app generates Dart. |
| `dist/` | Deployable firmware images, one folder per chip family, each with a `flash.sh`. Images are build artifacts, not committed. |

## Quick start

Toolchain, once:

```sh
cargo install espup && espup install   # ESP32 (Xtensa) toolchain; then `source ~/export-esp.sh` per shell
cargo install espflash                 # flashing + serial monitor
# Flutter stable is needed for calypso-app (https://docs.flutter.dev/get-started/install)
```

Then, per component (details in each component's README):

```sh
cd calypso-core && cargo test
cd calypso-esp32 && cargo run  # builds, flashes and monitors the device
cd calypso-app && flutter test
```

`cargo test` does not work in `calypso-esp32`: the target is `no_std` and Rust's
test harness needs `std`. Testable logic lives in `calypso-core`, which is where
the firmware tests run from.

## Formatting

Rust is formatted with `cargo fmt`, Dart with `dart format` — the ones the
toolchains ship, with no project config to disagree about. A pre-commit hook
checks the components a commit touches, and blocks on drift:

```sh
git config core.hooksPath .githooks   # once per clone
```

It only *checks*. Run `cargo fmt` in the crate (or `dart format .` in
`calypso-app`), re-stage, and commit again — a hook that rewrote the files would
leave the index holding the bytes it just replaced, so the commit would go in
unformatted while the working tree looked right. `git commit --no-verify` skips
it; so does a clone that has not run the config above.

