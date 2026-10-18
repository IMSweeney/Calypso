# Project Calypso

## Tooling

### First time setup
- `cargo install espup`
- `espup install`
- `~/export-esp.sh`
- `cargo install espflash`
- probe-rs?

## Editor setup

Open the repo in VS Code: `.vscode/extensions.json` recommends the extensions
that cover every language here — `rust-analyzer` (Rust), `Dart-Code.dart-code`
(Dart/Flutter in `calypso-app/`) and `zxh404.vscode-proto3` (`.proto`).
`.vscode/settings.json` wires up formatting and excludes build output.

Note: `calypso-esp32` targets Xtensa, so rust-analyzer needs the esp toolchain
(`source ~/export-esp.sh`) before that crate analyses cleanly; the other crates
work out of the box.

## Layout
- `calypso-esp32/` — ESP32 (Xtensa) firmware
- `calypso-core/` — platform-independent logic, unit-tested on the host
- `dist/` — deploy scripts and docs; flash images are generated, not committed
- `calypso-app/` — the iOS client (planning): architecture in
  `docs/app/README.md`, plan in `docs/app/plan.md`
- `proto/` — the protobuf wire format (`calypso.proto`), shared with the app;
  `calypso-core` generates its Rust types from it at build time (no `protoc`
  install needed — the build vendored one)

### Flashing
- `cd calypso-esp32 && cargo build`
- `cargo run`

### Testing
`cargo test` in `calypso-esp32/` does not work: the firmware target
(`xtensa-esp32-none-elf`) is `no_std`, and Rust's test harness requires `std`.
Platform-independent logic lives in the `calypso-core` crate instead, which is
tested on the host:
- `cd calypso-core && cargo test`

Client fixtures (`.pblog` log dumps the iOS app is developed against) are
generated from the same encoders the firmware uses:

- `cargo run -p calypso-core --example gen_fixtures`

For on-device tests, look at [`embedded-test`](https://crates.io/crates/embedded-test).

## TODO
- [X] Save calibration to memory between power cycles
- [X] Simple level sensor along one axis
- [X] Export data over BLE [Example](https://github.com/esp-rs/esp-hal/blob/main/examples/ble/bas_peripheral/src/main.rs)
- [ ] OTA code updates [Example](https://github.com/esp-rs/esp-hal/blob/main/examples/ota/update/src/main.rs)
- [ ] Storing path/time data. (how much can we store?)
- [ ] Detecting reps, calculating rep stats (speed, distance)
- [ ] Sending a rep's data over BLE
- [ ] Displaying the rep in 3d
- [ ] Calibration routine (if un-calibrated, BLE should still be available and calibration hint should display in UI)
