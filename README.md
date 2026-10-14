# Project Calypso

## Tooling

### First time setup
- `cargo install espup`
- `espup install`
- `~/export-esp.sh`
- `cargo install espflash`
- probe-rs?

### Flashing
- `cargo build`
- `cargo run`

### Testing
`cargo test` in `calypso/` does not work: the firmware target
(`xtensa-esp32-none-elf`) is `no_std`, and Rust's test harness requires `std`.
Platform-independent logic lives in the `calypso-core` crate instead, which is
tested on the host:
- `cd calypso-core && cargo test`

For on-device tests, look at [`embedded-test`](https://crates.io/crates/embedded-test).

## TODO
- [X] Save calibration to memory between power cycles
- [X] Simple level sensor along one axis
- [ ] Export data over BLE [Example](https://github.com/esp-rs/esp-hal/blob/main/examples/ble/bas_peripheral/src/main.rs)
- [ ] OTA code updates [Example](https://github.com/esp-rs/esp-hal/blob/main/examples/ota/update/src/main.rs)
- [ ] Storing path/time data. (how much can we store?)
- [ ] Detecting reps, calculating rep stats (speed, distance)
- [ ] Sending a rep's data over BLE
- [ ] Displaying the rep in 3d
