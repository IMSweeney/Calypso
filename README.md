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

## TODO
- [X] Save calibration to memory between power cycles
- [X] Simple level sensor along one axis
- [ ] Export data over BLE [Example](https://github.com/esp-rs/esp-hal/blob/main/examples/ble/bas_peripheral/src/main.rs)
- [ ] OTA code updates [Example](https://github.com/esp-rs/esp-hal/blob/main/examples/ota/update/src/main.rs)