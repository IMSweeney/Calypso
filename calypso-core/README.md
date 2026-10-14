# calypso-core

Platform-independent logic for the Calypso firmware. This crate has no
embedded dependencies (`esp-hal` etc.), so it compiles and is unit-tested on
the host with the standard Rust toolchain — the firmware target
(`xtensa-esp32-none-elf`) is `no_std` and cannot run Rust's test harness.

Anything that is pure logic (data formats, codecs, sensor math, rep
detection, ...) belongs here so it gets host test coverage. Anything that
touches hardware stays in the chip-family firmware crates
(e.g. `calypso-esp32/`).

## Build

```
cargo build
```

The crate is `#![no_std]` when built normally, so it can be linked into the
firmware (`calypso-esp32/` depends on it via a path dependency).

## Test

```
cargo test
```

Tests run on the host (macOS/Linux) with the stock toolchain — no espup or
ESP32 needed.

This includes integration tests (`tests/gatt_table.rs`) that act as a mock
BLE client: the trouble-host attribute table is platform-independent, so the
tests build the exact GATT table the firmware serves and read/write
characteristics the way a phone would (rename the device, read back the
value, check the persistence codec round-trips, verify the notify CCCD
exists).

What this does **not** cover: the radio itself (advertising, connections).
QEMU/Wokwi emulate the ESP32 CPU but not its BLE radio, and trouble-host's
own integration tests require real HCI hardware — so end-to-end radio
behavior still needs a device (see `embedded-test` for on-device tests).

## Deploy

This crate is a library; it is not deployed on its own. It is compiled into
the firmware image — see `dist/README.md` for firmware build/flash
instructions.
