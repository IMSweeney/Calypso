# calypso-core

Platform-independent logic for the Calypso firmware. This crate has no
embedded dependencies (`esp-hal` etc.), so it compiles and is unit-tested on
the host with the standard Rust toolchain — the firmware target
(`xtensa-esp32-none-elf`) is `no_std` and cannot run Rust's test harness.

Anything that is pure logic (data formats, codecs, sensor math, rep
detection, ...) belongs here so it gets host test coverage. Anything that
touches hardware stays in `calypso/`.

## Build

```
cargo build
```

The crate is `#![no_std]` when built normally, so it can be linked into the
firmware (`calypso/` depends on it via a path dependency).

## Test

```
cargo test
```

Tests run on the host (macOS/Linux) with the stock toolchain — no espup or
ESP32 needed.

## Deploy

This crate is a library; it is not deployed on its own. It is compiled into
the firmware image — see `dist/README.md` for firmware build/flash
instructions.
