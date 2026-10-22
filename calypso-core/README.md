# calypso-core

Platform-independent logic for Calypso, shared by the firmware and the tests.

No embedded dependencies, so it builds and is unit-tested on the host with the
stock Rust toolchain. It is `#![no_std]` when built for the device — the
firmware's target, `xtensa-esp32-none-elf`, cannot run Rust's test harness — and
`std`-enabled under `cargo test`.

```sh
cargo build
cargo test
```
