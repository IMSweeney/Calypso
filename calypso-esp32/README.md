# calypso-esp32

Calypso firmware for the original ESP32 (Xtensa, target
`xtensa-esp32-none-elf`). All code in here is specific to this chip family;
platform-independent logic lives in `calypso-core/` (a path dependency).

## Build

```
source ~/export-esp.sh   # one-time per shell; see the top-level README
cargo build
```

## Flash

```
cargo run                # builds, flashes via espflash, and opens the monitor
```

## Test

`cargo test` does not work here: the target is `no_std` and Rust's test
harness requires `std`. Host-testable logic lives in `calypso-core/`:

```
cd ../calypso-core && cargo test
```

## Deploy

To produce a standalone flash image for distribution, see `dist/README.md`.
