# calypso-esp32

Calypso firmware for the original ESP32 (Xtensa, target `xtensa-esp32-none-elf`), e.g. the ESP32-PICO-D4 this project targets.

It samples a BNO055 IMU over I²C at 100 Hz, detects reps, computes and stores a trace per rep in a flash log, and serves summaries and traces to a phone over BLE. Everything in here is chip or board specific; the logic lives in `calypso-core/`.

## Build

```sh
source ~/export-esp.sh   # once per shell; see "Quick start" in the top-level README
cargo build              # or: cargo build --release
```

The toolchain is pinned by `rust-toolchain.toml` to the `esp` channel installed by `espup`.

## Flash and monitor

```sh
cargo run                # builds, flashes with espflash, and opens the monitor
```

That is the normal dev loop. `cargo run --release` for an optimised build.

This writes the partition table along with the bootloader and the app. The firmware's `log` partition (see `partitions.csv`) must exist on the device or the rep log refuses to mount. The first flash repartitions the board. NVS keeps its offsets, so the stored device name and IMU calibration survive.

For a standalone image to hand to someone else, see [`dist/README.md`](../dist/README.md).

## Test

```sh
cargo test      # does not work here
```

The firmware target is `no_std` and Rust's test harness requires `std`, so host
tests run from the core crate instead:

```sh
cd ../calypso-core && cargo test
```

## Lint

```sh
cargo clippy
```

Expected clean. `clippy::large_stack_frames` is **denied** here with a 1 KB threshold (`.clippy.toml`) and `clippy::mem_forget` is denied in `main.rs`. `main` itself is exempted from the stack lint, so a large local added there will not be caught automatically — see [`docs/GOTCHAS.md`](../docs/GOTCHAS.md).

## Reading the boot log

A few figures are printed for exactly this purpose, and they are the first thing to look at when something is wrong on hardware:

- `Radio up, heap free N B`, `Rep detector ready, heap free N B`,
  `Sync buffers ready, heap free N B` — how much heap is left at each stage.
- `Stack in use before the tasks: N B` — `main`'s frame.
- `BLE connection established (stack N B)` — depth inside the GATT task.
- the 1 Hz sampler line, which carries `stack N B` beside the heap figure.

[`docs/GOTCHAS.md`](../docs/GOTCHAS.md) says what the numbers should be and what
each failure mode looks like.
