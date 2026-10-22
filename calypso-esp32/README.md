# calypso-esp32

Calypso firmware for the original ESP32 (Xtensa, target `xtensa-esp32-none-elf`),
e.g. the ESP32-PICO-D4 this project targets.

It samples a BNO055 IMU over I²C at 100 Hz, detects reps, computes and stores a
trace per rep in a flash log, and serves summaries and traces to a phone over BLE.
Everything in here is chip or board specific; the logic lives in `calypso-core/`.

## Build

```sh
source ~/export-esp.sh   # once per shell; see "Quick start" in the top-level README
cargo build              # or: cargo build --release
```

The toolchain is pinned by `rust-toolchain.toml` to the `esp` channel installed by
`espup`.

## Flash and monitor

```sh
cargo run                # builds, flashes with espflash, and opens the monitor
```

That is the normal dev loop; `cargo run --release` for an optimised build. It
writes the partition table along with the bootloader and the app, so the `log`
partition (see `partitions.csv`) exists on the device afterwards and the rep log
can mount. The first flash repartitions the board; NVS keeps its offsets, so the
stored IMU calibration survives.

For a standalone image to hand to someone else, see [`dist/README.md`](../dist/README.md).

## Bench testing without lifting

```sh
cargo run --features fake-reps
```

Stores a synthetic "rep" every 10 s — the last 8 s of samples, whatever the motion
was — instead of running the detector. Everything downstream is untouched, so this
exercises the whole path on a bench with nobody lifting: the flash log, the
summaries, the trace streaming, and the app syncing and plotting them.

- The calibration gate is deliberately not applied, so reps appear at a steady
  rate rather than at the chip's opinion of itself. Real builds still gate.
- It says so: one `fake-reps: storing a synthetic rep every 10 s, not detecting
  real ones` line at the start, so a bench session cannot be mistaken for a real
  one in a log.
- It is a cargo feature, not a runtime setting: a device should not be one
  accidental switch away from recording fiction as a measurement.

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

Expected clean. `clippy::large_stack_frames` is **denied** here with a 1 KB threshold
(`.clippy.toml`), and `clippy::mem_forget` is denied in `main.rs`. `main` itself is
exempted from the stack lint, so a large local added there will not be caught
automatically.

## The boot log

`cargo run` opens the monitor, and the firmware's log is the only window into the
device: heap at each setup stage, the stack figures, and the 1 Hz sampler line.
[`docs/GOTCHAS.md`](../docs/GOTCHAS.md) says what the numbers should be and what
each failure mode looks like.
