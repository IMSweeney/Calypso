# dist

Deployable firmware packages, one folder per chip family. Flash images
(`*.bin`) are build artifacts: they are generated locally (see below) and
not committed to git.

- `esp32/` — original ESP32 (Xtensa, `xtensa-esp32-none-elf`), e.g. the
  ESP32-PICO-D4 this project targets. Firmware source lives in
  `calypso-esp32/`.

Each chip-family folder contains a `flash.sh` script to deploy the generated
image.

## Build

From the repo root:

```
cd calypso-esp32
source ~/export-esp.sh    # one-time per shell; see "First time setup" in the top-level README
cargo build --release
espflash save-image --chip esp32 --merge \
  target/xtensa-esp32-none-elf/release/calypso ../dist/esp32/calypso.bin
```

## Test

The firmware target cannot run `cargo test` (it is `no_std`). Test the
platform-independent logic on the host instead:

```
cd calypso-core
cargo test
```

## Deploy

With the board connected over USB:

```
cd dist/esp32
./flash.sh            # writes calypso.bin, then starts the serial monitor
```

or equivalently `espflash write-bin --chip esp32 0x0 calypso.bin` followed by
`espflash monitor`.

When iterating on source, flashing from the firmware crate is simpler:
`cd calypso-esp32 && cargo run` (builds, flashes, and monitors in one step).
