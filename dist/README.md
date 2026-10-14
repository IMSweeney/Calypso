# dist

Pre-built, deployable firmware packages, one folder per chip family.

- `esp32/` — original ESP32 (Xtensa, `xtensa-esp32-none-elf`), e.g. the
  ESP32-PICO-D4 this project targets.

Each chip-family folder contains a merged flash image (`*.bin`) and a
`flash.sh` script to deploy it.

## Build

From the repo root:

```
cd calypso
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
`cd calypso && cargo run` (builds, flashes, and monitors in one step).
