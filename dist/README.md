# dist

Deployable firmware packages, one folder per chip family. Flash images (`*.bin`)
are build artifacts: generated locally (see below) and not committed.

- `esp32/` — original ESP32 (Xtensa, `xtensa-esp32-none-elf`), e.g. the
  ESP32-PICO-D4 this project targets. Firmware source lives in `calypso-esp32/`.

Each folder holds a `flash.sh` to deploy the image beside it.

## Build

From the repo root:

```
cd calypso-esp32
source ~/export-esp.sh    # once per shell; see the top-level README's "Quick start"
cargo build --release
espflash save-image --chip esp32 --merge \
  --partition-table partitions.csv \
  target/xtensa-esp32-none-elf/release/calypso ../dist/esp32/calypso.bin
```

The `--partition-table` flag bakes `calypso-esp32/partitions.csv` — which defines
the `log` data partition the rep log uses — into the merged image. **The first
flash with this table repartitions the device**; `nvs` keeps its default offset,
so the stored BNO055 calibration survives.

## Deploy

With the board connected over USB:

```
cd dist/esp32
./flash.sh            # writes calypso.bin, then starts the serial monitor
```

or equivalently `espflash write-bin --chip esp32 0x0 calypso.bin` followed by
`espflash monitor`. The merged image includes the partition table, so `write-bin`
applies the repartition too.

When iterating on source, flashing from the firmware crate is simpler:
`cd calypso-esp32 && cargo run` (builds, flashes and monitors in one step).
