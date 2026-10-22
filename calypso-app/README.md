# calypso-app

The Calypso client: a Flutter app that syncs reps from the device over BLE,
lists them, and plots a single rep's bar path in 3D.

Status: **working**, through the BLE transport. The remaining gap is a capture
from real hardware — every path below can be exercised on the Mac with fixtures.

## Build and run

```sh
flutter pub get

flutter run -d macos                  # fastest loop; CoreBluetooth works on the Mac
flutter run -d <iphone>               # real device
```

The app starts on the archive, seeded from a fixture so there is something to
look at with no hardware attached:

```sh
flutter run -d macos --dart-define=FIXTURE=dip-left   # any name from assets/fixtures
flutter run -d macos --dart-define=FIXTURE=           # empty: start with an empty archive
```

The **iOS Simulator has no CoreBluetooth at all**, so BLE only works on a real device or the Mac. Development against fixtures needs neither.

## Test

```sh
flutter test                 # 103 tests, no device, no simulator
flutter analyze              # expected clean
```

The suite covers the whole data path without hardware: `.pblog` decoding, trace
reassembly, the sync client, the plot model, the archive's torn-tail recovery,
the rep list and plot screens (widget tests), and `BleTransport` against a fake
link. Golden images pin the rendering:

```sh
flutter test --update-goldens   # after an intentional visual change; review the diff
```

Goldens are host-dependent (anti-aliasing differs between machines), so a golden
failure on a different machine is expected rather than alarming.

## Logging

The app's logs go to a rotating file: `calypso.log` under the application support
directory, 10 MB per file, three kept (`calypso.log.1`, `.2`). On macOS (sandboxed)
that is `~/Library/Containers/com.calypso.calypso/Data/Library/Application Support/
com.calypso.calypso/`; the app prints the full path as `calypso: log at …` when it
starts. It is turned on by `lib/logging/setup.dart` — and a test that never calls
that writes no file at all (see `docs/GOTCHAS.md`).

BLE is the only writer so far, at two levels, both under the `ble` logger:

- `UniversalBleLink` logs `raw` — every value the radio delivered, *before* the
  transport's per-characteristic streams filtered it, plus each scan result and
  connection change;
- `BleTransport` logs the session (`scanning`, `connecting`, `subscribed`,
  `connected`, `connection lost`) and each message it handled (`rx`, `tx`).

Bytes at the seam, and words above it: `describeMessage`
(`lib/logging/loggers.dart`) prints the frames, and the transport prints what it made
of them.

```
2026-10-18T20:41:03.482119 INFO    ble subscribed to 0000ca21-b0cd-11ec-871f-d45ddf138840
2026-10-18T20:41:03.631902 FINE    ble raw 0000ca21-b0cd-11ec-871f-d45ddf138840 13B 01 01 0c 0a 0a 00 …
2026-10-18T20:41:03.631955 FINE    ble rx 0000ca21-b0cd-11ec-871f-d45ddf138840 summary rep=…
2026-10-18T20:41:04.010004 FINE    ble rx 0000ca24-b0cd-11ec-871f-d45ddf138840 calibration sys=3 accel=3 gyro=3 mag=0 ready=true
```

The `raw`/`rx` pair is the point of the split: `raw` without a matching `rx` means
the plugin dropped the value, and no `raw` at all means the device never sent it.
The first line of the file is written at startup, so an empty log means logging is
broken rather than nothing having happened.

The calibration line has no `raw` above it on purpose: it is a **read** rather than a
notification, so it answers the app's own request or throws — there is nothing to
arrive out of the blue (`docs/APP_SYNC.md`).

## Codegen

The Dart message classes are **checked in**, so a fresh checkout builds and tests
without `protoc`. Regenerate them only when `proto/calypso.proto` changes:

```sh
./tool/gen_proto.sh          # needs protoc + `dart pub global activate protoc_plugin`
```

neither of which may be on your `PATH`. The plugin goes to `~/.pub-cache/bin`, and a
`protoc` the Rust build already fetched is under `~/.cargo/registry`: 

```sh
PATH="$HOME/.pub-cache/bin:$PATH" \
PROTOC="$(find ~/.cargo/registry/src -name protoc -path '*macos*' | head -1)" \
  ./tool/gen_proto.sh
```

The BLE client is generated from the same schema by `gatt-gen`, also checked in:

```sh
./tool/gen_gatt.sh          # needs a Rust toolchain; brings its own protoc
```

`lib/gatt/gatt_client.dart` is the schema's typed client, and `lib/transport/`
is what wraps it: scanning, connecting, reassembly and the timeouts a schema
cannot describe. `calypso-core/tests/gatt_generated.rs` fails if the committed
client and `proto/calypso_gatt.proto` disagree.

Fixtures are generated from the core crate, not written by hand:

```sh
cd .. && cargo run -p calypso-core --example gen_fixtures
```

See [`docs/FIXTURES.md`](../docs/FIXTURES.md) for what each fixture is for, and
[`docs/APP_SYNC.md`](../docs/APP_SYNC.md) for the sync protocol and the archive.
