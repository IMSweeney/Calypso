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
flutter test                 # 85 tests, no device, no simulator
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

## Codegen

The Dart message classes are **checked in**, so a fresh checkout builds and tests
without `protoc`. Regenerate them only when `proto/calypso.proto` changes:

```sh
./tool/gen_proto.sh          # needs protoc + `dart pub global activate protoc_plugin`
```

Fixtures are generated from the core crate, not written by hand:

```sh
cd .. && cargo run -p calypso-core --example gen_fixtures
```

See [`docs/FIXTURES.md`](../docs/FIXTURES.md) for what each fixture is for, and
[`docs/APP_SYNC.md`](../docs/APP_SYNC.md) for the sync protocol and the archive.
