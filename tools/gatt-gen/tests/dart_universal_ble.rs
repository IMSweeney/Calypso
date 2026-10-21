//! What the `dart-universal-ble` generator emits, checked against the example
//! schema.
//!
//! The output is Dart, so nothing here compiles it: the app that consumes it does
//! that (`calypso-app` has the generated client under `flutter analyze`, and a
//! test that drives it over a fake link). What is checked here is the shape the
//! schema implies — which methods exist, which do not, and what they say.

mod common;

use common::load;
use gatt_gen::generator::client::dart_universal_ble::{self, Config};
use gatt_gen::generator::{Config as TargetConfig, Target};

fn generated() -> String {
    let (gatt, _) = load("valid/device.proto").unwrap();
    let config = Config {
        messages_import: "../protocol/device.pb.dart".to_string(),
        messages_prefix: "pb".to_string(),
        regenerate_with: None,
    };
    dart_universal_ble::generate(&gatt, &config).unwrap().source
}

#[test]
fn the_file_is_named_by_the_target() {
    let (gatt, _) = load("valid/device.proto").unwrap();
    let output = Target::DartUniversalBle
        .generate(&gatt, &TargetConfig::DartUniversalBle(Config::default()))
        .expect("the dart target is written now");

    assert_eq!(output.file_name, "gatt_client.dart");
    assert!(output
        .source
        .starts_with("// GENERATED FILE -- DO NOT EDIT."));
}

#[test]
fn a_characteristic_gets_the_members_its_properties_imply() {
    let source = generated();

    // READ + NOTIFY on a bare u8: a stream of ints and a one-shot read, and no
    // write of any kind.
    assert!(source.contains("Stream<int> get levelUpdates"), "{source}");
    assert!(source.contains("Future<int> readLevel()"), "{source}");
    assert!(source.contains("Future<void> subscribeLevel()"), "{source}");
    assert!(!source.contains("writeLevel"), "{source}");

    // The pipe halves: a write of the request message, a stream of the other.
    assert!(
        source.contains("Future<void> writeRx(pb.Command value)"),
        "{source}"
    );
    assert!(
        source.contains("Stream<pb.Event> get txUpdates"),
        "{source}"
    );
    assert!(
        source.contains("pb.DeviceInfo.fromBuffer(bytes)"),
        "{source}"
    );
}

#[test]
fn the_uuids_and_the_link_come_from_the_schema() {
    let source = generated();

    assert!(
        source
            .contains("static const String serviceUuid = '0000180f-0000-1000-8000-00805f9b34fb';"),
        "{source}"
    );
    assert!(
        source.contains("static const String levelUuid = '00002a19-0000-1000-8000-00805f9b34fb';"),
        "{source}"
    );
    // One class per service, and the interface the app implements over
    // `universal_ble`.
    assert!(source.contains("class BatteryClient"), "{source}");
    assert!(source.contains("abstract class GattClientLink"), "{source}");
    assert!(
        source.contains("import '../protocol/device.pb.dart' as pb;"),
        "{source}"
    );
}

#[test]
fn the_write_flavour_follows_the_declared_properties() {
    let source = generated();

    // `Rx` offers write-without-response only, and asks for exactly that: the
    // one write that reports failure is not available on it.
    assert!(source.contains("withResponse: false,"), "{source}");
    // Nothing in this schema is written with a response, so the flag appears
    // once, on the one written characteristic.
    assert_eq!(source.matches("withResponse:").count(), 1, "{source}");
}
