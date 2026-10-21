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
        ..Config::default()
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

    // A written bare value: a typed write, and no stream.
    assert!(
        source.contains("Future<void> writeBeep(int value)"),
        "{source}"
    );
    assert!(
        source.contains("pb.DeviceInfo.fromBuffer(bytes)"),
        "{source}"
    );

    // The pipe's two halves get no members at all, which is the point: a chunk is
    // not a message. See `a_pipe_gets_the_glue_its_runtime_reads`.
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

    // `Beep` offers write-without-response only, and asks for exactly that: the
    // one write that reports failure is not available on it.
    assert!(source.contains("withResponse: false,"), "{source}");
    // Nothing else in this schema is written — the pipe's written half is the
    // runtime's — so the flag appears once.
    assert_eq!(source.matches("withResponse:").count(), 1, "{source}");
}

#[test]
fn a_pipe_gets_the_glue_its_runtime_reads() {
    let (gatt, _) = load("valid/device.proto").unwrap();
    let config = Config {
        messages_import: "../protocol/device.pb.dart".to_string(),
        pipe_msg_max: 4096,
        ..Config::default()
    };
    let source = dart_universal_ble::generate(&gatt, &config).unwrap().source;

    // One class per pipe, holding the pair and the numbers the runtime was
    // configured with.
    assert!(source.contains("class MainPipe {"), "{source}");
    assert!(
        source.contains("static const String downUuid = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';"),
        "{source}"
    );
    assert!(
        source.contains("static const int chunkMax = 244;"),
        "{source}"
    );
    assert!(
        source.contains("static const int messageMax = 4096;"),
        "{source}"
    );
    assert!(source.contains("static const int window = 8;"), "{source}");
    assert!(
        source.contains("static const int xferTimeoutMs = 5000;"),
        "{source}"
    );

    // The codecs, so the runtime never names a message class.
    assert!(
        source.contains("static Uint8List encodeRequest(pb.Command message)"),
        "{source}"
    );
    assert!(
        source.contains("static pb.Event decodeResponse(Uint8List bytes)"),
        "{source}"
    );

    // And *no* typed members for the pipe's two characteristics: a chunk is not a
    // message, so a stream over them would decode a fragment as one.
    assert!(!source.contains("rxUpdates"), "{source}");
    assert!(!source.contains("txUpdates"), "{source}");
    assert!(!source.contains("subscribeTx"), "{source}");
    assert!(!source.contains("writeRx"), "{source}");
}
