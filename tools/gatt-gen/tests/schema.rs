//! Reading the schema: custom options out of a descriptor set, and the model that comes out the other side.

mod common;

use common::{fixture_input, load};
use gatt_gen::model::{Bare, Perm, Prop, ValueType};
use gatt_gen::{descriptor, Error};

#[test]
fn the_example_schema_becomes_the_services_it_declares() {
    let (gatt, warnings) = load("valid/device.proto").expect("the example schema is valid");

    let names: Vec<&str> = gatt
        .services
        .iter()
        .map(|s| s.proto_name.as_str())
        .collect();
    assert_eq!(names, ["fixture.Battery", "fixture.Device"]);
    assert!(gatt.services.iter().all(|s| s.primary));

    let level = gatt.characteristic("fixture.Battery", "Level").unwrap();
    assert_eq!(level.uuid, "00002a19-0000-1000-8000-00805f9b34fb");
    assert_eq!(
        level.props.iter().copied().collect::<Vec<_>>(),
        [Prop::Read, Prop::Notify]
    );
    assert_eq!(level.response, ValueType::Bare(Bare::U8));
    assert!(level.request.is_empty());
    assert_eq!(level.max, Some(100));

    // The rpc's two directions are the two halves of the characteristic: the request is what the central writes, the response what it reads.
    let rx = gatt.characteristic("fixture.Device", "Rx").unwrap();
    assert_eq!(
        rx.request,
        ValueType::Message("fixture.Command".to_string())
    );
    assert!(rx.response.is_empty());
    assert_eq!(rx.pipe.as_deref(), Some("main"));
    assert_eq!(rx.max_len, 0);

    let info = gatt.characteristic("fixture.Device", "Info").unwrap();
    assert_eq!(
        info.response,
        ValueType::Message("fixture.DeviceInfo".to_string())
    );
    assert_eq!(info.max_len, 32);
    assert_eq!(info.perm, Perm::None);

    // Only the max_len warning: the example is otherwise a clean schema.
    assert_eq!(warnings.len(), 1, "{warnings:?}");
    assert!(warnings[0].contains("max_len 32 exceeds"), "{warnings:?}");
}

#[test]
fn the_pipe_pairs_the_written_half_with_the_notified_half() {
    let (gatt, _) = load("valid/device.proto").unwrap();

    let pipes = gatt.pipes();
    assert_eq!(pipes.len(), 1);
    let pipe = pipes[0];
    assert_eq!(pipe.name, "main");
    assert_eq!(pipe.to_device.name, "Rx");
    assert_eq!(pipe.to_host.name, "Tx");
}

#[test]
fn the_descriptor_set_is_the_interface_between_the_two_back_ends() {
    let out = std::env::temp_dir().join("gatt-gen-descriptor-set-test.fdset");
    descriptor::descriptor_set(&fixture_input("valid/device.proto"), &out)
        .expect("protoc accepts the example");

    // The bytes on disk have to describe the same schema as reading the proto directly: the other language's generator reads the descriptor set, not the proto.
    let (from_set, _) = descriptor::load_from_descriptor_set(&out).unwrap();
    let (from_proto, _) = load("valid/device.proto").unwrap();
    assert_eq!(from_set.to_string(), from_proto.to_string());

    let _ = std::fs::remove_file(&out);
}

#[test]
fn a_schema_carrying_its_own_vocabulary_is_checked_against_the_shipped_one() {
    // A vocabulary that renumbers the properties still decodes, and means something else, so it has to be caught rather than believed.
    let dir = std::env::temp_dir().join("gatt-gen-vocabulary-mismatch");
    std::fs::create_dir_all(&dir).unwrap();
    let renumbered = gatt_gen::OPTIONS_PROTO.replace("READ = 1;", "READ = 9;");
    assert_ne!(renumbered, gatt_gen::OPTIONS_PROTO);
    std::fs::write(dir.join("gatt_options.proto"), renumbered).unwrap();

    let mut input = fixture_input("valid/device.proto");
    // Ahead of the crate's own copy, which is what a consumer vendoring the vocabulary looks like.
    input.includes.insert(0, dir.clone());

    match gatt_gen::load(&input) {
        Err(Error::Descriptor(message)) => {
            assert!(message.contains("gatt.Prop = 1"), "{message}");
        }
        other => panic!("expected the vocabulary to be rejected, got {other:?}"),
    }

    let _ = std::fs::remove_dir_all(&dir);
}
