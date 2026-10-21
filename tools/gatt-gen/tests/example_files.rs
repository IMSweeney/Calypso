//! The example's generated files are what the tools write.
//!
//! `examples/device_server/` is two generated files and one hand-written example.
//! They are committed because the example is what compiles the generator's output,
//! and a file has to exist before the example can be built — so the test checks
//! them rather than producing them, and writes them when asked:
//!
//! ```sh
//! UPDATE_GENERATED=1 cargo test --test example_files
//! ```
//!
//! The same shape as `flutter test --update-goldens`, which this repo already uses
//! for images: a generated artefact that is committed, checked, and updated on
//! purpose.

use std::path::{Path, PathBuf};

use gatt_gen::generator::server::rust_trouble::{self, Config};
use gatt_gen::Input;

/// The fixture schema: a bare characteristic, a protobuf one, and a pipe.
const SCHEMA: &str = "tests/fixtures/valid/device.proto";

/// What the header of the generated table says, so the test names it the same way.
const REGENERATE_WITH: &str = "UPDATE_GENERATED=1 cargo test --test example_files";

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn example_dir() -> PathBuf {
    root().join("examples/device_server")
}

/// The configuration the committed table was written with. It is the shape a
/// device would use: one chunk per notification at a 247-byte MTU, a pipe that
/// carries a few kilobytes, and a window that suits a phone.
fn table_config() -> Config {
    Config {
        message_module: "crate::pb::msg".to_string(),
        server_name: "GattServer".to_string(),
        pipe_chunk_max: 244,
        pipe_msg_max: 4096,
        window: 8,
        xfer_timeout_ms: 5000,
        regenerate_with: Some(REGENERATE_WITH.to_string()),
    }
}

#[test]
fn the_example_table_is_what_the_generator_writes() {
    let (schema, warnings) =
        gatt_gen::load(&Input::new([root().join(SCHEMA)])).expect("the fixture schema is valid");
    assert_eq!(
        warnings.len(),
        1,
        "it warns once, about max_len: {warnings:?}"
    );

    let generated = rust_trouble::generate(&schema, &table_config()).expect("it emits");
    assert_eq!(generated.file_name, "gatt.rs");
    check_or_update(&example_dir().join(generated.file_name), &generated.source);
}

#[test]
fn the_example_messages_are_what_micropb_writes() {
    if std::env::var_os("PROTOC").is_none() {
        if let Ok(path) = protoc_bin_vendored::protoc_bin_path() {
            std::env::set_var("PROTOC", path);
        }
    }
    let out = std::env::temp_dir().join(format!(
        "gatt-gen-example-messages-{}.rs",
        std::process::id()
    ));

    let mut generator = micropb_gen::Generator::new();
    generator.use_container_heapless();
    // protoc resolves imports against `-I` paths only, and needs one that
    // prefixes the absolute file name it is given as well: the fixture's own
    // directory, then the vocabulary the schema imports, which ships with this
    // crate.
    generator.add_protoc_arg(format!(
        "-I{}",
        root().join("tests/fixtures/valid").display()
    ));
    generator.add_protoc_arg(format!("-I{}", root().join("proto").display()));

    generator.configure(
        ".fixture",
        micropb_gen::Config::new().max_bytes(64).max_len(8),
    );
    generator
        .compile_protos(
            &[root().join(SCHEMA).to_str().expect("utf-8")],
            out.to_str().expect("utf-8"),
        )
        .expect("micropb emits");

    let text = std::fs::read_to_string(&out).expect("it wrote the file");
    let _ = std::fs::remove_file(&out);
    check_or_update(&example_dir().join("messages.rs"), &text);
}

/// Compare against what is committed, or write it when the environment says so.
fn check_or_update(path: &Path, generated: &str) {
    if std::env::var_os("UPDATE_GENERATED").is_some() {
        std::fs::write(path, generated).expect("the file is writable");
        println!("wrote {}", path.display());
        return;
    }
    let committed = std::fs::read_to_string(path).unwrap_or_default();
    assert_eq!(
        committed,
        generated,
        "{} is not what the tools write: run {REGENERATE_WITH}",
        path.display()
    );
}
