//! Generates the messages and the table, pipe glue included.
//!
//! The same two calls as `examples/server`, with a schema that declares a pipe —
//! which is the difference that matters here: the generated table gains a module
//! of constants and codecs for it (`pipe_bulk`), and `gatt-pipe` drives them.

use std::path::Path;

use gatt_gen::generator::server::rust_trouble::{self, Config};
use gatt_gen::Input;

/// The schema: one service, both of whose characteristics belong to one pipe.
const PROTO: &str = "proto/pipe.proto";

fn main() {
    println!("cargo:rerun-if-changed={PROTO}");

    if std::env::var_os("PROTOC").is_none() {
        if let Ok(path) = protoc_bin_vendored::protoc_bin_path() {
            std::env::set_var("PROTOC", path);
        }
    }

    let out = std::env::var("OUT_DIR").expect("OUT_DIR is set by cargo");
    let out = Path::new(&out);

    let vocabulary = out.join("vocabulary");
    gatt_gen::write_options_proto(&vocabulary).expect("the vocabulary is writable");

    messages(out, &vocabulary);
    table(out);
}

fn messages(out: &Path, vocabulary: &Path) {
    let mut generator = micropb_gen::Generator::new();
    generator.use_container_heapless();
    generator.add_protoc_arg("-I.");
    generator.add_protoc_arg(format!("-I{}", vocabulary.display()));
    // A message carries a payload, so its containers are bounded generously
    // enough for a transfer to be worth making.
    generator.configure(".bulk", micropb_gen::Config::new().max_bytes(256));

    let dest = out.join("messages.rs");
    generator
        .compile_protos(&[PROTO], dest.to_str().expect("utf-8"))
        .expect("the messages are generated");
}

fn table(out: &Path) {
    let (schema, warnings) =
        gatt_gen::load(&Input::new([PROTO])).unwrap_or_else(|err| panic!("{err}"));
    for warning in &warnings {
        println!("cargo:warning={warning}");
    }

    let generated = rust_trouble::generate(
        &schema,
        &Config {
            message_module: "crate::pb::msg".to_string(),
            server_name: "Server".to_string(),
            // What the pipe is sized with. The chunk ceiling is what one
            // notification can hold at a 247-byte MTU; the message ceiling is
            // what the reassembly buffer holds, and the runtime asserts the
            // encoded message against it at compile time.
            pipe_chunk_max: 244,
            pipe_msg_max: 1024,
            window: 8,
            xfer_timeout_ms: 5000,
            regenerate_with: Some("cargo build".to_string()),
        },
    )
    .expect("the schema is valid");

    std::fs::write(out.join(generated.file_name), generated.source).expect("the table is writable");
}
