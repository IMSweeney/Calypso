//! Generates the server's message types and its GATT table.
//!
//! Two generators over one schema: `micropb-gen` for the messages the
//! characteristics carry, `gatt-gen` for the table they are declared in. Both
//! write into `$OUT_DIR`, which `src/main.rs` includes — so this is the shape of
//! the integration, and it is a build dependency and two calls.

use std::path::Path;

use gatt_gen::generator::server::rust_trouble::{self, Config};
use gatt_gen::Input;

/// The schema: the messages and the table they are served in, one file.
const PROTO: &str = "proto/server.proto";

fn main() {
    println!("cargo:rerun-if-changed={PROTO}");

    // Both generators shell out to protoc. Rather than requiring a system one,
    // point them at the binary this crate's build dependency ships.
    if std::env::var_os("PROTOC").is_none() {
        if let Ok(path) = protoc_bin_vendored::protoc_bin_path() {
            std::env::set_var("PROTOC", path);
        }
    }

    let out = std::env::var("OUT_DIR").expect("OUT_DIR is set by cargo");
    let out = Path::new(&out);

    // The GATT vocabulary ships with `gatt-gen` rather than with this project.
    // `gatt-gen` puts its own copy on protoc's path for its own run; `micropb-gen`
    // has to be told where it is.
    let vocabulary = out.join("vocabulary");
    gatt_gen::write_options_proto(&vocabulary).expect("the vocabulary is writable");

    messages(out, &vocabulary);
    table(out);
}

/// The message types, into `$OUT_DIR/messages.rs`.
fn messages(out: &Path, vocabulary: &Path) {
    let mut generator = micropb_gen::Generator::new();
    generator.use_container_heapless();
    // protoc resolves imports against `-I` paths only: the working directory is
    // one, and the vocabulary is another.
    generator.add_protoc_arg("-I.");
    generator.add_protoc_arg(format!("-I{}", vocabulary.display()));
    // Every container bounded: no allocator, and a message that does not fit is a
    // compile-time problem rather than a runtime one.
    generator.configure(".sensor", micropb_gen::Config::new().max_bytes(16));

    let dest = out.join("messages.rs");
    generator
        .compile_protos(&[PROTO], dest.to_str().expect("utf-8"))
        .expect("the messages are generated");
}

/// The GATT table, into `$OUT_DIR/gatt.rs`.
fn table(out: &Path) {
    // No include paths: the vocabulary `proto/server.proto` imports is on
    // `gatt-gen`'s own path.
    let (schema, warnings) =
        gatt_gen::load(&Input::new([PROTO])).unwrap_or_else(|err| panic!("{err}"));
    for warning in &warnings {
        // Things the schema asks for that this stack cannot do. A build is a
        // reasonable place to say so; failing the build would be worse.
        println!("cargo:warning={warning}");
    }

    let generated = rust_trouble::generate(
        &schema,
        &Config {
            // Where the generated table looks for the messages above.
            message_module: "crate::pb::msg".to_string(),
            server_name: "Server".to_string(),
            // No characteristic of this schema is part of a pipe, so these say
            // nothing yet: a pipe characteristic is sized by them when one is
            // declared (see `proto/server.proto` for the shape).
            pipe_chunk_max: 244,
            pipe_msg_max: 0,
            window: 8,
            xfer_timeout_ms: 5000,
            // Written into the generated file, so whoever finds it knows how to
            // remake it.
            regenerate_with: Some("cargo build".to_string()),
        },
    )
    .expect("the schema is valid");

    std::fs::write(out.join(generated.file_name), generated.source).expect("the table is writable");
}
