//! Generates the protobuf Rust types from `proto/calypso.proto` into `$OUT_DIR`, where `src/pb.rs` includes them.
//!
//! `micropb-gen` shells out to `protoc`; rather than requiring a system install we point it at the prebuilt binary from `protoc-bin-vendored`, so the build works on a clean machine with no extra setup.
//!
//! The GATT table (`src/gatt.rs`) is not generated here: `gatt-gen` writes it when someone runs `tool/gen_gatt.sh`, and it is committed.

use std::path::Path;

/// Max trace samples per `TraceChunk` record (must equal `calypso_core::pb::TRACE_CHUNK_SAMPLES`).
const TRACE_CHUNK_SAMPLES: usize = 48;

/// Length of a rep's UUID (must equal `calypso_core::uuid::UUID_LEN`).
const REP_UUID_LEN: usize = 16;

/// The messages, relative to the repo root.
const PROTO: &str = "proto/calypso.proto";

fn main() {
    // protoc needs an include path that is a prefix of the file names it is given, so run from the repo root and pass root-relative paths.
    let manifest = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is set by cargo");
    let root = Path::new(&manifest)
        .parent()
        .expect("crate is in the repo root");
    std::env::set_current_dir(root).expect("chdir to repo root");

    println!("cargo:rerun-if-changed={PROTO}");

    // Vendored protoc, so no system dependency. Both generators honour PROTOC.
    if std::env::var_os("PROTOC").is_none() {
        if let Ok(path) = protoc_bin_vendored::protoc_bin_path() {
            std::env::set_var("PROTOC", path);
        }
    }

    let out_dir = std::env::var("OUT_DIR").expect("OUT_DIR is set by cargo");
    messages(Path::new(&out_dir));
}

/// The protobuf message types, into `$OUT_DIR/calypso.rs`.
fn messages(out_dir: &Path) {
    let dest = out_dir.join("calypso.rs");

    let mut generator = micropb_gen::Generator::new();
    // Containers come from heapless (fixed capacity, no allocator); the generated structs stay useful on the device and on the host.
    generator.use_container_heapless();
    // protoc resolves imports against `-I` paths only, so naming the working directory is what makes `proto/calypso.proto` a legal file name.
    generator.add_protoc_arg("-I.");
    // Rep UUIDs are the only `bytes` fields; bound them so the generated types stay fixed-capacity (no allocator).
    generator.configure(
        ".calypso",
        micropb_gen::Config::new().max_bytes(REP_UUID_LEN as u32),
    );
    // Trace chunks are the only repeated field. The capacity must be a compile-time constant here; `pb::TRACE_CHUNK_SAMPLES` declares the same value and a unit test asserts they match.
    generator.configure(
        ".calypso.TraceChunk.samples",
        micropb_gen::Config::new().max_len(TRACE_CHUNK_SAMPLES as u32),
    );
    generator
        .compile_protos(&[PROTO], dest.to_str().unwrap())
        .expect("failed to generate protobuf types");
}
