//! Generates the protobuf Rust types (`src/pb.rs`) from `proto/calypso.proto`.
//!
//! `micropb-gen` shells out to `protoc`; rather than requiring a system
//! install we point it at the prebuilt binary from `protoc-bin-vendored`, so
//! the build works on a clean machine (and on CI) with no extra setup.

use std::path::Path;

/// Max trace samples per `TraceChunk` record (must equal
/// `calypso_core::pb::TRACE_CHUNK_SAMPLES`).
const TRACE_CHUNK_SAMPLES: usize = 48;

/// Length of a rep's UUID (must equal `calypso_core::uuid::UUID_LEN`).
const REP_UUID_LEN: usize = 16;

fn main() {
    // protoc needs an include path that is a prefix of the file names it is
    // given, so run from the repo root and pass root-relative paths.
    let manifest = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is set by cargo");
    let root = Path::new(&manifest)
        .parent()
        .expect("crate is in the repo root");
    std::env::set_current_dir(root).expect("chdir to repo root");

    let proto = root.join("proto/calypso.proto");
    println!("cargo:rerun-if-changed={}", proto.display());

    // Vendored protoc, so no system dependency.
    if std::env::var_os("PROTOC").is_none() {
        if let Ok(path) = protoc_bin_vendored::protoc_bin_path() {
            std::env::set_var("PROTOC", path);
        }
    }

    let out_dir = std::env::var("OUT_DIR").expect("OUT_DIR is set by cargo");
    let dest = Path::new(&out_dir).join("calypso.rs");

    let mut generator = micropb_gen::Generator::new();
    // Containers come from heapless (fixed capacity, no allocator); the
    // generated structs stay useful on the device and on the host.
    generator.use_container_heapless();
    // Rep UUIDs are the only `bytes` fields; bound them so the generated types
    // stay fixed-capacity (no allocator).
    generator.configure(
        ".calypso",
        micropb_gen::Config::new().max_bytes(REP_UUID_LEN as u32),
    );
    // Trace chunks are the only repeated field. The capacity must be a
    // compile-time constant here; `pb::TRACE_CHUNK_SAMPLES` declares the same
    // value and a unit test asserts they match.
    generator.configure(
        ".calypso.TraceChunk.samples",
        micropb_gen::Config::new().max_len(TRACE_CHUNK_SAMPLES as u32),
    );
    generator
        .compile_protos(&["proto/calypso.proto"], dest.to_str().unwrap())
        .expect("failed to generate protobuf types");
}
