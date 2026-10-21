# gatt-gen

Generates type-checked GATT code for both ends of a BLE link from `.proto`
files. A `service` in the proto is a GATT service, an `rpc` is a characteristic,
and the properties declared in the schema are the operations the generated code
offers — so a notify on a characteristic with no `NOTIFY` is a compile error
rather than a device that stays silent.

- [`SPEC.md`](SPEC.md) — the design (handoff spec, revision 2).
- [`PLAN.md`](PLAN.md) — the review of that spec against the pinned crates, the
  decisions taken, and the order things are being built in.

## Layout

The schema is read once, in the front half of the crate; a generator is a
function from a validated model to a file of text. One folder per side of the
link, one file inside each per language and BLE library:

```
src/
  descriptor.rs  model.rs  validate.rs  uuid.rs  vocabulary.rs   <- the schema
  generator/
    server/rust_trouble.rs         trouble-host, no_std, for the device
    client/dart_universal_ble.rs   universal_ble, for the app (not written yet)
```

Adding a target is adding a file: it declares `NAME`, `OUTPUT_FILE`, a `Config`
and a `generate`, and `generator::Target` picks it up.

## Using it

```sh
cargo install --path .                     # or `cargo run --` from a checkout
gatt-gen rust-trouble --out ../calypso-core/src \
  --messages crate::pb::msg --server CalypsoGatt ../proto/calypso_gatt.proto
gatt-gen --write-options proto             # the vocabulary, if you want a copy
```

The tool writes `<out>/<the target's file name>`, creates the directory if it
needs to, prints the warnings the schema earned, and never runs as part of
anyone's build: **the generated code is meant to be committed**, so a firmware
build needs no generator, no protoc and nothing from the network, and a change to
the interface shows up in review as a diff. (Calypso does exactly this — see
`calypso-core/tool/gen_gatt.sh` and `calypso-core/src/gatt.rs`. It also has
`calypso-core/tests/gatt_generated.rs`, which fails if the committed file and the
schema disagree.)

Because the output is committed, it is written **format-clean**: the file is an
ordinary source file of the crate that holds it, so the repo's own
`cargo fmt --check` has to pass on it. That is why the emitter lays out byte
arrays and attributes the way `rustfmt` would rather than relying on a
post-processing step — there is no rustfmt in the generation path.

The schema vocabulary ships with the crate: a consumer imports
`gatt_options.proto` and the include path is added for it, so nothing has to be
vendored, and a schema that carries a *different* vocabulary is rejected rather
than believed. The same generation as a library call:

```rust
use gatt_gen::generator::{Config, Target, server::rust_trouble};

let (gatt, warnings) = gatt_gen::load(&gatt_gen::Input::new(["proto/device.proto"]))?;
let file = Target::RustTrouble.generate(
    &gatt,
    &Config::RustTrouble(rust_trouble::Config::default()),
)?;
std::fs::write(file.file_name, file.source)?;
```

## Tests

`cargo test` covers the schema half: one failing fixture per rule, the example
schema parsed into the model it declares, and what `rust-trouble` emits.

## Notes for whoever works on this next

Two traps, both silent, both now covered by tests:

- protoc has to be run with `--include_imports`, or `gatt_options.proto` is not
  in the descriptor set, the extensions are not registered, and every option
  decodes as empty.
- Extension fields are **not** in `DynamicMessage::fields()`. An option that is
  set looks like a message with no fields if it is read from there; the
  extensions have to be read through `extensions()` or `get_extension()`.
