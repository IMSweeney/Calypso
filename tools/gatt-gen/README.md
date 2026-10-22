# gatt-gen

Generates type-checked GATT code for both ends of a BLE link from `.proto`
files. A `service` in the proto is a GATT service, an `rpc` is a characteristic,
and the properties declared in the schema are the operations the generated code
offers — so a notify on a characteristic with no `NOTIFY` is a compile error
rather than a device that stays silent.

- [`SPEC.md`](SPEC.md) — the design (handoff spec, revision 2).
- [`PLAN.md`](PLAN.md) — the review of that spec against the pinned crates, the
  decisions taken, what is still open, and the traps found on the way.

## Layout

The schema is read once, in the front half of the crate; a generator is a
function from a validated model to a file of text. One folder per side of the
link, one file inside each per language and BLE library:

```
src/
  descriptor.rs  model.rs  validate.rs  uuid.rs  vocabulary.rs   <- the schema
  generator/
    server/rust_trouble.rs         trouble-host, no_std, for the device
    client/dart_universal_ble.rs   universal_ble, for the app
```

Adding a target is adding a file: it declares `NAME`, `OUTPUT_FILE`, a `Config`
and a `generate`, and `generator::Target` picks it up.

## Using it

```sh
cargo install --path .                     # or `cargo run --` from a checkout
gatt-gen rust-trouble --out ../calypso-core/src \
  --messages crate::pb::msg --server CalypsoGatt ../proto/calypso_gatt.proto
gatt-gen dart-universal-ble --out ../calypso-app/lib/gatt \
  --dart-messages ../protocol/calypso.pb.dart --dart-prefix pb \
  ../proto/calypso_gatt.proto
gatt-gen --write-options proto             # the vocabulary, if you want a copy
```

The tool writes `<out>/<the target's file name>`, creates the directory if it
needs to, prints the warnings the schema earned, and never runs as part of
anyone's build: **the generated code is meant to be committed**, so a firmware
build needs no generator, no protoc and nothing from the network, and a change to
the interface shows up in review as a diff.

The generated Rust is **items only** — no `//!` docs, no `#![allow]` — because
both are inner attributes and an inner attribute is not permitted in an `include!`
expansion. Mount it in a module of your own and allow the lints there, and it
works whether you commit it and declare `pub mod`:

```rust
#[allow(clippy::all, nonstandard_style, unused, irrefutable_let_patterns)]
pub mod gatt;   // src/gatt.rs, generated and committed
```

or generate it in a build script and include it:

```rust
#[allow(clippy::all, nonstandard_style, unused, irrefutable_let_patterns)]
mod gatt {
    include!(concat!(env!("OUT_DIR"), "/gatt.rs"));
}
```

`examples/server` is the second shape, end to end. Calypso does exactly this — see
`calypso-core/tool/gen_gatt.sh` and `calypso-core/src/gatt.rs`. Both of Calypso's
generated files carry the command that regenerates them in their own header,
because nothing else checks them: no build runs the generator, and a table left
behind after a schema change is a device and an app disagreeing about the
interface.

Because the output is committed, it is written **format-clean**: the file is an
ordinary source file of the project that holds it, so the repo's own `cargo fmt
--check` (Rust) and `dart format --set-exit-if-changed` (Dart) both cover it.
Neither formatter skips a file that says `@generated`.

The Dart target talks to BLE through an interface it declares in its own output
(`GattClientLink`), which the app implements over `universal_ble` and a test
implements as a fake. `universal_ble`'s static API is never called directly.

A crate that takes the generated table needs `trouble-host` (with `gatt`),
`micropb` (with a heapless container) and `heapless`, plus two the macros expand
into: `static_cell` (each characteristic's storage) and `embassy-sync` (the
attribute table's mutex).

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
schema parsed into the model it declares, and what `rust-trouble` emits. It also
runs the two examples, which are default members because they are the only targets
that compile what this crate generates — and what they check is what a client sees:
the UUIDs in wire order, a CCCD on the notifiable characteristic and nowhere else, a
value through the table and back, and a frame that fills a pipe's buffer exactly.
