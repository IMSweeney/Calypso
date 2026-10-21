//! Generates type-checked GATT code for both ends of a BLE link from proto-defined
//! services.
//!
//! The schema is a set of `.proto` files: a `service` with `option
//! (gatt.service)` is a GATT service, an `rpc` with `option
//! (gatt.characteristic)` is a characteristic whose request type is what the
//! central writes and whose response type is what the peripheral reads or
//! notifies. `SPEC.md` is the design; `PLAN.md` records what of it this stack can
//! actually do.
//!
//! The crate reads the schema, checks it, and hands a validated model to one of
//! its generators — [`generator::server`] for the device, [`generator::client`]
//! for the app — which turn it into a file the consumer checks in. It runs on the
//! host, so it may use `std` and real dependencies; the code it emits may not.
//!
//! ```no_run
//! use gatt_gen::generator::{Config, server::rust_trouble};
//!
//! let (gatt, warnings) = gatt_gen::load(&gatt_gen::Input::new(["proto/device.proto"]))?;
//! let file = gatt_gen::Target::RustTrouble
//!     .generate(&gatt, &Config::RustTrouble(rust_trouble::Config::default()))?;
//! std::fs::write(file.file_name, file.source)?;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

pub mod descriptor;
pub mod error;
pub mod generator;
pub mod model;
pub mod uuid;
pub mod validate;
pub mod vocabulary;

pub use descriptor::{descriptor_set, load, load_from_descriptor_set, Input};
pub use error::Error;
pub use generator::{Output, Target, TARGETS};
pub use model::{Bare, Characteristic, Gatt, Perm, Pipe, Prop, Service, ValueType};
pub use vocabulary::{write_options_proto, OPTIONS_PROTO, OPTIONS_PROTO_FILE};
