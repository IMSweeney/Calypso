//! Dart for the app, over `universal_ble`.
//!
//! Not written yet. When it is, it generates the typed client the schema
//! describes — one class per service, `read<Name>()`, `<name>Updates` and
//! `write<Name>()` only for the properties the characteristic declares, over the
//! raw UUID access the runtime keeps private (see `SPEC.md` §8.2). Nothing in
//! the app consumes it yet: the hand-written transport in
//! `calypso-app/lib/transport/` speaks the same two characteristics.
//!
//! This file exists so the shape of the tool is settled — a folder per side of
//! the link, a file per language and library — and so `gatt-gen` can name a
//! target it cannot yet serve.

use crate::error::Error;
use crate::generator::Output;
use crate::model::Gatt;

/// The name the command line uses.
pub const NAME: &str = "dart-universal-ble";

/// The file this target will write.
pub const OUTPUT_FILE: &str = "gatt_client.dart";

/// Nothing to configure yet.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Config;

pub fn generate(_gatt: &Gatt, _config: &Config) -> Result<Output, Error> {
    Err(Error::Unsupported(format!(
        "{NAME}: not written yet; see tools/gatt-gen/PLAN.md"
    )))
}
