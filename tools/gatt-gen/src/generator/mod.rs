//! The back ends.
//!
//! One folder per side of the link — [`client`] for the app, [`server`] for the device — and one file inside each per language and BLE library. The schema is read once, in the shared front half of the crate; what differs between targets is only the code they emit, so a generator file is a function from a validated [`Gatt`] to text.
//!
//! ```text
//! proto -> descriptor -> model -> validate -+-> server/rust_trouble     -> gatt.rs
//!                                           +-> client/dart_universal_ble -> gatt_client.dart
//! ```

pub mod client;
pub mod server;

use crate::error::Error;
use crate::model::Gatt;

/// One generated file: the name it is written under, and its text.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Output {
    /// The file name, without a directory: the caller decides where it goes.
    pub file_name: String,
    pub source: String,
}

/// Which generator to run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Target {
    /// Rust for the device: `trouble-host` on embassy, `no_std`, no heap for the table itself.
    RustTrouble,
    /// Dart for the app: `universal_ble`, cross-platform.
    DartUniversalBle,
}

/// Every target, in the order a help message should list them.
pub const TARGETS: &[Target] = &[Target::RustTrouble, Target::DartUniversalBle];

/// A target's configuration, which is each generator's own.
#[derive(Debug, Clone)]
pub enum Config {
    RustTrouble(server::rust_trouble::Config),
    DartUniversalBle(client::dart_universal_ble::Config),
}

impl Target {
    /// The name the command line uses, and the name of the module that implements it.
    pub fn name(self) -> &'static str {
        match self {
            Target::RustTrouble => server::rust_trouble::NAME,
            Target::DartUniversalBle => client::dart_universal_ble::NAME,
        }
    }

    pub fn from_name(name: &str) -> Option<Self> {
        TARGETS.iter().copied().find(|target| target.name() == name)
    }

    /// The file this target writes, for the caller to place.
    pub fn output_file_name(self) -> &'static str {
        match self {
            Target::RustTrouble => server::rust_trouble::OUTPUT_FILE,
            Target::DartUniversalBle => client::dart_universal_ble::OUTPUT_FILE,
        }
    }

    pub fn generate(self, gatt: &Gatt, config: &Config) -> Result<Output, Error> {
        match (self, config) {
            (Target::RustTrouble, Config::RustTrouble(config)) => {
                server::rust_trouble::generate(gatt, config)
            }
            (Target::DartUniversalBle, Config::DartUniversalBle(config)) => {
                client::dart_universal_ble::generate(gatt, config)
            }
            (target, _) => Err(Error::Unsupported(format!(
                "{}: that is another target's configuration",
                target.name()
            ))),
        }
    }
}

impl std::fmt::Display for Target {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}
