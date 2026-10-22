//! The schema vocabulary: `gatt_options.proto`, and making sure the schema being compiled agrees with it.
//!
//! An extension is only meaningful if both ends agree on its number, its field numbers and its enum values, and protobuf will happily decode a *different*
//! `gatt_options.proto` that reuses the names. The vocabulary therefore ships with the tool, is put on the include path automatically, and is checked against the one the schema actually imported.

use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use prost_reflect::DescriptorPool;

use crate::error::Error;
use crate::model::{Perm, Prop};

/// The vocabulary, as shipped. Consumers that would rather write their own copy can take it from here (or `gatt-gen --write-options DIR`).
pub const OPTIONS_PROTO: &str = include_str!("../proto/gatt_options.proto");

/// The file name the schema has to import, and the name the vocabulary is written under.
pub const OPTIONS_PROTO_FILE: &str = "gatt_options.proto";

/// The package the vocabulary is declared in.
pub const OPTIONS_PACKAGE: &str = "gatt";

/// Write the vocabulary into `dir`, returning the file it wrote.
pub fn write_options_proto(dir: &Path) -> Result<PathBuf, Error> {
    std::fs::create_dir_all(dir).map_err(|source| Error::Io {
        path: dir.display().to_string(),
        source: source.to_string(),
    })?;
    let path = dir.join(OPTIONS_PROTO_FILE);
    std::fs::write(&path, OPTIONS_PROTO).map_err(|source| Error::Io {
        path: path.display().to_string(),
        source: source.to_string(),
    })?;
    Ok(path)
}

/// A directory holding the shipped vocabulary, for the include path.
///
/// Written once per process: `protoc` reads the file, so two writers racing on one path could hand it half a file.
pub fn vocabulary_dir() -> Result<&'static Path, Error> {
    static DIR: OnceLock<Result<PathBuf, (String, String)>> = OnceLock::new();
    match DIR.get_or_init(|| {
        let dir = std::env::temp_dir().join(format!("gatt-gen-vocabulary-{}", std::process::id()));
        write_options_proto(&dir).map_err(|err| match err {
            Error::Io { path, source } => (path, source),
            other => (dir.display().to_string(), other.to_string()),
        })
    }) {
        Ok(path) => Ok(path.parent().expect("the file is in the directory")),
        Err((path, source)) => Err(Error::Io {
            path: path.clone(),
            source: source.clone(),
        }),
    }
}

/// Check that the vocabulary in this descriptor set is the one this tool generates for, and say which part differs when it is not.
pub fn check(pool: &DescriptorPool) -> Result<(), Error> {
    let mismatch = |what: &str, expected: String, found: String| {
        Error::Descriptor(format!(
            "{what} is {found} where {OPTIONS_PACKAGE} expects {expected}: the schema imported a \
             different {OPTIONS_PROTO_FILE}. Use the one this tool ships \
             (`gatt-gen --write-options DIR`), or regenerate it."
        ))
    };

    let Some(props) = pool.get_enum_by_name(&format!("{OPTIONS_PACKAGE}.Prop")) else {
        return Err(mismatch("`gatt.Prop`", "an enum".into(), "missing".into()));
    };
    for prop in [
        Prop::Read,
        Prop::Write,
        Prop::WriteNoRsp,
        Prop::Notify,
        Prop::Indicate,
    ] {
        let expected = prop.name();
        let found = props
            .get_value(prop.number())
            .map(|value| value.name().to_string())
            .unwrap_or_default();
        if found != expected {
            return Err(mismatch(
                &format!("{OPTIONS_PACKAGE}.Prop = {}", prop.number()),
                expected.into(),
                if found.is_empty() {
                    "missing".into()
                } else {
                    found
                },
            ));
        }
    }

    let Some(perms) = pool.get_enum_by_name(&format!("{OPTIONS_PACKAGE}.Perm")) else {
        return Err(mismatch("`gatt.Perm`", "an enum".into(), "missing".into()));
    };
    for perm in [Perm::None, Perm::Encrypted, Perm::Authenticated] {
        let expected = perm.name();
        let found = perms
            .get_value(perm.number())
            .map(|value| value.name().to_string())
            .unwrap_or_default();
        if found != expected {
            return Err(mismatch(
                &format!("{OPTIONS_PACKAGE}.Perm = {}", perm.number()),
                expected.into(),
                if found.is_empty() {
                    "missing".into()
                } else {
                    found
                },
            ));
        }
    }

    Ok(())
}
