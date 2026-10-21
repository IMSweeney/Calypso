//! Failures the generator reports.
//!
//! A schema problem is `Invalid`, and carries every rule the schema broke
//! rather than the first one, so one run of the generator is one round trip.

use std::fmt;

#[derive(Debug)]
pub enum Error {
    /// Reading or writing a file failed.
    Io { path: String, source: String },
    /// `protoc` refused the schema.
    Protoc(String),
    /// The descriptor set could not be read, or an option could not be decoded.
    Descriptor(String),
    /// The schema is expressible protobuf but not a valid GATT table.
    Invalid(Vec<String>),
    /// The target asked for cannot do what it was asked to do.
    Unsupported(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io { path, source } => write!(f, "{path}: {source}"),
            Error::Protoc(stderr) => write!(f, "protoc failed: {}", stderr.trim()),
            Error::Descriptor(what) => write!(f, "descriptor set: {what}"),
            Error::Invalid(problems) => {
                writeln!(f, "the schema is not a valid GATT table:")?;
                for problem in problems {
                    writeln!(f, "  - {problem}")?;
                }
                Ok(())
            }
            Error::Unsupported(what) => write!(f, "{what}"),
        }
    }
}

impl std::error::Error for Error {}

/// Collects schema problems so every one of them is reported at once.
#[derive(Debug, Default)]
pub struct Problems {
    errors: Vec<String>,
    warnings: Vec<String>,
}

impl Problems {
    pub fn error(&mut self, message: impl Into<String>) {
        self.errors.push(message.into());
    }

    pub fn warn(&mut self, message: impl Into<String>) {
        self.warnings.push(message.into());
    }

    /// The warnings collected, or every error as one failure.
    pub fn finish(self) -> Result<Vec<String>, Error> {
        if self.errors.is_empty() {
            Ok(self.warnings)
        } else {
            Err(Error::Invalid(self.errors))
        }
    }
}
