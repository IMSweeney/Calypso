//! Reading the schema: protoc to a descriptor set, then the descriptor set to a [`Gatt`] model.
//!
//! The custom options (`gatt.service`, `gatt.characteristic`) are extension fields of `google.protobuf.ServiceOptions` / `MethodOptions`, so they are read out of the descriptor set rather than from the parsed proto. Two things that are easy to get wrong, both of which fail silently:
//!
//! - the descriptor set must be built with `--include_imports`, or `gatt_options.proto` is not in the pool and the options decode as empty;
//! - extensions are **not** in `DynamicMessage::fields()`; they have to be read through `extensions()` or `get_extension()`.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicUsize, Ordering};

use prost_reflect::{DescriptorPool, DynamicMessage, ExtensionDescriptor, Value};

use crate::error::{Error, Problems};
use crate::model::{Bare, Characteristic, Gatt, Perm, Prop, Service, ValueType};
use crate::{uuid, vocabulary};

/// The proto files to read the schema from, and where to find their imports.
#[derive(Debug, Clone)]
pub struct Input {
    pub protos: Vec<PathBuf>,
    pub includes: Vec<PathBuf>,
}

impl Input {
    pub fn new(protos: impl IntoIterator<Item = impl Into<PathBuf>>) -> Self {
        Input {
            protos: protos.into_iter().map(Into::into).collect(),
            includes: Vec::new(),
        }
    }

    pub fn include(mut self, path: impl Into<PathBuf>) -> Self {
        self.includes.push(path.into());
        self
    }
}

/// The extension number `gatt.characteristic` is declared with.
const CHARACTERISTIC_EXT: &str = "gatt.characteristic";
/// The extension number `gatt.service` is declared with.
const SERVICE_EXT: &str = "gatt.service";

/// `protoc`, either the one the environment names or the vendored build.
fn protoc() -> PathBuf {
    match std::env::var_os("PROTOC") {
        Some(path) => PathBuf::from(path),
        None => protoc_bin_vendored::protoc_bin_path().expect("a protoc binary is vendored"),
    }
}

/// Compile `input.protos` into a `FileDescriptorSet` at `out`.
///
/// Written to a file rather than parsed in memory because the Dart generator consumes the same bytes: one descriptor set, two back ends.
pub fn descriptor_set(input: &Input, out: &Path) -> Result<(), Error> {
    let mut command = Command::new(protoc());
    for include in &input.includes {
        command.arg(format!("-I{}", include.display()));
    }
    // protoc resolves imports against `-I` paths only, and the file name it is given has to sit under one of them: the working directory covers the relative names a build script passes, and a file's own directory covers absolute ones.
    command.arg("-I.");
    for proto in &input.protos {
        if let Some(dir) = proto.parent().filter(|dir| !dir.as_os_str().is_empty()) {
            command.arg(format!("-I{}", dir.display()));
        }
    }
    // The vocabulary comes with the tool, so `import "gatt_options.proto"`
    // resolves without every consumer vendoring a copy. Last, so a schema that deliberately ships its own copy is the one that gets checked.
    command.arg(format!("-I{}", vocabulary::vocabulary_dir()?.display()));
    command
        // Without this the custom options cannot be read back (see the module docs).
        .arg("--include_imports")
        .arg(format!("--descriptor_set_out={}", out.display()))
        .args(&input.protos);

    let output = command.output().map_err(|source| Error::Io {
        path: "protoc".to_string(),
        source: source.to_string(),
    })?;
    if !output.status.success() {
        return Err(Error::Protoc(
            String::from_utf8_lossy(&output.stderr).into(),
        ));
    }
    Ok(())
}

/// Parse and validate a descriptor set that `protoc` already produced.
pub fn load_from_descriptor_set(path: &Path) -> Result<(Gatt, Vec<String>), Error> {
    let bytes = std::fs::read(path).map_err(|source| Error::Io {
        path: path.display().to_string(),
        source: source.to_string(),
    })?;
    let pool = DescriptorPool::decode(bytes.as_slice())
        .map_err(|err| Error::Descriptor(err.to_string()))?;
    parse(&pool)
}

/// Compile `input` and parse the result.
pub fn load(input: &Input) -> Result<(Gatt, Vec<String>), Error> {
    let out = temporary_descriptor_set();
    descriptor_set(input, &out)?;
    let loaded = load_from_descriptor_set(&out);
    let _ = std::fs::remove_file(&out);
    loaded
}

/// A path no other call in this process shares: the descriptor set is written by protoc, so two concurrent loads must not pick the same file.
fn temporary_descriptor_set() -> PathBuf {
    static NEXT: AtomicUsize = AtomicUsize::new(0);
    let n = NEXT.fetch_add(1, Ordering::Relaxed);
    std::env::temp_dir().join(format!("gatt-gen-{}-{n}.fdset", std::process::id()))
}

/// Build the model from a descriptor pool.
pub fn parse(pool: &DescriptorPool) -> Result<(Gatt, Vec<String>), Error> {
    vocabulary::check(pool)?;

    let mut problems = Problems::default();
    let service_ext = pool
        .get_extension_by_name(SERVICE_EXT)
        .ok_or_else(|| missing_options_file(SERVICE_EXT))?;
    let char_ext = pool
        .get_extension_by_name(CHARACTERISTIC_EXT)
        .ok_or_else(|| missing_options_file(CHARACTERISTIC_EXT))?;

    let mut services = Vec::new();
    for service in pool.services() {
        // A proto service without the option is not a GATT service: the schema can be added to an existing file one service at a time.
        let Some(opts) = extension(&service.options(), &service_ext) else {
            continue;
        };
        let mut chars = Vec::new();
        for method in service.methods() {
            match extension(&method.options(), &char_ext) {
                Some(opts) => {
                    if let Some(char) = parse_characteristic(
                        &mut problems,
                        service.full_name(),
                        method.name(),
                        &opts,
                        &method,
                    ) {
                        chars.push(char);
                    }
                }
                None => problems.error(format!(
                    "{}: rpc {} has no `option (gatt.characteristic)`; every characteristic \
                     in a service with (gatt.service) needs one",
                    service.full_name(),
                    method.name()
                )),
            }
        }
        services.push(Service {
            proto_name: service.full_name().to_string(),
            uuid: string_field(&opts, "uuid").unwrap_or_default(),
            primary: bool_field(&opts, "primary").unwrap_or(false),
            characteristics: chars,
        });
    }

    if services.is_empty() {
        problems.error(format!(
            "no service carries `option (gatt.service)`; nothing to generate from {}",
            service_ext.name()
        ));
    }

    let gatt = Gatt { services };
    crate::validate::check(&gatt, &mut problems);
    let warnings = problems.finish()?;
    Ok((gatt, warnings))
}

fn missing_options_file(extension: &str) -> Error {
    Error::Descriptor(format!(
        "no `{extension}` extension in the descriptor set: was it built with \
         --include_imports, from a proto that imports gatt_options.proto?"
    ))
}

fn parse_characteristic(
    problems: &mut Problems,
    service: &str,
    name: &str,
    opts: &DynamicMessage,
    method: &prost_reflect::MethodDescriptor,
) -> Option<Characteristic> {
    let path = format!("{service}.{name}");

    let uuid_text = string_field(opts, "uuid").unwrap_or_default();
    let uuid = match uuid::parse_canonical(&uuid_text) {
        Some(bytes) => uuid::to_canonical(&bytes),
        None => {
            problems.error(format!(
                "{path}: uuid {uuid_text:?} is not a canonical 128-bit UUID \
                 (0000xxxx-0000-1000-8000-00805f9b34fb form)"
            ));
            String::new()
        }
    };

    let mut props = BTreeSet::new();
    for value in list_field(opts, "props") {
        match value.as_ref() {
            Value::EnumNumber(number) => match Prop::from_number(*number) {
                Some(prop) => {
                    props.insert(prop);
                }
                None => problems.error(format!(
                    "{path}: PROP_UNSPECIFIED is not a property; declare which operations \
                     the characteristic offers"
                )),
            },
            other => problems.error(format!("{path}: props: unexpected {other:?}")),
        }
    }

    let perm = match opts.get_field_by_name("perm").as_deref() {
        Some(Value::EnumNumber(number)) => Perm::from_number(*number).unwrap_or(Perm::None),
        _ => Perm::None,
    };

    Some(Characteristic {
        service: service.to_string(),
        name: name.to_string(),
        uuid,
        props,
        perm,
        max_len: u32_field(opts, "max_len").unwrap_or(0),
        max: u32_field(opts, "max"),
        pipe: non_empty(string_field(opts, "pipe")),
        custom_codec: non_empty(string_field(opts, "custom_codec")),
        request: ValueType::from_proto_name(method.input().full_name()),
        response: ValueType::from_proto_name(method.output().full_name()),
    })
}

/// The message an extension option holds, or `None` when the option is absent.
///
/// Extensions are not part of `DynamicMessage::fields()`: an option that is set looks like a message field with no fields set if you look in the wrong place.
fn extension(opts: &DynamicMessage, ext: &ExtensionDescriptor) -> Option<DynamicMessage> {
    opts.extensions()
        .find(|(field, _)| field.number() == ext.number())
        .and_then(|(_, value)| match value {
            Value::Message(message) => Some(message.clone()),
            _ => None,
        })
}

fn non_empty(value: Option<String>) -> Option<String> {
    value.filter(|v| !v.is_empty())
}

fn field(message: &DynamicMessage, name: &str) -> Option<Value> {
    message
        .get_field_by_name(name)
        .map(|value| value.into_owned())
}

fn string_field(message: &DynamicMessage, name: &str) -> Option<String> {
    match field(message, name) {
        Some(Value::String(text)) => Some(text.to_string()),
        _ => None,
    }
}

fn bool_field(message: &DynamicMessage, name: &str) -> Option<bool> {
    match field(message, name) {
        Some(Value::Bool(flag)) => Some(flag),
        _ => None,
    }
}

fn u32_field(message: &DynamicMessage, name: &str) -> Option<u32> {
    match field(message, name) {
        Some(Value::U32(number)) if number > 0 => Some(number),
        _ => None,
    }
}

fn list_field(message: &DynamicMessage, name: &str) -> Vec<std::borrow::Cow<'static, Value>> {
    match field(message, name) {
        Some(Value::List(values)) => values.into_iter().map(std::borrow::Cow::Owned).collect(),
        Some(Value::EnumNumber(number)) => {
            // A single-valued option is not a list; treat it as one of one.
            vec![std::borrow::Cow::Owned(Value::EnumNumber(number))]
        }
        _ => Vec::new(),
    }
}

/// Bare markers never travel as protobuf, so a characteristic using one has to be recognized before any codec is chosen.
pub fn is_bare(value: &ValueType) -> Option<Bare> {
    match value {
        ValueType::Bare(bare) => Some(*bare),
        _ => None,
    }
}
