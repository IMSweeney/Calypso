//! The GATT schema as the generator understands it, parsed from proto.
//!
//! One `service` in the proto is one GATT service; one `rpc` is one
//! characteristic, whose request type is what the central writes and whose
//! response type is what the peripheral reads or notifies (see `SPEC.md` §5).

use std::collections::BTreeSet;
use std::fmt;

/// A GATT characteristic property, as declared in `gatt.CharOpts.props`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Prop {
    Read,
    Write,
    WriteNoRsp,
    Notify,
    Indicate,
}

impl Prop {
    pub fn from_number(number: i32) -> Option<Self> {
        match number {
            1 => Some(Prop::Read),
            2 => Some(Prop::Write),
            3 => Some(Prop::WriteNoRsp),
            4 => Some(Prop::Notify),
            5 => Some(Prop::Indicate),
            _ => None,
        }
    }

    /// The number the vocabulary declares this property with.
    pub fn number(self) -> i32 {
        match self {
            Prop::Read => 1,
            Prop::Write => 2,
            Prop::WriteNoRsp => 3,
            Prop::Notify => 4,
            Prop::Indicate => 5,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Prop::Read => "READ",
            Prop::Write => "WRITE",
            Prop::WriteNoRsp => "WRITE_NO_RSP",
            Prop::Notify => "NOTIFY",
            Prop::Indicate => "INDICATE",
        }
    }

    /// Properties that carry a value from the device to the client.
    pub fn is_from_device(self) -> bool {
        matches!(self, Prop::Read | Prop::Notify | Prop::Indicate)
    }

    /// Properties that carry a value from the client to the device.
    pub fn is_to_device(self) -> bool {
        matches!(self, Prop::Write | Prop::WriteNoRsp)
    }
}

/// A link security requirement. `trouble-host` 0.5 has no per-characteristic
/// permissions, so the generator warns rather than pretending (see `PLAN.md`
/// errata E2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Perm {
    None,
    Encrypted,
    Authenticated,
}

impl Perm {
    pub fn from_number(number: i32) -> Option<Self> {
        match number {
            0 => Some(Perm::None),
            1 => Some(Perm::Encrypted),
            2 => Some(Perm::Authenticated),
            _ => None,
        }
    }

    /// The number the vocabulary declares this permission with.
    pub fn number(self) -> i32 {
        match self {
            Perm::None => 0,
            Perm::Encrypted => 1,
            Perm::Authenticated => 2,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Perm::None => "PERM_NONE",
            Perm::Encrypted => "PERM_ENCRYPTED",
            Perm::Authenticated => "PERM_AUTHENTICATED",
        }
    }
}

/// A bare-value marker from `gatt_options.proto`: a fixed raw encoding rather
/// than a serialized protobuf message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bare {
    U8,
    U16,
    U32,
    I8,
    I16,
    I32,
    F32,
    Bool,
    Utf8,
    Bytes,
}

impl Bare {
    /// The `gatt.*` marker for this bare kind, or `None` if it is not one.
    pub fn from_proto_name(name: &str) -> Option<Self> {
        match name {
            "gatt.U8" => Some(Bare::U8),
            "gatt.U16" => Some(Bare::U16),
            "gatt.U32" => Some(Bare::U32),
            "gatt.I8" => Some(Bare::I8),
            "gatt.I16" => Some(Bare::I16),
            "gatt.I32" => Some(Bare::I32),
            "gatt.F32" => Some(Bare::F32),
            "gatt.Bool" => Some(Bare::Bool),
            "gatt.Utf8" => Some(Bare::Utf8),
            "gatt.Bytes" => Some(Bare::Bytes),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Bare::U8 => "U8",
            Bare::U16 => "U16",
            Bare::U32 => "U32",
            Bare::I8 => "I8",
            Bare::I16 => "I16",
            Bare::I32 => "I32",
            Bare::F32 => "F32",
            Bare::Bool => "Bool",
            Bare::Utf8 => "Utf8",
            Bare::Bytes => "Bytes",
        }
    }

    /// Exact width in bytes, for the fixed-width kinds.
    pub fn width(self) -> Option<usize> {
        match self {
            Bare::U8 | Bare::I8 | Bare::Bool => Some(1),
            Bare::U16 | Bare::I16 => Some(2),
            Bare::U32 | Bare::I32 | Bare::F32 => Some(4),
            Bare::Utf8 | Bare::Bytes => None,
        }
    }

    pub fn is_integer(self) -> bool {
        matches!(
            self,
            Bare::U8 | Bare::U16 | Bare::U32 | Bare::I8 | Bare::I16 | Bare::I32
        )
    }
}

/// The type a characteristic carries in one direction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ValueType {
    /// `google.protobuf.Empty`: nothing travels in that direction.
    Empty,
    /// A bare marker: raw little-endian bytes on the wire.
    Bare(Bare),
    /// A protobuf message, by fully qualified proto name.
    Message(String),
}

impl ValueType {
    pub fn from_proto_name(name: &str) -> Self {
        if name == "google.protobuf.Empty" {
            ValueType::Empty
        } else if let Some(bare) = Bare::from_proto_name(name) {
            ValueType::Bare(bare)
        } else {
            ValueType::Message(name.to_string())
        }
    }

    pub fn is_empty(&self) -> bool {
        matches!(self, ValueType::Empty)
    }

    pub fn describe(&self) -> String {
        match self {
            ValueType::Empty => "google.protobuf.Empty".to_string(),
            ValueType::Bare(bare) => format!("gatt.{}", bare.name()),
            ValueType::Message(name) => name.clone(),
        }
    }
}

/// One characteristic: a proto `rpc` with a `gatt.characteristic` option.
#[derive(Debug, Clone)]
pub struct Characteristic {
    /// The proto service this belongs to.
    pub service: String,
    /// The rpc name, e.g. `Level`.
    pub name: String,
    /// Canonical 128-bit UUID text.
    pub uuid: String,
    pub props: BTreeSet<Prop>,
    pub perm: Perm,
    /// Required for `Utf8`/`Bytes`, and for a single-shot protobuf message.
    pub max_len: u32,
    /// Optional ceiling for a bare integer.
    pub max: Option<u32>,
    /// Pairs the write and notify characteristics of one chunked pipe.
    pub pipe: Option<String>,
    /// Names a user-supplied codec instead of a generated one.
    pub custom_codec: Option<String>,
    /// What the central writes.
    pub request: ValueType,
    /// What the peripheral reads or notifies.
    pub response: ValueType,
}

impl Characteristic {
    /// Fully qualified proto name, for error messages.
    pub fn path(&self) -> String {
        format!("{}.{}", self.service, self.name)
    }

    /// The type that travels, in whichever direction the properties imply.
    ///
    /// A characteristic has one value type in the attribute table, so a
    /// characteristic with traffic in both directions must use the same type
    /// both ways (see `validate`).
    pub fn value_type(&self) -> &ValueType {
        if self.props.iter().any(|p| p.is_to_device()) {
            &self.request
        } else {
            &self.response
        }
    }

    pub fn is_subscribable(&self) -> bool {
        self.props.contains(&Prop::Notify) || self.props.contains(&Prop::Indicate)
    }

    pub fn is_written(&self) -> bool {
        self.props.iter().any(|p| p.is_to_device())
    }
}

/// One GATT service: a proto `service` with a `gatt.service` option.
#[derive(Debug, Clone)]
pub struct Service {
    /// Fully qualified proto service name, e.g. `mydevice.v1.Battery`.
    pub proto_name: String,
    pub uuid: String,
    pub primary: bool,
    pub characteristics: Vec<Characteristic>,
}

impl Service {
    /// The last segment of the proto name, which is what generated Rust
    /// identifiers are built from.
    pub fn short_name(&self) -> &str {
        self.proto_name
            .rsplit('.')
            .next()
            .unwrap_or(&self.proto_name)
    }
}

/// A pipe: the written characteristic and the notified one that share a name.
#[derive(Debug, Clone, Copy)]
pub struct Pipe<'a> {
    pub name: &'a str,
    pub to_device: &'a Characteristic,
    pub to_host: &'a Characteristic,
}

/// The whole schema, validated.
#[derive(Debug, Clone)]
pub struct Gatt {
    pub services: Vec<Service>,
}

impl Gatt {
    pub fn characteristics(&self) -> impl Iterator<Item = &Characteristic> {
        self.services.iter().flat_map(|s| s.characteristics.iter())
    }

    pub fn characteristic(&self, service: &str, name: &str) -> Option<&Characteristic> {
        self.characteristics()
            .find(|c| c.service == service && c.name == name)
    }

    /// A hash of the schema, for the check at connect time: a firmware and an
    /// app built from the same schema agree, and anything else is caught before
    /// any typed traffic is believed.
    ///
    /// Over the canonical rendering of the model rather than over the
    /// descriptor set: protoc's bytes depend on its version, the import order
    /// and unknown fields, and prost re-encodes them differently again.
    pub fn schema_hash(&self) -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325u64; // FNV-1a
        for byte in format!("gatt-gen/1\n{self}").bytes() {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
        hash
    }

    /// Every pipe in the schema, in declaration order.
    pub fn pipes(&self) -> Vec<Pipe<'_>> {
        let mut names: Vec<&str> = self
            .characteristics()
            .filter_map(|c| c.pipe.as_deref())
            .collect();
        names.sort_unstable();
        names.dedup();

        names
            .into_iter()
            .filter_map(|name| {
                let mut group = self
                    .characteristics()
                    .filter(|c| c.pipe.as_deref() == Some(name));
                let to_device = group.find(|c| c.is_written())?;
                let to_host = group.find(|c| c.is_subscribable())?;
                Some(Pipe {
                    name,
                    to_device,
                    to_host,
                })
            })
            .collect()
    }
}

impl fmt::Display for Gatt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for service in &self.services {
            writeln!(
                f,
                "service {} ({}), primary={}",
                service.proto_name, service.uuid, service.primary
            )?;
            for char in &service.characteristics {
                let props: Vec<&str> = char.props.iter().map(|p| p.name()).collect();
                writeln!(
                    f,
                    "  {} ({}) [{}] {} -> {}{}{}{}",
                    char.name,
                    char.uuid,
                    props.join("+"),
                    char.request.describe(),
                    char.response.describe(),
                    if char.max_len > 0 {
                        format!(" max_len={}", char.max_len)
                    } else {
                        String::new()
                    },
                    match char.pipe {
                        Some(ref pipe) => format!(" pipe={pipe}"),
                        None => String::new(),
                    },
                    match char.custom_codec {
                        Some(ref codec) => format!(" codec={codec}"),
                        None => String::new(),
                    },
                )?;
            }
        }
        Ok(())
    }
}
