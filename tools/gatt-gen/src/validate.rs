//! The rules a schema has to satisfy to become a GATT table.
//!
//! Every rule here fails the build (see `SPEC.md` §6.4). The exceptions are warnings, and each of those is something the schema can ask for that this stack cannot do (per-link permissions) or that is legal but suspicious.

use std::collections::{BTreeMap, BTreeSet};

use crate::error::Problems;
use crate::model::{Bare, Characteristic, Gatt, Perm, Prop, ValueType};

/// Minimum ATT payload every stack guarantees: an MTU of 23 minus the 3-byte ATT header.
const GUARANTEED_NOTIFY_BYTES: u32 = 20;

/// Check the whole schema, reporting every violation.
pub fn check(gatt: &Gatt, problems: &mut Problems) {
    for service in &gatt.services {
        if service.uuid.is_empty() {
            problems.error(format!(
                "service {}: option (gatt.service) has no uuid",
                service.proto_name
            ));
        }
        if !service.primary {
            problems.warn(format!(
                "service {}: primary: false cannot be expressed -- trouble-host declares every \
                 service as a primary service",
                service.proto_name
            ));
        }
        if service.characteristics.is_empty() {
            problems.error(format!(
                "service {}: no characteristics (every rpc needs (gatt.characteristic))",
                service.proto_name
            ));
        }
    }

    check_unique_uuids(gatt, problems);

    for characteristic in gatt.characteristics() {
        check_properties(characteristic, problems);
        check_value(characteristic, problems);
        check_sizes(characteristic, problems);
        if characteristic.perm != Perm::None {
            problems.warn(format!(
                "{}: perm {} cannot be enforced: trouble-host has no per-characteristic \
                 permissions, so the link stays unencrypted whatever this says",
                characteristic.path(),
                characteristic.perm.name()
            ));
        }
    }

    check_pipes(gatt, problems);
}

/// Rule 2: a UUID identifies a characteristic, so two of them cannot share one.
fn check_unique_uuids(gatt: &Gatt, problems: &mut Problems) {
    let mut seen: BTreeMap<&str, &Characteristic> = BTreeMap::new();
    for characteristic in gatt.characteristics() {
        if characteristic.uuid.is_empty() {
            continue; // already reported by the parser
        }
        match seen.get(characteristic.uuid.as_str()) {
            Some(first) => problems.error(format!(
                "{}: uuid {} is already used by {}",
                characteristic.path(),
                characteristic.uuid,
                first.path()
            )),
            None => {
                seen.insert(&characteristic.uuid, characteristic);
            }
        }
    }

    let mut services: BTreeMap<&str, &str> = BTreeMap::new();
    for service in &gatt.services {
        if service.uuid.is_empty() {
            continue;
        }
        match services.get(service.uuid.as_str()) {
            Some(first) => problems.error(format!(
                "service {}: uuid {} is already used by {}",
                service.proto_name, service.uuid, first
            )),
            None => {
                services.insert(&service.uuid, &service.proto_name);
            }
        }
    }
}

/// Rule 1: the declared properties have to match the types in each direction.
fn check_properties(characteristic: &Characteristic, problems: &mut Problems) {
    let path = characteristic.path();
    let props = &characteristic.props;
    if props.is_empty() {
        problems.error(format!(
            "{path}: declares no properties, so no client can do anything with it"
        ));
        return;
    }

    let from_device = props.contains(&Prop::Read)
        || props.contains(&Prop::Notify)
        || props.contains(&Prop::Indicate);
    let to_device = props.contains(&Prop::Write) || props.contains(&Prop::WriteNoRsp);

    if from_device && characteristic.response.is_empty() {
        problems.error(format!(
            "{path}: {} needs a response type, but the response is google.protobuf.Empty",
            property_names(props, Prop::is_from_device)
        ));
    }
    if to_device && characteristic.request.is_empty() {
        problems.error(format!(
            "{path}: {} needs a request type, but the request is google.protobuf.Empty",
            property_names(props, Prop::is_to_device)
        ));
    }
    if (props.contains(&Prop::Notify) || props.contains(&Prop::Indicate))
        && !characteristic.request.is_empty()
    {
        problems.error(format!(
            "{path}: {} is device to client only, but the request type is {}",
            property_names(props, |p| matches!(p, Prop::Notify | Prop::Indicate)),
            characteristic.request.describe()
        ));
    }

    // One characteristic holds one value in the attribute table, so a characteristic with traffic both ways only works if both directions agree on the type.
    if from_device && to_device && characteristic.request != characteristic.response {
        problems.error(format!(
            "{path}: {} both ways, but the request is {} and the response is {}; one \
             characteristic has one value type",
            property_names(props, |_| true),
            characteristic.request.describe(),
            characteristic.response.describe()
        ));
    }
}

/// Rules 5 and 7: the type has to be one the generator can encode.
fn check_value(characteristic: &Characteristic, problems: &mut Problems) {
    let path = characteristic.path();
    let value = characteristic.value_type();

    if let Some(bare) = crate::descriptor::is_bare(value) {
        match bare {
            Bare::Utf8 | Bare::Bytes if characteristic.max_len == 0 => {
                problems.error(format!(
                    "{path}: gatt.{} carries raw bytes, so it needs max_len",
                    bare.name()
                ));
            }
            Bare::Utf8 | Bare::Bytes => {}
            fixed => {
                if characteristic.max_len != 0 {
                    problems.error(format!(
                        "{path}: max_len does not apply to gatt.{}; it is {} bytes wide",
                        fixed.name(),
                        fixed.width().expect("fixed width"),
                    ));
                }
                if characteristic.custom_codec.is_some() {
                    problems.error(format!(
                        "{path}: custom_codec does not apply to gatt.{}; use gatt.Bytes",
                        fixed.name()
                    ));
                }
            }
        }
        if characteristic.max.is_some() && !bare.is_integer() {
            problems.error(format!(
                "{path}: max applies to bare integers, not gatt.{}",
                bare.name()
            ));
        }
    }

    // Rule 7: a custom codec encodes a byte array; anything else gives the generator nothing to hand it.
    if characteristic.custom_codec.is_some()
        && crate::descriptor::is_bare(value) != Some(Bare::Bytes)
    {
        problems.error(format!(
            "{path}: custom_codec needs a gatt.Bytes value, but this one is {}",
            value.describe()
        ));
    }
}

/// Rules 3 and 6: a protobuf characteristic is either single-shot with a declared size, or part of a pipe -- and those are different mechanisms.
fn check_sizes(characteristic: &Characteristic, problems: &mut Problems) {
    let path = characteristic.path();
    if !matches!(characteristic.value_type(), ValueType::Message(_)) {
        return;
    }

    match (characteristic.max_len, characteristic.pipe.is_some()) {
        (0, false) => problems.error(format!(
            "{path}: a protobuf message needs max_len (a single read or write) or a pipe \
             (a chunked transfer), and has neither"
        )),
        (len, true) if len > 0 => problems.error(format!(
            "{path}: max_len and pipe are alternatives; a pipe sizes its transfers from \
             pipe_msg_max, not from max_len"
        )),
        (0, true) => {}
        (len, false) if len > GUARANTEED_NOTIFY_BYTES => problems.warn(format!(
            "{path}: max_len {len} exceeds the {GUARANTEED_NOTIFY_BYTES} byte payload every \
             stack guarantees, so it only works where the MTU is known to be larger"
        )),
        _ => {}
    }
}

/// Rule 4: a pipe is one written characteristic and one notified one.
fn check_pipes(gatt: &Gatt, problems: &mut Problems) {
    let mut groups: BTreeMap<&str, Vec<&Characteristic>> = BTreeMap::new();
    for characteristic in gatt.characteristics() {
        if let Some(name) = characteristic.pipe.as_deref() {
            groups.entry(name).or_default().push(characteristic);
        }
    }

    for (name, members) in groups {
        let to_device: Vec<&Characteristic> =
            members.iter().copied().filter(|c| c.is_written()).collect();
        let to_host: Vec<&Characteristic> = members
            .iter()
            .copied()
            .filter(|c| c.is_subscribable())
            .collect();

        if members.len() != 2 || to_device.len() != 1 || to_host.len() != 1 {
            problems.error(format!(
                "pipe {name:?}: expected exactly one written and one notified characteristic, \
                 found {}: {}",
                members.len(),
                members
                    .iter()
                    .map(|c| format!("{} [{}]", c.path(), property_names(&c.props, |_| true)))
                    .collect::<Vec<_>>()
                    .join(", ")
            ));
            continue;
        }

        let (down, up) = (to_device[0], to_host[0]);
        let services: BTreeSet<&str> = members.iter().map(|c| c.service.as_str()).collect();
        if services.len() != 1 {
            problems.error(format!(
                "pipe {name:?}: {} and {} are in different services; a pipe pairs two \
                 characteristics of one service",
                down.path(),
                up.path()
            ));
        }
        if down.props.contains(&Prop::Read) {
            problems.error(format!(
                "{}: the written half of pipe {name:?} is not readable; the pipe gets its \
                 data from transfers, not from a read",
                down.path()
            ));
        }
        if up.props.contains(&Prop::Write) || up.props.contains(&Prop::WriteNoRsp) {
            problems.error(format!(
                "{}: the notified half of pipe {name:?} is written to the client only",
                up.path()
            ));
        }
        if up.props.contains(&Prop::Indicate) {
            problems.error(format!(
                "{}: pipe {name:?} is acknowledged by its own ACK chunks, so its notified \
                 half cannot also be an indication",
                up.path()
            ));
        }
        // A pipe's ACK chunks are the acknowledgement; an ATT write response is an extra round trip that also queues behind the ATT response.
        if down.props.contains(&Prop::Write) {
            problems.warn(format!(
                "{}: pipe {name:?} is also written with WRITE (with response); the response \
                 is a round trip the ACK chunk already covers",
                down.path()
            ));
        }
    }
}

fn property_names(props: &BTreeSet<Prop>, want: impl Fn(Prop) -> bool) -> String {
    let names: Vec<&str> = props
        .iter()
        .filter(|p| want(**p))
        .map(|p| p.name())
        .collect();
    if names.is_empty() {
        "no property".to_string()
    } else {
        names.join("+")
    }
}
