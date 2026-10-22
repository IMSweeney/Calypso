//! A failing schema per rule (SPEC.md 6.4), each with the message it should produce.

mod common;

use common::{errors, load};

/// Every fixture under `invalid/`, and a fragment of the rule it breaks.
const INVALID: &[(&str, &str)] = &[
    ("notify_without_response.proto", "needs a response type"),
    ("read_without_response.proto", "needs a response type"),
    ("write_without_request.proto", "needs a request type"),
    ("notify_with_request.proto", "is device to client only"),
    (
        "mismatched_directions.proto",
        "one characteristic has one value type",
    ),
    ("no_props.proto", "declares no properties"),
    ("duplicate_uuid.proto", "is already used by"),
    ("bad_uuid.proto", "is not a canonical 128-bit UUID"),
    ("protobuf_without_size.proto", "needs max_len"),
    (
        "pipe_with_max_len.proto",
        "max_len and pipe are alternatives",
    ),
    (
        "pipe_missing_half.proto",
        "expected exactly one written and one notified",
    ),
    ("pipe_with_indicate.proto", "cannot also be an indication"),
    (
        "bytes_without_max_len.proto",
        "carries raw bytes, so it needs max_len",
    ),
    (
        "fixed_width_with_max_len.proto",
        "max_len does not apply to gatt.U8",
    ),
    (
        "custom_codec_without_bytes.proto",
        "custom_codec needs a gatt.Bytes value",
    ),
    (
        "missing_characteristic_option.proto",
        "has no `option (gatt.characteristic)`",
    ),
];

#[test]
fn each_rule_reports_what_it_is_for() {
    for (fixture, expected) in INVALID {
        let problems = errors(&format!("invalid/{fixture}"));
        assert!(
            problems.iter().any(|p| p.contains(expected)),
            "{fixture}: expected a problem containing {expected:?}, got {problems:#?}"
        );
    }
}

#[test]
fn a_schema_that_asks_for_what_the_stack_cannot_do_warns_rather_than_fails() {
    for (fixture, expected) in [
        ("encrypted.proto", "cannot be enforced"),
        ("max_len_over_20.proto", "max_len 32 exceeds"),
        (
            "pipe_write_with_response.proto",
            "is also written with WRITE",
        ),
    ] {
        let (_, warnings) = load(&format!("warn/{fixture}"))
            .unwrap_or_else(|err| panic!("{fixture}: expected warnings, got {err}"));
        assert!(
            warnings.iter().any(|w| w.contains(expected)),
            "{fixture}: expected a warning containing {expected:?}, got {warnings:#?}"
        );
    }
}
