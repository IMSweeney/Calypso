# BLE GATT Schema Codegen: Project Spec

Handoff spec for an implementing agent. Status: design agreed, nothing built yet. Revision 2: device side is `trouble-host` (embassy, no_std), not `esp32-nimble`.

## 1. Goal

Define a BLE GATT table once, in `.proto` files, and generate type-checked code for both ends:

- **Server:** Rust on ESP32, using `trouble-host` (embassy, `no_std`).
- **Client:** Flutter, using `universal_ble`, cross-platform.

The central requirement is **type checking on both sides**. Intended usage:

```rust
// Rust server
battery.level.notify(87);                    // ok: Level is a bare u8
battery.level.notify(&Command { .. });       // compile error
battery.rx.notify(..);                       // compile error: Rx has no NOTIFY
```

```dart
// Dart client
battery.levelUpdates.listen((int pct) => ...);   // Stream<int>
await battery.writeLevel(5);                      // doesn't exist: Level has no WRITE
```

Secondary requirements:

1. Characteristics carry either **protobuf messages** or **bare values** (a single raw little-endian integer, float, bool, string or byte array). Bare values must be byte-compatible with SIG-standard characteristics so stock tools (nRF Connect, OS battery UI) can read them.
2. Payloads larger than one ATT write or notification are supported through a **chunked data pipe** with predictable framing.
3. Mismatched firmware and app schema versions are detectable at connect time.

## 2. Non-goals

- L2CAP CoC for the client. `universal_ble` doesn't expose it. `trouble-host` does support it on the device, so it remains an upgrade path if throughput demands it (client candidate: `ble_plus`, or custom platform channels).
- Peripheral role in Flutter, or central role on ESP32.
- Full coverage of multi-field SIG characteristics (Heart Rate, Blood Pressure). These get an escape hatch (section 6.3), not first-class types.
- Cross-implementation byte-exact protobuf encoding (signing or hashing). Use deterministic CBOR for that, or hash decoded fields.
- Selective retransmit and concurrent transfers per pipe (v1).

## 3. Stack and key decisions

| Concern | Decision |
|---|---|
| Schema language | proto3 with custom options (field numbers 50000+) |
| Rust protobuf (device) | `micropb`: no_std, bounded containers. `prost` needs `std` or `alloc`, so it isn't used on the device |
| Rust BLE | `trouble-host` on embassy, `no_std`, no heap |
| Rust codegen | `build.rs` (runs on the host, so it may use `std`): `prost-build` emits a `FileDescriptorSet`, `prost-reflect` reads custom options. Only the *generated output* must be no_std |
| Dart protobuf | `protobuf` and `protoc_plugin` for message classes |
| Dart BLE | `universal_ble` |
| Dart GATT codegen | Dart script (or protoc plugin) over the same descriptor set |
| Byte order | Little-endian for all bare values |

**Verify before relying on:** the `trouble-host` and `universal_ble` API details below are written from memory and documentation snippets. Check them against the versions you pin before building on them. Specific things to confirm are listed in section 12.

## 4. Repository layout (proposed)

```
proto/
  gatt_options.proto      # shared options + bare-value markers
  <device>.proto          # services and messages for the product
rust/
  gatt-pipe/              # no_std chunker, reassembler, CRC, credit window (host-testable)
  gatt-runtime/           # no_std: codecs, GattValue wrappers, event helpers
  gatt-gen/               # std: descriptor loading, validation, code emission (used by build.rs)
  firmware/
    build.rs              # calls gatt-gen
    src/
dart/
  ble_gatt_runtime/       # generic Characteristic<T>, codecs, reassembler
  tool/gen_gatt.dart      # generation script
  app/
```

The `.proto` files are the single source of truth. Generated output is never edited by hand. Keep `gatt-pipe` free of any BLE or embassy dependency so it can be unit-tested on the host with injected faults.

## 5. Schema

### 5.1 `gatt_options.proto`

```proto
syntax = "proto3";
package gatt;

import "google/protobuf/descriptor.proto";

enum Prop {
  PROP_UNSPECIFIED = 0;
  READ = 1;
  WRITE = 2;
  WRITE_NO_RSP = 3;
  NOTIFY = 4;
  INDICATE = 5;
}

enum Perm {
  PERM_NONE = 0;
  PERM_ENCRYPTED = 1;
  PERM_AUTHENTICATED = 2;
}

message ServiceOpts {
  string uuid = 1;     // canonical 128-bit string
  bool primary = 2;
}

message CharOpts {
  string uuid = 1;
  repeated Prop props = 2;
  Perm perm = 3;
  uint32 max_len = 4;       // bounded size; 0 on a protobuf type = data pipe
  string pipe = 5;          // pairs an RX and a TX characteristic into one pipe
  string custom_codec = 6;  // escape hatch, see 6.3
  uint32 max = 7;           // optional numeric ceiling for bare ints
}

extend google.protobuf.ServiceOptions { ServiceOpts service = 50001; }
extend google.protobuf.MethodOptions  { CharOpts characteristic = 50002; }

// Bare-value markers. Never serialized as protobuf. The generator maps each
// to a fixed raw little-endian encoding. The `value` field exists only so
// the file is valid protobuf.
message U8    { uint32 value = 1; }   // 1 byte
message U16   { uint32 value = 1; }   // 2 bytes
message U32   { uint32 value = 1; }   // 4 bytes
message I8    { int32  value = 1; }
message I16   { int32  value = 1; }
message I32   { int32  value = 1; }
message F32   { float  value = 1; }
message Bool  { bool   value = 1; }   // 1 byte, 0 or 1
message Utf8  { string value = 1; }   // requires max_len
message Bytes { bytes  value = 1; }   // requires max_len
```

### 5.2 Generator configuration

Device-side buffer sizes must be compile-time constants, so the generator takes a small config (a TOML file or `build.rs` arguments, not part of the proto):

| Key | Meaning | Suggested default |
|---|---|---|
| `chunk_max` | Max bytes in one pipe chunk's backing buffer | 244 |
| `pipe_msg_max` | Max reassembled message size per pipe, per direction | set per product |
| `window` | Max unacknowledged chunks in flight (N) | 8 |
| `xfer_timeout_ms` | No-progress timeout per transfer | 5000 |

The runtime still clamps the actual chunk size to the negotiated MTU minus overhead. `chunk_max` is only the upper bound for static buffers.

### 5.3 Example `device.proto`

```proto
syntax = "proto3";
package mydevice.v1;

import "gatt_options.proto";
import "google/protobuf/empty.proto";

message DeviceInfo { string fw_version = 1; fixed32 serial = 2; }
message Command {
  uint32 id = 1;
  oneof body { SetLed set_led = 2; Reboot reboot = 3; }
}
message SetLed { bool on = 1; uint32 rgb = 2; }
message Reboot {}
message Event { uint32 id = 1; bytes data = 2; }

// Standard Battery service, bare u8
service Battery {
  option (gatt.service) = { uuid: "0000180f-0000-1000-8000-00805f9b34fb" primary: true };

  rpc Level(google.protobuf.Empty) returns (gatt.U8) {
    option (gatt.characteristic) = {
      uuid: "00002a19-0000-1000-8000-00805f9b34fb"
      props: [READ, NOTIFY]
      max: 100
    };
  }
}

// Custom service: small protobuf read + chunked data pipe
service Device {
  option (gatt.service) = { uuid: "6e400001-b5a3-f393-e0a9-e50e24dcca9e" primary: true };

  rpc Info(google.protobuf.Empty) returns (DeviceInfo) {
    option (gatt.characteristic) = {
      uuid: "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
      props: [READ] perm: PERM_ENCRYPTED max_len: 32
    };
  }
  rpc Rx(Command) returns (google.protobuf.Empty) {
    option (gatt.characteristic) = {
      uuid: "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
      props: [WRITE_NO_RSP] perm: PERM_ENCRYPTED pipe: "main"
    };
  }
  rpc Tx(google.protobuf.Empty) returns (Event) {
    option (gatt.characteristic) = {
      uuid: "6e400004-b5a3-f393-e0a9-e50e24dcca9e"
      props: [NOTIFY] perm: PERM_ENCRYPTED pipe: "main"
    };
  }
}
```

Modeling convention: **service** is a GATT service, **rpc** is a characteristic. The request type is what the central writes. The response type is what the peripheral reads or notifies. `google.protobuf.Empty` means "nothing in that direction".

## 6. Semantics

### 6.1 Properties to generated API

| Proto says | Rust server gets | Dart client gets |
|---|---|---|
| `READ` | `set(T)` | `read<Name>() -> Future<T>` |
| `NOTIFY` or `INDICATE` | `notify(conn, T)` (async) | `<name>Updates -> Stream<T>` |
| `WRITE` or `WRITE_NO_RSP` | a typed variant in the generated server event enum (see 8.1) | `write<Name>(T) -> Future<void>` |

A method exists **only** if the property is declared. This is what produces the compile errors in the goal section. `T` is a native type for bare values (`u8`, `int`) and the generated message class for protobuf.

### 6.2 Bare types

When a request or response type is one of the `gatt.*` markers, the generator emits raw encoding instead of protobuf:

- Fixed-width ints and floats: little-endian, exact width.
- `Bool`: one byte, 0 or 1.
- `Utf8` and `Bytes`: raw bytes, **`max_len` required**.
- `max_len` is **rejected** on fixed-width types.
- Device types: `u8`, `u16`, `u32`, `i8`, `i16`, `i32`, `f32`, `bool`, `heapless::String<N>`, `heapless::Vec<u8, N>`. Dart types: `int`, `double`, `bool`, `String`, `Uint8List`.
- If `max` is set, the generated setter or writer clamps or `debug_assert`s. Pick one behavior and document it.
- Bare characteristics never go through the chunker.

### 6.3 Escape hatch: custom codecs

For multi-field standard characteristics, set `custom_codec: "heart_rate"` and use `gatt.Bytes` with `max_len`. The generator emits calls to a function the user supplies by that name (Rust: a function path or trait impl. Dart: a registered codec). The generator only wires it up. It doesn't validate the encoding.

### 6.4 Validation rules (generator must fail the build)

1. Declared props must be consistent with types: `NOTIFY` and `INDICATE` need a non-Empty response and Empty request. `WRITE` and `WRITE_NO_RSP` need a non-Empty request and Empty response. `READ` needs a non-Empty response.
2. Every characteristic has a valid UUID. Every UUID is unique within the file.
3. A protobuf-typed characteristic must have either `max_len > 0` (single read or write, no framing) or belong to a `pipe`.
4. A `pipe` group must contain exactly one characteristic that is written to and one that is notified.
5. Bare-type rules from 6.2.
6. `max_len` must fit in the minimum ATT payload for single-shot characteristics (20 bytes unless MTU is guaranteed). Warn if it exceeds that.
7. A `custom_codec` requires `gatt.Bytes`.
8. Each protobuf message used on a pipe must have bounded size in `micropb` (all strings, bytes and repeated fields have a max configured), and that bound must not exceed `pipe_msg_max`. Fail otherwise.

Known unchecked case: the generator can't verify that a SIG-assigned UUID matches its spec'd type (for example `gatt.U16` on `0x2A19` would compile and be wrong). Optionally ship a small table of known SIG UUIDs and their types and warn on mismatch.

## 7. Data pipe protocol

A pipe is a pair of characteristics sharing a `pipe` name: one written by the central (write without response), one notified by the peripheral. **Both directions use the same chunk format.**

### 7.1 Chunk format (proposed)

```
| seq u8 | flags u8 | [total_len varint, only if START] | payload ... | crc16 LE |
```

- `flags`: `START=0x01`, `END=0x02`, `ACK=0x04`, `NAK=0x08`. A chunk may have START and END both set.
- `crc16`: CRC-16/CCITT-FALSE over every preceding byte of the chunk. Reject chunks that fail.
- `seq` increments per data chunk and wraps at 256, independently per direction.
- **One transfer carries exactly one protobuf message.** `total_len` is the encoded message length, so no extra length-delimiting is needed inside the transfer.
- Max payload per chunk = negotiated ATT MTU minus 3 minus header minus 2, and never above `chunk_max`. Read the MTU after connect and recompute. Never assume 247.

### 7.2 Flow control

Notifications have no acks, and `universal_ble` serializes writes, so the pipe needs its own flow control.

- Sender may have at most **N unacknowledged data chunks** in flight (`window`, default 8).
- Receiver sends an `ACK` chunk (empty payload, `seq` = highest in-order seq received) at least every N/2 chunks and on END.
- ACKs for central-to-peripheral data travel on the notify characteristic, interleaved with the peripheral's own data chunks, and vice versa.
- On CRC failure or a seq gap, the receiver drops the transfer and replies with a `NAK`. Sender may retry the whole transfer.
- Per-transfer timeout (`xfer_timeout_ms` with no progress) aborts both sides.

### 7.3 Reassembly

On START, check `total_len` against `pipe_msg_max` and reject larger transfers immediately (static buffers, no heap). Append in-order payloads into the static buffer. On END, check the length, decode the protobuf, and hand it to the typed event or stream.

### 7.4 Device-side runtime shape

- `gatt-pipe` is a `no_std`, executor-agnostic state machine: bytes in, actions out (send chunk, deliver message, abort). It has no BLE or embassy dependency.
- A thin embassy layer owns the tasks and channels that connect it to trouble: one task consumes write events for the RX characteristic, one task sends notifications for the TX characteristic, and a timer handles timeouts.
- All buffers are statically allocated with sizes taken from the generator config.

## 8. Generated API shape

Keep generated code **thin**. Hand-write the runtime and have the generator emit only typed declarations and glue.

### 8.1 Rust (trouble-host)

`trouble-host` already provides typed characteristics through its `#[gatt_server]`, `#[gatt_service(uuid = ...)]` and `#[characteristic(uuid = ..., read, write, notify, ...)]` macros, where the value type implements `GattValue`. The generator's job is to emit those macro invocations with the right value types, plus the glue below.

Generated per service and characteristic:

- The `#[gatt_server]` struct listing every service, and one `#[gatt_service]` struct per proto service, with each characteristic's UUID, properties and value type taken from the proto. Properties are emitted only if declared, so an undeclared operation isn't offered.
- Value types:
  - bare ints, floats and bool map to the native type (`u8`, `u16`, ...).
  - `Utf8` and `Bytes` map to `heapless::String<N>` and `heapless::Vec<u8, N>`.
  - small single-shot protobuf messages map to a generated wrapper type implementing `GattValue` with `micropb` encode and decode.
  - pipe characteristics map to a fixed-capacity byte buffer (`heapless::Vec<u8, CHUNK_MAX>`) and are never exposed raw. The pipe runtime owns them.
- **Typed event enum.** Trouble delivers writes as GATT events in the connection loop, not as callbacks. The generator emits something like:

  ```rust
  pub enum DeviceEvent {
      Rx(Command),            // from the Rx pipe, already reassembled and decoded
      // one variant per WRITE or WRITE_NO_RSP characteristic
  }
  ```

  plus a function that matches an incoming event's characteristic handle, decodes the value, and returns the variant. User code matches on the enum and gets compile-time exhaustiveness.
- **Typed notify wrappers.** If trouble's characteristic handle exposes `notify` regardless of the declared properties, the generator also emits a thin wrapper type per characteristic that exposes only the declared operations (`set`, `notify`) with the proto's value type. If trouble already restricts these, the wrapper can be skipped.

### 8.2 Dart

```dart
class BatteryClient {
  BatteryClient(this._ble, this.deviceId);
  Future<int> readLevel();
  Stream<int> get levelUpdates;
}
```

Both sides use only the runtime for raw UUID access. Keep raw access private so nobody can bypass the types.

### 8.3 Security mapping

`PERM_ENCRYPTED` and `PERM_AUTHENTICATED` must map onto trouble's security and bonding support. The generator should emit whatever trouble offers for per-characteristic or per-link requirements and fail the build, with a clear message, if a requested permission can't be expressed. Confirm what trouble currently supports before promising these.

## 9. Schema version check

Protobuf tolerates unknown and missing fields, so firmware and an app built from different `.proto` revisions would decode silently with defaults. Mitigation:

- Generator computes a hash of the descriptor set (normalized) and emits it as a constant on both sides.
- A read-only `SchemaVersion` characteristic (bare `gatt.U32` or `gatt.Bytes`) exposes the firmware's hash.
- The Dart runtime reads it on connect and surfaces a typed mismatch error before any other traffic.

## 10. Implementation plan

1. **Schema and parsing.** Write `gatt_options.proto` and `device.proto`. In a throwaway Rust program, load a `FileDescriptorSet` with `prost-reflect` and print each service and characteristic with its decoded options. This proves custom options (including the nested `CharOpts` message) can be read. Confirm the same descriptor set works for Dart.
2. **Trouble spike.** Hand-write a minimal trouble server on the ESP32 with one bare `u8` Battery Level characteristic (READ, NOTIFY) and a `heapless::Vec` characteristic. Confirm it works in nRF Connect. Record exactly what the macros accept and what the characteristic handle exposes. The generator's output is shaped by this result.
3. **Validator.** Implement the section 6.4 rules once, in `gatt-gen`, so both generators trust the same checks.
4. **Rust generator and runtime, bare and single-shot protobuf characteristics.** Emit `#[gatt_server]` and `#[gatt_service]` code, `GattValue` wrappers, the typed event enum and the optional typed wrappers. Test on hardware, including standard Battery Level in the phone's OS.
5. **Dart runtime and generator, same scope.** Typed read, stream and write over `universal_ble`. Test against the ESP32.
6. **Data pipe.** Build `gatt-pipe` and the Dart reassembler. Test them against each other in-process on the host with injected loss, reordering, duplication and corruption. Then add the embassy layer and test on hardware.
7. **Schema version check.**
8. **Hardening.** Reconnects, MTU renegotiation, backpressure under load, timeouts, memory use.

## 11. Acceptance tests

- Compile-fail tests (`trybuild`) on the device crate for: wrong value type passed to a notify wrapper, notify on a characteristic without NOTIFY, and an unhandled event variant (exhaustiveness).
- Dart analyzer tests confirming the corresponding client methods don't exist.
- Generator unit tests: each validation rule has a failing `.proto` fixture and the expected error.
- Bare-value byte tests: golden bytes for every marker type, both directions, both languages, identical.
- Pipe tests (host): transfers of 1 byte, exactly one chunk, many chunks, and `pipe_msg_max`. Injected CRC corruption, dropped chunk, duplicate chunk, stalled ACK, oversize `total_len`. MTU of 23 and 247.
- `micropb` bound check: a message whose bounded size exceeds `pipe_msg_max` fails the build.
- Hardware check: standard Battery Level read and notify visible in nRF Connect and the phone OS with no custom client.
- Schema hash mismatch is detected on connect.

## 12. Open questions and risks

- **`trouble-host` specifics (verify first):**
  - Which `GattValue` implementations exist for the value types above, and what a custom implementation needs.
  - How write events are delivered and how to match them to a characteristic handle.
  - Whether the characteristic handle exposes `notify` regardless of declared properties.
  - How notifications are sent and what backpressure the API gives.
  - What per-characteristic security it can express.
  - Maximum characteristic value size and its limits at compile time.
- **`micropb` bounds.** Confirm it can express the bounds the generator derives from `max_len` and field options, and that its generated types work alongside `heapless`.
- **`universal_ble` write queue.** Per secondhand descriptions, it runs writes through a queue that waits for each platform write callback, even without response. That caps pipe throughput and also provides some backpressure. Measure on target phones before tuning `window`.
- **MTU variance.** Android needs an explicit request (ask for 247 or more). iOS negotiates automatically. Web and Linux behavior may differ. The chunker must read the actual value.
- **`prost-reflect` and custom options.** Confirm extension options are readable from descriptors as expected.
- **Dart generator choice.** Script over the descriptor set versus a real protoc plugin. Start with the script.
- **Pairing and bonding.** `PERM_ENCRYPTED` relies on the OS triggering pairing when an encrypted characteristic is touched. Apple and web need a `pairingCommand` in `universal_ble`. Decide which characteristic serves that purpose.
- **L2CAP CoC upgrade.** Trouble supports it on the device. If pipe throughput is insufficient, evaluate a client-side channel (`ble_plus` or platform channels) and replace the pipe transport while keeping the typed API unchanged.
