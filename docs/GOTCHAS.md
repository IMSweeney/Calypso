# Gotchas

Traps that cost real time here, and what to do about them. Grouped by area;
each one is symptom → cause → what to do.

## Flash

### A flash write or erase needs the other core parked

**Symptom.** Every erase and write fails — including the log partition's first
format — while reads work perfectly. The stored calibration and device name read
back fine, so it looks like broken flash or a bad partition.

**Cause.** `esp-storage`'s default `MultiCoreStrategy::Error` refuses every write
and erase while CPU1 is running, and on this build CPU1 is up from the
bootloader onwards. Reads are unaffected.

**Fix.** `FlashStorage::new(..).multicore_auto_park()` is not optional here. It
parks the other core for the duration of each operation.

### `Rep log unusable` in the boot log

**Cause.** Almost always a stale partition table on the device: the `log`
partition from `calypso-esp32/partitions.csv` (`data`/`undefined`, 0x190000,
0x270000) is missing.

**Fix.** Flash the table along with the app — `cargo run` from `calypso-esp32/`,
a plain `espflash flash`, or the merged image from `dist/`, all write it. The
firmware now names the cause and, for a missing partition, dumps the table it
actually read, so the log says which one it got.

### NVS is not one big blob

Two separate regions, deliberately: the BNO055 calibration profile starts at
offset `0x00`, the BLE device name blob at `0x1000` — the second flash sector — so
erasing one can never touch the other. NOR flash bits only go from 1 to 0, so a
save is erase-then-write, not an overwrite.

## Stack and heap

### `main`'s stack is a remainder, and it is invisible to clippy

**Cause.** `main` runs on the linker's stack, which is not a sized buffer:
`_stack_end` is wherever the RWDATA sections ended, so **every static and every
heap region carved out of DRAM makes it smaller.** It was 22,532 bytes; removing
the Wi-Fi init made it 41,932 — the same change, twice the stack. It has to hold
`main`'s frame, every `join3` poll, and interrupt contexts.

**Two consequences.**

- `main` carries `#[allow(clippy::large_stack_frames)]`, even though the crate
  denies that lint at a 1 KB threshold. Anything large in `main` needs a human
  eye. A 3 KB array and a 2.2 KB temporary accumulated there without a murmur.
- **`Box::new(v)` builds `v` on the stack first** and then moves it, so a large
  value is large stack. Every big buffer here is allocated through `AllocVec` +
  `Box::leak`; `SampleRing` borrows its buffer for the same reason.

Check frames without hardware by reading the `entry a1, N` immediate out of the
ELF (N × 8 bytes):

```console
xtensa-esp32-elf-objdump -d target/xtensa-esp32-none-elf/debug/calypso \
  | grep -oE 'entry\s+a1, [0-9]+' | awk '{print $3}' | sort -n | tail
```

At the time of writing the largest frame is 240 units (1920 bytes) and nothing
allocates stack dynamically, so no single function is the problem.

### `Detected a write to the stack guard value`

That is a stack overflow, and by the time it prints, the task is dead. The boot
log's `stack N B` figures say where. Note the radio's own tasks have small heap
stacks too, so it is not always the main one. If a figure approaches ~20 KB, move
a buffer to the heap — there is no stack to grow into.

The probes, in boot order:

- `Radio up, heap free N B`, then the setup lines, then `Stack in use before the
  tasks: N B` — `main`'s frame, measured just before `join3` starts.
- `BLE connection established (stack N B)` — depth inside the GATT task.
- the 1 Hz sampler line, which carries `stack N B` beside the heap figure.

### A log that just stops is usually the heap

**Symptom.** The serial output simply ends. It reads exactly like a hang.

**Cause.** `Vec::resize` on an exhausted heap calls `handle_alloc_error`, which
panics — and the panic handler used to be `loop {}` with no output.

**Fix / how to read it.** The handler now prints `*** PANIC: ... ***`. The heap is
two `esp_alloc` regions (98,768 B reclaimed DRAM + 64 KB static) and the buffer
set is large: sample ring, 9 s rep window, two 512-point trace buffers, record
scratch, decode scratch, summary batch. The radio takes ~54 KB of it before any
of that.

The figures to watch, and what they mean:

| Line | Roughly |
|---|---|
| `Radio up, heap free N B` | ~109 KB free after the radio came up. This is the number that says whether the radio was cheap or expensive. |
| `Rep detector ready, heap free N B` | ~45 KB free. |
| `Sync buffers ready, heap free N B` | ~22 KB free. **Under ~8 KB** is a device that will not accept a connection. |

The 30 s sample ring was 36 KB more than the device needs (the detector never
asks for more than 9 s) and is now 12 s; Wi-Fi was removed and gave back most of
the radio's 54 KB. Do not grow either without reading these numbers.

## The radio

### The BLE controller has its own task and stack

Default 4 KB (`esp_radio::ble::Config::task_stack_size`), allocated from the heap,
and **doubled to 8 KB here**. A fault inside it looks nothing like a `main`
problem: the stack pointer in the panic dump points into DRAM (≈`0x3FFD…`), not
at the linker stack. It has the smallest default stack of anything on the board
and is the only one sizeable from here.

### There is no Wi-Fi initialisation

`wifi::new` is not called: this is a BLE-only device, and initialising Wi-Fi
started a task with its own stack plus driver buffers — most of the ~54 KB the
radio took, from a 164 KB heap. **If BLE ever stops coming up, put `wifi::new`
back first.**

Worth knowing, because the code and the notes have disagreed here: `esp-radio`'s
BLE bring-up contains `#[cfg(coex)] assert!(coex_init() == 0, "coex_init failed")`,
that assert needs the adapter functions Wi-Fi registers, and
`calypso-esp32/Cargo.toml` currently enables both the `coex` and `wifi` features —
the string *is* present in current builds. BLE still comes up, so `coex_init`
evidently returns 0 without Wi-Fi being initialised. Either the features get
trimmed or the reason gets documented; see [ROADMAP.md](../ROADMAP.md).

## BLE identity and discovery

Four faults found in one afternoon of first contact between a real device and the
app. None were visible from either end alone, and each one now has a test.

### The advertisement must carry the service UUID

A client that scans **filtered by service** — which is how the app looks — only
ever sees advertisements, not scan responses. A device that does not advertise its
service is invisible to the app while a generic scanner lists it happily. The
advertisement carries flags + service + local name (30 of the 31 bytes for
`Calypso`); a longer name falls back to the scan response.

### The same UUID needs two byte orders

In the GATT tree a client sees the bytes as the firmware wrote them, so those are
canonical text; in advertising data a 128-bit UUID goes on air least-significant
byte first, so the same UUID must be *reversed* there. The firmware therefore
stores **wire order in both places** and a client reverses both — `40 88 13 df …
20 ca 00 00` is `0000ca20-b0cd-11ec-871f-d45ddf138840` to a client.
`calypso-core/src/ble.rs` explains the rule; `tests/gatt_table.rs` and
`tests/advertising.rs` pin each side separately.

Getting this wrong the other way round is how the device once exposed every
service as `408813df-…` while the app looked for `0000ca20-…`.

### A client must discover services before subscribing

CoreBluetooth's peripheral has no characteristics until `discoverServices` has
run, and the plugin reports a lookup against that empty list as
`characteristicNotFound` — naming a UUID that is perfectly correct, so it reads
as a device fault. `BleTransport.connect` discovers between connecting and
subscribing, and `FakeBleLink` throws in that order so the suite fails if the
call is dropped.

### CoreBluetooth caches the GATT database per address

**Symptom.** A corrected firmware still looks unchanged from the Mac.

**Cause.** CoreBluetooth caches a peripheral's services and characteristics keyed
to its **Bluetooth address**, and `universal_ble` prefers that cache to asking
again. Reflashing does not change the address.

**Fix.** The firmware bumps its random address whenever the GATT layout changes
(see where the address is set in `main.rs`), and the app's error message names
both this cause and the stale-firmware one. Turning Bluetooth off and on clears
the cache.

### Diagnostic that actually found these

Make each side describe the other. The firmware logs the UUIDs it exposes, in both
byte orders, at boot (`sync service uuid … (wire), … (as a client sees it)`), and
the app reports the tree it discovered — `device offers …, not …`. Two rounds of
guessing from `characteristicNotFound` messages, which name a correct UUID, said
nothing about the device.

### A trace chunk can be too big to notify, silently

`sync::SYNC_CHUNK_MAX` (193 bytes) is the worst case for a 3-sample chunk, but the
trace-chunk characteristic is typed `BleVec<SYNC_MSG_MAX>` (128 bytes, sized for a
*summary*). A chunk whose encoding exceeds 128 bytes fails to notify, and the
stream just stops — mid-trace, with no error. Real 3-sample chunks are ~115–126
bytes, so it is close enough to bite. **Open**; the fix is sizing the
characteristic to `SYNC_CHUNK_MAX` plus its GATT table test.

## Tests and tooling

### `cargo test` does not work in `calypso-esp32`

The firmware target is `no_std` and Rust's test harness needs `std`. Host-testable
logic lives in `calypso-core`, which is where the tests run from.

### Real I/O never completes under a widget test's fake clock

**Symptom.** `pumpAndSettle timed out` with a spinner still running.

**Cause.** Reading a file or an asset in `initState` leaves real async work
pending, and the fake clock never advances it.

**Fix.** Inject the seam: the list screen takes a `RepArchive` and a
`RepTransport`, so tests pass an in-memory archive and a fixture-backed transport
that reads synchronously. `tester.runAsync` would only paper over it.

### Goldens are host-dependent

Anti-aliasing differs between machines, so a golden failure on another machine is
expected rather than alarming. `flutter test --update-goldens`, then review the
diff.

### A log file that exists and stays empty

`Logger.root.level` defaults to `Level.INFO`, and records below the root level are
dropped **before** any appender sees them. `lib/logging/setup.dart` sets
`Level.ALL`; without it the file opens fine and nothing lands in it. Tests that
want to read records call `captureLogs()`, which unhooks itself at the end — a
listener otherwise outlives the test that added it.

### Platform BLE permissions

macOS is sandboxed and needs `com.apple.security.device.bluetooth` in
`macos/Runner/*.entitlements` (already set); without it the plugin reports
`unsupported`, with nothing pointing at the entitlement. iOS needs
`NSBluetoothAlwaysUsageDescription` (already in `Info.plist`). The iOS Simulator
has no CoreBluetooth at all, so use a device or the Mac.

### Generated Dart is checked in

`calypso-app/lib/protocol/calypso.pb*.dart` is committed, so a fresh checkout
tests without `protoc`. Re-run `calypso-app/tool/gen_proto.sh` only when
`proto/calypso.proto` changes, and commit the result. Regenerating adds new
`SyncMessage` oneof arms, which makes the app's `switch` over them
non-exhaustive — the analyzer will say where.
