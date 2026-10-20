# Gotchas

Traps that cost real time here, and what to do about them. Grouped by area;
each one is symptom → cause → what to do.

## Flash

### A flash write or erase needs the other core parked

**Symptom.** Every erase and write fails — including the log partition's first
format — while reads work perfectly. The stored calibration reads back fine, so it
looks like broken flash or a bad partition.

**Cause.** `esp-storage`'s default `MultiCoreStrategy::Error` refuses every write
and erase while CPU1 is running, and on this build CPU1 is up from the
bootloader onwards. Reads are unaffected.

**Fix.** `FlashStorage::new(..).multicore_auto_park()` is not optional here. It
parks the other core for the duration of each operation.

### `Rep log unusable` in the boot log

The line carries the reason, and the reason decides what to do:

| It ends with | What it means |
|---|---|
| `no data/undefined partition in the flashed table` | The flashed table is not the one in `calypso-esp32/partitions.csv` (`data`/`undefined`, 0x190000, 0x270000). Fix by flashing the table with the app — `cargo run` from `calypso-esp32/`, a plain `espflash flash`, or the merged image from `dist/`. The firmware dumps the table it actually read. |
| `partition table unreadable: Invalid` | It read fine and is not a table. Same fix. |
| `partition table unreadable: StorageError` | Reading the table failed at the flash level. |
| `log partition: StorageError` | The flash failed while mounting the ring. |

A mount failure costs that boot only: the ring is mounted again for each append, so
a rep can still be stored once a later mount works.

**The interesting part is the `Rep log probe` line that follows a `log partition`
failure**, and it exists because the vendor partition wrapper flattens every storage
error into `partitions::Error::StorageError` — which cannot tell a flash that
*refused* an operation from one that failed it. The probe re-reads sector 0 with the
raw `FlashStorage`, whose `FlashStorageError` does distinguish them
(`OtherCoreRunning`, `IoTimeout`, `NotAligned`, …). The bytes it prints are the other
half:

- the read fails → its error is the real one;
- it reads all `ff` → the ring is unformatted, so `mount` was **formatting** it, and the
  erase is what failed;
- it reads a header → the partition was formatted, so `mount`'s **read scan** failed.

A write is deliberately not probed: repeating the erase to see whether it works is
not a diagnostic, it is the bug.

### A flash read's *length* must be a multiple of `READ_SIZE`

**Symptom.** Reps stop being stored as soon as there is something in the log to read:
`rep … could not be stored: log partition: reading records in sector 6: StorageError` —
and the boot mount fails the same way (`Rep log unusable`) once the ring holds records,
while a freshly erased ring mounts fine.

**Cause.** `flashlog` is generic over `N: NorFlash`, so its reads resolve to
`ReadNorFlash::read` — and `esp-storage`'s implementation **rejects any offset or length
that is not a multiple of `READ_SIZE`** (4 on the ESP32) with `NotAligned`. The
partition wrapper folds that into `partitions::Error::StorageError`, so the message says
nothing. The log reads records *by their length*: the CRC walk and the payload copy are
however long the record happens to be, which is never guaranteed to be a multiple of 4.

The asymmetry that made this hard to see: the **same region** implements the *other*
trait too, and `ReadStorage::read` (which the NVS calibration blob read resolves to)
copies through a whole-sector scratch buffer and accepts any length. So a 22-byte NVS
read worked, 8-byte sector and record headers worked, a 64-byte probe worked — and a
2459-byte record payload did not.

**Fix.** Every read goes through `flashlog::ReadBlocks::read_blocks`, which reads whole
64-byte blocks and copies out the window asked for — the shape the diagnostic probe had
proved worked. The test mock now advertises `READ_SIZE = 4` and **enforces** it (it used
to say `1` and accept anything, which is why a test already named
`unaligned_payloads_round_trip` never caught this).

**And the diagnostics were worth their keep:** naming the *phase* and *sector* narrowed
it to "a read in sector 6", and the probe reading that whole sector end to end from a
shallower frame is what proved the bytes were fine and pointed at the shape of the call
rather than the flash.

### NVS holds one thing, in one sector

The BNO055 calibration profile starts at offset `0x00`, and that is the whole of
it now. It used to share the partition with a BLE device-name blob at `0x1000`
— a second sector, so that erasing either could never touch the other — until the
name became a compile-time constant (the app can no longer rename the device).
NOR flash bits only go from 1 to 0, so a save is erase-then-write, not an
overwrite, and the sector erase is why one region per sector was ever the layout.

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
- the sampler line, which carries `stack N B` beside the heap figure. It prints
  when the IMU's state changes, and otherwise once a minute: the 1 Hz version said
  the same six numbers 3,600 times an hour and buried everything else.

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

## The IMU

### `sys` never reaches 3 in IMU mode, and nothing should wait for it

**Symptom.** The boot log says `imu acc:3 gyro:3 mag:0 sys:0 — rep recording paused
until the IMU is calibrated`, for ever: no rep is ever recorded, and the app's hint
never clears.

**Cause.** `CALIB_STAT`'s `sys` bits are the chip's aggregate and **count the
magnetometer**, which this build disables (IMU mode, no mag — steel barbell). So `sys`
stays at 0 however well the accelerometer and gyroscope are calibrated, and readiness
required it at 3.

**Fix.** `is_ready()` is `accel == 3 && gyro == 3`. `sys` and `mag` are still reported,
for a console that says which sensor is behind.

**It cost a power cycle to see the rest of it.** The calibration profile is saved on the
transition to `is_ready()`, so a gate that never opened also meant the accelerometer's
offsets were never written to NVS: after a reboot, `gyro` came back at 3 (the chip
re-derives that from stillness) and `accel` at 0 — which reads as "the save is broken"
rather than "the save never ran". When a save looks broken, first check that whatever
triggers it can actually happen.

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

### trouble-host's `GapConfig` builds GAP once per process, not once per table

**Symptom.** A test that builds the GATT table panics with `StaticCell is already
full, it can't be initialized twice`, but only when another test in the same
process built one first. The firmware — which builds one table, once — is fine.

**Cause.** `GapConfig::Peripheral(name, appearance).build(&mut table)` is exactly
what a device with a read-only name wants, and it replaces the hand-built GAP
service. But it keeps the name in a **function-local `static
StaticCell<String<22>>`** (`trouble-host/src/gap.rs`), initialised with `.init()`,
and `static_cell` panics when a cell is initialised twice. One table per process is
assumed, and a host test suite is many tables in one process.

**Fix.** GAP stays hand-built in `calypso_core::ble` — four lines, with the device
name at `&[CharacteristicProp::Read]`. Using the library's config would have made
`tests/gatt_table.rs` unrunnable, which is worth more than those four lines: that
suite is how the table is checked at all.

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

When *nothing arrives at all*, the app writes the session to `calypso.log` (in its
container, path printed at startup), and the shape of that file narrows it down:

| The file holds | It means |
|---|---|
| nothing at all | Logging is broken, not the link — the first line is written by the appender at startup. |
| `scanning`, then nothing | The scan never found the device, or the connect failed; both log their reason. |
| `subscribed`, but no `raw` | The device accepted the subscription and never sent — the CCCD trap below. |
| `raw` but no `rx` | The plugin received the value and the transport's per-characteristic filter dropped it: `raw` is logged *below* that filter. |
| `rx`, and the UI still says nothing | The bytes arrived and the app misread them. |

`subscribed` is the app's side of the handshake. The device's side is a CCCD write,
and the firmware reports it as `app subscribed to summary notifications` — which is
evidence, unlike a notification's result (see below).

### Notifications need a CCCD slot per characteristic

**Symptom.** A client subscribes, the write is accepted, the device logs that the
notification was delivered — and nothing ever arrives. Both our app and an
off-the-shelf scanner saw the silence, while another characteristic (the gravity
one the table had then) notified happily.

**Cause.** `AttributeServer`'s `CCCD_MAX` is how many characteristics a client can
subscribe to, and the firmware passed `1`. The server keeps one CCCD table per client
with exactly that many entries, fills it with the first CCCDs it finds in the
attribute table, and **drops the rest without a word**: `set_notify` matches no
handle and does nothing, `should_notify` is then false, and
`Characteristic::notify` returns `Ok(())` *without sending anything*. That one slot
went to the gravity characteristic, which came before the sync service in the table
— so it worked and `0xca21` / `0xca23` (calibration, summaries, traces) were dead.
Every host test passed: they check the table and the encoders, not that a
subscription survives. (The gravity characteristic is gone now, and the sync service
has one notifiable characteristic rather than two — so the count cannot be got wrong
in the same way again. It is still derived, and still checked against the table.)

**Fix.** `calypso_core::ble::NOTIFIABLE_CHARACTERISTICS`, passed as the server's
`CCCD_MAX`, with a host test asserting the constant equals the number of CCCDs
`build_gatt_table` actually created.

**Corollary: a notification's result is not evidence.** Because `notify` returns
`Ok` when nobody is subscribed, `calibration status delivered to the app` printed
happily once a second to a device that was sending nothing, and the summary cursor
advanced through the whole log into the void. The firmware now asks the server
(`is_subscribed` in `main.rs`) and logs the subscription where it can be known.

### A trace chunk can be too big to notify, silently

**Symptom.** The device stores reps and the app lists them, but opening one says it has
no trace (`rep xx: no trace in the archive`), and the app's BLE log shows the stream
stopping part-way. Nothing is logged at either end.

**Cause.** A payload that does not fit the characteristic it is notified on fails to
notify *silently*: `Characteristic::notify` is never reached, because the payload cannot
be built. The trace-chunk characteristic was typed for a *summary* (128 bytes) while
`sync::SYNC_CHUNK_MAX` is 193 for a 3-sample chunk — so a chunk past 128 bytes simply
ended the stream. Two things hid it: the notification was never attempted, so no
radio-level error was possible; and the worst-case test bounded chunks against the MTU
(244) rather than against the characteristic (128).

**Fix.** The characteristic is sized to `SYNC_CHUNK_MAX`, the firmware's store for it
with that, and `stream_trace`'s payload too — plus tests that make the sizes impossible
to get wrong quietly: the GATT table test types the notify characteristic as
`BleVec<SYNC_CHUNK_MAX>`, and the encoder test bounds a worst-case chunk, summary *and*
calibration status against it. `stream_trace` now also *says* when a chunk does not fit,
rather than returning in silence.

**Since.** There is one notify characteristic for everything the device sends
(`sync.messages`), so there is one ceiling to keep ahead of rather than one per
message kind, and the three senders encode into one shared buffer. The remaining way
to get this wrong is the *other* direction: a request the app writes is rejected by the
stack if it does not fit `sync.requests`, and the app sees the failure.

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
