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

### The linker stack is a remainder, and the log line measured the wrong end of it

**Symptom.** `Stack in use before the tasks: 39594 B` out of ~42 KB reads as a
device about to run out of stack, and the roadmap read it that way. It was
**headroom**, not use. The two are swapped by measuring from the wrong end: `Reset`
loads `_stack_start` into `a1` (esp-hal's `ld/sections/stack.x` puts `_stack_start`
at the *end* of the RWDATA region) and the stack grows down, so a stack pointer's
distance to `_stack_end` is the space left below it. In use on that build, at that
moment: 3,130 B.

**Cause.** The stack is not a sized buffer. `dram_seg` is 128 KB here
(`0x3ffc0000..0x3ffe0000`); `.data` and `.bss` are placed from the bottom and
`_stack_start` is pinned to the region's end, so **every static and every heap
region carved out of DRAM makes the stack smaller** — it is whatever is left over.
Removing the Wi-Fi init *grew* it from 22,532 to 41,932 bytes, which is the same
fact seen from the other side. It has to hold `main`'s frame, every poll chain
under it, and the interrupt that switched to this task.

Where the 128 KB goes in the debug build — 88,348 B below the stack, 42,724 B of
stack:

| Bytes | |
|---|---|
| 65,536 | the firmware's own **static heap region** (`heap_allocator!(size: 64 * 1024)` in `main`) |
| 14,552 | `.data` + `.data.wifi`: trouble-host's packet pool (4,036), esp-phy tables (1,944), esp-rtos' scheduler (1,408), esp-hal (2,100), xtensa-lx-rt (492) |
| 4,000 | radio/BTDM/PHY statics (`vflash_mem` 2,048, esp-radio, esp-phy) |
| 3,872 | the embassy task `POOL` — where `main`'s future lives, so it is *not* on the stack |
| 516 | the generated GATT value buffers (246 B each) |

The first row is 74% of everything the stack competes with, and it is the only
one that gives back stack one-for-one: the reclaimed DRAM region (98,768 B, at
`0x3ffe7e30`) is outside `dram_seg` and costs the stack nothing, and regions are
filled first-fit in the order `main` registers them, so the surplus at the top of
the static 64 KB is exactly what the stack does not get.

**The frames, which is what to watch** (debug / release):

| Frame | |
|---|---|
| 4,144–4,160 | `esp-storage`'s 4096-byte `FlashSectorBuffer` — **on every flash read, write and erase**; in release LTO inlines it into `flashlog`'s `read_blocks` (4,256 there) |
| ~2,200 | the `SyncMessage` the pipe and `pb` build by value (`chunk_message`, `encode_summary`, `Session::send_*`, `keep_sending`) |
| 896 / 4,688 | `Join3::poll` — release has all three poll bodies inlined into it |
| 1,312 / 2,000 | `TaskStorage::poll`, i.e. `main`'s own poll |
| 1,712 | `CalypsoGatt::new_with_config`, at boot only |
| ~1,700 | the largest radio/BTDM/PHY interrupt frames, on whatever stack they interrupt |

So the deepest path is a poll chain plus one of those — ~10–15 KB in release
against 44 KB — and nothing allocates stack dynamically. **The number to watch is
the deepest figure in the log line**, and it is measured rather than calculated:
`main` paints everything below its stack pointer with `0xA5` before
`esp_rtos::start`, and `stack_deepest()` scans up from the painted floor for the
lowest byte anything has written since. Two limits: the bottom 256 bytes are never
painted or scanned (the `stack-protector` guard is 60 bytes above `_stack_end`,
and a stack that got in there would have written it first), and the paint has to
happen before a task can be switched to inside an interrupt, since the interrupted
context's frame outlives the paint that wrote over it.

**Two consequences of the remainder.**

- `main` carries `#[allow(clippy::large_stack_frames)]`, even though the crate
  denies that lint at a 1 KB threshold. Anything large in `main` needs a human
  eye. A 3 KB array and a 2.2 KB temporary accumulated there without a murmur.
- **`Box::new(v)` builds `v` on the stack first** and then moves it, so a large
  value is large stack. Every big buffer here is allocated through `AllocVec` +
  `Box::leak`; `SampleRing` borrows its buffer for the same reason.

### Reading a frame size out of the ELF

The `entry a1, N` prologue is a function's stack use (N × 8 bytes), but on this
toolchain `objdump -d` prints **hex only for `.text`**: GNU objcopy/objdump
2.47.20260726 with esp-16.2.0 decodes `.rwtext` and the vectors, and nothing at all
in the 294 KB IROM section. A grep over the listing therefore covers the interrupt
handlers and misses every Rust function, including the 4,144-byte one above —
which is how "the largest frame is 240 units" survived so long.

Disassemble the section as a binary instead, which does work:

```console
xtensa-esp32-elf-objcopy -O binary --only-section=.text \
  target/xtensa-esp32-none-elf/debug/calypso /tmp/text.bin
xtensa-esp32-elf-objdump -b binary -m xtensa -D \
  --adjust-vma=0x400d0020 /tmp/text.bin | less
```

`0x400d0020` is `irom_seg`'s origin in esp-hal's `ld/esp32/memory.x`, and it is
where every address in the dump maps back to. The `entry` immediate decodes by
hand if a script is easier: three bytes `36 b1 b2` with `b0`'s low nibble `6` and
`b1`'s low nibble `1`, and frame = `(((b2 << 4) | (b1 >> 4)) & 0xfff) * 8`.

### `Detected a write to the stack guard value`

That is a stack overflow, and by the time it prints, the task is dead. The boot
log's stack figures say where, and by then the deepest one is the interesting one:
a guard hit means the deepest figure had been climbing towards the total for a
while. Note the radio's own tasks have small heap stacks too, so it is not always
the main one. If the deepest figure approaches the total, move a buffer to the
heap — or give the stack back some DRAM, which is one constant (see the entry
above).

The probes, in boot order:

- `Radio up, heap free N B`, then the setup lines, then `Stack before the tasks:
  N B used of M B, K B at the deepest` — just before `join3` starts.
- `BLE connection established (stack N B used of M B, K B at the deepest)` —
  depth inside the GATT task.
- the sampler line, which carries the same three stack figures beside the heap
  one. It prints when the IMU's state changes, and otherwise once a minute: the
  1 Hz version said the same six numbers 3,600 times an hour and buried everything
  else.

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
The UUIDs are declared once, in text form, in `proto/calypso.proto`;
`tools/gatt-gen`'s `uuid::to_wire_order` is the single place the reversal happens,
and the generated `calypso_core::gatt::uuid` carries the wire bytes (for the GATT
tree and for the advertisement, which wants the same array), the `Uuid` and the
text. `tests/advertising.rs` and `tests/gatt.rs` pin each side against the text.

Getting this wrong the other way round is how the device once exposed every
service as `408813df-…` while the app looked for `0000ca20-…`.

### trouble-host's `GapConfig` builds GAP once per process, not once per table

**Symptom.** A test that builds the GATT table panics with `StaticCell is already
full, it can't be initialized twice`, but only when another test in the same
process built one first. The firmware — which builds one table, once — is fine.

**Cause.** `GapConfig::Peripheral(name, appearance).build(&mut table)` is exactly
what a device with a read-only name wants, but it keeps the name in a
**function-local `static StaticCell<String<22>>`** (`trouble-host/src/gap.rs`),
initialised with `.init()`, and `static_cell` panics when a cell is initialised
twice. One table per process is assumed, and a host test suite is many tables in
one process. (`#[gatt_service]`, which the generated table uses, does the same
thing with each characteristic's storage — see `tools/gatt-gen/PLAN.md`, errata
E1.)

**Fix, for the hand-built table.** GAP was built by hand in `calypso_core::ble` —
four lines, with the device name at `&[CharacteristicProp::Read]` — because using
the library's config would have made the table's tests unrunnable, which was worth
more than those four lines.

**Now.** The table is generated (`proto/calypso.proto` → `calypso_core::gatt`),
which uses `GapConfig` and `#[gatt_service]`, so this is the price rather than a
choice: `tests/gatt.rs` is **one test** that builds the table once and runs its
checks as functions, and nothing else in that process may build one.

### A client must discover services before subscribing

CoreBluetooth's peripheral has no characteristics until `discoverServices` has
run, and the plugin reports a lookup against that empty list as
`characteristicNotFound` — naming a UUID that is perfectly correct, so it reads
as a device fault. `BleTransport.connect` discovers between connecting and
subscribing, and `FakeBleLink` throws in that order so the suite fails if the
call is dropped.

### CoreBluetooth caches the GATT database per address

**Symptom.** A corrected firmware still looks unchanged from the Mac — and, once the
cached tree is stale in a way the client's own checks miss, the connection fails with
an error that names nothing:

```
UniversalBleException: Code: UniversalBleErrorCode.unknownError,
  Message: The attribute could not be found., Details: 10
```

**Cause.** CoreBluetooth keeps a peripheral's services and characteristics keyed to
its **Bluetooth address**, and the plugin prefers that cache to asking again
(`UniversalBleAsyncServiceDiscovery`: `if let cachedServices = peripheral.services`).
Reflashing does not change the address, so a client keeps the tree it had — and
`setNotifiable` then resolves the characteristic *out of that tree*
(`peripheral.getCharacteristic(…)`) and writes the CCCD at the handle *it* has. That
handle is in another build's table, so the device answers ATT 0x0A, Attribute Not
Found; `universal_ble` maps a platform code that is not one of the names it knows to
`unknownError` and puts the raw code in `details`, which is why `Details: 10` is the
only part of that message that says anything. It is the ATT error, from the device.

On the device side the same connection looks healthy — `BLE connection established`,
and a subscription that never lands, because what was subscribed to was a handle this
table does not have.

**Fix.** The firmware's address carries two bytes of `SCHEMA_HASH`, so a build whose
table changed is a device no client has cached, and the rule needs nobody to remember
it. It was a literal before — *bump the address when the layout changes* — and it was
missed twice with the address unchanged: `01edf64` removed the gravity service and
`ca23`, `bee5c1e` turned the two characteristics into a pipe. What the hash cannot see
(the generator's own emission, which `gatt-gen/N` covers when it is bumped) still
wants a hand.

Two other levers, for when it happens anyway: turning Bluetooth off and on clears the
cache, and the app refuses a tree whose sync service carries a characteristic the
schema does not declare rather than subscribing on somebody else's handles.

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

**Fix.** The constant was derived from the table that was built, with a host test
asserting it equalled the table's own count. Then the generated table removed the
question: `#[gatt_server]` sums the attribute and CCCD counts from the service
declarations at compile time, so there is no number to keep in step.

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

**Since the pipe.** Both characteristics carry chunks rather than messages, and the
numbers are the schema's (`--chunk-max`, `--pipe-msg-max`): a message larger than one
value is cut up and reassembled by the other end, and the generated table asserts
`micropb`'s worst case for each message against the reassembly buffer *at build time*,
so a message that could not arrive is a failed build rather than a stream that stops.
What is left of the truncation is the stack's own: `gatt::assemble` truncates an
over-long PDU to the negotiated ATT MTU, which is why a chunk is sized from
`conn.raw().att_mtu()` and never from an assumed 247.

### A write that fills a chunk characteristic reads back empty

**Symptom.** The device stores a chunk and reads it back as nothing: the pipe sees an
empty write, refuses it as a frame that does not check out, and the app's transfer is
retried three times and then given up on. Nothing is logged, at either end.

**Cause.** `trouble-host`'s attribute table keeps a variable-length value as a fixed
buffer plus a length, and the length comes from the value the characteristic was *built*
with — `Vec::<u8, N>::default()`, whose length is zero. A write shorter than the buffer
takes `set_raw`'s variable-length branch and updates that length; a write that fills the
buffer *exactly* takes the equal-length branch, which copies the bytes and returns
without touching the length, so `table.get` reads `&value[..0]`.

**Fix.** The generator gives a pipe characteristic one byte more than the largest frame
it can receive (`PIPE_STORAGE_MAX`), so no frame can be exactly the buffer's size. The
spare byte is not slack — it is what keeps that branch out of reach.

**How it was found.** `calypso-core/tests/gatt.rs` writes a full frame through the table
and reads it back. It failed at 244 bytes into a 244-byte buffer and passes at 245.

### The app's trace timeout is about silence, not the size of the rep

**Symptom.** A long rep's trace stops part-way and the rep lands in the archive without
one, while the device is streaming it perfectly well.

**Cause.** The budget is *per gap*, not per stream. `BleTransport` restarts the timer on
every chunk, because how long a trace takes is a function of its length and the link, and
a whole-stream budget fails the large reps it was meant to protect. The log has a request
whose first chunk arrived 22 s after it was sent: inside a 30 s budget only because the
budget restarts, and a whole-stream one would have given up on a healthy device.

**Fix.** Leave it restarting. A stream that has gone *quiet* is the separate case, and
`chunkIdleTimeout` (700 ms) is what closes it — that is what ends a stream from firmware
that never set `total_chunks`.

### A request budget shorter than the peer's patience

**Symptom.** The app reports `trace …: request failed: Bad state: the device did not
answer: no acknowledgement of the request`, and the device logs the *same* rep requested
two or three times and then `sync trace chunk: unanswered after N attempts, dropped`. The
rep lands in the archive with no trace.

**Cause.** Two ends with different patience, and re-sends that are not free. The app
waited 2 s for a request's acknowledgement — a number its transport file had made up —
while the device waits `XFER_TIMEOUT_MS` (5 s) before it retries anything. Under a trace
stream the acknowledgement is queued behind the notifications already on the link: the
app's log has it landing ~3 s after the write, so every attempt timed out just before its
own ack arrived, and each re-send made the device *restart* the trace it was streaming.
The acks the app did receive read as `Acked.ignored`, because by then it had aborted the
transfer they were about.

**Fix.** The schema's number on both sides — `SyncPipe.xferTimeoutMs`, the same
`--xfer-timeout` the device was generated with. A budget shorter than the peer's patience
is a request re-sent into work that is still happening; longer is only slower.

**Related.** This is the same shape of bug as the window (127) and the frame size (23): a
number written by hand in one end that the schema already declares for both. When a
transfer misbehaves, check the numbers each end *thinks* it is using before reading the
protocol.

### A window of 127 is not flow control

**Symptom.** The device streams a trace and the app never moves: `sync trace chunk:
unanswered after 7 attempts, dropped` on the device, and on the app
`sync: no acknowledgement of the request (attempt 3 of 3)` for a request the device had
already answered. The app *is* receiving messages — summaries arrive and are logged — so
both ends look half-alive.

**Cause.** Two sizes in the send path, both wrong in the same direction — more traffic
than the link needed to see.

*The window.* It is how many chunks a sender may have in flight, and the device took the
127 it was generated with as licence to put a whole message on the air without waiting
for anything.

*The frame.* Each chunk was sized from `conn.raw().att_mtu()`, and this link reports
**23** — the spec's default, which trouble-host keeps until an Exchange MTU Request
arrives, and **macOS never sends one** (`connection_manager.rs`:
`storage.att_mtu = 23` when a connection is set up). Fourteen bytes of payload per
notification is what that number means, so a 2 KB trace chunk was ~150 notifications.

The two together sent those 150 as one blast. The phone's writes queue behind it,
including the acknowledgements the device was waiting for: thirty-five seconds of
retries later the device gave up on the chunk, the app's own request had timed out twice
in the same window, and the archive sync that request belonged to stopped where it was.
Summaries are *one* frame, which is why they kept arriving and made the app look
healthy.

**How to see it.** The app's log carries the frame size of everything the client
received: `raw 0000ca21-… 18B 55 00 …` is an 18-byte frame, and 18 bytes is what an MTU
of 23 leaves after the pipe's own overhead. Nothing else said so: the app's
`ATT MTU 515` is Apple's *write* limit, not the link's. The firmware logs both halves
now — `ATT MTU 23: notifications up to 244 bytes`.

**Fix.** Two, one per size.

- `--window 8`: the sender stops after 8 chunks in flight and the receiver acknowledges
every `window / 2`, so the acknowledgement *paces* the transfer instead of trailing
behind it — which is the whole point of the mechanism, and what 127 defeated.

  **And a message can be wider than the window.** A trace chunk is up to 11 frames, so a
transfer is sent in waves — and the acknowledgement that makes room is what has to
trigger the next wave. The device did neither: `keep_sending` returned early on a busy
pipe, so the rest of the wave sat in its buffer for ever while the app waited (8 frames
of 11, then thirty seconds of silence and a rep with no trace), and the trace cursor
advanced before it knew whether the send could even start, so a chunk asked for while
busy was *skipped*. Whoever flushes a message has to be callable while it is in flight,
and nothing may move a cursor until the send it is for has begun.
- **The characteristic is the ceiling and the ATT MTU only lowers it** (`pipe::frame_mtu`).
  A peer that has not said is not a peer that can only take 20 bytes: this device
  notified 193-byte payloads to this same client for as long as traces have existed, and
  the app's own 244-byte writes arrive whole. A peer that *did* negotiate a small MTU
  still gets frames that fit what it asked for.

**Corollary: a poll is traffic too.** The app's calibration read is once a second and
invisible next to that blast, but six of them were completing in the same millisecond —
queued behind the same congested link, in front of the writes the pipe was waiting on. A
poll keeps one request in flight for that reason.

### An acknowledgement is also a request for the next message

**Symptom.** One message repeats as fast as the link allows — about thirty a second on a
30 ms connection interval — while the bulk transfer behind it never moves. Nothing in
the device's log says so: every one of those transfers is acknowledged, so nothing
stalls, nothing retries, and the pipe looks healthy.

**Cause.** `keep_sending` runs on every write, and every message the device sends comes
back as an acknowledgement, which *is* a write. It asked for the calibration status
first and unconditionally, so its own acknowledgement was the trigger for the next copy:
status, ack, status, ack. The pipe carries one message at a time, so the same value also
held the pipe against the summaries queued behind it — a self-sustaining loop rather
than wasted notifications.

**Fix.** Calibration is not a message at all any more: it is a characteristic the app
**reads** (`0000ca24-…`, `proto/calypso_gatt.proto`), which is the design this trap
argues for. Anything a client can ask for on its own clock does not need to be pushed,
and what is not pushed cannot be pushed again by an acknowledgement of it.

The first attempt kept it on the pipe and had `Session` remember the levels it had told
that connection (`f868486`) — which worked, and put the reason in the sender rather than
in the message: *the pipe has no idempotent message*. A summary and a trace chunk each
advance a cursor, so each is news by construction, and calibration was the only thing on
the pipe that could say the same thing twice. Anything else added there needs the same
thought.

## Types

### The device keeps two types off the wire, on purpose

`proto/calypso.proto` describes the wire format, and both ends use those generated
types for anything that is stored, streamed or logged. Two types are deliberately not
the generated ones, because they are what the *maths* runs on rather than what travels:

- **`sampler::Sample`** — 20 bytes, fixed point, produced 100 times a second. The
  detector and the ZUPT integration read these directly; a message type here would be a
  conversion per sample.
- **`rep::TracePoint`** — 32 flat bytes (`pos: [f32; 3]`, `accel: [i16; 3]`,
  `quat: [i16; 4]`) against a `TraceSample` that carries a derived `t_offset_ms` and
  the wire's quantized ints. A rep's trace is up to 512 points.

What makes this more than tidiness: **a decoded message is ~2 KB** — a full trace chunk
is stored inline in a `TraceSample` list — and a frame that size is real stack on a
linker stack that is only a remainder (see the entry above). That is why
`pb::decode_record_into` reuses a caller-owned buffer, why the summary batch lives on
the heap, and why `stream_trace` carries the firmware's only `large_stack_frames`
exemption. Holding decoded messages in the sampling path would put kilobytes of
composed futures into every poll of it.

Everything else is the generated type, on the device too: `compute_rep` returns a
`msg::RepSummary` (the caller fills in the uuid it minted and the `chunk_count` its
storage will use), and a rep is stored, streamed and logged as the messages it is.

The app keeps no mirror types at all: `model/rep.dart` and `model/calibration.dart`
are extensions that *derive* from the messages (units, tilt ranges, the calibration
wording), so a field added to the proto reaches the UI instead of being dropped by a
hand-written conversion.

### A protobuf message is not a `const`

`package:protobuf` messages compare by *content* (`GeneratedMessage` overrides `==` and
`hashCode`), so a `ValueNotifier`/`ChangeNotifier` holding one behaves as expected — the
1 Hz calibration push does not notify when nothing changed. They cannot be `const`
though, so a test that wants a fixed message builds it in a `final`, not a `const`.

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

### The Dart test uuid generator is a port of the Rust one

Tests name a rep by index — `u(1)` in Dart, `fixture_uuid(1)` in Rust — and the two must
mint the same bytes, because a checked-in fixture carries the uuids the generator wrote
and the Dart suite asserts on them by index (`summary.uuid == u(i)`).
`calypso-app/test/support/test_uuid.dart` is a byte-for-byte port of
`calypso_core::uuid::fixture_uuid`: SplitMix64, four 32-bit words most significant byte
first, then RFC 4122 version and variant bits. **Change one, change the other**, or every
fixture-named test fails at once and the port stops being checkable.

### Attach an expectation before the failure it expects

**Symptom.** A test that drops a connection to fail an in-flight request reports an
*unhandled* error, or its `expectLater` never sees the error at all.

**Cause.** `expectLater` is what attaches the matcher to the future. If the future
completes with an error before anything is listening, the framework reports it as
unhandled rather than delivered to the test.

**Fix.** `final expectation = expectLater(pending, throwsA(…));` *before* triggering the
failure, then `await expectation;` — `ble_transport_test.dart`, the dropped connection.

### A widget test restarts nothing unless it unmounts first

Pumping the same widget type again matches the existing element, so the `State` is reused
and `initState` — and any sync it kicks off — never runs. A test that means "the app was
reopened" has to `await tester.pumpWidget(const SizedBox.shrink())` first
(`rep_list_test.dart`).

### A host GATT table cannot check a characteristic's properties

`trouble-host`'s table exposes a characteristic's value and type, not its properties, and
`table.set` does not consult them either. So a host test cannot tell a read-only
characteristic from a writable one: what keeps the declared properties honest is that
they come from the schema (`proto/calypso.proto`), which the device and the app both
generate from.
