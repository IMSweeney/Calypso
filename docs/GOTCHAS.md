# Gotchas

Traps that cost real time here, and what to do about them. Grouped by area;
each one is symptom → cause → what to do.

## Flash

### A flash write or erase needs the other core parked

**Symptom.** Every erase and write fails — including the log partition's first
format — while reads work perfectly. A read of an earlier write comes back fine, so it
looks like broken flash or a bad partition.

**Cause.** `esp-storage`'s default `MultiCoreStrategy::Error` refuses every write
and erase while CPU1 is running, and on this build CPU1 is up from the
bootloader onwards. Reads are unaffected.

**Fix.** `FlashStorage::new(..).multicore_auto_park()` is not optional here. It
parks the other core for the duration of each operation.

### A walk of the log stops the sampler

**Symptom.** Reps in a real capture carry ~2.2 s jumps in their sample times, one per rep,
and the phone's once-a-second notification arrives in runs of four with 3.4–3.8 s of
silence between them — through a session in which nothing was wrong with the link.

**Cause.** Everything runs in one task (`join3` in `main.rs`), so a synchronous flash
walk on the BLE arm is the sampler not running: it is not a slow tick, it is no tick. Two
paths walked the whole log whenever they ran — `collect_summaries`, which scans every
record to find the summaries past a cursor, and `read_trace`, which scans every record to
find the chunks of one rep — and the first of them ran on *every* idle second, because a
spent batch asked for another collect and a caught-up stream collected zero summaries
every time.

**What to do.** Never add a walk to a path something else waits on, and ask for a record
by id rather than for the log: `FlashLog::for_each_from` finds a record by binary
searching the sectors on their first record id (a sector's first id only increases along
the ring), so the reader pays for one sector search and the records it came for.
`read_trace_from` uses that with the place the summary scan recorded, and a summary that
the log has not changed since is not collected again (`SummaryBatch::stale`, fed by
`LOG_APPENDS`). The place of a rep's chunks is a *cache*: the summary and chunks are
written together under the log's lock, so the chunks follow the summary, but a request
for a rep the cache has no place for must fall back to the walk rather than be refused.

### An erased sector header is not an erased sector

**Symptom.** Two ways to get this wrong. A ring that skips an erase because the sector's header
reads `0xFF` writes over bytes that survived underneath: the CRC fails, the sector reads as
corrupt, and the record is gone with nothing reporting it. And a reserve that erases ahead of the
ring takes history with it — the log holds that many sectors fewer, silently.

**Cause.** The sector header is written *before* any record is, so a sector with an erased header
holds nothing a reader is owed. It does **not** follow that the data below is erased: a write that
failed between the header and the records, or the power loss in that same window, leaves programmed
bytes under a blank header. A NOR write only clears bits, so writing over them produces a record
that reads back as garbage rather than as an error.

**Fix.** `advance_sector` checks the bytes rather than a claim about them: `is_erased` walks the
whole sector and the erase is skipped only when every byte reads `0xFF` — a millisecond against the
forty the erase costs, and it stops at the header for a sector that obviously holds records. The
reserve's *planning* may go on headers alone, because a mistaken plan can only cause an erase that
was not needed, never a missing one. The reserve's other cost is stated where it is chosen: the
sectors it erases are the oldest, so a reserve `n` deep is `n` sectors less history.

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
trait too, and `ReadStorage::read` copies through a whole-sector scratch buffer and accepts
any length. So a 22-byte NVS read worked, 8-byte sector and record headers worked, a
64-byte probe worked — and a 2459-byte record payload did not.

**Fix.** Every read goes through `flashlog::ReadBlocks::read_blocks`, which reads whole
64-byte blocks and copies out the window asked for — the shape the diagnostic probe had
proved worked. The test mock now advertises `READ_SIZE = 4` and **enforces** it (it used
to say `1` and accept anything, which is why a test already named
`unaligned_payloads_round_trip` never caught this).

**And the diagnostics were worth their keep:** naming the *phase* and *sector* narrowed
it to "a read in sector 6", and the probe reading that whole sector end to end from a
shallower frame is what proved the bytes were fine and pointed at the shape of the call
rather than the flash.

### NVS is empty, and stays in the table anyway

The BNO055 calibration profile used to live at offset `0x00` of the `nvs` partition,
and it was the whole of it — one region per 4 KB sector, because NOR flash bits only go
from 1 to 0, so a save is erase-then-write rather than an overwrite. A BLE device-name
blob at `0x1000` was the other, until the name became a compile-time constant.

Nothing writes that partition now: there is no profile to save once the chip is read raw,
and the log has its own. It stays in `partitions.csv` because removing it would move every
partition after it, and 4 KB nobody uses is cheaper than a layout change with nothing to
show for it.

## Stack and heap

### The linker stack is a remainder, and the log line measured the wrong end of it

**Symptom.** `Stack in use before the tasks: 39594 B` out of ~42 KB reads as a
device about to run out of stack, and the roadmap read it that way. It was
**headroom**, not use. The two are swapped by measuring from the wrong end: `Reset`
loads `_stack_start` into `a1` (esp-hal's `ld/sections/stack.x` puts `_stack_start`
at the *end* of the RWDATA region) and the stack grows down, so a stack pointer's
distance to `_stack_end` is the space left below it. In use on that build, at that
moment: a small fraction of it.

**Cause.** The stack is not a sized buffer. `dram_seg` is 128 KB here
(`0x3ffc0000..0x3ffe0000`); `.data` and `.bss` are placed from the bottom and
`_stack_start` is pinned to the region's end, so **every static and every heap
region carved out of DRAM makes the stack smaller** — it is whatever is left over.
Removing the Wi-Fi init *grew* it by most of what Wi-Fi had been taking, which is
the same fact seen from the other side. It has to hold `main`'s frame, every poll chain
under it, and the interrupt that switched to this task.

Below the stack, in the debug build, the largest single consumer is the
firmware's own **static heap region** (`heap_allocator!(size: 64 * 1024)` in
`main`). The rest is `.data` and `.bss` — trouble-host's packet pool, the esp-phy
tables, esp-rtos' scheduler, esp-hal, xtensa-lx-rt — plus the radio's own statics,
the embassy task `POOL` that `main`'s future lives in (so it is *not* on the
stack), and the generated GATT value buffers.

The static heap is the only one of those that gives back stack one-for-one: the
reclaimed DRAM region is outside `dram_seg` and costs the stack nothing, and
regions are filled first-fit in the order `main` registers them, so the surplus at
the top of the static heap is exactly what the stack does not get.

**The frames, which is what to watch.** The largest by far is `esp-storage`'s
4 KB `FlashSectorBuffer`, on every flash read, write and erase; in release LTO it
inlines into `flashlog`'s `read_blocks`, so the name in a frame listing moves
while the size does not. Next is the `SyncMessage` that the pipe and `pb` build by
value (`chunk_message`, `encode_summary`, `Session::send_*`, `keep_sending`).
Then the poll chains — `Join3::poll`, which release has all three poll bodies
inlined into, and `TaskStorage::poll`, which is `main`'s own. `CalypsoGatt::new_with_config`
takes a large frame but only at boot, and the largest radio/BTDM/PHY interrupt
frames land on whatever stack they interrupt.

So the deepest path is a poll chain plus one of those, well inside the stack in
release, and nothing allocates stack dynamically. **The number to watch is
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
  eye. Large arrays and large temporaries accumulate there without a murmur.
- **`Box::new(v)` builds `v` on the stack first** and then moves it, so a large
  value is large stack. Every big buffer here is allocated through `AllocVec` +
  `Box::leak`; `SampleRing` borrows its buffer for the same reason.

### Reading a frame size out of the ELF

The `entry a1, N` prologue is a function's stack use (N × 8 bytes), but on this
toolchain `objdump -d` prints **hex only for `.text`**: GNU objcopy/objdump
2.47.20260726 with esp-16.2.0 decodes `.rwtext` and the vectors, and nothing at all
in the IROM section. A grep over the listing therefore covers the interrupt
handlers and misses every Rust function, including the sector buffer above —
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
two `esp_alloc` regions — one reclaimed DRAM, one the 64 KB static region above —
and the buffer set is large: sample ring, 10 s rep window, two plotted-trace buffers, record
scratch, decode scratch, summary batch, and the live stream's queue. The radio takes most of
it before any of that. A raw trace read out of a record-mode build adds a window's worth of
samples, allocated on the request rather than at boot, and refused when the heap cannot spare
it alongside the headroom that constant asks for.

The figures to watch, and what they mean: `Radio up, heap free N B` says whether
the radio was cheap or expensive, and the free figure should then fall at each
setup stage — the detector, then the sync buffers — in the order the buffers are
allocated. What matters is the last of them: **under ~8 KB free is a device that
will not accept a connection.**

The sample ring is sized to what the detector actually asks for (10 s of samples)
rather than to a round number, and Wi-Fi's removal is what gave the radio's share
back. Do not grow any of this without reading those figures first.

## The IMU

### A status register that cannot advance is a gate that never opens

**Symptom.** The old build gated recording on the chip's calibration levels, and one of
them never reached 3: `sys` is the aggregate and counts the magnetometer, which this build
disables (no mag — steel barbell). A gate that waits on it is a gate that never opens.

**Cause and cost.** `is_ready()` ended up as `accel == 3 && gyro == 3`, which fixed the
symptom and hid the shape of the bug: the calibration profile was saved on the same
transition, so a gate that never opened also meant the accelerometer's offsets were never
written to NVS. After a reboot `gyro` came back at 3 (the chip re-derives it from
stillness) and `accel` at 0, which reads as "the save is broken" rather than "the save
never ran", and it cost a power cycle to tell them apart. **When a save looks broken,
first check that whatever triggers it can actually happen.**

The whole surface is gone now: an IMU in a non-fusion mode has no autocalibration at all
(§3.6, §3.11 of the datasheet), `CALIB_STAT` cannot advance, and the offsets are the
host's to estimate — see [`RAW_IMU.md`](RAW_IMU.md).

### A fusion output is not a sensor reading

**Symptom.** Moving the device off the chip's fusion left two things broken that nothing
reported: the rep detector was fed an acceleration it had been reconstructing from the
chip's *estimate*, and the 1 Hz level verdict read `gravity()`, which a non-fusion mode
does not have.

**Cause.** Fusion mode's outputs are derived — linear acceleration is "gravity removed"
by definition, and the gravity vector is the estimate it was removed with. Anything read
from them is an estimate's opinion, and in a mode that does not fuse there is nothing to
read at all: `linear_acceleration*`, `gravity*` and `quaternion` return `InvalidMode`
without touching the bus.

**Fix.** The filter estimates the attitude itself, from the gyroscope pulled onto gravity,
and every orientation the device reports is that estimate. If an orientation is ever read
off a chip again, ask first whether it is a measurement or an inference.

### A gyro bias learned only at rest is not learned during a set

**Symptom.** A hand-held set recorded windows that opened carrying 8 and 15 m/s of velocity,
and replayed from rest still reached 10 m/s horizontally on a bar that never went sideways.
The attitude looked plausible, and every still window in the same capture read nothing.

**Cause.** Both offsets were learned from one predicate: the accelerometer's reading had
stopped changing. That is the right question for the accelerometer's offset and the wrong one
for the gyro's — “at 1 g” is not true of a bar moving up and down — so a set with no 400 ms
rest in it never learned the gyro's bias at all. A degree per second then tilted the vertical
by six degrees across thirty seconds, which is a metre per second squared of gravity in the
horizontal and metres per second of velocity a window later. Correcting the attitude only
while still made it worse in the same direction: the one signal that could have pulled it back
was switched off exactly while the set was moving.

**Fix.** Learn the gyro's bias from the accelerometer's *direction*, which is gravity whether
the bar is moving or not: a bias turns the attitude while gravity stays put in the sensor's
frame, and a real rotation moves it through. A set now teaches the gyro between reps without
ever being called at rest, and the attitude is pulled onto gravity gently while moving as well
as hard at rest. The regression is `a_gyro_bias_that_arrives_during_a_set_does_not_walk_the_ground`.

### The IMU driver configures the accelerometer and not the gyroscope

**Symptom.** A plan that says "set both ranges in CONFIG mode" meets a driver whose
`AccConfig` covers `ACC_Config` and whose `GYR_Config` is not exposed at all.

**Cause.** `bno055` 0.4.0 writes page 1's `ACC_Config` through `set_acc_config` and reads
it back through `get_acc_config`; the gyroscope's register (`0x0F`) has no accessor and
the register writes behind it are private.

**Fix.** The accelerometer is set (±8 g, 62.5 Hz) and read back before the mode switch;
the gyroscope keeps the chip's defaults, **2000 dps at 32 Hz**, which cannot clip a
barbell and is recorded in `RepConfig` rather than left at 0. Neither range changes the
scale a trace decodes with — the data registers are fixed at 1 m/s² = 100 LSB and
1 dps = 16 LSB (§3.6.5.1, §3.6.5.3) — so the range is about clipping and internal noise
and nothing else.

### The first acceleration read after the mode switch is all zeros

**Symptom.** A record-mode window that opens at boot reports a hundred metres of travel in nine
seconds, and its first sample reads `0, 0, 0 m/s²` while the gyroscope beside it reads a plausible
rate. Every window after it is ordinary.

**Cause.** The chip's acceleration registers have not been written when the loop's first poll lands,
so that sample is a reading nobody took — and it arrives as a whole g of step at the head of the
first window, which the integration carries for the rest of the window.

**Fix.** The first sample is dropped when it reads no specific force at all, and the log says so. A
still sensor never reads zero, so a real first reading cannot be lost to it. The window that starts
at boot is still not a still window — the filter has not been still long enough to know which way is
down — but it is now the motion that is missing from it rather than the sensor's.

### A reset in the middle of a read leaves the bus locked

**Symptom.** One boot in about eight halts at 0.2 s with `I2c(Timeout)` unwrapped out of the
sensor's `init`, and the device stays dead until it is unplugged. Every other boot is normal, and
nothing in the log leads up to it because it is the first I²C transaction of the boot.

**Cause.** `Error::Timeout` is the bus timeout, not a missing ACK: SCL never came back. Resetting the
ESP32 in the middle of a 100 Hz read leaves the sensor holding SDA low, waiting for clocks that are
not coming, so the next transaction cannot even send a START. The fault is made by the reset, which
is why it hides on a normal boot and appears when the device is reset while it is sampling — a
reflash, a watchdog, a brownout.

**Fix.** A failed attempt is retried: esp-hal clears the bus on a timeout, so each attempt runs on a
clean one, and five of them cover a sensor that was mid-transfer or still coming up. If all five
fail the device reboots instead of halting, because a halted one needs a person to unplug it. Two
failures in eight resets before the change, ten clean boots after it — but the retry never fired in
those ten, so what is established is that the failure is no longer fatal, not that a retry clears
it.

### A rep only counts if it ends in a pause

**Symptom.** A set of reps stores one rep, or none, and the reps that survive are 3–8 s
long. Nothing in the app or in either log says a rep was discarded, so the set looks like
it was never done. **Find it with a test, not with a session**: the one real capture that
looked like this had stored nothing at all for two and a half hours, which was the
recording gate and not the detector (see the IMU section above). The mechanism below is
real and reproducible — a no-pause set is one span — but a missing set on hardware is
more often the gate, because the gate leaves no reps to look at.

**Cause.** A rep closed only after `|a|` stayed under 0.25 m/s² for 250 ms. A set nobody
pauses through never produces that, so the span stayed open and ran from the first rep
into the next: three reps of motion are already past `MAX_REP_MS` (8 s), and a span that
long was `return None` — thrown away whole, with no report and no log line. The one rep
per set that did arrive was whichever closure landed inside 8 s.

**Fix.** A rep is out and back, so it ends where the bar turns around, and a pause is
only one of the ways that happens. The detector integrates the acceleration into a
velocity and closes on the end of the out-and-back as well as on the rest. Two details
make that work: the accelerometer's bias is learned **only while the bar is still**
(otherwise the bias estimate eats the rep it is measuring), and so is the velocity
damping — either one running between two reps of a no-pause set quietly merges them
again.

**The gate had the same shape of bug.** The detector is not fed while the recording gate
is shut, and it used to be reopened by a status that flapped: a span after a flap was
measured from a timestamp the ring no longer held, one `RepWindowUnavailable` landed
1.1 s after the gate reopened, and every other flap cost a rep silently. Closing the gate
resets the detector, and that is the rule to keep: **whatever shuts a gate must also
forget any span measured across it.** The gate is a latch now — the filter settles once and
never un-settles — so nothing can shut it, and nothing can strand a span in the first place.

**Test it against a set that never pauses.** `sim/tests/sets.rs` has a no-rest set, five
reps back to back with rest only after the last: before the change it was one span of the
whole set, now it is five. That test also had to fix its own motion model — a sine is a
push that never comes back, and a detector tuned on one has nothing to learn about reps.
The generated motion lives in `calypso-sim`, which is where the detector's behaviour is
pinned now; `calypso-core/tests/fusion.rs` is the filter under it and
`calypso-core/tests/rep.rs` the trace on top — the angles, the integration, the span — on
hand-written samples with answers worked out on paper, so a failure there says the maths
is wrong rather than that a threshold moved.

### A turn is a speed, so a gentle return is invisible

**Symptom.** A rep whose lowering is slow and controlled is not stored at all. For a bench
the boundary sits between 0.12 and 0.14 m/s of peak return speed — at the very slow end
of a deliberate negative, and inside the range of a lifter lowering under control.

**Cause.** The detector delimits a rep by turning points, and a turn is the vertical speed
falling below `VEL_MIN` (0.07 m/s) for `T_TURN_MS`. A stroke that never reaches that speed never
turns, so it never becomes a leg, and no leg means no rep — regardless of how far the bar
actually went.

**Fix.** There is none yet, and this is a decision rather than an oversight: the turning
point model fixed stalls and ROM fidelity, and the cost was accepted. Closing a leg
on bounded travel as well as on a turn would cover it. Until then,
`sim/tests/detector_tolerance.rs::a_gentle_return_is_not_detected_yet` pins the behaviour
so it fails — loudly, with instructions — if it is ever fixed.

### A turn read off the whole velocity never happens on a real frame

**Symptom.** The device is recording — the sampler says the filter is settled and the detector
line prints a bias and an acceleration — but `legs` climbs slowly or not at all and nothing is
stored. `|v|` in that line never falls to the centimetres a turnaround should, while the
acceleration it is integrating looks small.

**Cause.** The turn test compared `VEL_MIN` against the *whole* velocity. A leg is a signed
*vertical* travel measured on Z, but X and Y carry the frame's residue, and the fusion's own
documentation allows a couple of degrees of it — `g·sin(2°)` is 0.34 m/s², which over a
one-second stroke is 0.34 m/s of horizontal velocity, five times `VEL_MIN`, and it stays there
because the detector damps velocity only while no leg is open. The bar turned; the speed did
not. Nothing the sensor reads is wrong: the vertical is clean, and the horizontal was never
part of the rep.

**Fix.** `Detector::vertical_speed` is what a turn is read on, and what `moved` is set from, so
a bar carried sideways while it turns is still turning. `Detector::speed` still reports the whole
velocity for the log — it is the number that says the frame is drifting — and it is deliberately
not the number the turn test reads. `sim/tests/turns.rs` pins a gyro offset that arrives with the
rep, which is how this arrives on a real device.

### A leg under the travel threshold consumes the rep before it

**Symptom.** A set stores one rep fewer than it should, or it splits a rep in two, and the
missing one is the drive that followed a bar settling back at its turnaround. Nothing logs a
rejection: the detector simply never pairs that drive with anything.

**Cause.** Every leg was allowed to replace or clear the pending one. A bar gives a few
centimetres back at lockout, which is motion in the opposite direction and fast enough to turn,
so a 3–6 cm leg arrived between a drive and its return — under `MIN_ROM_M`, so it could not pair,
but it replaced the drive it followed. The drive was then the one thrown away, and the return
that came after it had nothing to pair with.

**Fix.** A leg under `MIN_ROM_M` is dropped and leaves the pending leg alone. The one exception
is a same-direction leg with no rest between them, which is a stall inside a stroke: its travel
is added to that stroke whatever its size. `sim/tests/turns.rs` pins the settle at three sizes.

### A history ring answers a stale question with a default

**Symptom.** The velocity a rep reports as its starting velocity is exactly zero for every
rep longer than the ring, which at 100 Hz and 48 slots is 0.48 s — so every real rep. The
travel then reads long, because the integration begins as though the bar were still.

**Cause.** `Detector::velocity_at` searches a ring of recent samples for the newest at or
before a timestamp and returns zero when nothing qualifies. Asking it for a point that has
already been overwritten is not an error, it is a silent default. Anything that captures a
value "at the left edge" has to capture it **when the edge happens**, not when the rep
closes.

**Fix.** A leg records `v0` as it opens (`Detector::open_leg`), not at the pair. The same
trap applies to anything else read out of the state history.

### A fake rep's window is not the span it claims

**Symptom.** In a `--features fake-reps` build a summary says its window is 24–30 s and the raw
samples in it are 17–27 s: a trace shifted seconds earlier than the rep it belongs to, and cut
short before that rep's own end.

**Cause.** The window is sliced from `Detector::rep_state()`, and bench mode never feeds the
detector, so it answers with its initial `(0, LEVEL)`. `copy_window` then starts at the ring's
oldest sample instead of at the span's start, and stops when its output buffer fills instead of
at the span's end.

**Fix.** A window nothing detected has to name its own start: `--features record-mode` does,
because it opens each window itself and keeps the filter state from that moment. The fake-reps
branch now does too: a detected rep answers with a `DetectedRep` carrying its own start, and a
window nothing detected starts at its span's own start rather than asking a detector that never
ran.

### A rep window must not begin inside a rest

**Symptom.** Every rep after the first in a set reads high — 24% over travel — and the
velocity the trace starts from is a bar descending at 0.29 m/s.

**Cause.** Legs are contiguous by design: each turn is both the end of a leg and the start
of the next, which is what makes a turnaround with no pause either side two legs. The
consequence is that a leg can *open* in the middle of a hold, so the rep's span — and its
`MARGIN_MS` — begins while the bar is still finishing the previous stroke, and the
unexplained travel in the window is the previous stroke's tail.

**Fix.** A confirmed rest before the leg has moved abandons the open leg, and the next
one starts at the motion onset instead. The rule is: a turn continues a chain, a hold
breaks it. "Confirmed" is the filter's stillness — the reading holding at 1 g for 400 ms
— rather than a threshold on a single sample, so the rule does not depend on how slowly
the bar was set down.

### A hole in the timeline is bridged as one step

**Symptom.** A stored rep reports travel the bar cannot have made — one capture's largest
ROM is 1.28 m off a deadlift — and its trace times jump 2–4 s, then a burst of samples
1–7 ms apart, then the normal spacing.

**Cause.** The BLE arm walked the flash log, on the sampler's thread (see *A walk of the
log stops the sampler*, below): every store already in the log is a payload read, twice,
and the sampler does not run until the walk ends. `Ticker::every` advances its deadline by
one period rather than resetting it to now, so every deadline missed while the loop was
stopped fires the instant it resumes: the burst is that catch-up, and the jump is the gap
it leaves behind. `compute_rep` integrates the gap as one trapezoid, which adds seconds of
acceleration to the velocity at once; that rep climbs about 1 m in the 1.1 s after its
gap. Nothing downstream can see the gap — `Sample` carries a timestamp and no count of
what should have been there.

**The timestamps are not wrong.** Measured on the capture, the stamped intervals still
add up to wall clock and the point count is still about one per 10 ms, because the
catch-up hands the missed deadlines their samples back: what is missing is the *motion*
inside the gap, not the axis. A reading of the trace therefore cannot be repaired by
re-spacing it — the gap is a hole in what the device knows.

**Fix.** The sampler logs `sampler hole: … ms without a sample` where the gap is made, on
the boot clock, so the flash windows and link events above it are the candidate causes,
and raises `REPORT_KIND_SAMPLER_HOLE` so the phone says a rep from then may read long
while the person is still in the gym. The walks themselves are gone from both paths that
ran them uninvited (see *A walk of the log stops the sampler*); a *collect* still walks
the log once per connection and once per batch of summaries, which is a freeze worth
knowing about but not one that repeats.

**A gap is also inside the stored window, not just between reps.** The tracer copies a
window out of the ring, so the jump comes with it: of 61 replayed windows from one
capture, 54 had a hole wider than 50 ms or a burst of samples closer than 4 ms — the three
"real reps" in the fixture set had 158 ms, 2.1 s and 2.3 s of missing time inside them.
`replay_dump` now labels every window `hole N ms` or `burst N ms` and only counts the ones
with a timeline, because judging a detector on a window that does not describe its own
motion is not a measurement. What the integration should do with the time is
still open (`ROADMAP.md`): read a rep that spans a hole as short, or as unusable, or
remove the holes.

**A clean timeline is not the whole story.** The first rep stored after the holes were
fixed still cannot be replayed as a stream, and not because anything is wrong with it. The
window is the motion span with `MARGIN_MS` asked for before it and whatever had been
sampled after it — the tail is cut at the close — and on this rep even the 200 ms before
the onset are inside motion: the first sample carries 7.0 m/s² against a 0.6 m/s²
on-threshold, the last 1.1 against a 0.25 m/s² rest threshold. Neither end is rest, so the
detector fed that window cold closes no span at all.

**And the trace cannot reproduce its own summary.** The trace keeps every second sample of
the window, and on this motion travel depends on the step it is integrated at: re-running
`compute_rep` over the 20 ms trace reads 1.10 m where the summary stored 0.94, and every
second sample again (40 ms) reads 1.42. On a synthetic rep of the same length the same
decimation costs under 1%, so this is the real motion's own structure, not the maths.
What that means: a summary is a function of the samples it was computed from, and a stored
trace is not that sample set. So `sim/tests/real_capture.rs` pins what the bytes *are* —
identity, completeness, timeline, date — and a detector is still judged on synthetic
motion and on a live stream.

### A detected rep is a hole in the live stream

**Symptom.** A recording taken while the device is detecting has a gap of a few tens of
milliseconds at each rep's close: a jump in `t_offset_ms` inside a chunk, or a
chunk-to-chunk step past 80 ms — with `seq` unbroken and no `sampler hole` in the boot
log, because nothing was dropped and nothing passed 500 ms.

**Cause.** Closing a rep is `compute_rep` plus a flash write, and both run in the
sampler's own loop. The firmware is one task — sampler, BLE and housekeeping are branches
of one `join3` — so moving the write to the BLE branch would move the gap, not remove it:
same core, same task, and on ESP32 the write parks whoever issues it. The firmware
measures the pause as `append_ms`, which is the number a `sampler hole` line would show
if it were longer than the report's 500 ms threshold.

**Fix.** `DevMode` pauses the detector: while the stream runs nothing closes a rep, so
nothing writes flash. The detector is restarted from rest when the stream ends, because
a leg left open across the pause would close as one rep spanning the whole stream, and a
step measured from the sample before it would integrate the pause as motion. The erased
reserve is not refilled during a stream either — an erase parks the chip ~45 ms, and it
would be a hole in the one thing the client asked for.

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
radio took, from a 164 KB heap.

The build is BLE-only, so that distinction no longer comes up: `esp-radio`'s BLE
bring-up has a `#[cfg(coex)] assert!(coex_init() == 0, "coex_init failed")`, `coex`
is only set when the `wifi` and `ble` features are both on, and this manifest now
enables only `ble` — see "The radio is built for a Wi-Fi stack the firmware never
starts". The assert is not compiled, and there is no coexistence layer to fail.

**If BLE stops coming up after a radio dependency bump**, re-enabling `wifi` and
`coex` and calling `wifi::new` is the first thing to try: that is where this
device has been lost before, and it is why the trim is a `cargo check` change
until it has run on a board.

### The radio is built for a Wi-Fi stack the firmware never starts

**Symptom.** The image carries the Wi-Fi driver and the coexistence layer for a
device that only ever does BLE, and nothing anywhere complains.

**Cause.** `esp-radio`'s features are additive and the starting template's set was
never trimmed. `coex` is the trap in it: it arbitrates between Wi-Fi and BLE, so on
its own it changes nothing and only leaves a build-script warning (`coex is enabled
but wifi is not`) — it reads like a safety flag, and is not one.

**Fix.** BLE only: `features = ["ble", "defmt", "esp-alloc", "esp32",
"unstable"]`, as `calypso-esp32/Cargo.toml` says. `unstable` is not optional — the
build script refuses `ble` without it. With `wifi` gone, `smoltcp`, `embassy-net`,
`embedded-io` and `embedded-io-async` have no user in the manifest either.

## BLE identity and discovery

Four faults found in one afternoon of first contact between a real device and the
app. None were visible from either end alone, and each one now has a test.

### The advertisement must carry the service UUID

A client that scans **filtered by service** — which is how the app looks — only
ever sees advertisements, not scan responses. A device that does not advertise its
service is invisible to the app while a generic scanner lists it happily. The
advertisement carries flags + service + local name (30 of the 31 bytes for
`Calypso`), and a name too long for one advertisement leaves it empty rather than half-built — the firmware warns on an empty one, so a too-long name shows up in the boot log.

### The same UUID needs two byte orders

In the GATT tree a client sees the bytes as the firmware wrote them, so those are
canonical text; in advertising data a 128-bit UUID goes on air least-significant
byte first, so the same UUID must be *reversed* there. The firmware therefore
stores **wire order in both places** and a client reverses both — `40 88 13 df …
20 ca 00 00` is `0000ca20-b0cd-11ec-871f-d45ddf138840` to a client.
The UUIDs are declared once, in text form, in `proto/calypso_gatt.proto`;
`tools/gatt-gen`'s `uuid::to_wire_order` is the single place the reversal happens,
and the generated `calypso_core::gatt::uuid` carries the wire bytes (for the GATT
tree and for the advertisement, which wants the same array), the `Uuid` and the
text. `calypso-core/tests/advertising.rs` pins the advertised bytes against it, and
`gatt-gen`'s `examples/server` test pins the generated `Uuid` and wire bytes against
the same text.

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
thing with each characteristic's storage.)

**Fix, for the hand-built table.** GAP was built by hand in `calypso_core::ble` —
four lines, with the device name at `&[CharacteristicProp::Read]` — because using
the library's config would have made the table's tests unrunnable, which was worth
more than those four lines.

**Now.** The table is generated (`proto/calypso_gatt.proto` → `calypso_core::gatt`),
which uses `GapConfig` and `#[gatt_service]`, so this is the price rather than a
choice. The tests that build a table are `gatt-gen`'s two examples, and each is
**one test** that builds its table once and runs its checks as functions, because
nothing else in that process may build one.

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
— so it worked, and the sync service's messages and its acknowledgements were dead.
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

**How it was found.** A full frame written through the table and read back: it failed
at 244 bytes into a 244-byte buffer, and passes at 245. That check is now
`a_full_frame_survives_the_characteristic` in `gatt-gen`'s `examples/pipe`, which is
where the storage is sized.

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

**Still true with the sizes right.** The framing that made that wait ~3 s is gone — a
chunk is ~8 notifications of 244 bytes now, not ~150 of 14 — and the acknowledgement
still lands a median 5 s after the write, which is the whole `XFER_TIMEOUT_MS` budget.
Most requests in the real session were re-sent for it. What a re-send costs is no longer
the trace: the device answers the second request, streams it, and a second first chunk is
dropped by the reassembler. What it costs is the seconds — the wait before the first
chunk was 40% of the time the traces took, and the device acts on the write and replies
at once, so the seconds are on the air or in the phone, not in the protocol.

**Fix.** The schema's number on both sides — `SyncPipe.xferTimeoutMs`, the same
`--xfer-timeout` the device was generated with. A budget shorter than the peer's patience
is a request re-sent into work that is still happening; longer is only slower.

**What the number cannot fix.** A budget long enough to outlast a slow acknowledgement
is a trace that starts late by construction, and the wait has nothing to do with the
trace: the device streams as soon as the write lands, so a chunk for the rep being asked
for is already proof the request arrived. The ack is wanted to know the *request* was
received, and the answer to that question arrives twice — once as the ack, once as data.

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
arrives, and **macOS never sends one unasked** (`connection_manager.rs`:
`storage.att_mtu = 23` when a connection is set up). Fourteen bytes of payload per
notification is what that number means, so a 2 KB trace chunk was ~150 notifications.
The app asks for one now (`BleLink.requestMtu`) and the link reports 247, so 23 is only
what a client that says nothing gets.

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

**And it now says so.** `sync trace chunk: 8 frame(s) of a 2477 B message out, window
full at 8 — the rest goes when the acknowledgement lands` is the device's own view of a
wave that stopped short; read beside the app's `raw 0000ca21-… 242B` lines, the two ends
describe the same transfer. Nothing is logged when the window is not what stopped it — a
summary is one frame and always the whole message — so the line appears once per wave that
matters. (`Sender::awaiting_ack` is the flag behind it; a wave that never appears at all is
a transfer that never started.)
- **The characteristic is the ceiling and the ATT MTU only lowers it** (`pipe::frame_mtu`).
  A peer that has not said is not a peer that can only take 20 bytes: this device
  notified 193-byte payloads to this same client for as long as traces have existed, and
  the app's own 244-byte writes arrive whole. A peer that *did* negotiate a small MTU
  still gets frames that fit what it asked for.

**Corollary: a poll is traffic too.** The app used to read a characteristic once a second,
invisible next to that blast, and six of those reads were completing in the same
millisecond — queued behind the same congested link, in front of the writes the pipe was
waiting on. A poll keeps one request in flight for that reason, and today the app polls
nothing at all: the two characteristics it uses are a write and a subscription.

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

**Fix.** The calibration status stopped being a message at all: it became a characteristic
the app **reads** on its own clock, which is the design this trap argues for — and that
characteristic is gone too now, because there is no calibration to report. The rule it was
an instance of is what stays: anything a client can ask for on its own clock does not need
to be pushed, and what is not pushed cannot be pushed again by an acknowledgement of it.

The first attempt kept it on the pipe and had `Session` remember the levels it had told
that connection (`f868486`) — which worked, and put the reason in the sender rather than
in the message: *the pipe has no idempotent message*. A summary and a trace chunk each
advance a cursor, so each is news by construction, and calibration was the only thing on
the pipe that could say the same thing twice. Anything else added there needs the same
thought.

### Nothing arrives from a device whose clock is not set

**Symptom.** The app connects, says it is connected, and no rep ever appears. The
device's boot log has no `rep … stored` line either, so it looks like nothing was
ever recorded.

**Cause.** The device sends only summaries it can date, and it dates them from the
clock the client writes on connecting (`Time`, [APP_SYNC.md](APP_SYNC.md)). Reps
of the boot running now wait for that write rather than going out without a time, and
a boot that never saw a client has no clock at all — so a device switched on and used
with no app in that boot records reps that are never offered.

**Fix.** Nothing to do but connect: the app writes the clock as soon as the link is
up, and says `clock: device set to … s`. If it never appears, the client is a build
from before the characteristic existed and was refused at connect for the missing
characteristic — rebuild it from the same schema as the firmware. The device says so
on its side once per connection: `sync: no clock this boot, so no summary goes out
until the app sets one`.

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

The app keeps no mirror types at all: `model/rep.dart` is extensions that *derive* from
the messages (units, tilt ranges, what a rep's counts mean), so a field added to the proto
reaches the UI instead of being dropped by a hand-written conversion.

### A protobuf message is not a `const`

`package:protobuf` messages compare by *content* (`GeneratedMessage` overrides `==` and
`hashCode`), so a `ValueNotifier`/`ChangeNotifier` holding one behaves as expected — a
value pushed once a second does not notify when nothing changed. They cannot be `const`
though, so a test that wants a fixed message builds it in a `final`, not a `const`.

## Tests and tooling

### A log line's timestamp is the host's, not the device's

**Symptom.** The device's log showed 130–240 ms between `record: window … closed` and `flash rep
append: starting`, it was larger on one mounting than on another — and the marks the firmware then
put around the same code measured 38 ms, with nothing about the motion changing anything.

**Cause.** The `0.000 [INFO ]` prefix is stamped at the host end of the serial link, after the line
has been through the UART and `defmt`'s decoder. Subtracting two of them gives a range whose width
depends on what else was in flight, not a duration — which is why an interval that looked like a
mystery looked like a property of the mount instead.

**Fix.** Put both ends of an interval on the device's clock (`Instant::now()` in the sampler loop)
and log the difference as one number: `rep timing:`. The host timestamps are good for the order of
events and for nothing else.

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
`NSBluetoothAlwaysUsageDescription` (`ios/Runner/Info.plist`): without it the system
**terminates the app** the moment the plugin instantiates its central manager, which
is a crash on the first tap of Connect and on nothing else — the Simulator never
reaches it, because there is no CoreBluetooth there at all. Use a device or the Mac.

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
they come from the schema (`proto/calypso_gatt.proto`), which the device and the app
both generate from.
