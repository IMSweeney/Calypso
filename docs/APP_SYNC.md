# App sync

How the app gets reps from the device: the BLE interface, the sync protocol, and
the archive the client keeps.

The client is the **archive of record**. The device's log is a ring that erases its
oldest sectors, so the app keeps everything it has ever synced; the device is a
buffer between sessions.

## The device's BLE interface

Service `0000ca20-b0cd-11ec-871f-d45ddf138840` — advertised, so a scan can filter on it:

| UUID | Direction | Carries |
|---|---|---|
| `0000ca21-…` | notify | everything the device says: chunks of a `SyncMessage`, and the acknowledgements of what the app sent |
| `0000ca22-…` | write-without-response | everything the app asks for: chunks of a `SyncRequest`, and the acknowledgements of what the device sent |
| `0000ca25-…` | notify | the device's own faults, as a `Report` |
| `0000ca23-…` | write | the device's clock, as unix seconds — the app sets it on every connection |
| `0000ca26-…` | read | the IMU as the sampler last read it, as an `ImuState` |

**One characteristic each way for messages, and that is the design.** The body of the
message says what it is — a `summary` or a `trace` chunk — so nothing here
distinguishes a response from a push, and a new kind of message is a new arm of the
oneof rather than a new characteristic, a new UUID and a new subscription. The
acknowledgements travel on the characteristic they are *not* about, which is what
lets two carry both directions.

The faults are the exception on the other side of that rule, and the reason the
pipe could not carry them. They *are* events, so they are pushed — but not as pipe
transfers, for three reasons that all come from what a transfer costs: the pipe carries
one message at a time, so a fault would queue behind the trace it is waiting on; the
most useful faults are the pipe's *own* failures (`notify failed`, a transfer given up
on), which cannot be reported by the thing that is broken; and the pipe has no
idempotent message (`GOTCHAS.md`, "An acknowledgement is also a request for the next
message"), while a fault that keeps happening is the same three bytes over and over.
So it is a value-sized characteristic carrying a kind and nothing else — one
notification, no acknowledgement, no pacing, and the app owns every sentence. The
device holds what it raised while nobody was listening and sends it when a subscriber
appears, which is how a log that would not mount reaches the phone of the person who
turns up an hour later.

The clock is the other one, and the same argument the other way round: it is a state
the app **writes**, once per device boot. On the pipe it would queue behind the
summaries it exists to date, because the device holds this boot's summaries until it
has a clock — see [Time](#time). It is required at connect like the rest — a device
missing any of them is refused, by name, before anything is read or written — and
the clock and the faults are the two whose absence would otherwise be silent.

The IMU's own values are the one **diagnostic** in the table: the sample the rep
detector is fed — specific force, the attitude the filter computed for it, and when
the sampler read it — at the fixed point a stored trace carries, so a readout can be
compared against a rep's own numbers. The device republishes it ten times a second,
so a reader gets the most recent sample whenever it asks, and `t_uptime_ms` says how
recent. The attitude is the firmware's rather than the chip's: a non-fusion mode has
no orientation to report ([`RAW_IMU.md`](RAW_IMU.md)).

The app reads it once per connection and logs it; **nothing on screen shows it yet**.
A generic BLE client — nRF Connect and the like, or a script on a laptop — is the
other reader, and gets the same value. The characteristic is known to the app rather
than required: one a build does not know makes every attribute past it a different
table's, which is why adding one to the firmware needs an app that already names it,
while a device from before it existed still syncs reps.

GAP is the other service: `0x2A00` (the device name) and `0x2A01` (the appearance),
both **read only**. The name is a compile-time constant in the firmware. The
advertisement carries the flags, the service UUID and the local name.

### The pipe

Neither characteristic carries a whole message. A message is cut into chunks of at
most 244 bytes, each framed and carried in one write or one notification, and
reassembled at the other end — `tools/gatt-pipe` (Rust, the device) and
`calypso-app/lib/gatt/pipe.dart` (Dart, the app) are the two implementations, checked
against the same cases in `tools/gatt-pipe/tests/vectors/pipe.txt`. The frame:

```text
| seq u8 | flags u8 | [total_len varint, START only] | payload | crc16 LE |
```

`START` and `END` say where a chunk is in a message; `ACK` and `NAK` are the
acknowledgements. What the pipe buys, and what it costs:

- **A trace chunk is one transfer, not three points a notification.** A whole
  `TraceChunk` (48 samples, ~2 KB) crosses as a few notifications, and
  `total_chunks` still tells the app when it has the whole trace — that delimits a
  *trace*, not a transfer.
- **Nothing is lost quietly.** Writes have no response and notifications no
  acknowledgement of their own, so the count is the acknowledgement. Either end
  gives up on a message after three attempts spaced by the schema's
  `--xfer-timeout` (5 s), which the device and the app are both generated with: a
  shorter budget re-sends a request into work that is still happening.
- **One message at a time, and at most eight chunks of it on the wire.** A transfer
  is finished by its acknowledgement, so neither end starts the next message until
  the one before it is done: the device orders what it owes the app — the trace it
  asked for, then the summaries — and the app sends one request at a time. The
  *window* is the flow control for the message itself: the sender stops at 8 chunks
  in flight and the receiver acknowledges every `window / 2`, so a large message is
  paced by acknowledgements rather than sent as a burst.
- **Chunks are sized from the link, or from the characteristic when the link says
  nothing.** The device reads the ATT MTU off the connection and frames each chunk
  to fit it — but `23` is the spec's default and what a central reports when it
  never sent the Exchange MTU Request (macOS never does), so a connection reporting
  23 is framed at the characteristic's 244 bytes instead. A peer that *did*
  negotiate something smaller gets frames that fit it. The app asks its own stack
  for an MTU after connecting (`requestMtu`) and clamps to the same ceiling, so
  neither end ever sends a frame bigger than the characteristic holds.

## Transports

Everything except the radio sits behind one interface, so the whole data path runs
in `flutter test` and in the Simulator (which has no CoreBluetooth at all):

```dart
abstract class RepTransport {
  Future<List<pb.RepSummary>> summaries();
  Stream<pb.RepSummary> watchSummaries();
  Future<List<pb.TraceComputedPoint>> trace(String uuid);
  Future<void> close();
}
```

The types are the generated ones throughout, down to the plot: the archive stores
records and the UI reads fields off the messages, so a field added to
`proto/calypso.proto` reaches the screen without anyone remembering to copy it.
What `lib/model/` holds is extensions that *derive* from the messages — physical
units, tilt ranges, what a rep's counts mean.

## Summary sync

Two paths, one destination: the archive.

- **Catch-up on connect.** `BleTransport` collects summaries from the moment it is
  built, before anything asks for them, so `archiveFromTransport` drains what it
  holds: for each rep the archive does not already have, request its trace and
  append its records.
- **Live.** The list screen subscribes to `watchSummaries()` once and writes each
  summary to the archive as the device reports it, so a rep that closes while the
  app is connected appears without another Connect. The subscription is not
  per-connection: the transport's stream outlives a link, and summaries arrive on it
  only while one is up.
- **Incremental, on the device's side.** The device keeps a cursor (`SummaryCursor`)
  of how far its subscriber has been brought up to date, so re-subscribing sends
  only what is new, and it advances the cursor only for a message the pipe took.
  A batch that drained the log is not read again until something is stored, so a
  subscriber that is up to date costs the device nothing to keep.
  Nothing goes out to a subscriber that is not there: the firmware asks the server
  whether the characteristic is subscribed rather than inferring it from a notify
  that always reports success. What the batch does not send this tick is still in
  the batch, and the cursor has not moved past it.
- **The clock first.** The app writes the device's clock as soon as the link is up,
  because the device holds every summary of this boot until it has one, and the
  write dates the reps it already stored as well as the ones to come. Nothing it
  cannot date is sent at all — see [Time](#time).
- **Wrap-safe.** Old reps disappear from the device's ring but not from the client's
  archive. The cursor is clamped to the log's actual count, so a device that wrapped
  re-syncs from the oldest id it still has.
- **Idempotent, at the log.** Both paths ask the archive what it holds and skip
  that, so a rep summarised twice — around a reconnect, say — is written once. The
  writes are chained, so the two paths cannot interleave on the same rep either.
- A dropped connection costs nothing already archived: the stream goes quiet and the
  next connect catches up.

Both paths end in the same place, and the list is built from that place: the UI
holds no rep state of its own.

## Time

The device has no RTC, so the only time it knows is the one the app writes: unix
seconds, once per connection, on the `Time` characteristic.

- **An epoch belongs to one boot.** The device stores what its uptime zero was worth
  on the wall clock, and throws it away at the next boot; the app sets it again every
  time it connects, so nothing is lost by that. It is what turns a stored rep's
  uptime into a time: a rep keeps `t_start_ms`, and its boot's epoch dates it.
- **The log holds it, the RAM copy is the live one.** Every boot opens with a
  `BootClock` record (`epoch_s` 0, which is what stops a boot that never had a clock
  from being read as the one before it) and the client's write replaces it. A record's
  clock is the newest one **at or before** it, in order, which is the only rule a
  reader of a dump can apply as well as the device.
- **Every summary the app receives carries `t_unix_ms`**, so the archive needs no
  side file to know when a rep happened, and the list is ordered and labelled by it.
- **Untimed reps are not sent.** A record's clock is the newest clock record at or
  before it, so an untimed summary is one stored before its boot's clock was written.
  For a past boot those reps were offered in that boot's own session — the write
  happens at the start of a session and a session re-reads the log from the start —
  so they are in the archive already, unless that session did not finish, in which
  case they are stepped over for good. A summary of the boot *running now* is
  different: it is only waiting for the write, and the batch waits with it, since one
  write dates the whole boot and the reps already stored are dated by it too.
- **It says so.** `clock: epoch … (uptime … ms)` when the write lands, and one
  `sync: no clock this boot, so no summary goes out until the app sets one` per
  connection where nothing can be sent.

## Faults

What the device says when something goes wrong, on its own initiative, as a `Report`:
a `ReportKind` and nothing else.

- **A kind, not a sentence.** The device's own sentence stays in its console log; the
  app turns the kind into words (`calypso-app/lib/state/device_reports.dart`), which
  is the only place a kind is spelled out. So a kind names a **class** of failure, not
  an incident, and adding detail to one on the device side does not change what the app
  shows.
- **An event, not a state.** The characteristic is notify-only and nothing reads it
  back: there is no "and it is now fine" message, and a latest-one-you-can-read would
  be a state the device would have to call current. What the app shows is the latest
  fault of this connection and how many have arrived.
- **Closing one closes its kind.** The banner has a close button, and what it closes is
  the class of failure rather than the single arrival: the device repeats a fault that
  is still happening once a second, so a dismissal a repeat could undo would undo
  itself. A different kind opens it again, and so does a new connection.
- **Every arrival is in the app log**, as `device REPORT_KIND_…: <sentence>` — the kind
  name so that a line here can be matched to a firmware log or a bug report, and the
  sentence so the log reads without the app in front of you. Closing the banner stops
  neither.
- **Once a second, however often it happens.** A fault is latched at the source and
  coalesced per kind (`calypso_core::report`), so an IMU that has stopped answering at
  100 Hz is one line a second rather than a hundred, and a *different* kind is news at
  once. Nothing is retried: a fault the link would not carry now is not one the next
  attempt says better.
- **Nothing is lost while nobody is listening.** The latch keeps what was raised before
  the app subscribed — including everything raised during a boot that failed before a
  client ever arrived — and the device sends it on the first tick after the
  subscription lands. A log that would not mount is therefore something the app can
  show, rather than a `warn!` on a console nobody has open.
- **It is a value-sized characteristic for a reason.** A `Report` is two bytes against
  a `max_len` of 20, so it can be notified on any link, at any moment, without the
  pipe's pacing — see the interface section above for why the pipe could not carry it.

## Trace sync

The app writes a `TraceRequest { kind, rep_uuid, all_chunks, chunk_index }` (as a
transfer of its own) and the device streams chunks in response, one per transfer:
`TraceChunk` messages for the plotted trace and `RawChunk` ones for the raw trace.

- **`kind` names which trace.** `TRACE_KIND_COMPUTED` is the default and the one the
  app asks for: the plotted path, which every rep has. `TRACE_KIND_RAW` is what a dev
  session recorded, and a rep that has none reads back empty rather than as the other
  kind — a client reads `computed_chunks` / `raw_chunks` off the summary and knows
  which it may ask for.

- **One transfer per stored chunk.** The log stores 48 points per record (sized to
  fit a flash sector) and each record is one message: it crosses as a few
  notifications of at most 244 bytes. How many notifications a message takes is the
  pipe's business, not the client's.
- **The device knows where the records are.** A rep's chunks are the records after
  its summary, and the summary scan records that place as it hands the summary out,
  so the usual request costs a walk of that rep's records rather than a pass over
  the log — which, on the device, is the difference between milliseconds and seconds
  of the sampler not running ([docs/GOTCHAS.md](GOTCHAS.md)). A request for a rep the
  device has no place for is found by walking the log, so nothing is refused.
- **End of stream.** Every streamed chunk carries `total_chunks`, the number of
  chunks in that stream, so the client knows which is the last. A streamed chunk is
  a stored one now, so that number is also the summary's `computed_chunks`; the field
  stays because a *stream* is what the client is reassembling, and a record in the
  log leaves it 0.
- **Reassembly is tolerant.** `TraceAssembler` keys chunks by index, so out-of-order
  arrival works, a repeated index replaces the earlier one, and chunks for another
  rep are ignored. `missingIndices()` reports gaps so a single chunk can be
  re-requested.
- **The raw trace is the whole window at full rate**, so it is several times the
  plotted one: about 19 KB for a nine-second rep against 5 KB, and 8 to 21 chunks
  rather than 3. It exists to be replayed rather than drawn, which is also why the
  device holds it differently — the plotted trace is read into the session's buffer,
  and the raw one needs a buffer of its own, allocated on the first raw request
  rather than at boot.

### Dev mode

Raw traces are **recorded only while a client asks for them**. The client writes a
`SyncRequest`'s `dev_mode` arm with `enabled`, and from then on every rep the device
stores keeps its raw trace too; `enabled: false` stops it.

- **An operation, not a value.** Nothing reads it back, and the device forgets it at
  every boot rather than persisting it: a switch left on by accident fills the log
  three times as fast, and forgetting it costs a reflash rather than a discovery.
- **It travels on the pipe**, like a trace request, rather than on a characteristic
  of its own — see [the interface](#the-devices-ble-interface) for why an operation
  is not a state.
- **The raw trace is recorded per boot, from the moment it is set.** Reps stored
  before it are plotted-only, and they say so: `raw_chunks` is 0.
- **What the samples are is [`RAW_IMU.md`](RAW_IMU.md)'s subject.** Until the chip is
  read raw, the sampler's samples carry no angular rate, so a trace recorded now
  exercises the path rather than replaying faithfully.

## The archive

`HistoryStore` is the client's copy of everything synced, and **the only source of
truth for what the app shows**:

- Records live in `history.pblog` — the same `[u32 LE length][record]` framing as
  fixtures and captures, so history, a fixture and a dumped log are one format
  ([`FIXTURES.md`](FIXTURES.md)).
- **What exists is read out of the log**: `contains(uuid)`, the list, its order and
  each rep's trace all come from the records. Nothing else can make a rep exist, and
  nothing else can stop one existing — which is also what makes re-syncing a no-op
  rather than a second copy of a rep.
- **The list is a day at a time.** It opens on today, its arrows walk one day per
  press, and the heading says which day that is — `Yesterday` for one back, the date
  otherwise. Forward stops at today, because nothing has happened there. A calendar
  button beside the arrows comes back to today from any page in one press, since the
  arrows would take that many presses; it is disabled while the screen is on today,
  and the page follows the clock rather than pinning a date, so a screen left open
  across midnight turns over. A page holds its reps newest first, each row labelled
  with the time of day: the date is the heading's, and the seconds are not, since a
  set is reps a minute or less apart. A record that carries no time at all — a capture
  of a boot whose clock was never set — is on no day, so no page shows it. A build
  seeded from a fixture opens on the day that fixture was recorded rather than on
  today, since a replay is not a day ([`FIXTURES.md`](FIXTURES.md)).
- **A rep's time is in its record.** The device dates every summary it sends
  ([Time](#time)), so the archive holds when each rep happened and there is no
  second file to keep in step with it. The list is ordered by that time rather than
  by the order the archive was written in, which is what makes a catch-up land in
  the order it happened rather than in the order it arrived.
- **Whether a rep can be replayed is in its summary.** `raw_chunks > 0` is "this
  rep's raw trace is in the archive", and the list says `replayable` on the row.
  Nothing else is claimed per rep: the trace's thinning is derivable from the counts
  (`computed_len` against a full-rate window), and the summary carries what the
  detector was given rather than a flag per possibility.
