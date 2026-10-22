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
| `0000ca24-…` | read | the IMU's calibration state, as a `CalibrationStatus` |

**One characteristic each way, and that is the design.** The body of the message
says what it is — a `summary` or a `trace` chunk — so nothing here distinguishes a
response from a push, and a new kind of message is a new arm of the oneof rather
than a new characteristic, a new UUID and a new subscription. The acknowledgements
travel on the characteristic they are *not* about, which is what lets two carry
both directions.

Calibration is the exception, and says what the rule is about: a level is a
**state**, not a message about something happening, so it is a value the app reads
when its UI wants it rather than a body pushed at the device's choice of moment.
See [`CALIBRATION.md`](CALIBRATION.md) for what the levels mean.

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
  Future<List<pb.TraceSample>> trace(String uuid);
  Future<void> close();
}
```

The types are the generated ones throughout, down to the plot: the archive stores
records and the UI reads fields off the messages, so a field added to
`proto/calypso.proto` reaches the screen without anyone remembering to copy it.
What `lib/model/` holds is extensions that *derive* from the messages — physical
units, tilt ranges, the calibration wording.

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
  Nothing goes out to a subscriber that is not there: the firmware asks the server
  whether the characteristic is subscribed rather than inferring it from a notify
  that always reports success. What the batch does not send this tick is still in
  the batch, and the cursor has not moved past it.
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

## Trace sync

The app writes a `TraceRequest { rep_uuid, all_chunks, chunk_index }` (as a transfer
of its own) and the device streams `TraceChunk` messages in response, one per
transfer.

- **One transfer per stored chunk.** The log stores 48 points per record (sized to
  fit a flash sector) and each record is one message: it crosses as a few
  notifications of at most 244 bytes. How many notifications a message takes is the
  pipe's business, not the client's.
- **End of stream.** Every streamed chunk carries `total_chunks`, the number of
  chunks in that stream, so the client knows which is the last. A streamed chunk is
  a stored one now, so that number is also the summary's `chunk_count`; the field
  stays because a *stream* is what the client is reassembling, and a record in the
  log leaves it 0.
- **Reassembly is tolerant.** `TraceAssembler` keys chunks by index, so out-of-order
  arrival works, a repeated index replaces the earlier one, and chunks for another
  rep are ignored. `missingIndices()` reports gaps so a single chunk can be
  re-requested.

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
- Client receipt times live in a small `received.json` beside it, because the device
  has no RTC: its timestamps are uptime milliseconds and are meaningless across
  reboots. **Decoration, never truth** — losing it costs the list its timestamps (a
  rep with no time shows `—`) and nothing else, and entries for reps the log does
  not hold are dropped when it is loaded.
