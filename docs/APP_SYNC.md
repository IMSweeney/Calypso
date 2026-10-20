# App sync

How the app gets reps from the device: the BLE interface, the sync
protocol, and the archive the client keeps.

The client is the **archive of record**. The device's log is a ring that erases
its oldest sectors, so the app keeps everything it has ever synced; the device is a buffer between sessions.

## The device's BLE interface

Service `0000ca20-b0cd-11ec-871f-d45ddf138840` — advertised, so a scan can filter on it:

| UUID | Direction | Carries |
|---|---|---|
| `0000ca21-…` | notify | everything the device says: a `SyncMessage`, whatever its body |
| `0000ca22-…` | write (+ write-without-response) | everything the app asks for: a `SyncRequest` |

**One characteristic each way, and that is the design.** The body of the message
says what it is — a `summary`, a `trace` chunk, a `calibration` status — so nothing
here distinguishes a response from a push, and a new kind of message is a new arm of
the oneof rather than a new characteristic, a new UUID and a new subscription. The
notify side is sized for the largest message (a trace chunk); the requests side is
sized for a `TraceRequest` and its envelope.

They were three characteristics once: summaries, trace chunks and a bare
`TraceRequest` each had their own. Collapsing them removed the asymmetry between the
two directions, the two subscriptions the app had to make, and a CCCD-slot count
that could be got wrong in silence (`docs/GOTCHAS.md`).

GAP is the other service: `0x2A00` (the device name) and `0x2A01` (the
appearance), both **read only**. The name is a compile-time constant in the
firmware — the app could rename the device once, which meant a writable
characteristic and a name blob in NVS, and it does not any more.

Calibration rides the messages characteristic as `SyncMessage.calibration` rather
than having a characteristic of its own — see [`CALIBRATION.md`](CALIBRATION.md).

The advertisement carries the flags, the **service UUID** and the local name (30
of the 31 available bytes for `Calypso`).

## Transports

Everything except the radio sits behind one interface, so the whole data path
runs in `flutter test` and in the Simulator (which has no CoreBluetooth at all):

```dart
abstract class RepTransport {
  /// Summaries on the device, oldest first.
  Future<List<RepSummary>> summaries();

  /// Follow the device as new reps arrive.
  Stream<RepSummary> watchSummaries();

  /// One rep's trace, reassembled from chunks.
  Future<List<TracePoint>> trace(String repUuid);
}
```

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
- **Incremental, on the device's side.** The device keeps a cursor
  (`SummaryCursor`) of how far its subscriber has been brought up to date, so
  re-subscribing sends only what is new, and it advances the cursor only on a
  successful notify — where "successful" is checked properly, since
  `Characteristic::notify` reports success whether or not anybody is subscribed
  (see [`GOTCHAS.md`](GOTCHAS.md)). An app that is not subscribed means the next tick
  retries from the same position.
- **Wrap-safe.** Old reps disappear from the device's ring but not from the
  client's archive. The cursor is clamped to the log's actual count, so a device
  that wrapped re-syncs from the oldest id it still has.
- **Idempotent, at the log.** Both paths ask the archive what it holds and skip
  that, so a rep summarised twice — around a reconnect, say — is written once. The
  writes are chained, so the two paths cannot interleave on the same rep either.
- A dropped connection costs nothing already archived: the stream goes quiet and the
  next connect catches up.

Both paths end in the same place, and the list is built from that place: the UI
holds no rep state of its own.

## Trace sync

The app writes a `TraceRequest { rep_uuid, all_chunks, chunk_index }` and the
device streams `TraceChunk` notifications in response.

- **Re-chunked on the way out.** The log stores 48 points per record (sized to fit
  a flash sector); BLE re-chunks to `sync::BLE_CHUNK_SAMPLES` = 3 points per notification, chosen for Apple's ~185-byte MTU. The client does not care how many points arrive per chunk.
- **End of stream.** Every streamed chunk carries `total_chunks`, the number of
  chunks in that stream, so the client knows which is the last. It cannot be derived from the summary: the log's `chunk_count` counts 48-point records, while the stream is 3-point chunks. Stored records leave the field 0.
- **Reassembly is tolerant.** `TraceAssembler` keys chunks by index, so
  out-of-order arrival works, a repeated index replaces the earlier one, and chunks for another rep are ignored. `missingIndices()` reports gaps so a single chunk can be re-requested.

## The archive

`HistoryStore` is the client's copy of everything synced, and **the only source of
truth for what the app shows**:

- Records live in `history.pblog` — the same `[u32 LE length][RepRecord]` framing
  as fixtures and captures, so history, a fixture and a dumped log are one format ([`FIXTURES.md`](FIXTURES.md)).
- **What exists is read out of the log**: `contains(uuid)`, the list, its order and
  each rep's trace all come from the records. Nothing else can make a rep exist, and
  nothing else can stop one existing — which is also what makes re-syncing a no-op
  rather than a second copy of a rep.
- Client receipt times live in a small `received.json` beside it, because the
  device has no RTC: its timestamps are uptime milliseconds and are meaningless across reboots.
  **Decoration, never truth** — losing it costs the list its timestamps (a rep with
  no time shows `—`) and nothing else, and entries for reps the log does not hold
  are dropped when it is loaded.
