# App sync

How the app gets reps from the device: the BLE interface, the sync
protocol, and the archive the client keeps.

The client is the **archive of record**. The device's log is a ring that erases
its oldest sectors, so the app keeps everything it has ever synced; the device is a buffer between sessions.

## The device's BLE interface

Service `0000ca20-b0cd-11ec-871f-d45ddf138840` — advertised, so a scan can filter on it:

| UUID | Direction | Carries |
|---|---|---|
| `0000ca21-…` | notify + read | `SyncMessage` with a `summary` body, auto-streamed to a subscriber |
| `0000ca22-…` | write (+ write-without-response) | a **bare** `TraceRequest` |
| `0000ca23-…` | notify | `SyncMessage` with a `trace` body, on demand |

Alongside it:

| UUID | Direction | Carries |
|---|---|---|
| `0000ca10-…` / `0000ca11-…` | read + notify | the IMU's gravity vector, X axis only (m/s²) |
| GAP `0x2A00` | read + write | the device name: rename from the app, persisted in NVS |

Calibration rides the summary characteristic as `SyncMessage.calibration` rather
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

```
idle ──connect──► syncing ──caught up──► live
                     │                    │
                     └── trace request ───┴──► streamingTrace ──► live
```

- **Incremental.** The device keeps a cursor (`SummaryCursor`) of how far its
  subscriber has been brought up to date, so re-subscribing sends only what is new. It advances the cursor only on a successful notify, so "is the app subscribed?" needs no CCCD bookkeeping — an unsubscribed app simply means the
  next tick retries from the same position.
- **Idempotent on the client.** Summaries are merged by `rep_uuid`; duplicates are
  ignored, which makes re-syncing cheap and a ring wrap on the device harmless.
- **Wrap-safe.** Old reps disappear from the device's ring but not from the
  client's archive. The cursor is clamped to the log's actual count, so a device that wrapped re-syncs from the oldest id it still has.
- A dropped connection must not lose the archive: the subscription is dropped,
  the client stops, and a later `syncSummaries()` picks up where it left off.

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

`HistoryStore` is the client's copy of everything synced:

- Records live in `history.pblog` — the same `[u32 LE length][RepRecord]` framing
  as fixtures and captures, so history, a fixture and a dumped log are one format ([`FIXTURES.md`](FIXTURES.md)).
- Client receipt times live in a small `received.json` beside it, because the
  device has no RTC: its timestamps are uptime milliseconds and are meaningless across reboots.
