# calypso-app

The client side: syncs reps from the device over BLE, lists rep summaries, and
plots a single rep in 3D.

**Status: planning.** No code yet.

- [Architecture](../docs/app/README.md) — stack and why, layers, project layout,
  protocol client, transports, plot model, fixtures, testing, dev loop.
- [Plan](../docs/app/plan.md) — goal, decisions (including what was rejected),
  phases, TODO checklist, open questions.

Decided: **Flutter**, **`flutter_reactive_ble`**, plot drawn by hand with
`vector_math` + `CustomPainter`, Dart messages generated from
`proto/calypso.proto`, and the client as the archive of record.

Planned layout:

- `lib/protocol/` — generated message classes + record/chunk decoding
- `lib/sync/` — summary cursor, trace reassembly, session grouping
- `lib/transport/` — `RepTransport` implementations: fixture, BLE, recording
- `lib/plot/` — plot model (pure Dart) + `CustomPainter` renderer
- `lib/state/`, `lib/ui/` — state controllers and screens
- `test/` — unit, widget and golden tests
- `assets/fixtures/` — checked-in `.pblog` logs from the firmware's encoders
