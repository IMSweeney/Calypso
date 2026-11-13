# Fixtures

The app is developed and tested against checked-in device data, so no work on the
client needs hardware.

## What a `.pblog` is

A `.pblog` is a **log dump**: the `RepRecord` messages the device stores, without the
flash sector framing, written as a sequence of

```
[u32 little-endian length][record bytes]
```

**a history file, a fixture and a capture are one format**: a capture is just a copy of somebody's archive.

## The captures

A capture is a raw recording of IMU data.
They live in an object store rather than in the repository. 
```sh
export CAPTURES_REMOTE=r2/calypso-captures
tools/captures.sh ls
tools/captures.sh push                      # every capture in `captures/`
tools/captures.sh pull working-sets-2026-11-10.pblog
```

`captures/captures.json` stores a manifest of these files.
