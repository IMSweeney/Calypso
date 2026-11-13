# Roadmap

Where Calypso is and what is left. Feature documentation lives in
[`docs/`](docs/); this file is only scope and status.

## Detector

- [ ] **Monte Carlo sim** - Generate syntetic reps/sets using a physical model and a noise model and measure the detector performance against these reps to find edge cases in the detector.

## Firmware

- [ ] **OTA code updates** — the app partition is 1.5 MB with headroom; dual slots would mean shrinking the log partition.
- [ ] On-device tests with [`embedded-test`](https://crates.io/crates/embedded-test): the flash partition layer first, then the radio path.
- [ ] **Read the cell over the STEMMA bus, not off the charger's divider.** The chosen part is a MAX17048, in-line between the cell and the BFF's battery JST, which yields a percentage.

## Client

- [ ] **Plot performance** - camera panning is super laggy in the rep plot view 
- [ ] **Session grouping** — Grouping reps into sets and sets into sessions
- [ ] Fault detection / coaching hints on the traces (bar dip, forward lean).
- [ ] Real-device polish: reconnect UX, sync progress, permission prompts.
- [ ] Install story: a free Apple ID gives 7-day builds; TestFlight and the App
      Store need a paid account.
