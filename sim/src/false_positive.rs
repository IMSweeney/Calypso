//! False-positive map.
//!
//! The inverse of the detectability map: sweep the motion that is *not* a rep and report where the detector starts storing it. A cell is `#` if the motion produced one stored rep (`rom >= MIN_ROM_M`), `X` if it produced more than one — an oscillation that re-opens a span at every turnaround — and `.` if it produced none. The output is the boundary of how exaggerated a non-rep has to be before the device records it as one, and how badly once it does.
//!
//! Two classes, both already generators in [`crate::motion`]:
//!
//!   `Push`   — the unrack, a re-rack, a bump, a partial drive: one stroke, no return.
//!   `Wobble` — a hand on the rack: oscillation with no net travel.
//!
//! The calibration transient (a roll) and the unrack/re-rack assembled *around* a rep both belong to phase two, so they are not here.
//!
//! The knobs are swept with the same [`Sweep`] the detectability map uses, so the two maps are read at the same resolution. The ranges are not the classes': this map's job is the motion that is
//! *not* a rep, and it has to reach past the point where one would be stored, so travel runs from a nudge to three times `MIN_ROM_M` and speed from a slow drag to faster than any pull.
//!
//! The slices are defined here, once, so [`crate::stats`] counts the same cells this draws.

use crate::map::{draw, Knob, Slice, Sweep};
use crate::motion::{Motion, Push};

/// The bar's excursion: below `MIN_ROM_M`, then well past it.
const TRAVEL: Sweep = Sweep::new(0.02, 0.30, 21);
/// How fast the non-rep moves: a slow drag to faster than a rep is ever driven.
const SPEED: Sweep = Sweep::new(0.30, 2.50, 21);
/// How fast a wobble oscillates: a hand on the rack, to a rattle.
const HZ: Sweep = Sweep::new(0.5, 5.0, 21);

/// A push needs rest at the top long enough for the filter to call the bar at rest, so the hold is fixed comfortably above it; the wobble is fixed at a few seconds of shaking.
const HOLD_MS: u32 = 500;
const WOBBLE_MS: u32 = 3000;

/// A one-way push swept over travel and speed.
pub fn push_slice() -> Slice {
    Slice::scan(
        Knob::new("travel", "m", TRAVEL),
        Knob::new("speed", "m/s", SPEED),
        &|t, s| {
            Motion::Push(Push {
                travel: t,
                speed: s,
                hold_ms: HOLD_MS,
            })
        },
    )
}

/// An oscillation with no net travel, swept over travel and frequency.
pub fn wobble_slice() -> Slice {
    Slice::scan(
        Knob::new("travel", "m", TRAVEL),
        Knob::new("hz", "Hz", HZ),
        &|t, h| Motion::Wobble {
            travel: t,
            hz: h,
            ms: WOBBLE_MS,
        },
    )
}

/// Draw both slices: `#` where the detector stored the non-rep once, `X` where it split it, and `.` where it rejected it.
pub fn render() {
    println!("false-positive map: how exaggerated must a non-rep be before it is stored?");

    let legend = "'#' = one stored rep, 'X' = more than one, '.' = rejected";

    println!("\n### Push (unrack / re-rack / bump / partial drive), hold {HOLD_MS} ms ###");
    draw(&push_slice(), ['.', '#', 'X'], legend);

    println!("\n### Wobble (hand on rack), {WOBBLE_MS} ms ###");
    draw(&wobble_slice(), ['.', '#', 'X'], legend);
}
