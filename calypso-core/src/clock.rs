//! The device's wall clock: what the client sets, and the time a stored rep gets.
//!
//! The device has no RTC, so the only time it knows is the one the client writes (`Time` on the sync service, unix seconds).
//! It is kept **per boot**: an epoch is what that boot's uptime zero was worth on the wall clock, which is what turns a stored rep's uptime into a time.
//! It is forgotten at the next boot, and the client sets it again on every connection, so the forgetting costs nothing.
//!
//! Where the epoch is kept is the schema's business (`BootClock` in `proto/calypso.proto`): the log holds it, so a rep keeps its time across a restart, and the RAM copy is the boot running now — the one boot whose clock is often not in the log yet, because nothing has set it since the device came up.
//!
//! **The clock arrives after the reps it applies to**, which is the whole shape of the rule here: a clock record dates the records after it, and a boot with none stays undated rather than borrowing the boot before it.

use crate::pb::msg;

/// The earliest time the device will believe.
/// Nothing this project could have recorded exists before it, so a value below it is a client whose clock is wrong rather than a time, and storing it would put a plausible lie in the archive.
/// A number rather than a date because the device has no calendar.
pub const EARLIEST_S: u32 = 1_700_000_000;

/// A boot's clock: what its uptime zero was worth on the wall clock, or 0 when no client has set it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Boot {
    pub epoch_s: u32,
}

impl Boot {
    /// The rep's wall-clock time: the boot's epoch plus the rep's own uptime.
    /// `None` when this boot never had a clock, which is what makes such a rep undateable rather than wrongly dated.
    ///
    /// Milliseconds, while the epoch is to the second: every rep of one boot carries the same rounding, so two reps in the same second still order exactly.
    pub fn rep_time_ms(&self, t_start_ms: u32) -> Option<u64> {
        if self.epoch_s == 0 {
            return None;
        }
        Some(self.epoch_s as u64 * 1000 + t_start_ms as u64)
    }
}

/// The epoch a client's `now` implies for a device that has been up `uptime_ms`, or `None` for a time the device will not believe (see [`EARLIEST_S`]).
pub fn epoch_from(now_s: u32, uptime_ms: u32) -> Option<u32> {
    if now_s < EARLIEST_S {
        return None;
    }
    now_s.checked_sub(uptime_ms / 1000)
}

/// The clock each record of a log belongs to, following it in order.
///
/// Feed it every record as you meet it: [`Boots::on_clock`] for a clock record, [`Boots::boot_of`] for a rep summary.
/// A clock record dates the records after it, so the answer only changes where one appears — which is what makes a boot with no clock of its own undated rather than dated by the boot before it.
///
/// The boot this device is running is the exception, and is passed in: its reps may already be in the log *before* the client set its clock (which is what setting it later is for), and only the device knows where its own boot starts, because a dump of the log does not carry record ids.
pub struct Boots {
    /// The first record id of the boot this device is running, and that boot's epoch in RAM.
    /// `None` when its log has no record of where the boot started — every boot writes one at mount, so this is a log that could not be written to — and then nothing is dated at all: without a boundary there is no telling this boot's reps from the boot before it, and a guess would be a plausible wrong time.
    our_from_record_id: Option<u32>,
    our_epoch_s: u32,
    /// The epoch of the boot the walk is in, as the log has it.
    epoch_s: u32,
}

impl Boots {
    /// `our_epoch_s` is read once, at the start of a walk: the client can set the clock between two walks, never during one.
    pub fn new(our_boot_from_record_id: Option<u32>, our_epoch_s: u32) -> Self {
        Boots {
            our_from_record_id: our_boot_from_record_id,
            our_epoch_s,
            epoch_s: 0,
        }
    }

    /// A clock record, in log order. Every boot writes one at mount with `epoch_s` 0 — which is how the boot before it stops applying — and the client's write replaces it.
    pub fn on_clock(&mut self, clock: &msg::BootClock) {
        self.epoch_s = clock.epoch_s;
    }

    /// Whether a record is one this boot wrote: the boot whose clock is in RAM, because the client set it now.
    pub fn is_ours(&self, id: u32) -> bool {
        matches!(self.our_from_record_id, Some(from) if id >= from)
    }

    /// A rep summary, in log order: the clock of the boot that recorded it.
    pub fn boot_of(&self, id: u32) -> Boot {
        let Some(from) = self.our_from_record_id else {
            // The log cannot say which boot a record came from, so it dates none of them.
            return Boot { epoch_s: 0 };
        };
        if id >= from {
            return Boot {
                epoch_s: self.our_epoch_s,
            };
        }
        Boot {
            epoch_s: self.epoch_s,
        }
    }
}
