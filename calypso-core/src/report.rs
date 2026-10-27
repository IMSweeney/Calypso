//! The device's own faults, on their way to the phone.
//!
//! `Report` (`proto/calypso.proto`) is a kind and nothing else: the app owns the
//! sentence, the console owns the detail. What is left to decide here is when a kind is
//! worth sending, because a fault that keeps happening must not be a notification per
//! failed read, and a fault raised before the client subscribed is still owed to it.

use crate::pb::msg;

/// How long the same kind must stay quiet before it is reported again, in milliseconds.
///
/// The IMU is read at 100 Hz, so a bus that has stopped answering raises its fault a
/// hundred times a second; one line a second is the same news at a rate a person can
/// read. A *different* kind is due at once, because the app is showing whatever it was
/// last told.
pub const COALESCE_MS: u32 = 1000;

/// What has been raised and not yet sent.
///
/// One slot, not a queue: a kind names a class of failure, so a second fault of the same
/// kind before the first has gone out is the same news, and the newest kind is the one
/// worth showing. The cost is that two *different* kinds raised while nothing is
/// listening leave only the newer one — this is a nudge to the person holding the phone,
/// not a journal, and the console has both.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Latch {
    /// The newest kind raised, sent or not.
    latest: Option<msg::ReportKind>,
    /// The kind last sent and the uptime it went out at, which is what [`COALESCE_MS`]
    /// is measured from.
    last_sent: Option<(msg::ReportKind, u32)>,
}

impl Latch {
    pub const fn new() -> Self {
        Latch {
            latest: None,
            last_sent: None,
        }
    }

    /// Note a fault. Infinitely cheap: it is called from the sampling path and from the
    /// arms that are already reporting a failure, so it may not allocate, block or fail.
    pub fn raise(&mut self, kind: msg::ReportKind) {
        self.latest = Some(kind);
    }

    /// The fault owed to the app at `now_ms`, marking it sent.
    ///
    /// Due when something is latched and either it has never gone out, it is not the kind
    /// that last went out, or [`COALESCE_MS`] has passed since that one did. What goes out
    /// is **unlatched**, so a fault that happened once is one line however long the app
    /// stays connected, and a fault that is still happening is raised again by its own
    /// failure — which is the only thing that can tell the two apart.
    ///
    /// Taking and marking in one call is what keeps a caller from sending the same fault
    /// twice: a notification the link refused is not retried, because a fault that cannot
    /// be notified now is not one the next attempt says better.
    pub fn take_due(&mut self, now_ms: u32) -> Option<msg::ReportKind> {
        let kind = self.latest?;
        let due = match self.last_sent {
            None => true,
            Some((sent, at)) => sent != kind || now_ms.wrapping_sub(at) >= COALESCE_MS,
        };
        if !due {
            // Still latched: a burst that has not gone quiet gets its line when the window
            // closes, on a tick, without being dropped.
            return None;
        }
        self.last_sent = Some((kind, now_ms));
        self.latest = None;
        Some(kind)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gatt::{max_len, SyncReportsValue};
    use micropb::size::max_encoded_size;

    /// The schema's kind numbers, written out.
    ///
    /// A generated enum is an open newtype with no runtime list of its values, so this is
    /// the one place a new kind has to be added by hand for the tests to cover it.
    fn all_kinds() -> impl Iterator<Item = msg::ReportKind> {
        [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
            .into_iter()
            .map(msg::ReportKind)
    }

    #[test]
    fn nothing_raised_is_nothing_owed() {
        let mut latch = Latch::new();
        assert_eq!(latch.take_due(0), None);
    }

    #[test]
    fn a_raised_kind_is_due_once() {
        let mut latch = Latch::new();
        latch.raise(msg::ReportKind::RepNotStored);
        assert_eq!(latch.take_due(10), Some(msg::ReportKind::RepNotStored));
        assert_eq!(latch.take_due(11), None);
    }

    #[test]
    fn the_same_kind_waits_out_the_window() {
        let mut latch = Latch::new();
        latch.raise(msg::ReportKind::ImuReadFailed);
        assert!(latch.take_due(0).is_some());
        // A hundred failures inside the window are the same line.
        for now in (1..COALESCE_MS).step_by(10) {
            latch.raise(msg::ReportKind::ImuReadFailed);
            assert_eq!(latch.take_due(now), None, "at {now} ms");
        }
        latch.raise(msg::ReportKind::ImuReadFailed);
        assert_eq!(
            latch.take_due(COALESCE_MS),
            Some(msg::ReportKind::ImuReadFailed)
        );
    }

    #[test]
    fn a_one_off_fault_is_one_line() {
        let mut latch = Latch::new();
        latch.raise(msg::ReportKind::RepNotStored);
        assert_eq!(latch.take_due(0), Some(msg::ReportKind::RepNotStored));
        // Nothing raised it again, so there is nothing to send however long the app stays
        // connected: a fault that happened once is not a fault that is still happening.
        assert_eq!(latch.take_due(COALESCE_MS * 10), None);
    }

    #[test]
    fn a_different_kind_does_not_wait() {
        let mut latch = Latch::new();
        latch.raise(msg::ReportKind::ImuReadFailed);
        assert_eq!(latch.take_due(0), Some(msg::ReportKind::ImuReadFailed));
        latch.raise(msg::ReportKind::TraceUnreadable);
        assert_eq!(latch.take_due(1), Some(msg::ReportKind::TraceUnreadable));
    }

    #[test]
    fn a_kind_relatched_after_the_window_is_due_again() {
        let mut latch = Latch::new();
        latch.raise(msg::ReportKind::NotifyFailed);
        assert!(latch.take_due(0).is_some());
        // The link failed again a while later: that is news, even though it is the same kind.
        latch.raise(msg::ReportKind::NotifyFailed);
        assert_eq!(
            latch.take_due(COALESCE_MS * 3),
            Some(msg::ReportKind::NotifyFailed)
        );
    }

    /// The window is a `u32` of milliseconds, so it wraps after ~49 days of uptime. The
    /// elapsed time is the wrapping difference, which is why a fault sent just before the
    /// wrap is still quiet a moment after it — and due a window after it, rather than
    /// silenced by a subtraction that overflowed the other way.
    #[test]
    fn a_wrapped_clock_is_still_a_window() {
        let mut latch = Latch::new();
        latch.raise(msg::ReportKind::RadioFailed);
        assert!(latch.take_due(u32::MAX - 10).is_some());

        // 16 ms later, across the wrap: the same window, still open.
        latch.raise(msg::ReportKind::RadioFailed);
        assert_eq!(latch.take_due(5), None);

        // 2011 ms after it went out: due again.
        latch.raise(msg::ReportKind::RadioFailed);
        assert_eq!(latch.take_due(2000), Some(msg::ReportKind::RadioFailed));
    }

    /// The characteristic's `max_len` and the message have to stay in step: a `Report`
    /// that outgrew it could not be notified at all, which is the failure mode this
    /// whole feature exists to remove (`docs/GOTCHAS.md`, "A trace chunk can be too big
    /// to notify, silently").
    #[test]
    fn a_report_fits_the_characteristic() {
        assert!(
            max_encoded_size::<msg::Report>() <= max_len::SYNC_REPORTS,
            "a Report encodes to {} bytes, the characteristic holds {}",
            max_encoded_size::<msg::Report>(),
            max_len::SYNC_REPORTS
        );
    }

    #[test]
    fn every_kind_round_trips_through_the_value() {
        for kind in all_kinds() {
            // The wire form is one field, so the message is the kind wrapped up.
            let report = msg::Report { kind };
            let value = SyncReportsValue::encode(&report).expect("a Report always fits");
            assert_eq!(value.decode(), Some(report));
        }
    }
}
