//! What time a stored rep gets, and when the device refuses to date one at all.

use calypso_core::clock::{self, Boots};
use calypso_core::pb::msg;

/// The boot running now, and an earlier one: the ids the log gave their first records.
const OUR_FROM: u32 = 400;
const EARLIER: u32 = 200;

/// What the client said, and the epoch that implies for a device 1000 s into its boot.
const NOW_S: u32 = 1_760_000_000;
const EPOCH_S: u32 = 1_759_999_000;

/// A clock record as every boot writes one at mount: no epoch, which is what stops the boot before it applying.
fn mount() -> msg::BootClock {
    msg::BootClock { epoch_s: 0 }
}

/// A clock record as the client's write leaves one.
fn set(epoch_s: u32) -> msg::BootClock {
    msg::BootClock { epoch_s }
}

#[test]
fn the_epoch_is_the_clients_now_minus_the_uptime() {
    assert_eq!(clock::epoch_from(NOW_S, 1_000_000), Some(EPOCH_S));
    assert_eq!(
        clock::epoch_from(NOW_S, 500),
        Some(NOW_S),
        "sub-second uptime is not a second of the device's life"
    );
}

#[test]
fn a_time_before_this_project_existed_is_refused() {
    assert_eq!(clock::epoch_from(clock::EARLIEST_S - 1, 0), None);
    assert_eq!(clock::epoch_from(0, 0), None);
    assert_eq!(
        clock::epoch_from(clock::EARLIEST_S, 0),
        Some(clock::EARLIEST_S)
    );
}

#[test]
fn a_rep_is_dated_by_its_boots_epoch_plus_its_own_uptime() {
    let boot = clock::Boot { epoch_s: EPOCH_S };
    assert_eq!(boot.rep_time_ms(9_500), Some(1_759_999_009_500));
    assert_eq!(
        clock::Boot { epoch_s: 0 }.rep_time_ms(9_500),
        None,
        "a boot with no clock dates nothing"
    );
}

#[test]
fn a_clock_record_dates_the_records_after_it_and_no_others() {
    let mut boots = Boots::new(Some(OUR_FROM), 0);
    boots.on_clock(&mount());
    assert_eq!(
        boots.boot_of(EARLIER).rep_time_ms(5_000),
        None,
        "the reps stored before the client wrote anything are not dated by it"
    );

    boots.on_clock(&set(EPOCH_S));
    assert_eq!(
        boots.boot_of(EARLIER + 1).rep_time_ms(5_000),
        Some(1_759_999_005_000)
    );
}

#[test]
fn a_boot_with_no_clock_does_not_borrow_the_one_before_it() {
    let mut boots = Boots::new(Some(OUR_FROM), 0);
    boots.on_clock(&set(EPOCH_S));
    assert!(boots.boot_of(EARLIER).rep_time_ms(1_000).is_some());

    boots.on_clock(&mount());
    assert_eq!(
        boots.boot_of(EARLIER + 1).rep_time_ms(1_000),
        None,
        "the device restarted: this boot has no clock until the client sets one"
    );
}

#[test]
fn a_log_that_cannot_say_where_this_boot_started_dates_nothing() {
    // What a log that could not be written to at mount looks like. Every boot records where it began, and without that record its reps cannot be told from the boot before it — so none of them is dated, rather than the previous boot's epoch being borrowed for them.
    let mut boots = Boots::new(None, EPOCH_S);
    boots.on_clock(&set(EPOCH_S - 1_000));
    assert_eq!(boots.boot_of(EARLIER).rep_time_ms(1_000), None);
    assert_eq!(
        boots.boot_of(OUR_FROM).rep_time_ms(1_000),
        None,
        "not even with a clock set in RAM"
    );
}

#[test]
fn the_boot_running_now_is_dated_from_ram() {
    let mut boots = Boots::new(Some(OUR_FROM), 0);
    boots.on_clock(&mount());
    assert_eq!(
        boots.boot_of(OUR_FROM).rep_time_ms(9_000),
        None,
        "nothing has set this boot's clock yet"
    );

    // The client connects and writes it. The reps stored earlier in this boot are the point of that write, and they are dated without the log being read again.
    let boots = Boots::new(Some(OUR_FROM), EPOCH_S);
    assert_eq!(
        boots.boot_of(OUR_FROM).rep_time_ms(9_000),
        Some(1_759_999_009_000)
    );
    assert_eq!(
        boots.boot_of(EARLIER).rep_time_ms(9_000),
        None,
        "and it says nothing about an earlier boot's reps"
    );
}
