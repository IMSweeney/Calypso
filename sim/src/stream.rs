//! The samples a recording holds, on the device's own clock.
//!
//! A phone's recording holds samples and nothing else: the device was streaming for the whole session and stored no window, no summary and no state. So the samples go back through [`crate::replay`] from the state a device that has just powered on starts in, and what comes out is what the device would have stored had it been detecting.
//!
//! One thing a replay cannot recover is the stream's own beginning. A recording starts wherever the record button was pressed, so the filter spends its first `READY_S` of quiet earning a gate the device had already earned before the recording began — a rep inside that stretch is unmeasurable here and was measurable on the device.

use calypso_core::pb::{decode_record, raw_point_from_message, Record};
use calypso_core::sampler::Sample;

use crate::fixtures::read_dump;

/// The samples one recording holds, on the device's own clock.
///
/// A point's time is an offset from its chunk's `first_t_ms`, which is the uptime of the chunk's first sample: adding it back is what puts the whole session on the clock the detector steps in.
pub fn samples(pblog: &[u8]) -> Vec<Sample> {
    let (records, truncated) = read_dump(pblog);
    assert!(!truncated, "the recording is torn at the end");
    let mut samples: Vec<Sample> = Vec::new();
    for record in &records {
        match decode_record(record).expect("a record in the recording decodes") {
            Record::Imu(c) => {
                for p in c.samples.iter() {
                    let mut s = raw_point_from_message(p);
                    s.t_ms = c.first_t_ms.saturating_add(s.t_ms);
                    samples.push(s);
                }
            }
            // The boot's epoch, for dating the stream on a wall clock; `epoch_s` is what reads it.
            Record::Clock(_) => {}
            other => panic!("a live stream recording cannot hold a {other:?}"),
        }
    }
    samples
}

/// The wall clock the recording's uptime counts from, in unix seconds.
///
/// The app writes one `BootClock` when it opens a file and writes it again if the phone's clock is re-set, and the device writes 0 at mount for a boot no client has dated yet; so the newest non-zero one is the epoch a recording is dated by.
/// `None` is a recording whose boot never had a clock, which is undated rather than dated at 1970.
pub fn epoch_s(pblog: &[u8]) -> Option<u32> {
    let (records, truncated) = read_dump(pblog);
    assert!(!truncated, "the recording is torn at the end");
    let mut epoch_s = None;
    for record in &records {
        if let Record::Clock(clock) =
            decode_record(record).expect("a record in the recording decodes")
        {
            if clock.epoch_s != 0 {
                epoch_s = Some(clock.epoch_s);
            }
        }
    }
    epoch_s
}
