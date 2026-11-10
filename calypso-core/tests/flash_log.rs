mod common;

use calypso_core::flashlog::{Error, FlashLog};
use common::{
    erases, mount, reads, reset_erases, reset_reads, BigFlash, MockError, MockFlash, SECTOR,
};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

#[test]
fn append_and_read_back() {
    let mut flash = MockFlash::new();
    let mut log = mount(&mut flash);
    let id = log.append(b"hello rep").unwrap();
    assert_eq!(id, 1);
    let mut buf = [0u8; 256];
    let len = log.read(id, &mut buf).unwrap().unwrap();
    assert_eq!(&buf[..len], b"hello rep");
}

#[test]
fn ids_are_monotonic() {
    let mut flash = MockFlash::new();
    let mut log = mount(&mut flash);
    let a = log.append(b"one").unwrap();
    let b = log.append(b"two").unwrap();
    assert_eq!(b, a + 1);
}

#[test]
fn unaligned_payloads_round_trip() {
    let mut flash = MockFlash::new();
    let mut log = mount(&mut flash);
    let mut buf = [0u8; 256];
    for len in [1usize, 2, 3, 5, 7, 13, 63] {
        let payload = vec![0xABu8; len];
        let id = log.append(&payload).unwrap();
        let n = log.read(id, &mut buf).unwrap().unwrap();
        assert_eq!(&buf[..n], &payload[..]);
    }
}

#[test]
fn survives_remount() {
    let mut flash = MockFlash::new();
    let (id1, id2) = {
        let mut log = mount(&mut flash);
        (
            log.append(b"persist me").unwrap(),
            log.append(b"and me").unwrap(),
        )
    };

    let mut log = FlashLog::mount(&mut flash).unwrap();
    let id3 = log.append(b"after remount").unwrap();
    assert_eq!(id3, id2 + 1);

    let mut buf = [0u8; 256];
    let n = log.read(id1, &mut buf).unwrap().unwrap();
    assert_eq!(&buf[..n], b"persist me");
    let n = log.read(id3, &mut buf).unwrap().unwrap();
    assert_eq!(&buf[..n], b"after remount");
}

#[test]
fn wrap_drops_oldest_sectors() {
    let mut flash = MockFlash::new();
    let mut log = mount(&mut flash);
    let payload = [0x42u8; 100];
    let mut ids = Vec::new();
    for _ in 0..30 {
        ids.push(log.append(&payload).unwrap());
    }
    let mut live = Vec::new();
    let mut buf = [0u8; 256];
    log.for_each(&mut buf, |id, _| live.push(id)).unwrap();
    assert!(
        live.len() < ids.len(),
        "expected some records to be dropped"
    );
    assert_eq!(*live.last().unwrap(), *ids.last().unwrap());
    let first_live = live[0];
    assert!(live.iter().copied().eq(first_live..=*ids.last().unwrap()));
    let n = log.read(*live.last().unwrap(), &mut buf).unwrap().unwrap();
    assert_eq!(&buf[..n], &payload[..]);
}

#[test]
fn corrupt_record_is_skipped_on_remount() {
    let mut flash = MockFlash::new();
    {
        let mut log = mount(&mut flash);
        log.append(b"good").unwrap();
        log.append(b"good2").unwrap();
    }

    // Record 2's payload: 8-byte sector header + 16 bytes of record 1 + 8-byte header.
    flash.inner.data[32] ^= 0xFF;

    let mut log = FlashLog::mount(&mut flash).unwrap();
    let mut buf = [0u8; 256];
    let n = log.read(1, &mut buf).unwrap().unwrap();
    assert_eq!(&buf[..n], b"good");
    assert!(log.read(2, &mut buf).unwrap().is_none());
    let id = log.append(b"new").unwrap();
    let n = log.read(id, &mut buf).unwrap().unwrap();
    assert_eq!(&buf[..n], b"new");
}

/// A reserve is what keeps an erase out of an append, and this is that property stated as the flash sees it.
///
/// The ring is eight sectors and a hundred-byte record is two to a sector, so forty records leave sixteen live across all eight — the ring has wrapped and every sector the ring can drop next holds records.
#[test]
fn the_ring_advances_into_a_prepared_sector_without_erasing() {
    let mut flash = MockFlash::new();
    let mut log = mount(&mut flash);
    let payload = [0x5Au8; 100];
    for _ in 0..40 {
        log.append(&payload).unwrap();
    }
    // Nothing is erased ahead, so the next advance would pay for one.
    assert_eq!(log.prepared_ahead(1).unwrap(), 0);

    assert!(log.prepare_one(7).unwrap());
    assert_eq!(log.prepared_ahead(1).unwrap(), 1);

    reset_erases();
    log.append(&payload).unwrap();
    assert_eq!(erases(), 0, "the advance into a prepared sector erased");

    // The prepared sector was the oldest one, and its two records are what the reserve spent.
    let mut buf = [0u8; 128];
    assert!(log.read(25, &mut buf).unwrap().is_none());
    assert!(log.read(26, &mut buf).unwrap().is_none());
    assert_eq!(log.read(27, &mut buf).unwrap(), Some(100));
}

/// Each prepare has to land on the next sector rather than on the same one again, or a reserve stays one deep however many ticks it is given.
#[test]
fn each_prepare_deepens_the_reserve_by_one() {
    let mut flash = MockFlash::new();
    let mut log = mount(&mut flash);
    let payload = [0x66u8; 100];
    for _ in 0..40 {
        log.append(&payload).unwrap();
    }
    for depth in 1..=3 {
        assert!(log.prepare_one(3).unwrap());
        assert_eq!(log.prepared_ahead(3).unwrap(), depth);
    }
    assert!(
        !log.prepare_one(3).unwrap(),
        "a reserve that is deep enough is left alone"
    );
}

#[test]
fn a_reserve_survives_a_remount() {
    let mut flash = MockFlash::new();
    let payload = [0x33u8; 100];
    {
        let mut log = mount(&mut flash);
        for _ in 0..40 {
            log.append(&payload).unwrap();
        }
        log.prepare_one(7).unwrap();
    }

    let mut log = FlashLog::mount(&mut flash).unwrap();
    assert_eq!(log.prepared_ahead(2).unwrap(), 1);
    reset_erases();
    log.append(&payload).unwrap();
    assert_eq!(erases(), 0, "the remount forgot the reserve");
}

/// A flash that has never been written is erased, so the first pass through the ring costs nothing: fourteen records are seven sectors of an eight-sector ring, and it has not been round yet.
#[test]
fn a_ring_that_has_never_wrapped_erases_nothing() {
    let mut flash = MockFlash::new();
    let mut log = mount(&mut flash);
    reset_erases();
    let payload = [0x77u8; 100];
    for _ in 0..14 {
        log.append(&payload).unwrap();
    }
    assert_eq!(erases(), 0);
}

/// The check the reserve rests on is a check of the bytes, not of the header that claims them: a write that failed between the two leaves a sector whose header reads erased and whose records are still there, and an append must not write over them.
#[test]
fn a_sector_that_only_looks_erased_is_still_erased() {
    let mut flash = MockFlash::new();
    let payload = [0x11u8; 100];
    {
        let mut log = mount(&mut flash);
        for _ in 0..40 {
            log.append(&payload).unwrap();
        }
    }
    let prepared = 4 * SECTOR;
    flash.inner.data[prepared..prepared + 8].fill(0xFF);

    let mut log = FlashLog::mount(&mut flash).unwrap();
    assert_eq!(log.prepared_ahead(1).unwrap(), 1, "the header erases too");
    reset_erases();
    log.append(&payload).unwrap();
    assert_eq!(
        erases(),
        1,
        "a sector with records still in it was written over"
    );
}

/// A record is found, not walked to.
///
/// Measured in bytes the flash hands out, because that is what the cost is: a walk reads every payload twice (once for the CRC, once to deliver it), and on the device a walk of a full log is seconds the sampler spends stopped The comparison, not the constant, is what this test is about.
#[test]
fn finding_a_record_reads_a_fraction_of_the_log() {
    let mut flash = BigFlash::new();
    let payload = [0x5Au8; 180];
    let mut ids = Vec::new();
    {
        let mut log = mount(&mut flash);
        for _ in 0..150 {
            ids.push(log.append(&payload).unwrap());
        }
    }

    let mut log = FlashLog::mount(&mut flash).unwrap();
    let mut buf = [0u8; 256];

    reset_reads();
    let mut walked = 0u32;
    log.for_each(&mut buf, |_, _| walked += 1).unwrap();
    let walk = reads();
    assert_eq!(walked, 150);

    reset_reads();
    let n = log.read(ids[147], &mut buf).unwrap().unwrap();
    let point = reads();
    assert_eq!(&buf[..n], &payload[..]);
    assert!(
        point * 4 < walk,
        "{point} bytes read to find one record, {walk} to walk them all"
    );
}

#[test]
fn a_walk_from_an_id_crosses_a_sector_boundary() {
    let mut flash = BigFlash::new();
    let mut ids = Vec::new();
    {
        let mut log = mount(&mut flash);
        for i in 0..50u8 {
            ids.push(log.append(&[i; 180]).unwrap());
        }
    }

    let mut log = FlashLog::mount(&mut flash).unwrap();
    let mut buf = [0u8; 256];
    let mut seen = Vec::new();
    let delivered = log
        .for_each_from(ids[18], 10, &mut buf, |id, payload| {
            let i = ids.iter().position(|other| *other == id).unwrap();
            assert_eq!(payload, &[i as u8; 180], "record {i} came back as another");
            seen.push(id);
            true
        })
        .unwrap();
    assert_eq!(delivered, 10);
    assert_eq!(seen, ids[18..28].to_vec());
}

#[test]
fn a_walk_from_a_dropped_id_starts_at_the_oldest_record() {
    let payload = [0x42u8; 100];
    let mut flash = MockFlash::new();
    let mut log = mount(&mut flash);
    for _ in 0..30 {
        log.append(&payload).unwrap();
    }

    let mut buf = [0u8; 128];
    let mut live = Vec::new();
    log.for_each(&mut buf, |id, _| live.push(id)).unwrap();
    let oldest = live[0];

    // Id 1 is older than the ring: the walk still starts somewhere real rather than reading nothing.
    let mut seen = Vec::new();
    log.for_each_from(1, 3, &mut buf, |id, _| {
        seen.push(id);
        true
    })
    .unwrap();
    assert_eq!(seen, vec![oldest, oldest + 1, oldest + 2]);
    assert!(log.read(1, &mut buf).unwrap().is_none());
}

#[test]
fn too_large_payload_rejected() {
    let mut flash = MockFlash::new();
    let mut log = mount(&mut flash);
    let big = vec![0u8; SECTOR];
    assert!(matches!(log.append(&big), Err(Error::TooLarge)));
}

#[test]
fn region_too_small_rejected() {
    struct Tiny(MockFlash);
    impl embedded_storage::nor_flash::ErrorType for Tiny {
        type Error = MockError;
    }
    impl ReadNorFlash for Tiny {
        const READ_SIZE: usize = 1;
        fn capacity(&self) -> usize {
            self.0.capacity()
        }
        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            self.0.read(offset, bytes)
        }
    }
    impl NorFlash for Tiny {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = SECTOR;
        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            self.0.write(offset, bytes)
        }
        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            self.0.erase(from, to)
        }
    }
    let mut tiny = Tiny(MockFlash::new());
    tiny.0.inner.data.truncate(SECTOR);
    assert!(matches!(
        FlashLog::mount(&mut tiny),
        Err(Error::RegionTooSmall)
    ));
}
