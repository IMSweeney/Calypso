mod common;

use calypso_core::flashlog::{Error, FlashLog};
use common::{mount, MockError, MockFlash, SECTOR};
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
    assert!(live.len() < ids.len(), "expected some records to be dropped");
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
    assert!(matches!(FlashLog::mount(&mut tiny), Err(Error::RegionTooSmall)));
}
