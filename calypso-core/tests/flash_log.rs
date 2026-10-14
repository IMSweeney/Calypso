//! Host tests for the flash log ring, against a mock NOR flash that
//! reproduces real flash semantics (writes can only clear bits, erase
//! resets a sector to 0xFF).

use calypso_core::flashlog::{Error, FlashLog};
use embedded_storage::nor_flash::{NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash};

/// Small sectors so tests wrap the ring quickly.
const SECTOR: usize = 256;
const SECTORS: usize = 8;

#[derive(Debug, PartialEq, Eq)]
struct MockError;

impl NorFlashError for MockError {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

struct MockFlash {
    data: Vec<u8>,
}

impl MockFlash {
    fn new() -> Self {
        MockFlash { data: vec![0xFF; SECTOR * SECTORS] }
    }
}

impl embedded_storage::nor_flash::ErrorType for MockFlash {
    type Error = MockError;
}

impl ReadNorFlash for MockFlash {
    const READ_SIZE: usize = 1;

    fn capacity(&self) -> usize {
        self.data.len()
    }

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let start = offset as usize;
        if start + bytes.len() > self.data.len() {
            return Err(MockError);
        }
        bytes.copy_from_slice(&self.data[start..start + bytes.len()]);
        Ok(())
    }
}

impl NorFlash for MockFlash {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = SECTOR;

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        assert_eq!(offset as usize % Self::WRITE_SIZE, 0, "unaligned write offset");
        assert_eq!(bytes.len() % Self::WRITE_SIZE, 0, "unaligned write length");
        let start = offset as usize;
        if start + bytes.len() > self.data.len() {
            return Err(MockError);
        }
        for (i, &b) in bytes.iter().enumerate() {
            self.data[start + i] &= b; // NOR flash: writes can only clear bits
        }
        Ok(())
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        assert_eq!(from as usize % Self::ERASE_SIZE, 0);
        assert_eq!(to as usize % Self::ERASE_SIZE, 0);
        for b in &mut self.data[from as usize..to as usize] {
            *b = 0xFF;
        }
        Ok(())
    }
}

fn mount() -> FlashLog<MockFlash> {
    FlashLog::mount(MockFlash::new()).unwrap()
}

#[test]
fn append_and_read_back() {
    let mut log = mount();
    let id = log.append(b"hello rep").unwrap();
    assert_eq!(id, 1);
    let mut buf = [0u8; 256];
    let len = log.read(id, &mut buf).unwrap().unwrap();
    assert_eq!(&buf[..len], b"hello rep");
}

#[test]
fn ids_are_monotonic() {
    let mut log = mount();
    let a = log.append(b"one").unwrap();
    let b = log.append(b"two").unwrap();
    assert_eq!(b, a + 1);
}

#[test]
fn unaligned_payloads_round_trip() {
    let mut log = mount();
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
    let mut log = mount();
    let id1 = log.append(b"persist me").unwrap();
    let id2 = log.append(b"and me").unwrap();

    // Remount the same flash and continue where we left off.
    let flash = log.into_inner();
    let mut log = FlashLog::mount(flash).unwrap();
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
    let mut log = mount();
    // Payload sized so each sector holds ~2 records; write many to force wraps.
    let payload = [0x42u8; 100];
    let mut ids = Vec::new();
    for _ in 0..30 {
        ids.push(log.append(&payload).unwrap());
    }
    // Collect what survived.
    let mut live = Vec::new();
    let mut buf = [0u8; 256];
    log.for_each(&mut buf, |id, _| live.push(id)).unwrap();
    assert!(live.len() < ids.len(), "expected some records to be dropped");
    // Live records are the newest ones, in order.
    assert_eq!(*live.last().unwrap(), *ids.last().unwrap());
    let first_live = live[0];
    assert!(live.iter().copied().eq(first_live..=*ids.last().unwrap()));
    // Data reads back intact.
    let n = log.read(*live.last().unwrap(), &mut buf).unwrap().unwrap();
    assert_eq!(&buf[..n], &payload[..]);
}

#[test]
fn corrupt_record_is_skipped_on_remount() {
    let mut log = mount();
    log.append(b"good").unwrap();
    log.append(b"good2").unwrap();

    // Corrupt the payload region of the second record (simulate a torn write).
    let mut flash = log.into_inner();
    // Sector layout: 8-byte sector header, then record 1 ("good":
    // 8 hdr + 4 payload + 4 trailer = 16 bytes), so record 2's payload starts
    // at byte 8 + 16 + 8 = 32.
    flash.data[32] ^= 0xFF;

    let mut log = FlashLog::mount(flash).unwrap();
    // The corrupt tail can't be rewritten (bits already programmed), so mount
    // advances to a fresh sector; the good record in sector 0 survives.
    let mut buf = [0u8; 256];
    let n = log.read(1, &mut buf).unwrap().unwrap();
    assert_eq!(&buf[..n], b"good");
    assert!(log.read(2, &mut buf).unwrap().is_none());
    // New appends keep working.
    let id = log.append(b"new").unwrap();
    let n = log.read(id, &mut buf).unwrap().unwrap();
    assert_eq!(&buf[..n], b"new");
}

#[test]
fn too_large_payload_rejected() {
    let mut log = mount();
    let big = vec![0u8; SECTOR]; // way over max_payload for 256-byte sectors
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
    tiny.0.data.truncate(SECTOR); // only one sector
    assert!(matches!(FlashLog::mount(tiny), Err(Error::RegionTooSmall)));
}
