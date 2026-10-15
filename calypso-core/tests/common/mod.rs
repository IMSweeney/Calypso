//! Shared test support: mock NOR flashes that reproduce real flash semantics
//! (writes can only clear bits, erase resets a sector to 0xFF, writes must be
//! 4-byte aligned). Two sizes are used: small sectors wrap the ring quickly,
//! and 4 KB sectors match the real ESP32 flash so record sizes are realistic.
#![allow(dead_code)] // not every test target uses every mock


use calypso_core::flashlog::FlashLog;
use embedded_storage::nor_flash::{NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash};

/// Small sectors so tests wrap the ring quickly.
pub const SECTOR: usize = 256;
pub const SECTORS: usize = 8;
/// Real ESP32 sector size, for tests that care about record capacity.
pub const BIG_SECTOR: usize = 4096;
pub const BIG_SECTORS: usize = 8;

#[derive(Debug, PartialEq, Eq)]
pub struct MockError;

impl NorFlashError for MockError {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

pub struct MockData {
    pub data: Vec<u8>,
    sector: usize,
}

impl MockData {
    pub fn new(sector: usize, sectors: usize) -> Self {
        MockData {
            data: vec![0xFF; sector * sectors],
            sector,
        }
    }

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), MockError> {
        let start = offset as usize;
        if start + bytes.len() > self.data.len() {
            return Err(MockError);
        }
        bytes.copy_from_slice(&self.data[start..start + bytes.len()]);
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), MockError> {
        let start = offset as usize;
        if start + bytes.len() > self.data.len() {
            return Err(MockError);
        }
        for (i, &b) in bytes.iter().enumerate() {
            self.data[start + i] &= b; // NOR flash: writes can only clear bits
        }
        Ok(())
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), MockError> {
        assert_eq!(from as usize % self.sector, 0, "unaligned erase");
        assert_eq!(to as usize % self.sector, 0, "unaligned erase");
        for b in &mut self.data[from as usize..to as usize] {
            *b = 0xFF;
        }
        Ok(())
    }
}

macro_rules! define_mock_flash {
    ($name:ident, $sector:expr, $sectors:expr) => {
        pub struct $name {
            pub inner: MockData,
        }

        impl $name {
            pub fn new() -> Self {
                $name {
                    inner: MockData::new($sector, $sectors),
                }
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self::new()
            }
        }

        impl embedded_storage::nor_flash::ErrorType for $name {
            type Error = MockError;
        }

        impl ReadNorFlash for $name {
            const READ_SIZE: usize = 1;

            fn capacity(&self) -> usize {
                self.inner.data.len()
            }

            fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
                self.inner.read(offset, bytes)
            }
        }

        impl NorFlash for $name {
            const WRITE_SIZE: usize = 4;
            const ERASE_SIZE: usize = $sector;

            fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
                assert_eq!(
                    offset as usize % Self::WRITE_SIZE,
                    0,
                    "unaligned write offset"
                );
                assert_eq!(bytes.len() % Self::WRITE_SIZE, 0, "unaligned write length");
                self.inner.write(offset, bytes)
            }

            fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
                self.inner.erase(from, to)
            }
        }
    };
}

define_mock_flash!(MockFlash, SECTOR, SECTORS);
// Flash with real 4 KB ESP32 sectors, for record-capacity tests.
define_mock_flash!(BigFlash, BIG_SECTOR, BIG_SECTORS);

/// Mount a log over a mock flash. The flash is owned by the caller so it can
/// be inspected (or remounted) after the log is dropped.
pub fn mount<N: NorFlash>(flash: &mut N) -> FlashLog<'_, N> {
    FlashLog::mount(flash).unwrap()
}
