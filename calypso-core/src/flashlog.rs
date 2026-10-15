//! Append-only record log on NOR flash, structured as a ring of erase
//! sectors. Used for the workout/rep store (see `docs/rep-tracking`).
//!
//! Layout:
//! - The region is split into `N::ERASE_SIZE` sectors.
//! - Each sector starts with an 8-byte header: magic + epoch (monotonic,
//!   incremented every time the ring advances to a sector).
//! - Records follow: `[magic u16, len u16, id u32, payload, crc16 u16, pad]`,
//!   padded to 4-byte alignment (the ESP32 flash write granularity).
//! - Records never span sectors; when a record doesn't fit, the ring advances
//!   to the next sector (erasing it first — this is what makes the oldest
//!   data fall off the end).
//!
//! Power-loss behaviour: a crash mid-write leaves a record that fails its
//! CRC check (or was never written, reading as 0xFF). On mount, the first
//! invalid record marks the end of the sector's data; if the failure was on
//! already-programmed bytes (can't rewrite without erasing), the log advances
//! to a fresh sector instead. At most the record being written is lost.
//!
//! Sectors are always written in circular order with increasing epochs, so
//! the on-flash order is fully determined by the head sector — no index
//! tables are needed at mount time.

use embedded_storage::nor_flash::NorFlash;

const SECTOR_MAGIC: u32 = 0x4341_4C59; // "CALY"
const RECORD_MAGIC: u16 = 0xCA10;
const SECTOR_HEADER_LEN: u32 = 8;
const RECORD_HEADER_LEN: u32 = 8; // magic u16 + len u16 + id u32
const RECORD_TRAILER_LEN: u32 = 4; // crc16 + 2 bytes padding
const ERASED: u8 = 0xFF;

#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Flash(E),
    /// Region smaller than two erase sectors.
    RegionTooSmall,
    /// Payload does not fit in a single sector (records never span sectors).
    TooLarge,
}

fn pad4(n: u32) -> u32 {
    (n + 3) & !3
}

/// CRC-16/CCITT (poly 0x1021, init 0xFFFF).
fn crc16(mut crc: u16, data: &[u8]) -> u16 {
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

pub struct FlashLog<'a, N> {
    flash: &'a mut N,
    sector_size: u32,
    sectors: u32,
    /// Sector currently being appended to.
    head: u32,
    head_epoch: u32,
    /// Next write position within the head sector.
    write_offset: u32,
    next_id: u32,
}

impl<'a, N: NorFlash> FlashLog<'a, N> {
    /// Mount the log over the whole flash region `flash` exposes (borrowed, so
    /// the caller keeps ownership and can reuse it after the log is dropped),
    /// scanning for the current head sector and write position. Formats
    /// (erases everything) if no valid sector is found.
    pub fn mount(flash: &'a mut N) -> Result<Self, Error<N::Error>> {
        let sector_size = N::ERASE_SIZE as u32;
        let capacity = flash.capacity() as u32;
        let sectors = capacity / sector_size;
        if sectors < 2 {
            return Err(Error::RegionTooSmall);
        }

        // Single pass: find the sector with the highest epoch (the head).
        let mut head: Option<(u32, u32)> = None; // (sector, epoch)
        for s in 0..sectors {
            let mut hdr = [0u8; 8];
            flash.read(s * sector_size, &mut hdr).map_err(Error::Flash)?;
            let magic = u32::from_le_bytes(hdr[0..4].try_into().unwrap());
            let epoch = u32::from_le_bytes(hdr[4..8].try_into().unwrap());
            if magic == SECTOR_MAGIC && head.is_none_or(|(_, e)| epoch > e) {
                head = Some((s, epoch));
            }
        }

        let mut log = FlashLog {
            flash,
            sector_size,
            sectors,
            head: 0,
            head_epoch: 0,
            write_offset: SECTOR_HEADER_LEN,
            next_id: 1,
        };

        match head {
            None => log.format()?,
            Some((head, epoch)) => {
                log.head = head;
                log.head_epoch = epoch;
                let (end, last_id, clean) = log.scan_sector(head)?;
                if !clean {
                    // Tail is corrupt on programmed bytes: they cannot be
                    // rewritten without an erase, so start a fresh sector.
                    // Ids still continue from the last valid record.
                    log.next_id = match last_id {
                        Some(id) => id + 1,
                        None => log.last_id_before_head()?.map_or(1, |id| id + 1),
                    };
                    log.advance_sector()?;
                } else {
                    log.write_offset = end;
                    log.next_id = match last_id {
                        Some(id) => id + 1,
                        // Head has no records (power lost right after
                        // advancing): continue ids from the previous sector.
                        None => log.last_id_before_head()?.map_or(1, |id| id + 1),
                    };
                }
            }
        }
        Ok(log)
    }

    /// Erase everything and start a fresh log.
    pub fn format(&mut self) -> Result<(), Error<N::Error>> {
        self.flash
            .erase(0, self.sectors * self.sector_size)
            .map_err(Error::Flash)?;
        self.head = 0;
        self.head_epoch = 0;
        self.write_offset = SECTOR_HEADER_LEN;
        self.next_id = 1;
        self.write_sector_header()
    }

    /// Append a record; returns its id (monotonically increasing, starting
    /// at 1). Erases the oldest sector when the ring wraps.
    pub fn append(&mut self, payload: &[u8]) -> Result<u32, Error<N::Error>> {
        if payload.len() > self.max_payload() {
            return Err(Error::TooLarge);
        }
        let rec_len = RECORD_HEADER_LEN + pad4(payload.len() as u32) + RECORD_TRAILER_LEN;
        if self.write_offset + rec_len > self.sector_size {
            self.advance_sector()?;
        }

        let id = self.next_id;
        let pos = self.head * self.sector_size + self.write_offset;

        // Header: magic, len, id.
        let mut hdr = [0u8; RECORD_HEADER_LEN as usize];
        hdr[0..2].copy_from_slice(&RECORD_MAGIC.to_le_bytes());
        hdr[2..4].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        hdr[4..8].copy_from_slice(&id.to_le_bytes());
        self.flash.write(pos, &hdr).map_err(Error::Flash)?;

        // Payload, in 4-byte-aligned writes (ESP32 flash granularity):
        // full words straight from the payload, then the tail word padded
        // with 0xFF.
        let full = payload.len() & !3;
        if full > 0 {
            self.flash
                .write(pos + RECORD_HEADER_LEN, &payload[..full])
                .map_err(Error::Flash)?;
        }
        if full < payload.len() {
            let mut tail = [ERASED; 4];
            tail[..payload.len() - full].copy_from_slice(&payload[full..]);
            self.flash
                .write(pos + RECORD_HEADER_LEN + full as u32, &tail)
                .map_err(Error::Flash)?;
        }

        // Trailer: CRC-16 over header[2..] + payload, padded to 4 bytes.
        let crc = crc16(crc16(0xFFFF, &hdr[2..]), payload);
        let trailer = [crc.to_le_bytes()[0], crc.to_le_bytes()[1], ERASED, ERASED];
        self.flash
            .write(pos + RECORD_HEADER_LEN + pad4(payload.len() as u32), &trailer)
            .map_err(Error::Flash)?;

        self.write_offset += rec_len;
        self.next_id += 1;
        Ok(id)
    }

    /// Largest payload that fits one record in one sector.
    pub fn max_payload(&self) -> usize {
        (self.sector_size - SECTOR_HEADER_LEN - RECORD_HEADER_LEN - RECORD_TRAILER_LEN) as usize
    }

    /// Number of erase sectors in the ring.
    pub fn sectors(&self) -> u32 {
        self.sectors
    }

    /// Iterate all live records, oldest first. Records larger than `buf`
    /// are skipped.
    pub fn for_each(
        &mut self,
        buf: &mut [u8],
        mut f: impl FnMut(u32, &[u8]),
    ) -> Result<(), Error<N::Error>> {
        self.for_each_while(buf, |id, payload| {
            f(id, payload);
            true
        })
    }

    /// Like `for_each`, but iteration stops when `f` returns false.
    pub fn for_each_while(
        &mut self,
        buf: &mut [u8],
        mut f: impl FnMut(u32, &[u8]) -> bool,
    ) -> Result<(), Error<N::Error>> {
        // Count live sectors (valid headers within the ring's epoch window).
        let mut live = 0u32;
        for s in 0..self.sectors {
            let mut hdr = [0u8; 8];
            self.flash.read(s * self.sector_size, &mut hdr).map_err(Error::Flash)?;
            let magic = u32::from_le_bytes(hdr[0..4].try_into().unwrap());
            let epoch = u32::from_le_bytes(hdr[4..8].try_into().unwrap());
            if magic == SECTOR_MAGIC && self.head_epoch >= epoch && self.head_epoch - epoch < self.sectors {
                live += 1;
            }
        }
        // Sectors are written circularly with increasing epochs, so the
        // oldest live sector is `live - 1` steps behind the head.
        for k in (0..live).rev() {
            let sector = (self.head + self.sectors - k) % self.sectors;
            if !self.scan_records(sector, buf, &mut f)? {
                break;
            }
        }
        Ok(())
    }

    /// Read back a single record by id. Returns the payload length written
    /// into `buf`, if found.
    pub fn read(&mut self, want_id: u32, buf: &mut [u8]) -> Result<Option<usize>, Error<N::Error>> {
        let mut found = None;
        self.for_each_while(buf, &mut |id: u32, payload: &[u8]| {
            if id == want_id {
                found = Some(payload.len());
                false // stop at first match
            } else {
                true
            }
        })?;
        Ok(found)
    }

    // --- internals ---------------------------------------------------------

    fn write_sector_header(&mut self) -> Result<(), Error<N::Error>> {
        let mut hdr = [0u8; 8];
        hdr[0..4].copy_from_slice(&SECTOR_MAGIC.to_le_bytes());
        hdr[4..8].copy_from_slice(&self.head_epoch.to_le_bytes());
        self.flash
            .write(self.head * self.sector_size, &hdr)
            .map_err(Error::Flash)
    }

    /// Move to the next sector in the ring, erasing it (this is what drops
    /// the oldest data on wrap) and writing a fresh header.
    fn advance_sector(&mut self) -> Result<(), Error<N::Error>> {
        self.head = (self.head + 1) % self.sectors;
        self.head_epoch = self.head_epoch.wrapping_add(1);
        let base = self.head * self.sector_size;
        self.flash
            .erase(base, base + self.sector_size)
            .map_err(Error::Flash)?;
        self.write_sector_header()?;
        self.write_offset = SECTOR_HEADER_LEN;
        Ok(())
    }

    /// Validate the records in one sector. Returns `(end_offset, last_id,
    /// clean)`: `clean` is false if scanning stopped on corrupt *programmed*
    /// bytes (which cannot be rewritten without erasing) rather than on
    /// erased space.
    fn scan_sector(&mut self, sector: u32) -> Result<(u32, Option<u32>, bool), Error<N::Error>> {
        let mut last_id = None;
        self.scan_sector_with(sector, &mut [], |id, _| {
            last_id = Some(id);
            true
        })
        .map(|(end, stop)| (end, last_id, !matches!(stop, ScanStop::Corrupt)))
    }

    fn last_id_before_head(&mut self) -> Result<Option<u32>, Error<N::Error>> {
        if self.head_epoch == 0 {
            return Ok(None);
        }
        let prev = (self.head + self.sectors - 1) % self.sectors;
        let (_, last, _) = self.scan_sector(prev)?;
        Ok(last)
    }

    /// Deliver each valid record's payload in a sector.
    /// Returns false if iteration was stopped by `f` (a corrupt tail just
    /// ends the sector's records, not the whole iteration).
    fn scan_records(
        &mut self,
        sector: u32,
        buf: &mut [u8],
        f: &mut impl FnMut(u32, &[u8]) -> bool,
    ) -> Result<bool, Error<N::Error>> {
        self.scan_sector_with(sector, buf, |id, payload| f(id, payload))
            .map(|(_, stop)| !matches!(stop, ScanStop::Requested))
    }

    /// Walk one sector's records, validating magic/length/CRC. Delivers
    /// `f(id, payload)` for each valid record whose payload fits in `buf`
    /// (a zero-length `buf` validates structure without delivering data);
    /// iteration stops when `f` returns false.
    /// Returns `(end_offset, stop_reason)`.
    fn scan_sector_with(
        &mut self,
        sector: u32,
        buf: &mut [u8],
        mut f: impl FnMut(u32, &[u8]) -> bool,
    ) -> Result<(u32, ScanStop), Error<N::Error>> {
        let mut pos = SECTOR_HEADER_LEN;
        loop {
            let base = sector * self.sector_size + pos;
            let mut hdr = [0u8; RECORD_HEADER_LEN as usize];
            self.flash.read(base, &mut hdr).map_err(Error::Flash)?;
            if hdr.iter().all(|&b| b == ERASED) {
                break; // erased space: clean end of data
            }
            let magic = u16::from_le_bytes(hdr[0..2].try_into().unwrap());
            let len = u16::from_le_bytes(hdr[2..4].try_into().unwrap()) as u32;
            let rec_len = RECORD_HEADER_LEN + pad4(len) + RECORD_TRAILER_LEN;
            if magic != RECORD_MAGIC || len as usize > self.max_payload() || pos + rec_len > self.sector_size {
                return Ok((pos, ScanStop::Corrupt));
            }
            // CRC over header[2..8] + payload (read in small chunks).
            let mut crc = crc16(0xFFFF, &hdr[2..8]);
            let mut chunk = [0u8; 64];
            let mut remaining = len as usize;
            let mut off = 0u32;
            while remaining > 0 {
                let n = remaining.min(chunk.len());
                self.flash
                    .read(base + RECORD_HEADER_LEN + off, &mut chunk[..n])
                    .map_err(Error::Flash)?;
                crc = crc16(crc, &chunk[..n]);
                remaining -= n;
                off += n as u32;
            }
            let mut trailer = [0u8; RECORD_TRAILER_LEN as usize];
            self.flash
                .read(base + RECORD_HEADER_LEN + pad4(len), &mut trailer)
                .map_err(Error::Flash)?;
            if u16::from_le_bytes(trailer[0..2].try_into().unwrap()) != crc {
                return Ok((pos, ScanStop::Corrupt));
            }
            let id = u32::from_le_bytes(hdr[4..8].try_into().unwrap());
            let keep_going = if !buf.is_empty() && len as usize <= buf.len() {
                self.flash
                    .read(base + RECORD_HEADER_LEN, &mut buf[..len as usize])
                    .map_err(Error::Flash)?;
                f(id, &buf[..len as usize])
            } else if buf.is_empty() {
                f(id, &[])
            } else {
                true // record too big for buf: skip delivery, keep scanning
            };
            if !keep_going {
                return Ok((pos + rec_len, ScanStop::Requested));
            }
            pos += rec_len;
        }
        Ok((pos, ScanStop::Clean))
    }
}

/// Why scanning a sector stopped.
#[derive(Debug, PartialEq, Eq)]
enum ScanStop {
    /// Reached erased space: normal end of data.
    Clean,
    /// Hit corrupt programmed bytes (torn write): cannot append here.
    Corrupt,
    /// The caller's closure asked to stop.
    Requested,
}
