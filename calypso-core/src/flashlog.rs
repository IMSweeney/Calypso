//! Append-only record log on NOR flash, structured as a ring of erase sectors.
//!
//! Layout: the region is split into `N::ERASE_SIZE` sectors, each starting with an 8-byte header (magic + epoch, incremented as the ring advances).
//! Records follow as `[magic u16, len u16, id u32, payload, crc16 u16, pad]`, padded to 4 bytes (the ESP32 flash write granularity).
//! Records never span sectors: when one does not fit, the ring advances to the next sector and erases it first, which is what drops the oldest data.
//! Sectors are written circularly with increasing epochs, so the head sector determines the on-flash order and no index table is needed.
//!
//! Power loss: a crash mid-write leaves a record that fails its CRC or was never written (reading as 0xFF).
//! On mount the first invalid record ends that sector's data; if it failed on already-programmed bytes, which cannot be rewritten without erasing, the log advances to a fresh sector instead.
//! At most the record being written is lost.

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

const SECTOR_MAGIC: u32 = 0x4341_4C59; // "CALY"
const RECORD_MAGIC: u16 = 0xCA10;
const SECTOR_HEADER_LEN: u32 = 8;
const RECORD_HEADER_LEN: u32 = 8; // magic u16 + len u16 + id u32
const RECORD_TRAILER_LEN: u32 = 4; // crc16 + 2 bytes padding
const ERASED: u8 = 0xFF;

#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    /// The flash refused or failed an operation, and this says which: see [`Phase`].
    Flash {
        phase: Phase,
        /// The sector the operation was on. In the mount scan that is where the scan stopped.
        sector: u32,
        source: E,
    },
    /// Region smaller than two erase sectors.
    RegionTooSmall,
    /// Payload does not fit in a single sector (records never span sectors).
    TooLarge,
}

impl<E> Error<E> {
    /// [`Error::Flash`] for one operation, rather than at each of the fifteen call sites where one of them would eventually be wrong.
    fn flash(phase: Phase, sector: u32) -> impl FnOnce(E) -> Error<E> {
        move |source| Error::Flash {
            phase,
            sector,
            source,
        }
    }
}

/// What a failing flash operation was doing.
/// The repairs differ, and a bare "flash error" cannot tell them apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Reading sector headers while looking for the head.
    ScanHeaders,
    /// Reading a sector's records.
    ReadRecords,
    /// Erasing: the ring advancing a sector, a mount starting a fresh sector because the head's tail was corrupt, or [`FlashLog::format`] erasing all of them (in which case the sector is where it starts).
    Erase,
    /// Writing a record, or a sector header.
    Write,
}

fn pad4(n: u32) -> u32 {
    (n + 3) & !3
}

/// Read in whole aligned blocks.
///
/// `esp-storage` rejects a read whose offset or length is not a multiple of `READ_SIZE`, and the reads here are however long a record happens to be.
/// So every read goes a whole `BLOCK` at a time and copies out the window asked for, which needs no buffer bigger than 64 bytes.
/// `BLOCK` must be a multiple of the driver's `READ_SIZE` (4 on the ESP32).
trait ReadBlocks: ReadNorFlash {
    fn read_blocks(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        const BLOCK: usize = 64;
        let mut block = [0u8; BLOCK];
        let mut done = 0usize;
        while done < bytes.len() {
            let want = offset as usize + done;
            let start = want / BLOCK * BLOCK;
            let skip = want - start;
            let n = (bytes.len() - done).min(BLOCK - skip);
            self.read(start as u32, &mut block)?;
            bytes[done..done + n].copy_from_slice(&block[skip..skip + n]);
            done += n;
        }
        Ok(())
    }
}

impl<T: ReadNorFlash> ReadBlocks for T {}

/// CRC-16/CCITT (poly 0x1021, init 0xFFFF).
fn crc16(mut crc: u16, data: &[u8]) -> u16 {
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

pub struct FlashLog<'a, N> {
    flash: &'a mut N,
    sector_size: u32,
    sectors: u32,
    head: u32,
    head_epoch: u32,
    write_offset: u32,
    next_id: u32,
}

impl<'a, N: NorFlash> FlashLog<'a, N> {
    /// Mount the log over the whole flash region `flash` exposes, scanning for the current head sector and write position.
    /// The flash is borrowed, so the caller can reuse it after the log is dropped. Formats (erases everything) if no valid sector is found.
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
            flash
                .read_blocks(s * sector_size, &mut hdr)
                .map_err(Error::flash(Phase::ScanHeaders, s))?;
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
            // Nothing to invalidate: no sector has a valid header, and `advance_sector` erases each one as the ring reaches it.
            // Calling `format()` here instead would erase all 624 sectors up front, ~19 s of boot.
            None => {
                log.flash
                    .erase(0, log.sector_size)
                    .map_err(Error::flash(Phase::Erase, 0))?;
                log.write_sector_header()?;
            }
            Some((head, epoch)) => {
                log.head = head;
                log.head_epoch = epoch;
                let (end, last_id, clean) = log.scan_sector(head)?;
                if !clean {
                    // Tail is corrupt on programmed bytes, which cannot be rewritten without an erase, so start a fresh sector; ids still continue from the last valid record.
                    log.next_id = match last_id {
                        Some(id) => id + 1,
                        None => log.last_id_before_head()?.map_or(1, |id| id + 1),
                    };
                    log.advance_sector()?;
                } else {
                    log.write_offset = end;
                    log.next_id = match last_id {
                        Some(id) => id + 1,
                        // Head has no records (power lost right after advancing): continue ids from the previous sector.
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
            .map_err(Error::flash(Phase::Erase, 0))?;
        self.head = 0;
        self.head_epoch = 0;
        self.write_offset = SECTOR_HEADER_LEN;
        self.next_id = 1;
        self.write_sector_header()
    }

    /// Append a record; returns its id (monotonically increasing, starting at 1). Erases the oldest sector when the ring wraps.
    pub fn append(&mut self, payload: &[u8]) -> Result<u32, Error<N::Error>> {
        if payload.len() > self.max_payload() {
            return Err(Error::TooLarge);
        }
        let rec_len = RECORD_HEADER_LEN + pad4(payload.len() as u32) + RECORD_TRAILER_LEN;
        if self.write_offset + rec_len > self.sector_size {
            self.advance_sector()?;
        }

        let id = self.next_id;
        let sector = self.head;
        let pos = sector * self.sector_size + self.write_offset;

        // Header: magic, len, id.
        let mut hdr = [0u8; RECORD_HEADER_LEN as usize];
        hdr[0..2].copy_from_slice(&RECORD_MAGIC.to_le_bytes());
        hdr[2..4].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        hdr[4..8].copy_from_slice(&id.to_le_bytes());
        self.flash
            .write(pos, &hdr)
            .map_err(Error::flash(Phase::Write, sector))?;

        // Payload, in 4-byte-aligned writes (ESP32 flash granularity): full words straight from the payload, then the tail word padded with 0xFF.
        let full = payload.len() & !3;
        if full > 0 {
            self.flash
                .write(pos + RECORD_HEADER_LEN, &payload[..full])
                .map_err(Error::flash(Phase::Write, sector))?;
        }
        if full < payload.len() {
            let mut tail = [ERASED; 4];
            tail[..payload.len() - full].copy_from_slice(&payload[full..]);
            self.flash
                .write(pos + RECORD_HEADER_LEN + full as u32, &tail)
                .map_err(Error::flash(Phase::Write, sector))?;
        }

        // Trailer: CRC-16 over header[2..] + payload, padded to 4 bytes.
        let crc = crc16(crc16(0xFFFF, &hdr[2..]), payload);
        let trailer = [crc.to_le_bytes()[0], crc.to_le_bytes()[1], ERASED, ERASED];
        self.flash
            .write(
                pos + RECORD_HEADER_LEN + pad4(payload.len() as u32),
                &trailer,
            )
            .map_err(Error::flash(Phase::Write, sector))?;

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

    /// Iterate all live records, oldest first. Records larger than `buf` are skipped.
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
            self.flash
                .read_blocks(s * self.sector_size, &mut hdr)
                .map_err(Error::flash(Phase::ScanHeaders, s))?;
            let magic = u32::from_le_bytes(hdr[0..4].try_into().unwrap());
            let epoch = u32::from_le_bytes(hdr[4..8].try_into().unwrap());
            if magic == SECTOR_MAGIC
                && self.head_epoch >= epoch
                && self.head_epoch - epoch < self.sectors
            {
                live += 1;
            }
        }
        // Sectors are written circularly with increasing epochs, so the oldest live sector is `live - 1` steps behind the head.
        for k in (0..live).rev() {
            let sector = (self.head + self.sectors - k) % self.sectors;
            if !self.scan_records(sector, buf, &mut f)? {
                break;
            }
        }
        Ok(())
    }

    /// Read back a single record by id. Returns the payload length written into `buf`, if found.
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
        let sector = self.head;
        self.flash
            .write(sector * self.sector_size, &hdr)
            .map_err(Error::flash(Phase::Write, sector))
    }

    /// Move to the next sector in the ring, erasing it (this is what drops the oldest data on wrap) and writing a fresh header.
    fn advance_sector(&mut self) -> Result<(), Error<N::Error>> {
        self.head = (self.head + 1) % self.sectors;
        self.head_epoch = self.head_epoch.wrapping_add(1);
        let sector = self.head;
        let base = sector * self.sector_size;
        self.flash
            .erase(base, base + self.sector_size)
            .map_err(Error::flash(Phase::Erase, sector))?;
        self.write_sector_header()?;
        self.write_offset = SECTOR_HEADER_LEN;
        Ok(())
    }

    /// Validate the records in one sector. Returns `(end_offset, last_id, clean)`: `clean` is false if scanning stopped on corrupt *programmed* bytes (which cannot be rewritten without erasing) rather than on erased space.
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
    /// Returns false if iteration was stopped by `f` (a corrupt tail just ends the sector's records, not the whole iteration).
    fn scan_records(
        &mut self,
        sector: u32,
        buf: &mut [u8],
        f: &mut impl FnMut(u32, &[u8]) -> bool,
    ) -> Result<bool, Error<N::Error>> {
        self.scan_sector_with(sector, buf, |id, payload| f(id, payload))
            .map(|(_, stop)| !matches!(stop, ScanStop::Requested))
    }

    /// Walk one sector's records, validating magic/length/CRC.
    /// Delivers `f(id, payload)` for each valid record whose payload fits in `buf` (a zero-length `buf` validates structure without delivering data); iteration stops when `f` returns false.
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
            self.flash
                .read_blocks(base, &mut hdr)
                .map_err(Error::flash(Phase::ReadRecords, sector))?;
            if hdr.iter().all(|&b| b == ERASED) {
                break; // erased space: clean end of data
            }
            let magic = u16::from_le_bytes(hdr[0..2].try_into().unwrap());
            let len = u16::from_le_bytes(hdr[2..4].try_into().unwrap()) as u32;
            let rec_len = RECORD_HEADER_LEN + pad4(len) + RECORD_TRAILER_LEN;
            if magic != RECORD_MAGIC
                || len as usize > self.max_payload()
                || pos + rec_len > self.sector_size
            {
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
                    .read_blocks(base + RECORD_HEADER_LEN + off, &mut chunk[..n])
                    .map_err(Error::flash(Phase::ReadRecords, sector))?;
                crc = crc16(crc, &chunk[..n]);
                remaining -= n;
                off += n as u32;
            }
            let mut trailer = [0u8; RECORD_TRAILER_LEN as usize];
            self.flash
                .read_blocks(base + RECORD_HEADER_LEN + pad4(len), &mut trailer)
                .map_err(Error::flash(Phase::ReadRecords, sector))?;
            if u16::from_le_bytes(trailer[0..2].try_into().unwrap()) != crc {
                return Ok((pos, ScanStop::Corrupt));
            }
            let id = u32::from_le_bytes(hdr[4..8].try_into().unwrap());
            let keep_going = if !buf.is_empty() && len as usize <= buf.len() {
                self.flash
                    .read_blocks(base + RECORD_HEADER_LEN, &mut buf[..len as usize])
                    .map_err(Error::flash(Phase::ReadRecords, sector))?;
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
