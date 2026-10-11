//! The flight log's flash layout: one chunk per erase page, appended from the start of a region
//! and erased as a whole.
//!
//! A page is `magic: u16 | tlog_start: u32 | chunk_len: u16 | crc: u16 | chunk` (little-endian).
//! `tlog_start` is where the chunk's records begin in its boot's `.tlog`, which puts any offset a
//! binary search away. The CRC covers the page header and the chunk's own header, so one short read
//! per page is enough to index the whole log.
//!
//! The layout survives power loss at any point without a repair step:
//! - The valid pages form a prefix of the region; the first invalid one ends it.
//! - A chunk is programmed before its page header, so a torn write reads as invalid.
//! - A page is erased before it is written unless it already reads blank, which also clears torn
//!   writes and whatever an older layout left behind.
//! - Erasing works down from the end, so an interrupted erase leaves a shorter valid prefix. It
//!   covers the whole region, so the pages ahead of the log are blank and appending needs no erase.

use core::ops::Range;

use embedded_storage_async::nor_flash::NorFlash;

use super::{CHUNK_HEADER_LEN, ChunkHeader, decompress_chunk, read_tlog};

pub const PAGE_SIZE: u32 = 4096;
pub const PAGE_HEADER_LEN: usize = 10;

/// The longest chunk a page holds.
pub const MAX_CHUNK: usize = PAGE_SIZE as usize - PAGE_HEADER_LEN;

/// Boots listed; older ones stay in flash, but are no longer offered for download.
pub const MAX_LOGS: usize = 64;

const PROBE_LEN: usize = PAGE_HEADER_LEN + CHUNK_HEADER_LEN;
const MAGIC: u16 = 0x4c47;

/// How much one [`PageLog::erase_step`] covers: a W25Q block, its fastest erase. Reading one to see
/// whether it is blank already takes a sixth of the time erasing it does.
const ERASE_STEP: u32 = 0x1_0000;

const CRC: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_16_IBM_3740);

/// One boot's run of pages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LogSpan {
    /// The boot that wrote it.
    pub id: u16,
    /// Length of its `.tlog`.
    pub size: u32,
    first_page: u32,
    pages: u32,
}

#[derive(Debug, PartialEq, Eq)]
pub enum PageLogError<E> {
    Flash(E),
    Full,
    /// Too long for a page, or without a chunk header.
    InvalidChunk,
    /// An erase is underway.
    Erasing,
}

/// What a read needs to hold on to. Static-friendly: a new one is all zeroes.
pub struct ReadBuffers<const RAW: usize> {
    page: [u8; PAGE_SIZE as usize],
    raw: [u8; RAW],
}

/// The chunk currently decompressed into [`ReadBuffers::raw`]. Pages never change once written,
/// so only an erase invalidates it.
#[derive(Clone, Copy)]
struct Decoded {
    page: u32,
    tlog_start: u32,
    tlog_len: u32,
    raw_len: usize,
}

#[derive(Clone, Copy)]
struct PageHeader {
    tlog_start: u32,
    chunk_len: u16,
    chunk: ChunkHeader,
}

/// The flight log on a [`NorFlash`] region whose erase size is [`PAGE_SIZE`]. `RAW` is the
/// longest decompressed chunk, as for the [`super::ChunkBuilder`] that cut them.
pub struct PageLog<'b, F, const RAW: usize> {
    flash: F,
    /// Page aligned.
    range: Range<u32>,
    /// Pages from the start of the region that may hold data: the valid prefix, or what an erase
    /// has left to do.
    end: u32,
    /// Pages below which an erase still has to go.
    erasing: Option<u32>,
    logs: heapless::Vec<LogSpan, MAX_LOGS>,
    buffers: &'b mut ReadBuffers<RAW>,
    decoded: Option<Decoded>,
}

impl<E> From<E> for PageLogError<E> {
    fn from(error: E) -> Self {
        Self::Flash(error)
    }
}

impl<const RAW: usize> ReadBuffers<RAW> {
    #[allow(
        clippy::large_stack_arrays,
        reason = "only evaluated at compile time, to initialize statics in place"
    )]
    pub const fn new() -> Self {
        Self {
            page: [0; PAGE_SIZE as usize],
            raw: [0; RAW],
        }
    }
}

impl<const RAW: usize> Default for ReadBuffers<RAW> {
    fn default() -> Self {
        Self::new()
    }
}

impl PageHeader {
    fn encode(tlog_start: u32, chunk: &[u8]) -> Option<[u8; PAGE_HEADER_LEN]> {
        let chunk_header = chunk.first_chunk::<CHUNK_HEADER_LEN>()?;
        let [l0, l1] = u16::try_from(chunk.len()).ok()?.to_le_bytes();
        let [m0, m1] = MAGIC.to_le_bytes();
        let [t0, t1, t2, t3] = tlog_start.to_le_bytes();
        let fields = [m0, m1, t0, t1, t2, t3, l0, l1];

        let mut digest = CRC.digest();
        digest.update(&fields);
        digest.update(chunk_header);
        let [c0, c1] = digest.finalize().to_le_bytes();
        Some([m0, m1, t0, t1, t2, t3, l0, l1, c0, c1])
    }

    fn parse(probe: &[u8; PROBE_LEN]) -> Option<Self> {
        let (fields, rest) = probe.split_first_chunk::<8>()?;
        let (crc, chunk_header) = rest.split_first_chunk::<2>()?;
        let [m0, m1, t0, t1, t2, t3, l0, l1] = *fields;

        let mut digest = CRC.digest();
        digest.update(fields);
        digest.update(chunk_header);
        if u16::from_le_bytes([m0, m1]) != MAGIC || digest.finalize() != u16::from_le_bytes(*crc) {
            return None;
        }

        let chunk_len = u16::from_le_bytes([l0, l1]);
        (CHUNK_HEADER_LEN..=MAX_CHUNK)
            .contains(&usize::from(chunk_len))
            .then_some(Self {
                tlog_start: u32::from_le_bytes([t0, t1, t2, t3]),
                chunk_len,
                chunk: ChunkHeader::parse(chunk_header)?,
            })
    }

    fn tlog_end(&self) -> u32 {
        self.tlog_start
            .saturating_add(u32::from(self.chunk.tlog_len))
    }
}

impl<'b, F: NorFlash, const RAW: usize> PageLog<'b, F, RAW> {
    /// An empty log over `range`, which has to be page aligned. [`Self::scan`] indexes what is
    /// already there.
    pub fn new(flash: F, range: Range<u32>, buffers: &'b mut ReadBuffers<RAW>) -> Self {
        const {
            assert!(F::ERASE_SIZE == PAGE_SIZE as usize);
            assert!(F::WRITE_SIZE == 1 && F::READ_SIZE == 1);
        };

        Self {
            flash,
            range,
            end: 0,
            erasing: None,
            logs: heapless::Vec::new(),
            buffers,
            decoded: None,
        }
    }

    /// Indexes the pages in the region, up to the first that holds no valid page. On an error the
    /// index ends where reading stopped.
    pub async fn scan(&mut self) -> Result<(), F::Error> {
        self.end = 0;
        self.logs.clear();
        self.decoded = None;
        while self.end < self.capacity()
            && let Some(header) = self.probe(self.end).await?
        {
            self.note(self.end, header);
            self.end = self.end.saturating_add(1);
        }
        Ok(())
    }

    /// The flash, for whatever else lives on it outside the log region.
    pub fn flash(&mut self) -> &mut F {
        &mut self.flash
    }

    /// Oldest first.
    pub fn logs(&self) -> &[LogSpan] {
        &self.logs
    }

    /// The id the next boot's chunks should carry: one past the last, never zero.
    pub fn next_boot(&self) -> u16 {
        self.logs
            .last()
            .map_or(1, |last| last.id.wrapping_add(1).max(1))
    }

    /// Bytes of the region.
    pub fn total(&self) -> u32 {
        self.capacity().saturating_mul(PAGE_SIZE)
    }

    /// Bytes of the region in use.
    pub fn used(&self) -> u32 {
        self.end.saturating_mul(PAGE_SIZE)
    }

    /// Writes `chunk` into the next page, as part of the log of the boot its header names.
    pub async fn append(&mut self, chunk: &[u8]) -> Result<(), PageLogError<F::Error>> {
        if self.erasing.is_some() {
            return Err(PageLogError::Erasing);
        }
        if self.end >= self.capacity() {
            return Err(PageLogError::Full);
        }
        let boot = ChunkHeader::parse(chunk)
            .ok_or(PageLogError::InvalidChunk)?
            .boot;
        let tlog_start = match self.logs.last() {
            Some(last) if last.id == boot => last.size,
            _ => 0,
        };
        let header = PageHeader::encode(tlog_start, chunk)
            .filter(|_| chunk.len() <= MAX_CHUNK)
            .ok_or(PageLogError::InvalidChunk)?;

        let address = self.address(self.end);
        if !self
            .is_blank(address..address.saturating_add(PAGE_SIZE))
            .await?
        {
            self.flash
                .erase(address, address.saturating_add(PAGE_SIZE))
                .await?;
        }
        let body = address.saturating_add(PAGE_HEADER_LEN as u32);
        self.flash.write(body, chunk).await?;
        self.flash.write(address, &header).await?;

        let mut probe = [0; PROBE_LEN];
        if let Some((page_header, chunk_header)) = probe.split_first_chunk_mut::<PAGE_HEADER_LEN>()
            && let Some(written) = chunk.first_chunk::<CHUNK_HEADER_LEN>()
        {
            *page_header = header;
            chunk_header.copy_from_slice(written);
        }
        if let Some(header) = PageHeader::parse(&probe) {
            self.note(self.end, header);
        }
        self.end = self.end.saturating_add(1);
        Ok(())
    }

    /// Fills `out` from `offset` of log `id`'s `.tlog`, framed for `system_id`, and returns how much
    /// it wrote: less than `out.len()` only at the end of the log, or where it is unreadable.
    pub async fn read(
        &mut self,
        id: u16,
        offset: u32,
        system_id: u8,
        out: &mut [u8],
    ) -> Result<usize, F::Error> {
        let Some(log) = self.logs.iter().find(|log| log.id == id).copied() else {
            return Ok(0);
        };

        let mut written = 0;
        while let Some(rest) = out.get_mut(written..)
            && !rest.is_empty()
        {
            let position = offset.saturating_add(written as u32);
            if position >= log.size {
                break;
            }
            let Some(decoded) = self.decode(&log, position).await? else {
                break;
            };
            let raw = self.buffers.raw.get(..decoded.raw_len).unwrap_or_default();
            let from = position.saturating_sub(decoded.tlog_start) as usize;
            match read_tlog(raw, system_id, from, rest) {
                Some(n) if n > 0 => written = written.saturating_add(n),
                _ => break,
            }
        }
        Ok(written)
    }

    /// Empties the index at once; the flash follows through [`Self::erase_step`]. Nothing can be
    /// appended until that is done.
    pub fn begin_erase(&mut self) {
        self.logs.clear();
        self.decoded = None;
        self.erasing = Some(self.capacity());
    }

    /// Erases the topmost block of the region not yet erased, unless it is blank already. Returns
    /// whether any is left.
    pub async fn erase_step(&mut self) -> Result<bool, F::Error> {
        let Some(top_page) = self.erasing.filter(|&page| page > 0) else {
            self.erasing = None;
            return Ok(false);
        };
        let top = self.address(top_page);
        let last = self.address(top_page.saturating_sub(1));
        let bottom = last.saturating_sub(last % ERASE_STEP).max(self.range.start);

        if !self.is_blank(bottom..top).await? {
            self.flash.erase(bottom, top).await?;
        }
        let bottom_page = bottom.saturating_sub(self.range.start) / PAGE_SIZE;
        self.end = self.end.min(bottom_page);
        self.erasing = (bottom_page > 0).then_some(bottom_page);
        Ok(self.erasing.is_some())
    }

    async fn is_blank(&mut self, range: Range<u32>) -> Result<bool, F::Error> {
        for address in range.step_by(PAGE_SIZE as usize) {
            self.flash.read(address, &mut self.buffers.page).await?;
            if self.buffers.page.iter().any(|&b| b != 0xff) {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn capacity(&self) -> u32 {
        self.range.end.saturating_sub(self.range.start) / PAGE_SIZE
    }

    fn address(&self, page: u32) -> u32 {
        self.range
            .start
            .saturating_add(page.saturating_mul(PAGE_SIZE))
    }

    async fn probe(&mut self, page: u32) -> Result<Option<PageHeader>, F::Error> {
        let mut probe = [0; PROBE_LEN];
        self.flash.read(self.address(page), &mut probe).await?;
        Ok(PageHeader::parse(&probe))
    }

    /// Accounts for a valid page in the index.
    fn note(&mut self, page: u32, header: PageHeader) {
        let id = header.chunk.boot;
        match self.logs.last_mut() {
            Some(last) if last.id == id => {
                last.pages = last.pages.saturating_add(1);
                last.size = header.tlog_end();
            }
            _ => {
                if self.logs.is_full() {
                    self.logs.remove(0);
                }
                let _ = self.logs.push(LogSpan {
                    id,
                    size: header.tlog_end(),
                    first_page: page,
                    pages: 1,
                });
            }
        }
    }

    /// Makes sure the chunk holding `position` of `log` is the one decompressed, finding its page
    /// by binary search over the page headers. `None` if that chunk is unreadable.
    #[allow(
        clippy::arithmetic_side_effects,
        reason = "lo < hi throughout, both within the log's pages"
    )]
    async fn decode(&mut self, log: &LogSpan, position: u32) -> Result<Option<Decoded>, F::Error> {
        let pages = log.first_page..log.first_page.saturating_add(log.pages);
        if let Some(decoded) = self.decoded
            && pages.contains(&decoded.page)
            && (decoded.tlog_start..decoded.tlog_start + decoded.tlog_len).contains(&position)
        {
            return Ok(Some(decoded));
        }

        // The last page starting at or before `position`.
        let (mut lo, mut hi) = (pages.start, pages.end);
        while hi - lo > 1 {
            let mid = lo + (hi - lo) / 2;
            let Some(header) = self.probe(mid).await? else {
                return Ok(None);
            };
            if header.tlog_start <= position {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        let Some(header) = self.probe(lo).await? else {
            return Ok(None);
        };
        let body = self.address(lo).saturating_add(PAGE_HEADER_LEN as u32);
        let chunk = self
            .buffers
            .page
            .get_mut(..usize::from(header.chunk_len))
            .unwrap_or_default();
        self.flash.read(body, chunk).await?;

        self.decoded = decompress_chunk(chunk, &mut self.buffers.raw).map(|raw| Decoded {
            page: lo,
            tlog_start: header.tlog_start,
            tlog_len: u32::from(header.chunk.tlog_len),
            raw_len: raw.len(),
        });
        Ok(self
            .decoded
            .filter(|d| (d.tlog_start..d.tlog_start + d.tlog_len).contains(&position)))
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::indexing_slicing,
        clippy::arithmetic_side_effects,
        reason = "test assertions over fixed buffers"
    )]

    extern crate std;

    use super::*;
    use crate::Downlink;
    use crate::log::ChunkBuilder;
    use embassy_futures::block_on;
    use embedded_storage_async::nor_flash::{
        ErrorType, NorFlashError, NorFlashErrorKind, ReadNorFlash,
    };
    use rapid_dialect::rapid::messages::Attitude;
    use std::boxed::Box;
    use std::vec;
    use std::vec::Vec;

    const RAW: usize = 16384;
    const SYSTEM_ID: u8 = 4;
    /// Four 64 KiB blocks behind one for something else, as the params are on the vehicle.
    const REGION: Range<u32> = 0x1_0000..0x5_0000;

    type Builder = ChunkBuilder<RAW, MAX_CHUNK>;
    type Log<'a> = PageLog<'a, &'a mut MemFlash, RAW>;

    /// NOR semantics: programming can only clear bits, erasing sets a whole page.
    struct MemFlash {
        data: Vec<u8>,
    }

    #[derive(Debug, PartialEq, Eq)]
    struct MemError;

    impl NorFlashError for MemError {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    impl ErrorType for MemFlash {
        type Error = MemError;
    }

    impl ReadNorFlash for MemFlash {
        const READ_SIZE: usize = 1;

        async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), MemError> {
            let offset = offset as usize;
            bytes.copy_from_slice(&self.data[offset..offset + bytes.len()]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.data.len()
        }
    }

    impl NorFlash for MemFlash {
        const WRITE_SIZE: usize = 1;
        const ERASE_SIZE: usize = PAGE_SIZE as usize;

        async fn erase(&mut self, from: u32, to: u32) -> Result<(), MemError> {
            assert_eq!(from % PAGE_SIZE, 0);
            assert_eq!(to % PAGE_SIZE, 0);
            self.data[from as usize..to as usize].fill(0xff);
            Ok(())
        }

        async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), MemError> {
            let offset = offset as usize;
            for (cell, byte) in self.data[offset..offset + bytes.len()]
                .iter_mut()
                .zip(bytes)
            {
                *cell &= byte;
            }
            Ok(())
        }
    }

    impl MemFlash {
        fn new() -> Self {
            Self {
                data: vec![0xff; REGION.end as usize],
            }
        }

        fn page_mut(&mut self, page: u32) -> &mut [u8] {
            let start = (REGION.start + page * PAGE_SIZE) as usize;
            &mut self.data[start..start + PAGE_SIZE as usize]
        }
    }

    fn new_buffers() -> Box<ReadBuffers<RAW>> {
        Box::new(ReadBuffers::new())
    }

    fn open<'a>(flash: &'a mut MemFlash, buffers: &'a mut ReadBuffers<RAW>) -> Log<'a> {
        let mut log = PageLog::new(flash, REGION, buffers);
        block_on(log.scan()).unwrap();
        log
    }

    /// A boot's chunks, from `records` attitude messages that compress about as badly as real
    /// telemetry, and the `.tlog` they should read back as.
    fn boot(boot: u16, records: u32) -> (Vec<Vec<u8>>, Vec<u8>) {
        let mut builder = Box::new(Builder::new());
        let mut out = vec![0; Builder::OUT_LEN];
        let mut raw = vec![0; RAW];
        let (mut chunks, mut tlog) = (Vec::new(), Vec::new());

        for t in 0..records {
            let message = Downlink::from_self(Attitude {
                time_boot_ms: t,
                roll: (t.wrapping_mul(2_654_435_761) as f32).sin(),
                ..Default::default()
            });
            assert!(builder.push(t, &message));
            while builder.is_ready() || (t + 1 == records && !builder.is_empty()) {
                let len = builder.take_chunk(boot, &mut out).unwrap();
                let chunk = out[..len].to_vec();
                let header = ChunkHeader::parse(&chunk).unwrap();
                let records = decompress_chunk(&chunk, &mut raw).unwrap();
                let mut part = vec![0; usize::from(header.tlog_len)];
                read_tlog(records, SYSTEM_ID, 0, &mut part).unwrap();
                tlog.extend(part);
                chunks.push(chunk);
            }
        }
        (chunks, tlog)
    }

    fn append_all(log: &mut Log<'_>, chunks: &[Vec<u8>]) {
        for chunk in chunks {
            block_on(log.append(chunk)).unwrap();
        }
    }

    fn read(log: &mut Log<'_>, id: u16, offset: u32, len: usize) -> Vec<u8> {
        let mut out = vec![0; len];
        let n = block_on(log.read(id, offset, SYSTEM_ID, &mut out)).unwrap();
        out.truncate(n);
        out
    }

    /// Reads in the window size LOG_DATA uses, front to back.
    fn read_whole(log: &mut Log<'_>, id: u16) -> Vec<u8> {
        let mut tlog = Vec::new();
        loop {
            let part = read(log, id, tlog.len() as u32, 90);
            tlog.extend(&part);
            if part.len() < 90 {
                return tlog;
            }
        }
    }

    #[test]
    fn boots_read_back_after_reopening_from_any_offset() {
        let mut flash = MemFlash::new();
        let boots = [(1, boot(1, 3000)), (2, boot(2, 10)), (3, boot(3, 6000))];
        {
            let mut buffers = new_buffers();
            let mut log = open(&mut flash, &mut buffers);
            assert_eq!(log.next_boot(), 1);
            for (_, (chunks, _)) in &boots {
                append_all(&mut log, chunks);
            }
        }

        let mut buffers = new_buffers();
        let mut log = open(&mut flash, &mut buffers);
        assert_eq!(log.next_boot(), 4);
        let listed: Vec<_> = log.logs().iter().map(|l| (l.id, l.size)).collect();
        let expected: Vec<_> = boots
            .iter()
            .map(|(id, (_, tlog))| (*id, tlog.len() as u32))
            .collect();
        assert_eq!(listed, expected);
        let pages: usize = boots.iter().map(|(_, (chunks, _))| chunks.len()).sum();
        assert_eq!(log.used(), pages as u32 * PAGE_SIZE);

        for (id, (_, tlog)) in &boots {
            assert_eq!(&read_whole(&mut log, *id), tlog, "boot {id}");

            // Backwards and forwards, across chunk boundaries, as gap fills would.
            let mut offset = 7_u32;
            for _ in 0..50 {
                offset =
                    (offset.wrapping_mul(1_103_515_245).wrapping_add(12345)) % tlog.len() as u32;
                let end = (offset as usize + 300).min(tlog.len());
                assert_eq!(read(&mut log, *id, offset, 300), tlog[offset as usize..end]);
            }
        }
        assert!(read(&mut log, 9, 0, 90).is_empty());
    }

    #[test]
    fn a_torn_write_ends_the_log_and_is_cleared_by_the_next() {
        let mut flash = MemFlash::new();
        let (chunks, tlog) = boot(1, 4000);
        let (first, rest) = chunks.split_at(2);
        {
            let mut buffers = new_buffers();
            let mut log = open(&mut flash, &mut buffers);
            append_all(&mut log, first);
        }
        // Power lost after the chunk, before its header.
        flash.page_mut(2)[PAGE_HEADER_LEN..PAGE_HEADER_LEN + 100].fill(0x00);

        let mut buffers = new_buffers();
        let mut log = open(&mut flash, &mut buffers);
        assert_eq!(log.used(), 2 * PAGE_SIZE);
        append_all(&mut log, rest);
        assert_eq!(read_whole(&mut log, 1), tlog);
    }

    #[test]
    fn foreign_data_reads_as_an_empty_log_and_is_written_over() {
        let mut flash = MemFlash::new();
        for page in 0..8 {
            flash.page_mut(page).fill(0x5a);
        }
        let (chunks, tlog) = boot(1, 4000);

        let mut buffers = new_buffers();
        let mut log = open(&mut flash, &mut buffers);
        assert!(log.logs().is_empty());
        append_all(&mut log, &chunks);
        assert_eq!(read_whole(&mut log, 1), tlog);
    }

    #[test]
    fn an_interrupted_erase_leaves_a_shorter_log_and_a_full_one_a_blank_region() {
        let mut flash = MemFlash::new();
        // 40 pages, into the third 16-page block of the region, and something stale in the last.
        let (chunks, _) = boot(1, 20_000);
        assert!(chunks.len() >= 40);
        flash.page_mut(60).fill(0x5a);
        {
            let mut buffers = new_buffers();
            let mut log = open(&mut flash, &mut buffers);
            append_all(&mut log, &chunks[..40]);
            log.begin_erase();
            assert!(log.logs().is_empty());
            assert_eq!(
                block_on(log.append(&chunks[40])),
                Err(PageLogError::Erasing)
            );
            // The stale block, then the one the log ends in.
            assert!(block_on(log.erase_step()).unwrap());
            assert!(block_on(log.erase_step()).unwrap());
        }

        let mut buffers = new_buffers();
        let mut log = open(&mut flash, &mut buffers);
        assert_eq!(log.used(), 32 * PAGE_SIZE);
        log.begin_erase();
        while block_on(log.erase_step()).unwrap() {}
        assert_eq!(log.used(), 0);
        block_on(log.append(&chunks[0])).unwrap();
        assert_eq!(log.used(), PAGE_SIZE);
        log.begin_erase();
        while block_on(log.erase_step()).unwrap() {}

        let mut buffers = new_buffers();
        let log = open(&mut flash, &mut buffers);
        assert!(log.logs().is_empty());
        assert_eq!(log.next_boot(), 1);
        assert!(
            flash.data[REGION.start as usize..]
                .iter()
                .all(|&b| b == 0xff)
        );
    }

    #[test]
    fn a_full_region_refuses_more() {
        let mut flash = MemFlash::new();
        let mut buffers = new_buffers();
        let mut log = open(&mut flash, &mut buffers);
        let (chunks, _) = boot(1, 100);
        let pages = (REGION.end - REGION.start) / PAGE_SIZE;

        for _ in 0..pages {
            block_on(log.append(&chunks[0])).unwrap();
        }
        assert_eq!(log.used(), log.total());
        assert_eq!(block_on(log.append(&chunks[0])), Err(PageLogError::Full));
    }
}
