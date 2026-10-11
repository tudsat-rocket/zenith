//! On-board flight log codec.
//!
//! Downlink messages are stored as records of `time_ms: u32 | component_id: u8 | msg_id: u24 |
//! len: u8 | payload[len]` (little-endian, payload as truncated by MAVLink v2). Records are packed
//! into chunks of `raw_len: u16 | tlog_len: u16 | boot: u16 | LZ4 block`, each decodable on its own,
//! and served back as a `.tlog`: every record becomes `time_us: u64 BE | MAVLink v2 frame`.
//! `tlog_len` lets a reader seek through the `.tlog` by chunk headers alone, `boot` tells apart the
//! runs of chunks one boot wrote.

use core::num::NonZeroUsize;

use lz4_flex::block::{
    CompressTable, compress_into_with_table, decompress_into, get_maximum_output_size,
};
use mavio::Frame;
use mavio::protocol::{Dialect, IntoPayload, MavLinkVersion, MessageSpec, V2};
use rapid_dialect::Rapid;

use crate::Downlink;

mod pages;

pub use pages::{
    LogSpan, MAX_CHUNK, MAX_LOGS, PAGE_HEADER_LEN, PAGE_SIZE, PageLog, PageLogError, ReadBuffers,
};

pub const RECORD_HEADER_LEN: usize = 9;
pub const CHUNK_HEADER_LEN: usize = 6;

/// What a record grows by as a `.tlog` entry: timestamp, v2 header and checksum replace its header.
const TLOG_GROWTH: usize = 8 + 10 + 2 - RECORD_HEADER_LEN;

/// The largest record a [`ChunkBuilder`] can take.
pub const MAX_RECORD_LEN: usize = RECORD_HEADER_LEN + 255;

/// Chunk sizes, read without decompressing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunkHeader {
    pub raw_len: u16,
    pub tlog_len: u16,
    pub boot: u16,
}

/// One decoded record, borrowing its payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Record<'a> {
    pub time_ms: u32,
    pub component_id: u8,
    pub msg_id: u32,
    pub payload: &'a [u8],
}

/// Collects records and cuts them into chunks.
///
/// `N` bounds the raw bytes held; it must leave room for a few chunks' worth, since how much raw
/// data fits into one chunk depends on how well it compresses. The builder tracks that with a
/// target raw length, re-estimated from each chunk's compression ratio.
///
/// Chunks are at most `MAX_CHUNK` bytes. A new builder is all zeroes, so a static one costs no
/// initialized data.
pub struct ChunkBuilder<const N: usize, const MAX_CHUNK: usize> {
    raw: [u8; N],
    len: usize,
    /// `None` until the first chunk, which assumes no compression.
    target: Option<NonZeroUsize>,
    table: CompressTable,
}

impl ChunkHeader {
    pub fn parse(chunk: &[u8]) -> Option<Self> {
        let [r0, r1, t0, t1, b0, b1, ..] = *chunk else {
            return None;
        };
        Some(Self {
            raw_len: u16::from_le_bytes([r0, r1]),
            tlog_len: u16::from_le_bytes([t0, t1]),
            boot: u16::from_le_bytes([b0, b1]),
        })
    }
}

impl<'a> Record<'a> {
    /// Splits the first record off `raw`.
    fn split(raw: &'a [u8]) -> Option<(Self, &'a [u8])> {
        let (header, rest) = raw.split_first_chunk::<RECORD_HEADER_LEN>()?;
        let [t0, t1, t2, t3, component_id, m0, m1, m2, len] = *header;
        let (payload, rest) = rest.split_at_checked(usize::from(len))?;
        let record = Self {
            time_ms: u32::from_le_bytes([t0, t1, t2, t3]),
            component_id,
            msg_id: u32::from_le_bytes([m0, m1, m2, 0]),
            payload,
        };
        Some((record, rest))
    }

    /// Iterates the records of a decompressed chunk, stopping at the first truncated one.
    pub fn iter(raw: &'a [u8]) -> impl Iterator<Item = Record<'a>> {
        let mut rest = raw;
        core::iter::from_fn(move || {
            let (record, tail) = Record::split(rest)?;
            rest = tail;
            Some(record)
        })
    }

    #[allow(
        clippy::arithmetic_side_effects,
        reason = "a payload is at most 255 bytes"
    )]
    fn tlog_len(&self) -> usize {
        self.payload.len() + RECORD_HEADER_LEN + TLOG_GROWTH
    }

    /// Writes this record as a `.tlog` entry, returning its length.
    ///
    /// A message id the dialect no longer knows still produces a frame of the right length, so
    /// chunk `tlog_len`s stay valid; its checksum is computed without CRC_EXTRA and will not verify.
    fn write_tlog(&self, system_id: u8, sequence: u8, out: &mut [u8]) -> Option<usize> {
        let crc_extra = Rapid::message_info(self.msg_id).map_or(0, MessageSpec::crc_extra);
        let frame = Frame::builder()
            .version(V2)
            .sequence(sequence)
            .system_id(system_id)
            .component_id(self.component_id)
            .message_id(self.msg_id)
            .payload(self.payload)
            .crc_extra(crc_extra)
            .build();

        let (time, rest) = out.split_first_chunk_mut::<8>()?;
        *time = (u64::from(self.time_ms).saturating_mul(1000)).to_be_bytes();
        let frame_len = frame.serialize(rest).ok()?;
        frame_len.checked_add(8)
    }
}

impl<const N: usize, const MAX_CHUNK: usize> ChunkBuilder<N, MAX_CHUNK> {
    /// How long [`Self::take_chunk`]'s output buffer has to be: LZ4 wants room for the worst case.
    pub const OUT_LEN: usize = CHUNK_HEADER_LEN + get_maximum_output_size(N);

    pub const fn new() -> Self {
        Self {
            raw: [0; N],
            len: 0,
            target: None,
            table: CompressTable::small(),
        }
    }

    fn target(&self) -> usize {
        self.target.map_or(MAX_CHUNK, NonZeroUsize::get)
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Whether enough is buffered that [`Self::take_chunk`] should be called.
    pub fn is_ready(&self) -> bool {
        self.len >= self.target() || N.saturating_sub(self.len) < MAX_RECORD_LEN
    }

    /// Appends one record. Returns false, dropping the message, if it does not fit.
    #[allow(
        clippy::arithmetic_side_effects,
        clippy::indexing_slicing,
        reason = "the end index is checked against N before any write"
    )]
    pub fn push(&mut self, time_ms: u32, downlink: &Downlink) -> bool {
        let Ok(payload) = downlink.message.encode(MavLinkVersion::V2) else {
            return false;
        };
        let bytes = payload.bytes();
        let end = self.len + RECORD_HEADER_LEN + bytes.len();
        if end > N {
            return false;
        }

        let [t0, t1, t2, t3] = time_ms.to_le_bytes();
        let [m0, m1, m2, _] = payload.id().to_le_bytes();
        let header = [
            t0,
            t1,
            t2,
            t3,
            downlink.component_id,
            m0,
            m1,
            m2,
            payload.length(),
        ];
        let record = &mut self.raw[self.len..end];
        record[..RECORD_HEADER_LEN].copy_from_slice(&header);
        record[RECORD_HEADER_LEN..].copy_from_slice(bytes);
        self.len = end;
        true
    }

    /// Compresses as many whole records as fit into one chunk of at most `MAX_CHUNK` bytes, tagged
    /// with `boot`, into `out`, removes them from the builder and returns the chunk length. Returns
    /// `None` if the builder is empty or `out` is shorter than [`Self::OUT_LEN`].
    #[allow(
        clippy::arithmetic_side_effects,
        clippy::indexing_slicing,
        reason = "cut and header lengths are bounded by self.len <= N and out.len()"
    )]
    pub fn take_chunk(&mut self, boot: u16, out: &mut [u8]) -> Option<usize> {
        let (header, body) = out.split_first_chunk_mut::<CHUNK_HEADER_LEN>()?;
        let space = MAX_CHUNK - CHUNK_HEADER_LEN;

        loop {
            let (cut, tlog_len) = self.prefix(self.target())?;
            let compressed =
                compress_into_with_table(&self.raw[..cut], body, &mut self.table).ok()?;
            // Aim at 15/16 of the space, assuming the data compresses as this attempt did.
            let fill = cut * space / compressed.max(1);
            let target = (fill - fill / 16).clamp(MAX_RECORD_LEN, N - MAX_RECORD_LEN);

            if compressed <= space {
                *header = chunk_header(cut, tlog_len, boot)?;
                self.raw.copy_within(cut..self.len, 0);
                self.len -= cut;
                self.target = NonZeroUsize::new(target);
                return Some(CHUNK_HEADER_LEN + compressed);
            }
            if cut <= MAX_RECORD_LEN {
                return None;
            }
            self.target = NonZeroUsize::new(target.min(cut - cut / 8));
        }
    }

    /// The longest run of whole records from the start that is at most `limit` bytes, or the
    /// first record alone if even that is longer, with the run's `.tlog` length.
    #[allow(
        clippy::arithmetic_side_effects,
        clippy::indexing_slicing,
        reason = "offsets advance by record lengths within self.len"
    )]
    fn prefix(&self, limit: usize) -> Option<(usize, usize)> {
        let mut cut = 0;
        let mut tlog_len = 0;
        for record in Record::iter(&self.raw[..self.len]) {
            let len = RECORD_HEADER_LEN + record.payload.len();
            if cut > 0 && cut + len > limit {
                break;
            }
            cut += len;
            tlog_len += record.tlog_len();
        }
        (cut > 0).then_some((cut, tlog_len))
    }
}

impl<const N: usize, const MAX_CHUNK: usize> Default for ChunkBuilder<N, MAX_CHUNK> {
    fn default() -> Self {
        Self::new()
    }
}

fn chunk_header(raw_len: usize, tlog_len: usize, boot: u16) -> Option<[u8; CHUNK_HEADER_LEN]> {
    let [r0, r1] = u16::try_from(raw_len).ok()?.to_le_bytes();
    let [t0, t1] = u16::try_from(tlog_len).ok()?.to_le_bytes();
    let [b0, b1] = boot.to_le_bytes();
    Some([r0, r1, t0, t1, b0, b1])
}

/// Decompresses a chunk into `raw`, returning the records' bytes.
pub fn decompress_chunk<'a>(chunk: &[u8], raw: &'a mut [u8]) -> Option<&'a [u8]> {
    let header = ChunkHeader::parse(chunk)?;
    let body = chunk.get(CHUNK_HEADER_LEN..)?;
    let len = decompress_into(body, raw).ok()?;
    (len == usize::from(header.raw_len)).then(|| raw.get(..len))?
}

/// Writes the bytes at `offset..` of a decompressed chunk's `.tlog` form into `out`, returning how
/// many were written (fewer than `out.len()` only at the chunk's end). Sequence numbers restart at
/// zero per chunk, so the bytes at a `.tlog` offset do not depend on where a reader started.
#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    reason = "positions are bounded by entry lengths and out.len()"
)]
pub fn read_tlog(raw: &[u8], system_id: u8, offset: usize, out: &mut [u8]) -> Option<usize> {
    let mut entry = [0; 8 + 12 + 255];
    let mut position = 0;
    let mut written = 0;

    for (sequence, record) in Record::iter(raw).enumerate() {
        if written == out.len() {
            break;
        }
        let len = record.tlog_len();
        if position + len > offset {
            record.write_tlog(system_id, sequence as u8, &mut entry)?;
            let from = offset.saturating_sub(position);
            let n = (len - from).min(out.len() - written);
            out[written..written + n].copy_from_slice(&entry[from..from + n]);
            written += n;
        }
        position += len;
    }

    Some(written)
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
    use rapid_dialect::rapid::messages::{Attitude, Heartbeat};
    use std::vec;
    use std::vec::Vec;

    fn attitude(time_boot_ms: u32) -> Downlink {
        Downlink::from_self(Attitude {
            time_boot_ms,
            roll: 0.1,
            pitch: -0.2,
            yaw: time_boot_ms as f32 * 1e-3,
            ..Default::default()
        })
    }

    #[test]
    fn records_round_trip_through_chunks_and_tlog() {
        let mut builder = ChunkBuilder::<16384, 4086>::new();
        let mut messages = Vec::new();
        for t in 0..2000u32 {
            let message = if t % 10 == 0 {
                Downlink::new(3, Heartbeat::default())
            } else {
                attitude(t)
            };
            assert!(builder.push(t, &message));
            messages.push((t, message));
            if builder.is_ready() {
                break;
            }
        }

        let mut chunk = vec![0; ChunkBuilder::<16384, 4086>::OUT_LEN];
        let chunk_len = builder.take_chunk(7, &mut chunk).unwrap();
        assert!(chunk_len <= 4086);
        let header = ChunkHeader::parse(&chunk).unwrap();
        assert_eq!(header.boot, 7);

        let mut raw = vec![0; 16384];
        let raw = decompress_chunk(&chunk[..chunk_len], &mut raw).unwrap();
        assert_eq!(raw.len(), usize::from(header.raw_len));

        let mut tlog = vec![0; 65536];
        let tlog_len = read_tlog(raw, 4, 0, &mut tlog).unwrap();
        assert_eq!(tlog_len, usize::from(header.tlog_len));

        // Any window reads the same bytes as the whole.
        for (offset, len) in [(0, 1), (5, 90), (17, 300), (tlog_len - 10, 90)] {
            let mut window = [0; 300];
            let n = read_tlog(raw, 4, offset, &mut window[..len]).unwrap();
            assert_eq!(n, len.min(tlog_len - offset));
            assert_eq!(window[..n], tlog[offset..offset + n]);
        }

        let mut rest = &tlog[..tlog_len];
        let mut decoded = 0;
        while let Some((time, frame_bytes)) = rest.split_first_chunk::<8>() {
            let frame = unsafe { Frame::<V2>::deserialize(frame_bytes) }.unwrap();
            frame.validate_checksum::<Rapid>().unwrap();
            assert_eq!(frame.sequence(), decoded as u8);
            let (t, message) = &messages[decoded];
            assert_eq!(u64::from_be_bytes(*time), u64::from(*t) * 1000);
            assert_eq!(frame.component_id(), message.component_id);
            assert_eq!(frame.decode::<Rapid>().unwrap(), message.message);
            rest = &frame_bytes[frame.size()..];
            decoded += 1;
        }
        assert!(decoded > 1);
        assert_eq!(builder.is_empty(), decoded == messages.len());
    }

    #[test]
    fn chunks_never_exceed_the_limit_and_keep_every_record() {
        let mut builder = ChunkBuilder::<16384, 1000>::new();
        let mut chunk = vec![0; ChunkBuilder::<16384, 4086>::OUT_LEN];
        let mut raw = vec![0; 16384];
        let mut next = 0u32;
        let mut expected = 0u32;

        for t in 0..20_000u32 {
            assert!(builder.push(t, &attitude(t.wrapping_mul(2_654_435_761))));
            while builder.is_ready() || (t == 19_999 && !builder.is_empty()) {
                let len = builder.take_chunk(7, &mut chunk).unwrap();
                assert!(len <= 1000);
                let raw = decompress_chunk(&chunk[..len], &mut raw).unwrap();
                for record in Record::iter(raw) {
                    assert_eq!(record.time_ms, expected);
                    expected += 1;
                }
                next += 1;
            }
        }
        assert_eq!(expected, 20_000);
        assert!(next > 1);
    }
}
