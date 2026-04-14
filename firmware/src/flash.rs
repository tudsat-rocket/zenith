/// Uncomplete driver for the Winbond W25Q128JV Spi Flash.
/// Data sheet: https://www.pjrc.com/teensy/W25Q128FV.pdf
use bitflags::{Flags, bitflags};
use core::fmt::DebugTuple;
use core::mem::swap;
use core::{fmt::Debug, future::Future};
use defmt::{info, warn};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::spi::{Error as SpiErrorTrait, ErrorKind, ErrorType};
use embedded_hal_async::spi::{Operation, SpiDevice};
use embedded_storage_async::nor_flash::{
    self, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

const _4KB: u32 = 4 * 1024;
const _32KB: u32 = 32 * 1024;
const _64KB: u32 = 64 * 1024;

pub struct W25Q128<SPI> {
    spi: SPI,
}

#[derive(Debug)]
pub enum FlashError<E> {
    Spi(E),
    Init,
    Busy,
    OutOfBounds,
    NotAligned,
    MultipleCommandsRequired,
    Timeout,
    Other,
}

impl<E: core::fmt::Debug> NorFlashError for FlashError<E> {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::Spi(_) => NorFlashErrorKind::Other,
            Self::Init => NorFlashErrorKind::Other,
            Self::Busy => NorFlashErrorKind::Other,
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::NotAligned => NorFlashErrorKind::NotAligned,
            Self::MultipleCommandsRequired => NorFlashErrorKind::Other,
            Self::Timeout => NorFlashErrorKind::Other,
            Self::Other => NorFlashErrorKind::Other,
        }
    }
}

impl<SPI: SpiDevice> nor_flash::ErrorType for W25Q128<SPI> {
    type Error = FlashError<SPI::Error>;
}

impl<SPI: SpiDevice> ReadNorFlash for W25Q128<SPI> {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.read(offset, bytes).await
    }

    fn capacity(&self) -> usize {
        self.capacity()
    }
}

impl<SPI: SpiDevice> NorFlash for W25Q128<SPI> {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = 4096;
    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.erase_flexible(from, to).await
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.flexible_write(offset, bytes).await
    }
}

impl<SPI: SpiDevice> W25Q128<SPI> {
    pub fn capacity(&self) -> usize {
        (128 / 8) * 1024 * 1024
    }

    pub async fn new(mut spi: SPI) -> Result<Self, FlashError<SPI::Error>> {
        defmt::info!("started flash new");
        // TODO: check
        // Do we need to check if busy?
        let mut ids: [u8; 3] = [0; 3];
        let res = spi
            .transaction(&mut [
                Operation::Write(&[OpCode::JedecId.into()]),
                Operation::Read(&mut ids),
            ])
            .await
            .map_err(FlashError::Spi);
        match res {
            Ok(_) => defmt::info!("flash ok"),
            Err(ref e) => {
                match e {
                    FlashError::Spi(e) => match e.kind() {
                        ErrorKind::Other => defmt::info!("spi error other"),
                        ErrorKind::Overrun => defmt::info!("spi error overrun"),
                        ErrorKind::ModeFault => defmt::info!("spi error mode fault"),
                        ErrorKind::FrameFormat => defmt::info!("spi error FrameFormat"),
                        ErrorKind::ChipSelectFault => defmt::info!("spi error ChipSelectFault"),
                        _ => defmt::info!("spi error non_exhaustive"),
                    },
                    _ => defmt::info!("non spi error"),
                };
            }
        }
        res?;
        info!("jedec: {}", &ids);
        let (man_id, mem_id, capacity) = (ids[0], ids[1], ids[2]);
        info!(
            "man_id: 0x{:x}, mem_id: 0x{:x}, capacity: 0x{:x}",
            man_id, mem_id, capacity
        );
        let (man_id, dev_id) = (ids[0], u16::from_le_bytes([ids[2], ids[1]]));
        let (name, size_mbit) = match (man_id, dev_id) {
            (0xef, 0x4019) => ("W25Q256JV-IQ", 256),
            (0xef, 0x7019) => ("W25Q256JV-IM", 256),
            (0x17, 0x7018) => ("W25Q128JV", 128),
            (0xef, 0x4018) => ("OurFlash", 128),
            _ => ("unknown", 0),
        };

        if size_mbit != 128 {
            defmt::error!(
                "Failed to initialize flash (0x{:02x}, 0x{:04x}).",
                man_id,
                dev_id
            );
            return Err(FlashError::Init);
        }
        defmt::info!("{} initialized", name);
        Ok(Self { spi })
    }

    pub async fn erase_sector(&mut self, offset: u32) -> Result<(), FlashError<SPI::Error>> {
        if offset % 4096 != 0 {
            return Err(FlashError::NotAligned);
        }
        self.erase_sector_4kb_unchecked(offset).await
    }

    // pub async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), FlashError<SPI::Error>> {
    //     self.flexible_write(offset, bytes).await
    // }

    pub async fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), FlashError<SPI::Error>> {
        // TODO: test out of bounds check
        if (offset as usize) + bytes.len() > self.capacity() {
            return Err(FlashError::OutOfBounds);
        }
        if self.is_busy().await? {
            return Err(FlashError::Busy);
        }
        self.spi
            .transaction(&mut [
                Operation::Write(&make_op_3b_slice(OpCode::ReadData, offset)),
                Operation::Read(bytes),
            ])
            .await
            .map_err(FlashError::Spi)?;
        Ok(())
    }

    async fn erase_flexible(&mut self, from: u32, to: u32) -> Result<(), FlashError<SPI::Error>> {
        if from > to || to as usize > self.capacity() {
            return Err(FlashError::OutOfBounds);
        }
        if from == 0 && to as usize == self.capacity() {
            self.chip_erase().await?;
            return Ok(());
        }
        if from == to {
            return Ok(());
        }
        if !aligned_4kb(from) || !aligned_4kb(to) {
            return Err(FlashError::NotAligned);
        }
        let mut from = from;
        loop {
            from = self.erase_next(from, to).await?;
            if from == to {
                break;
            }
        }

        Ok(())
    }
    // returns index of not erased section
    // Returns the offset of the
    async fn erase_next(&mut self, offset: u32, max: u32) -> Result<u32, FlashError<SPI::Error>> {
        // TODO: add debug panics to unreachable
        let remaining: u32 = max - offset;

        let max_block_size: u32 = match remaining {
            0.._4KB => 0, // unreachable
            _4KB.._32KB => _4KB,
            _32KB.._64KB => _32KB,
            _ => _64KB,
        };

        let max_alignment = if aligned_64kb(offset) {
            _64KB
        } else if aligned_32kb(offset) {
            _32KB
        } else if aligned_4kb(offset) {
            _4KB
        } else {
            // unreachable if index is 4KB aligned
            unreachable!();
            0
        };

        let erase_size = max_block_size.min(max_alignment);

        match erase_size {
            _4KB => self.erase_sector_4kb_unchecked(offset).await?,
            _32KB => self.erase_block_32kb_unchecked(offset).await?,
            _64KB => self.erase_block_64kb_unchecked(offset).await?,
            0 => (),
            // unreachable
            _ => unreachable!(),
        }
        Ok(offset + erase_size)
    }

    // --- private methods ---

    /// Writes bytes at the given offset.
    /// If the input is longer than 256 bytes, multiple flash program operations will be executed.
    ///
    /// After the bytes are transfered, the flash needs an additional processing time:
    /// First byte: app. 30 µs
    /// Every additional byte: app. 2.5 µs
    async fn flexible_write(
        &mut self,
        offset: u32,
        bytes: &[u8],
    ) -> Result<(), FlashError<SPI::Error>> {
        const PAGE_SIZE: usize = 256;
        let offset: usize = offset as usize;
        if bytes.is_empty() {
            return Ok(());
        }
        let global_upper = offset as usize + bytes.len();
        if global_upper > self.capacity() {
            return Err(FlashError::OutOfBounds);
        }
        let mut lower_index = offset as usize;
        while lower_index < global_upper {
            let upper = {
                let upper_page = ((lower_index / PAGE_SIZE) * PAGE_SIZE + PAGE_SIZE) as usize;
                upper_page.min(global_upper)
            };
            // do in page write
            let partial_data = &bytes
                [((lower_index - offset as usize) as usize)..((upper - offset as usize) as usize)];
            debug_assert!(partial_data.len() < PAGE_SIZE);
            debug_assert_eq!(
                lower_index / PAGE_SIZE,
                (lower_index + partial_data.len() - 1) / PAGE_SIZE
            );
            self.partial_page_write_unchecked(lower_index as u32, partial_data)
                .await?;

            lower_index = upper;
        }
        Ok(())
    }

    // Wrappers for Flash Instructions:

    /// Returns true if the flash chip is busy.
    pub async fn is_busy(&mut self) -> Result<bool, FlashError<SPI::Error>> {
        let is_busy = self
            .read_status_reg1
            .await?
            .contains(StatusReg1::ERASE_WRITE_IN_PROGRESS);
        if is_busy {
            info!("flash is busy");
        }

        Ok(is_busy)
    }

    async fn assure_finished(
        &mut self,
        min_duration: Duration,
        max_duration: Duration,
    ) -> Result<(), FlashError<SPI::Error>> {
        let retry_dur: Duration = min_duration / 10;

        // TODO: check for overflows
        let start_t = Instant::now();
        Timer::after_millis(min_duration).await;
        while start_t.elapsed().as_millis() < max_duration {
            if !self.is_busy() {
                return Ok(());
            }
            Timer::after(retry_dur).await;
        }
        return Err(FlashError::Timeout);
    }

    /// Reads the ERASE_WRITE_IN_PROGRESS status register.
    async fn read_status_reg1(&mut self) -> Result<StatusReg1, FlashError<SPI::Error>> {
        // TODO: test
        let mut reg1: &mut [u8] = &mut [0; 1];
        self.spi
            .transaction(&mut [
                Operation::Write(&[OpCode::ReadStatusRegister1.into()]),
                Operation::Read(reg1),
            ])
            .await
            .map_err(FlashError::Spi)?;
        Ok(StatusReg1::from_bits_truncate(reg1[0]));
    }

    async fn partial_page_write_unchecked(
        &mut self,
        offset: u32,
        bytes: &[u8],
    ) -> Result<(), FlashError<SPI::Error>> {
        info!("partial_page_write: offs: {}, bytes: {}", offset, bytes);
        if self.is_busy().await? {
            return Err(FlashError::Busy);
        };
        // TODO: debug asserts
        self.spi
            .write(&[OpCode::WriteEnable.into()])
            .await
            .map_err(FlashError::Spi)?;
        Timer::after(Duration::from_micros(10)).await;
        let slice = make_op_3b_slice(OpCode::PageProgram, offset);
        info!("slice: {:02X}", slice);
        Timer::after(Duration::from_micros(10)).await;
        self.spi
            .transaction(&mut [
                Operation::Write(&make_op_3b_slice(OpCode::PageProgram, offset)),
                Operation::Write(bytes),
            ])
            .await
            .map_err(FlashError::Spi)
    }

    async fn enable_write(&mut self) -> FlashError<SPI::Error> {
        self.spi
            .write(&[OpCode::WriteEnable.into()])
            .await
            .map_err(FlashError::Spi)?;
        if !self
            .read_status_reg1()
            .await?
            .contains(StatusReg1::WRITE_ENABLE_LATCH)
        {
            return Err(FlashError::Other);
        }
        Ok(())
    }

    /// Performs a sector erase 4KB without checking for address correctness.
    async fn erase_sector_4kb_unchecked(
        &mut self,
        address: u32,
    ) -> Result<(), FlashError<SPI::Error>> {
        if self.is_busy().await? {
            return Err(FlashError::Busy);
        };
        debug_assert_eq!(address % _4KB, 0);
        self.enable_write().await?;
        self.spi
            .write(&make_op_3b_slice(OpCode::SectorErase4KB, address))
            .await
            .map_err(FlashError::Spi)?;

        self.assure_finished(SECTOR_ERASE_4KB_TIME_TYP_MS, SECTOR_ERASE_4KB_TIME_MAX_MS)
            .await
    }

    /// Performs a block erase 32KB without checking for address correctness.
    async fn erase_block_32kb_unchecked(
        &mut self,
        address: u32,
    ) -> Result<(), FlashError<SPI::Error>> {
        if self.is_busy().await? {
            return Err(FlashError::Busy);
        };
        debug_assert_eq!(address % _32KB, 0);
        self.enable_write().await?;
        self.spi
            .write(&make_op_3b_slice(OpCode::BlockErase32KB, address))
            .await
            .map_err(FlashError::Spi)?;

        self.assure_finished(BLOCK_ERASE_32KB_TIME_TYP_MS, BLOCK_ERASE_32KB_TIME_MAX_MS)
            .await
    }
    /// Performs a block erase 64KB without checking for address correctness.
    async fn erase_block_64kb_unchecked(
        &mut self,
        address: u32,
    ) -> Result<(), FlashError<SPI::Error>> {
        if self.is_busy().await? {
            return Err(FlashError::Busy);
        };
        debug_assert_eq!(address % _64KB, 0);
        self.enable_write().await?;
        self.spi
            .write(&make_op_3b_slice(OpCode::BlockErase64KB, address))
            .await
            .map_err(FlashError::Spi)?;

        self.assure_finished(BLOCK_ERASE_64KB_TIME_TYP_MS, BLOCK_ERASE_64KB_TIME_MAX_MS)
            .await
    }
    /// Performs a chip erase.
    async fn chip_erase(&mut self) -> Result<(), FlashError<SPI::Error>> {
        if self.is_busy().await? {
            return Err(FlashError::Busy);
        };
        self.enable_write().await?;
        self.spi
            .write(&[OpCode::ChipErase.into()])
            .await
            .map_err(FlashError::Spi)?;
        self.assure_finished(CHIP_ERASE_TIME_TYP_MS, CHIP_ERASE_TIME_MAX_MS)
            .await
    }
}

/// Helper function that produces a 4 byte pattern containing a 1 byte opcode and a 3 byte
/// address in big endian byte order.
/// | opcode | A23-A16 | A15-A8 | A7-A0 |
fn make_op_3b_slice(opcode: OpCode, address: u32) -> [u8; 4] {
    let byte23_16 = ((address >> 16) & 0xFF) as u8;
    let byte15_8 = ((address >> 8) & 0xFF) as u8;
    let byte7_0 = (address & 0xFF) as u8;
    [opcode.into(), byte23_16, byte15_8, byte7_0]
}

fn aligned_4kb(value: u32) -> bool {
    value % _4KB == 0
}
fn aligned_32kb(value: u32) -> bool {
    value % _32KB == 0
}
fn aligned_64kb(value: u32) -> bool {
    value % _64KB == 0
}

// Timings for W25Q128JV

const SECTOR_ERASE_4KB_TIME_TYP_MS: Duration = Duration::from_millis(45);
const SECTOR_ERASE_4KB_TIME_MAX_MS: Duration = Duration::from_millis(400);

const BLOCK_ERASE_32KB_TIME_TYP_MS: Duration = Duration::from_millis(120);
const BLOCK_ERASE_32KB_TIME_MAX_MS: Duration = Duration::from_millis(1_600);

const BLOCK_ERASE_64KB_TIME_TYP_MS: Duration = Duration::from_millis(150);
const BLOCK_ERASE_64KB_TIME_MAX_MS: Duration = Duration::from_millis(2_000);

const CHIP_ERASE_TIME_TYP_MS: Duration = Duration::from_secs(40);
const CHIP_ERASE_TIME_MAX_MS: Duration = Duration::from_secs(200);

bitflags! {
    struct StatusReg1: u8 {
        const STATUS_REGISTER_PROTECT = 1 << 7;
        const SECTOR_PROTECT = 1 << 6;
        const TOP_BOTTOM_PROTECT = 1 << 5;
        const BLOCK_PROTECT2 = 1 << 4;
        const BLOCK_PROTECT1 = 1 << 3;
        const BLOCK_PROTECT0 = 1 << 2;
        const WRITE_ENABLE_LATCH = 1 << 1;
        const ERASE_WRITE_IN_PROGRESS = 1 << 0;
    }
}

#[repr(u8)]
enum OpCode {
    WriteEnable = 0x06,
    VolatileSrWriteEnable = 0x50,
    WriteDisable = 0x04,

    ReleasePowerDown = 0xab,
    ManufacturerDeviceId = 0x90,
    JedecId = 0x9f,
    ReadUniqueId = 0x4b,

    ReadData = 0x03,
    FastRead = 0x0b,

    PageProgram = 0x02,

    SectorErase4KB = 0x20,
    BlockErase32KB = 0x52,
    BlockErase64KB = 0xd8,
    ChipErase = 0xc7,

    // Reread data sheet; some variations might be missing.
    ReadStatusRegister1 = 0x05,
    ReadStatusRegister2 = 0x35,
    ReadStatusRegister3 = 0x15,

    ReadSfdpRegister = 0x5a,
    EraseSecurityRegister = 0x44,
    ProgramSecurityRegister = 0x42,
    ReadSecurityRegister = 0x48,

    GlobalBlockLock = 0x7e,
    GlobalBlockUnlock = 0x98,
    ReadBlockLock = 0x3d,
    IndividualBlockLock = 0x36,
    IndividualBlockUnlock = 0x39,

    EraseProgramSuspend = 0x75,
    EraseProgramResume = 0x7a,
    PowerDown = 0xb9,

    EnterQpiMode = 0x38,
    EnableReset = 0x66,
    ResetDevice = 0x99,
}

impl From<OpCode> for u8 {
    fn from(opcode: OpCode) -> u8 {
        opcode as u8
    }
}
