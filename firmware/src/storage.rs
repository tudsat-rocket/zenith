//! On-board NOR flash storage.
//!
//! The flash is split into two regions: the first 16 sectors hold the parameter store (a
//! sequential-storage map of raw u32 values keyed by the stable param ids from
//! [`mission::params`], or [`telemetry::params::GcsParams`] on a ground station), everything after
//! that the flight log: a [`PageLog`] of [`links::log`] chunks, filled until full and kept until
//! erased.
//!
//! A dedicated task owns the flash chip; parameters are read once at boot, and param writes and log
//! records are queued through statics so callers (ultimately the 1 kHz main loop) never wait on
//! flash I/O. Ground stations list and download the log while it keeps logging; only an erase
//! pauses it.

use core::ops::Range;

use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Instant;
use static_cell::ConstStaticCell;

use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage};

use defmt::*;

use links::log::{ChunkBuilder, MAX_CHUNK, PageLog, PageLogError, ReadBuffers};
use links::protocols::log::{DATA_LEN, LogInfo, LogStore, StorageUsage};
use links::{Downlink, UplinkCommand};
use mission::flight_log::{LOG_STATUS, LOG_USAGE, LogState};
use mission::health::STORAGE_ERRORS;
use mission::mavlink::VehicleSnapshot;
use mission::params::ParameterGroup;
use mission::{Params, TelemetryLink, params};
use rapid_dialect::FlightMode;
use rapid_dialect::rapid::messages::Heartbeat;

use crate::OurSpiDevice;

pub mod w25q;

pub const PARAMS_FLASH_RANGE: Range<u32> = 0..0x1_0000;

/// Big enough for the longest key + value (2 + 4 bytes) with margin.
const DATA_BUFFER_SIZE: usize = 32;

/// Raw log bytes held back for compression.
const RAW_LEN: usize = 16 * 1024;

type FlashDriver = w25q::W25Q<OurSpiDevice<'static>>;
type Builder = ChunkBuilder<RAW_LEN, MAX_CHUNK>;
type FlightLog = PageLog<'static, FlashDriver, RAW_LEN>;

static STORAGE_REQUESTS: Channel<CriticalSectionRawMutex, StorageRequest, 16> = Channel::new();
static LOG_RECORDS: Channel<CriticalSectionRawMutex, LogItem, 64> = Channel::new();
static LOG_QUERIES: Channel<CriticalSectionRawMutex, LogQuery, 1> = Channel::new();
static LOG_REPLIES: Channel<CriticalSectionRawMutex, LogReply, 1> = Channel::new();

static LOG_BUFFERS: ConstStaticCell<LogBuffers> = ConstStaticCell::new(LogBuffers::new());

/// Live parameter mirror served to ground stations by the per-interface param protocol tasks.
pub static PARAM_STORE: params::SharedParams = params::SharedParams::new();

/// A ground station's own params, served alongside the vehicle's it relays.
#[cfg(feature = "gcs")]
pub static GCS_PARAM_STORE: params::SharedParams<telemetry::params::GcsParams> =
    params::SharedParams::new();

#[derive(Debug, Clone, Copy, Format)]
pub enum StorageRequest {
    WriteParam { id: u16, raw: u32 },
    EraseParams,
    EraseLog,
}

#[allow(
    clippy::large_enum_variant,
    reason = "a channel slot has to fit a Downlink either way"
)]
enum LogItem {
    Record(u32, Downlink),
    /// Writes out whatever is buffered, short chunk or not.
    Flush,
}

#[derive(Debug, Clone, Copy, Format)]
enum LogQuery {
    Count,
    Log(u16),
    Read { id: u16, offset: u32 },
}

enum LogReply {
    Count(u16),
    Log(Option<LogInfo>),
    Data([u8; DATA_LEN], usize),
}

/// Cheap handle for queueing storage work from anywhere.
#[derive(Clone, Copy)]
pub struct FlashHandle {
    _private: (),
}

/// The main loop's end of the flight log: queues records for the storage task, never waiting.
#[derive(Default)]
pub struct FlashLog {
    time_ms: u32,
    mode: Option<FlightMode>,
}

/// The flight log as the LOG_* protocol sees it, answered by the storage task.
pub struct FlashLogReader;

pub struct Flash {
    chip: Chip,
}

enum Chip {
    /// The boot-time probe failed; every access is skipped.
    Unhealthy,
    Healthy(FlashDriver),
}

/// The `Storage` impl handed to the vehicle.
pub struct FlashStorage {
    params: Params,
    handle: FlashHandle,
}

struct LogBuffers {
    builder: Builder,
    /// A chunk on its way to flash; LZ4 wants the worst case room.
    chunk: [u8; Builder::OUT_LEN],
    read: ReadBuffers<RAW_LEN>,
}

/// The storage task's state.
struct Storage {
    log: FlightLog,
    builder: &'static mut Builder,
    chunk: &'static mut [u8; Builder::OUT_LEN],
    system_id: u8,
    /// The id this boot's chunks carry: one past the last boot in the log.
    boot: u16,
}

/// Brings the storage subsystem up: publishes the boot params to the mirror ground stations read
/// from, starts the task that owns the flash chip, and returns the vehicle's [`mission::Storage`].
/// `system_id` goes into the frames of downloaded logs.
pub fn spawn(flash: Flash, boot_params: Params, system_id: u8, spawner: &Spawner) -> FlashStorage {
    PARAM_STORE.init(boot_params.clone());
    start(flash, system_id, spawner);
    FlashStorage::new(boot_params, FlashHandle::new())
}

/// Starts the task that owns the flash chip, which [`FlashHandle`]s queue their requests for.
pub fn start(flash: Flash, system_id: u8, spawner: &Spawner) {
    if spawner.spawn(run(flash, system_id)).is_err() {
        error!("storage: failed to spawn task");
    }
}

#[embassy_executor::task]
async fn run(flash: Flash, system_id: u8) -> ! {
    let Chip::Healthy(driver) = flash.chip else {
        LOG_STATUS.set(LogState::Stopped);
        loop {
            match select3(
                STORAGE_REQUESTS.receive(),
                LOG_RECORDS.receive(),
                LOG_QUERIES.receive(),
            )
            .await
            {
                Either3::First(request) => {
                    STORAGE_ERRORS.record();
                    warn!("storage: flash unavailable, dropping {}", request);
                }
                Either3::Second(_) => {}
                Either3::Third(query) => LOG_REPLIES.send(LogReply::empty(query)).await,
            }
        }
    };

    let Some(buffers) = LOG_BUFFERS.try_take() else {
        error!("storage: log buffers already taken");
        loop {
            embassy_time::Timer::after_secs(60).await;
        }
    };

    let log_range = PARAMS_FLASH_RANGE.end..driver.size();
    let storage = Storage {
        log: PageLog::new(driver, log_range, &mut buffers.read),
        builder: &mut buffers.builder,
        chunk: &mut buffers.chunk,
        system_id,
        boot: 1,
    };
    storage.run().await
}

impl Default for FlashHandle {
    fn default() -> Self {
        Self::new()
    }
}

impl FlashHandle {
    pub const fn new() -> Self {
        Self { _private: () }
    }

    pub fn write_param(&self, id: u16, raw: u32) {
        let request = StorageRequest::WriteParam { id, raw };
        if STORAGE_REQUESTS.try_send(request).is_err() {
            STORAGE_ERRORS.record();
            warn!(
                "storage: request queue full, dropping write for param {}",
                id
            );
        }
    }

    pub fn erase_params(&self) {
        if STORAGE_REQUESTS
            .try_send(StorageRequest::EraseParams)
            .is_err()
        {
            STORAGE_ERRORS.record();
            warn!("storage: request queue full, dropping param erase");
        }
    }

    pub fn erase_log(&self) {
        if STORAGE_REQUESTS.try_send(StorageRequest::EraseLog).is_err() {
            STORAGE_ERRORS.record();
            warn!("storage: request queue full, dropping log erase");
        }
    }
}

impl FlashLog {
    /// Queues this tick's share of the log schedule. A mode change first writes out what the old
    /// mode left buffered, which may be nothing more for a long while if the new one logs little
    /// or nothing, then records the change to the tick with a heartbeat.
    pub fn log(&mut self, snapshot: &VehicleSnapshot<'_>) {
        self.time_ms = snapshot.time.0;
        if self.mode != Some(snapshot.mode) {
            self.mode = Some(snapshot.mode);
            self.flush();
            self.send_message(Downlink::from_self(Heartbeat::from(snapshot)));
        }
        snapshot.send_log(self);
    }

    /// Has the storage task write out everything queued so far, e.g. ahead of a reboot.
    pub fn flush(&mut self) {
        if LOG_RECORDS.try_send(LogItem::Flush).is_err() {
            LOG_STATUS.note_dropped();
        }
    }
}

impl TelemetryLink for FlashLog {
    fn send_message(&mut self, message: Downlink) {
        if LOG_RECORDS
            .try_send(LogItem::Record(self.time_ms, message))
            .is_err()
        {
            LOG_STATUS.note_dropped();
        }
    }

    fn try_recv_command(&mut self) -> Option<UplinkCommand> {
        None
    }
}

impl FlashLogReader {
    async fn query(query: LogQuery) -> LogReply {
        LOG_QUERIES.send(query).await;
        LOG_REPLIES.receive().await
    }
}

impl LogStore for FlashLogReader {
    async fn count(&self) -> u16 {
        match Self::query(LogQuery::Count).await {
            LogReply::Count(count) => count,
            _ => 0,
        }
    }

    async fn log(&self, index: u16) -> Option<LogInfo> {
        match Self::query(LogQuery::Log(index)).await {
            LogReply::Log(log) => log,
            _ => None,
        }
    }

    async fn read(&self, id: u16, offset: u32, out: &mut [u8; DATA_LEN]) -> usize {
        match Self::query(LogQuery::Read { id, offset }).await {
            LogReply::Data(data, len) => {
                *out = data;
                len
            }
            _ => 0,
        }
    }

    async fn usage(&self) -> Option<StorageUsage> {
        LOG_USAGE.get()
    }
}

impl LogReply {
    /// The answer to `query` when there is no usable flash.
    fn empty(query: LogQuery) -> Self {
        match query {
            LogQuery::Count => Self::Count(0),
            LogQuery::Log(_) => Self::Log(None),
            LogQuery::Read { .. } => Self::Data([0; DATA_LEN], 0),
        }
    }
}

impl Flash {
    /// Probes the flash chip and reads all stored parameters. Parameters missing from flash stay at
    /// their defaults; a corrupted param region is erased and reported.
    ///
    /// A chip that fails to probe is reported and the flash marked unhealthy: the vehicle boots on
    /// default params with no persistence and no log rather than not booting at all.
    pub async fn init<G: ParameterGroup + Default>(spi: OurSpiDevice<'static>) -> (Flash, G) {
        let mut driver = w25q::W25Q::new(spi);

        if let Err(e) = driver.probe().await {
            STORAGE_ERRORS.record();
            error!(
                "storage: flash probe failed ({}), continuing without persistence",
                e
            );
            return (
                Flash {
                    chip: Chip::Unhealthy,
                },
                G::default(),
            );
        }

        let params = read_params(&mut driver).await;
        (
            Flash {
                chip: Chip::Healthy(driver),
            },
            params,
        )
    }

    /// Whether the boot-time JEDEC probe succeeded. A false here means all storage access is being
    /// skipped.
    pub fn is_healthy(&self) -> bool {
        matches!(self.chip, Chip::Healthy(_))
    }

    /// On-target self-test: store a value under a reserved key (outside the parameter registry, so
    /// real parameters are untouched) and read it back. Exercises the full driver + map stack on
    /// real hardware.
    pub async fn selftest_roundtrip(&mut self) -> bool {
        const TEST_KEY: u16 = 0xfffe;
        const TEST_VALUE: u32 = 0xdead_beef;

        let Chip::Healthy(driver) = &mut self.chip else {
            return false;
        };
        let mut map = param_map(driver);
        let mut buffer = [0u8; DATA_BUFFER_SIZE];
        if let Err(e) = map.store_item(&mut buffer, &TEST_KEY, &TEST_VALUE).await {
            error!("storage: selftest store failed: {:?}", Debug2Format(&e));
            return false;
        }
        match map.fetch_item::<u32>(&mut buffer, &TEST_KEY).await {
            Ok(Some(value)) => value == TEST_VALUE,
            Ok(None) => false,
            Err(e) => {
                error!("storage: selftest fetch failed: {:?}", Debug2Format(&e));
                false
            }
        }
    }
}

fn param_map(
    flash: &mut FlashDriver,
) -> MapStorage<u16, &mut FlashDriver, Cache<Uncached, Uncached, Uncached, u16>> {
    MapStorage::new(
        flash,
        MapConfig::new(PARAMS_FLASH_RANGE),
        Cache::new_uncached(),
    )
}

async fn read_params<G: ParameterGroup + Default>(flash: &mut FlashDriver) -> G {
    let mut map = param_map(flash);
    let mut buffer = [0u8; DATA_BUFFER_SIZE];
    let mut params = G::default();

    for index in 0..G::PARAM_COUNT {
        let Some(descriptor) = G::descriptor(index) else {
            continue;
        };

        match map
            .fetch_item::<u32>(&mut buffer, &descriptor.id.get())
            .await
        {
            Ok(Some(raw)) => {
                params.set(descriptor.id, descriptor.ty.decode_raw(raw));
            }
            Ok(None) => {}
            Err(sequential_storage::Error::Corrupted { .. }) => {
                // fetch_item already attempted an automatic repair, so this region is beyond
                // saving. Start fresh.
                STORAGE_ERRORS.record();
                error!("storage: param region corrupted, erasing");

                if let Err(e) = map.erase_all().await {
                    STORAGE_ERRORS.record();
                    error!("storage: erase failed: {:?}", Debug2Format(&e));
                }

                return G::default();
            }
            Err(e) => {
                STORAGE_ERRORS.record();
                error!(
                    "storage: failed to fetch param {}: {:?}",
                    descriptor.id.get(),
                    Debug2Format(&e)
                );
            }
        }
    }

    params
}

impl LogBuffers {
    #[allow(
        clippy::large_stack_arrays,
        reason = "only evaluated at compile time, to initialize LOG_BUFFERS in place"
    )]
    const fn new() -> Self {
        Self {
            builder: Builder::new(),
            chunk: [0; Builder::OUT_LEN],
            read: ReadBuffers::new(),
        }
    }
}

impl Storage {
    async fn run(mut self) -> ! {
        let started = Instant::now();
        if let Err(e) = self.log.scan().await {
            STORAGE_ERRORS.record();
            error!("storage: flight log scan stopped early: {}", e);
        }
        // Zero means "no log" in LOG_ENTRY.
        self.boot = self.log.next_boot();
        info!(
            "storage: flight log holds {} logs in {} bytes, indexed in {} ms; this is boot {}",
            self.log.logs().len(),
            self.log.used(),
            started.elapsed().as_millis(),
            self.boot
        );
        LOG_STATUS.set(LogState::Logging);
        LOG_USAGE.set(self.usage());

        loop {
            match select3(
                STORAGE_REQUESTS.receive(),
                LOG_RECORDS.receive(),
                LOG_QUERIES.receive(),
            )
            .await
            {
                Either3::First(StorageRequest::EraseLog) => self.erase_log().await,
                Either3::First(request) => self.handle_params(request).await,
                Either3::Second(LogItem::Record(time_ms, downlink)) => {
                    self.record(time_ms, &downlink).await;
                }
                Either3::Second(LogItem::Flush) => self.flush().await,
                Either3::Third(query) => {
                    let reply = self.answer(query).await;
                    LOG_REPLIES.send(reply).await;
                }
            }
        }
    }

    fn usage(&self) -> StorageUsage {
        StorageUsage {
            total: self.log.total(),
            used: self.log.used(),
        }
    }

    async fn answer(&mut self, query: LogQuery) -> LogReply {
        match query {
            LogQuery::Count => LogReply::Count(self.log.logs().len() as u16),
            LogQuery::Log(index) => {
                LogReply::Log(self.log.logs().get(usize::from(index)).map(|log| LogInfo {
                    id: log.id,
                    size: log.size,
                }))
            }
            LogQuery::Read { id, offset } => {
                let mut data = [0; DATA_LEN];
                let len = match self.log.read(id, offset, self.system_id, &mut data).await {
                    Ok(len) => len,
                    Err(e) => {
                        STORAGE_ERRORS.record();
                        error!("storage: log read failed: {}", e);
                        0
                    }
                };
                LogReply::Data(data, len)
            }
        }
    }

    /// Everything but [`StorageRequest::EraseLog`], which is already underway when this is called
    /// during one.
    async fn handle_params(&mut self, request: StorageRequest) {
        let mut buffer = [0u8; DATA_BUFFER_SIZE];
        match request {
            StorageRequest::WriteParam { id, raw } => {
                info!("storage: writing param {} = {:#010x}", id, raw);
                let mut map = param_map(self.log.flash());
                if let Err(e) = map.store_item(&mut buffer, &id, &raw).await {
                    STORAGE_ERRORS.record();
                    error!(
                        "storage: failed to store param {}: {:?}",
                        id,
                        Debug2Format(&e)
                    );
                }
            }
            StorageRequest::EraseParams => {
                info!("storage: erasing param region");
                if let Err(e) = param_map(self.log.flash()).erase_all().await {
                    STORAGE_ERRORS.record();
                    error!("storage: erase failed: {:?}", Debug2Format(&e));
                }
            }
            StorageRequest::EraseLog => {}
        }
    }

    /// Lists the log as empty at once, then erases it a block at a time, answering queries and
    /// param writes in between. Records arriving meanwhile are dropped.
    async fn erase_log(&mut self) {
        info!("storage: erasing flight log");
        LOG_STATUS.set(LogState::Paused);
        self.builder.clear();
        self.log.begin_erase();

        let state = loop {
            match self.log.erase_step().await {
                Ok(true) => {}
                Ok(false) => break LogState::Logging,
                Err(e) => {
                    STORAGE_ERRORS.record();
                    error!("storage: log erase failed: {}", e);
                    break LogState::Stopped;
                }
            }
            LOG_USAGE.set(self.usage());

            while let Ok(query) = LOG_QUERIES.try_receive() {
                let reply = self.answer(query).await;
                LOG_REPLIES.send(reply).await;
            }
            while let Ok(request) = STORAGE_REQUESTS.try_receive() {
                self.handle_params(request).await;
            }
            while LOG_RECORDS.try_receive().is_ok() {}
        };

        self.boot = self.log.next_boot();
        LOG_USAGE.set(self.usage());
        LOG_STATUS.set(state);
        info!("storage: flight log erased");
    }

    fn is_logging() -> bool {
        matches!(LOG_STATUS.get(), LogState::Logging | LogState::Degraded)
    }

    async fn record(&mut self, time_ms: u32, downlink: &Downlink) {
        if !Self::is_logging() {
            return;
        }

        if !self.builder.push(time_ms, downlink) {
            LOG_STATUS.note_dropped();
        }
        if self.builder.is_ready() {
            self.write_chunk().await;
        }
    }

    async fn flush(&mut self) {
        while Self::is_logging() && !self.builder.is_empty() {
            self.write_chunk().await;
        }
    }

    /// Cuts one chunk off the builder and appends it to the log.
    async fn write_chunk(&mut self) {
        let Some(len) = self.builder.take_chunk(self.boot, &mut self.chunk[..]) else {
            error!("storage: failed to build a log chunk");
            self.builder.clear();
            LOG_STATUS.note_dropped();
            return;
        };
        let chunk = self.chunk.get(..len).unwrap_or_default();

        match self.log.append(chunk).await {
            Ok(()) => LOG_USAGE.set(self.usage()),
            Err(PageLogError::Full) => {
                warn!("storage: flight log full, logging stopped");
                LOG_STATUS.set(LogState::Stopped);
            }
            Err(PageLogError::Erasing) => LOG_STATUS.note_dropped(),
            Err(PageLogError::InvalidChunk) => {
                error!("storage: log chunk does not fit a page");
                LOG_STATUS.note_dropped();
            }
            Err(PageLogError::Flash(e)) => {
                STORAGE_ERRORS.record();
                error!("storage: failed to write log chunk: {}", e);
                LOG_STATUS.note_dropped();
            }
        }
    }
}

impl FlashStorage {
    fn new(params: Params, handle: FlashHandle) -> Self {
        Self { params, handle }
    }
}

impl mission::Storage for FlashStorage {
    fn read_params(&mut self) -> Option<Params> {
        Some(self.params.clone())
    }

    fn write_param(&mut self, id: params::ParamId, value: params::ParamValue) {
        self.handle.write_param(id.get(), value.encode_raw());
    }

    fn erase_log(&mut self) {
        self.handle.erase_log();
    }
}
