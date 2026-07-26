//! On-board NOR flash storage.
//!
//! The flash is split into two regions: the first 16 sectors hold the parameter store (a
//! sequential-storage map of raw u32 values keyed by the stable param ids from
//! [`mission::params`]), everything after that is reserved for a future flight data log.
//!
//! A dedicated task owns the flash chip; parameters are read once at boot and writes are queued
//! through the cheap-to-clone [`FlashHandle`] so callers (ultimately the 1 kHz main loop) never
//! wait on flash I/O.

use core::ops::Range;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage};

use defmt::*;

use mission::params::ParameterGroup;
use mission::{Params, params};

use crate::OurSpiDevice;

pub mod w25q;

pub const PARAMS_FLASH_RANGE: Range<u32> = 0..0x1_0000;

/// Big enough for the longest key + value (2 + 4 bytes) with margin.
const DATA_BUFFER_SIZE: usize = 32;

type FlashDriver = w25q::W25Q<OurSpiDevice<'static>>;
type ParamCache = Cache<Uncached, Uncached, Uncached, u16>;
type ParamMap = MapStorage<u16, FlashDriver, ParamCache>;

static STORAGE_REQUESTS: Channel<CriticalSectionRawMutex, StorageRequest, 16> = Channel::new();

/// Live parameter mirror served to ground stations by the per-interface param protocol tasks.
pub static PARAM_STORE: params::SharedParams = params::SharedParams::new();

#[derive(Debug, Clone, Copy, Format)]
pub enum StorageRequest {
    WriteParam { id: u16, raw: u32 },
    EraseParams,
}

/// Cheap handle for queueing storage work from anywhere.
#[derive(Clone, Copy)]
pub struct FlashHandle {
    _private: (),
}

pub struct Flash {
    map: ParamMap,
    healthy: bool,
    size: u32,
}

/// The `Storage` impl handed to the vehicle.
pub struct FlashStorage {
    params: Params,
    handle: FlashHandle,
}

/// Brings the storage subsystem up: publishes the boot params to the mirror ground stations read
/// from, starts the task that owns the flash chip, and returns the vehicle's [`mission::Storage`].
pub fn spawn(flash: Flash, boot_params: Params, spawner: &Spawner) -> FlashStorage {
    PARAM_STORE.init(boot_params.clone());

    if spawner.spawn(run(flash)).is_err() {
        error!("storage: failed to spawn task");
    }

    FlashStorage::new(boot_params, FlashHandle::new())
}

#[embassy_executor::task]
async fn run(mut flash: Flash) -> ! {
    let mut buffer = [0u8; DATA_BUFFER_SIZE];

    loop {
        let request = STORAGE_REQUESTS.receive().await;

        if !flash.healthy {
            warn!("storage: flash unavailable, dropping {}", request);
            continue;
        }

        match request {
            StorageRequest::WriteParam { id, raw } => {
                info!("storage: writing param {} = {:#010x}", id, raw);
                if let Err(e) = flash.map.store_item(&mut buffer, &id, &raw).await {
                    error!(
                        "storage: failed to store param {}: {:?}",
                        id,
                        Debug2Format(&e)
                    );
                }
            }
            StorageRequest::EraseParams => {
                info!("storage: erasing param region");
                if let Err(e) = flash.map.erase_all().await {
                    error!("storage: erase failed: {:?}", Debug2Format(&e));
                }
            }
        }
    }
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
            warn!("storage: request queue full, dropping param erase");
        }
    }
}

impl Flash {
    /// Probes the flash chip and reads all stored parameters. Parameters missing from flash stay at
    /// their defaults; a corrupted param region is erased and reported.
    ///
    /// A chip that fails to probe is reported and the flash marked unhealthy: the vehicle boots on
    /// default params with no persistence rather than not booting at all. Unhealthy means every
    /// later access is skipped.
    pub async fn init(spi: OurSpiDevice<'static>) -> (Flash, Params) {
        let mut driver = w25q::W25Q::new(spi);

        let healthy = match driver.probe().await {
            Ok(()) => true,
            Err(e) => {
                error!(
                    "storage: flash probe failed ({}), continuing without persistence",
                    e
                );
                false
            }
        };

        let size = driver.size();
        let map = MapStorage::new(
            driver,
            MapConfig::new(PARAMS_FLASH_RANGE),
            Cache::new_uncached(),
        );

        let mut flash = Flash { map, healthy, size };
        let params = if healthy {
            flash.read_params().await
        } else {
            Params::default()
        };

        (flash, params)
    }

    /// Whether the boot-time JEDEC probe succeeded. A false here means all storage access is being
    /// skipped.
    pub fn is_healthy(&self) -> bool {
        self.healthy
    }

    pub fn log_range(&self) -> Range<u32> {
        PARAMS_FLASH_RANGE.end..self.size.max(PARAMS_FLASH_RANGE.end)
    }

    /// On-target self-test: store a value under a reserved key (outside the parameter registry, so
    /// real parameters are untouched) and read it back. Exercises the full driver + map stack on
    /// real hardware.
    pub async fn selftest_roundtrip(&mut self) -> bool {
        const TEST_KEY: u16 = 0xfffe;
        const TEST_VALUE: u32 = 0xdead_beef;

        let mut buffer = [0u8; DATA_BUFFER_SIZE];
        if let Err(e) = self
            .map
            .store_item(&mut buffer, &TEST_KEY, &TEST_VALUE)
            .await
        {
            error!("storage: selftest store failed: {:?}", Debug2Format(&e));
            return false;
        }
        match self.map.fetch_item::<u32>(&mut buffer, &TEST_KEY).await {
            Ok(Some(value)) => value == TEST_VALUE,
            Ok(None) => false,
            Err(e) => {
                error!("storage: selftest fetch failed: {:?}", Debug2Format(&e));
                false
            }
        }
    }

    async fn read_params(&mut self) -> Params {
        let mut buffer = [0u8; DATA_BUFFER_SIZE];
        let mut params = Params::default();

        for index in 0..Params::PARAM_COUNT {
            let Some(descriptor) = Params::descriptor(index) else {
                continue;
            };

            match self
                .map
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
                    error!("storage: param region corrupted, erasing");

                    if let Err(e) = self.map.erase_all().await {
                        error!("storage: erase failed: {:?}", Debug2Format(&e));
                    }

                    return Params::default();
                }
                Err(e) => {
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
}
