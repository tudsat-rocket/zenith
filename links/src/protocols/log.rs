//! This task implements MAVLink log download (LOG_REQUEST_LIST, LOG_REQUEST_DATA, LOG_ERASE,
//! LOG_REQUEST_END) for the on-board flight log, one log per boot, and answers requests for
//! STORAGE_INFORMATION about the flash holding it.
//!
//! Erasing goes through the vehicle as [`UplinkCommand::EraseLog`], which knows whether it is safe.
//!
//! On the wire a log's id is its index, oldest first: ground stations file LOG_ENTRY by id into
//! `0..num_logs` (QGroundControl only shifts that by one for ArduPilot), so the store's own ids,
//! which count boots, never reach them.

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};

use rapid_dialect::Rapid;
use rapid_dialect::rapid::enums::{StorageStatus, StorageType, StorageUsageFlag};
use rapid_dialect::rapid::messages::{LogData, LogEntry, StorageInformation};

use crate::{
    DOWNLINK_N, Downlink, InterfaceCommandPublisher, InterfaceCommandSubscriber,
    InterfaceRxSubscriber, InterfaceTxPublisher, SELF_COMPONENT_ID, UplinkCommand,
};

pub const DATA_LEN: usize = 90;

/// The one storage there is.
pub const STORAGE_ID: u8 = 1;

/// One boot's log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LogInfo {
    /// The store's own id, for [`LogStore::read`]; ground stations see the index instead.
    pub id: u16,
    /// In bytes of `.tlog`.
    pub size: u32,
}

/// How much of the flash the log takes, in bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StorageUsage {
    pub total: u32,
    pub used: u32,
}

pub trait LogStore {
    /// How many logs there are.
    async fn count(&self) -> u16;

    /// The `index`-th log, oldest first.
    async fn log(&self, index: u16) -> Option<LogInfo>;

    /// Fills `out` from `offset` of log `id`, returning the bytes written: fewer than [`DATA_LEN`]
    /// only at the end of the log.
    async fn read(&self, id: u16, offset: u32, out: &mut [u8; DATA_LEN]) -> usize;

    /// `None` if the flash is unusable.
    async fn usage(&self) -> Option<StorageUsage>;
}

/// A LOG_* request addressed to us.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogRequest {
    List { start: u16, end: u16 },
    Data { id: u16, ofs: u32, count: u32 },
    Erase,
    End,
}

/// Whether a message targeting `(target_system, target_component)` is for us.
fn addressed(target_system: u8, target_component: u8, system_id: u8) -> bool {
    (target_system == 0 || target_system == system_id)
        && (target_component == 0 || target_component == SELF_COMPONENT_ID)
}

impl LogRequest {
    /// The next request on `rx` addressed to `system_id`.
    pub async fn receive(rx: &mut InterfaceRxSubscriber, system_id: u8) -> Self {
        loop {
            let frame = rx.next_message_pure().await;
            if let Some(request) = Self::from_frame(&frame, system_id) {
                return request;
            }
        }
    }

    /// A request that arrived on `rx` since it was last read, without waiting for one.
    fn try_receive(rx: &mut InterfaceRxSubscriber, system_id: u8) -> Option<Self> {
        while let Some(frame) = rx.try_next_message_pure() {
            if let Some(request) = Self::from_frame(&frame, system_id) {
                return Some(request);
            }
        }
        None
    }

    fn from_frame(frame: &mavio::Frame<mavio::prelude::V2>, system_id: u8) -> Option<Self> {
        // This is likely not a packet intended for us, see commands.rs.
        if frame.system_id() < 0x7f {
            return None;
        }
        Self::parse(&frame.decode::<Rapid>().ok()?, system_id)
    }

    fn parse(msg: &Rapid, system_id: u8) -> Option<Self> {
        match msg {
            Rapid::LogRequestList(req)
                if addressed(req.target_system, req.target_component, system_id) =>
            {
                Some(Self::List {
                    start: req.start,
                    end: req.end,
                })
            }
            Rapid::LogRequestData(req)
                if addressed(req.target_system, req.target_component, system_id) =>
            {
                Some(Self::Data {
                    id: req.id,
                    ofs: req.ofs,
                    count: req.count,
                })
            }
            Rapid::LogErase(req)
                if addressed(req.target_system, req.target_component, system_id) =>
            {
                Some(Self::Erase)
            }
            Rapid::LogRequestEnd(req)
                if addressed(req.target_system, req.target_component, system_id) =>
            {
                Some(Self::End)
            }
            _ => None,
        }
    }
}

/// Sends a LOG_ENTRY for every log with an index in `start..=end`, or the empty one if there are
/// no logs at all.
async fn list(store: &impl LogStore, tx: &InterfaceTxPublisher, start: u16, end: u16) {
    let count = store.count().await;
    let Some(last_index) = count.checked_sub(1) else {
        tx.publish(Downlink::from_self(LogEntry::default())).await;
        return;
    };

    for index in start..=end.min(last_index) {
        let Some(log) = store.log(index).await else {
            continue;
        };
        tx.publish(Downlink::from_self(LogEntry {
            id: index,
            num_logs: count,
            last_log_num: last_index,
            time_utc: 0,
            size: log.size,
        }))
        .await;
    }
}

/// Sends LOG_DATA for `ofs..ofs + count` of the log at index `id` until it ends, or returns a request that
/// arrived meanwhile, which preempts the rest. The stream only takes half the downlink channel, so
/// telemetry keeps flowing.
#[allow(
    clippy::arithmetic_side_effects,
    reason = "position stays below end, n is at most DATA_LEN"
)]
async fn stream(
    store: &impl LogStore,
    tx: &InterfaceTxPublisher,
    rx: &mut InterfaceRxSubscriber,
    system_id: u8,
    (id, ofs, count): (u16, u32, u32),
) -> Option<LogRequest> {
    let Some(log) = store.log(id).await else {
        // The empty packet that ends a log.
        tx.publish(Downlink::from_self(LogData {
            id,
            ofs,
            count: 0,
            data: [0; DATA_LEN],
        }))
        .await;
        return None;
    };
    let end = ofs.saturating_add(count);
    let mut position = ofs;

    while position < end {
        if let Some(request) = LogRequest::try_receive(rx, system_id) {
            return Some(request);
        }
        while tx.free_capacity() < DOWNLINK_N / 2 {
            Timer::after(Duration::from_millis(1)).await;
        }

        let mut data = [0; DATA_LEN];
        let n = store
            .read(log.id, position, &mut data)
            .await
            .min((end - position) as usize);
        tx.publish(Downlink::from_self(LogData {
            id,
            ofs: position,
            count: n as u8,
            data,
        }))
        .await;

        // A short packet is the last one; an empty one says there is nothing more.
        if n < DATA_LEN {
            break;
        }
        position += n as u32;
    }
    None
}

/// What this vehicle says about its flash, on request and on the telemetry schedule alike.
pub fn storage_information(time_boot_ms: u32, usage: Option<StorageUsage>) -> StorageInformation {
    const MIB: f32 = 1024.0 * 1024.0;
    const NAME: &[u8] = b"flight log";

    let mut name = [0; 32];
    if let Some(prefix) = name.get_mut(..NAME.len()) {
        prefix.copy_from_slice(NAME);
    }

    let (status, total, used) = match usage {
        Some(StorageUsage { total, used: 0 }) => (StorageStatus::Empty, total, 0),
        Some(StorageUsage { total, used }) => (StorageStatus::Ready, total, used),
        None => (StorageStatus::NotSupported, 0, 0),
    };

    StorageInformation {
        time_boot_ms,
        storage_id: STORAGE_ID,
        storage_count: 1,
        status,
        total_capacity: total as f32 / MIB,
        used_capacity: used as f32 / MIB,
        available_capacity: total.saturating_sub(used) as f32 / MIB,
        read_speed: 0.0,
        write_speed: 0.0,
        type_: StorageType::Other,
        name,
        storage_usage: StorageUsageFlag::SET | StorageUsageFlag::LOGS,
    }
}

pub async fn run<L: LogStore>(
    system_id: u8,
    tx: InterfaceTxPublisher,
    mut rx: InterfaceRxSubscriber,
    mut commands: InterfaceCommandSubscriber,
    cmd_tx: InterfaceCommandPublisher,
    store: L,
) {
    log::info!("log: task started");
    let mut next = None;
    loop {
        let request = match next.take() {
            Some(request) => request,
            None => match select(
                LogRequest::receive(&mut rx, system_id),
                commands.next_message_pure(),
            )
            .await
            {
                Either::First(request) => request,
                Either::Second(UplinkCommand::RequestStorageInformation(storage_id)) => {
                    if storage_id == 0 || storage_id == STORAGE_ID {
                        let time_boot_ms = Instant::now().as_millis() as u32;
                        let info = storage_information(time_boot_ms, store.usage().await);
                        tx.publish(Downlink::from_self(info)).await;
                    }
                    continue;
                }
                Either::Second(_) => continue,
            },
        };

        match request {
            LogRequest::List { start, end } => list(&store, &tx, start, end).await,
            LogRequest::Data { id, ofs, count } => {
                next = stream(&store, &tx, &mut rx, system_id, (id, ofs, count)).await;
            }
            LogRequest::Erase => cmd_tx.publish(UplinkCommand::EraseLog).await,
            // A stream it interrupts is all there is to end.
            LogRequest::End => {}
        }
    }
}
