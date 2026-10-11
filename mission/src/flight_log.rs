//! The platform's on-board flight log state, as reported in SYS_STATUS and STORAGE_INFORMATION.
//!
//! The SITL keeps no log and never writes here.

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use links::protocols::log::StorageUsage;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LogState {
    /// No log on this platform, or not started yet.
    Absent = 0,
    Logging = 1,
    /// Still logging, but records have been dropped since boot.
    Degraded = 2,
    /// Full, or the flash is unusable.
    Stopped = 3,
    /// Not logging while the log is being downloaded or erased.
    Paused = 4,
}

pub struct LogStatus(AtomicU8);

impl LogStatus {
    pub const fn new() -> Self {
        Self(AtomicU8::new(LogState::Absent as u8))
    }

    pub fn set(&self, state: LogState) {
        self.0.store(state as u8, Ordering::Relaxed);
    }

    /// Moves `Logging` to `Degraded`, leaving any other state alone.
    pub fn note_dropped(&self) {
        let _ = self.0.compare_exchange(
            LogState::Logging as u8,
            LogState::Degraded as u8,
            Ordering::Relaxed,
            Ordering::Relaxed,
        );
    }

    pub fn get(&self) -> LogState {
        match self.0.load(Ordering::Relaxed) {
            1 => LogState::Logging,
            2 => LogState::Degraded,
            3 => LogState::Stopped,
            4 => LogState::Paused,
            _ => LogState::Absent,
        }
    }
}

impl Default for LogStatus {
    fn default() -> Self {
        Self::new()
    }
}

/// How much of the flash the log takes. The total is only set once, so the two halves never tear in
/// a way that matters.
pub struct LogUsage {
    total: AtomicU32,
    used: AtomicU32,
}

impl LogUsage {
    pub const fn new() -> Self {
        Self {
            total: AtomicU32::new(0),
            used: AtomicU32::new(0),
        }
    }

    pub fn set(&self, usage: StorageUsage) {
        self.total.store(usage.total, Ordering::Relaxed);
        self.used.store(usage.used, Ordering::Relaxed);
    }

    /// `None` without a usable flash.
    pub fn get(&self) -> Option<StorageUsage> {
        let total = self.total.load(Ordering::Relaxed);
        (total > 0).then(|| StorageUsage {
            total,
            used: self.used.load(Ordering::Relaxed),
        })
    }
}

impl Default for LogUsage {
    fn default() -> Self {
        Self::new()
    }
}

pub static LOG_STATUS: LogStatus = LogStatus::new();
pub static LOG_USAGE: LogUsage = LogUsage::new();
