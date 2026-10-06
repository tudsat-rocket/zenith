//! Fault counters for the flight computer itself, reported in its SYS_STATUS.
//!
//! These are written from the platform's background tasks (CAN, sensors, storage), which do not
//! flow through `Vehicle`. The SITL never writes them.

use core::sync::atomic::{AtomicU16, Ordering};

/// A cumulative count of faults. Wraps rather than saturating: the ground station alerts on a
/// counter going up, which a saturated counter would stop doing.
pub struct ErrorCounter(AtomicU16);

impl ErrorCounter {
    pub const fn new() -> Self {
        Self(AtomicU16::new(0))
    }

    pub fn record(&self) {
        self.record_n(1);
    }

    pub fn record_n(&self, n: u16) {
        self.0.fetch_add(n, Ordering::Relaxed);
    }

    pub fn get(&self) -> u16 {
        self.0.load(Ordering::Relaxed)
    }
}

impl Default for ErrorCounter {
    fn default() -> Self {
        Self::new()
    }
}

/// CAN controller errors, and frames lost to full queues in either direction.
pub static CAN_ERRORS: ErrorCounter = ErrorCounter::new();
/// On-board sensor reads that failed and left their reading unknown. The high-g accelerometer is
/// left out: no board in use populates it, so it would only ever count.
pub static SENSOR_ERRORS: ErrorCounter = ErrorCounter::new();
/// Flash writes and erases that failed, or were dropped before reaching the flash.
pub static STORAGE_ERRORS: ErrorCounter = ErrorCounter::new();
