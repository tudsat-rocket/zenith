//! The platform's CPU utilization estimate.
//!
//! The SITL deliberately never writes here.

use core::sync::atomic::{AtomicU16, Ordering};

const UNKNOWN: u16 = u16::MAX;

/// The share of wall-clock time the CPU spent doing anything other than sleeping.
///
/// Per mille rather than percent because that is the unit MAVLink's `SYS_STATUS.load` wants.
pub struct CpuLoad(AtomicU16);

impl CpuLoad {
    pub const fn new() -> Self {
        Self(AtomicU16::new(UNKNOWN))
    }

    /// Publish a new estimate, clamped to the 0-1000 range `SYS_STATUS.load` is defined over.
    pub fn set_permille(&self, permille: u16) {
        self.0.store(permille.min(1000), Ordering::Relaxed);
    }

    /// The most recent estimate, or `None` on a platform that doesn't measure CPU load.
    pub fn permille(&self) -> Option<u16> {
        match self.0.load(Ordering::Relaxed) {
            UNKNOWN => None,
            permille => Some(permille),
        }
    }
}

impl Default for CpuLoad {
    fn default() -> Self {
        Self::new()
    }
}

/// The one live estimate, written by the platform's idle accounting.
pub static CPU_LOAD: CpuLoad = CpuLoad::new();

/// Whether the main loop is still meeting its 1 kHz deadline.
pub struct LoopHealth {
    overruns: AtomicU16,
    peak_latency_us: AtomicU16,
}

impl LoopHealth {
    pub const fn new() -> Self {
        Self {
            overruns: AtomicU16::new(0),
            peak_latency_us: AtomicU16::new(0),
        }
    }

    /// Cumulative count of iterations that took longer than the tick period, saturating.
    pub fn record_overrun(&self) {
        let previous = self.overruns.load(Ordering::Relaxed);
        self.overruns
            .store(previous.saturating_add(1), Ordering::Relaxed);
    }

    pub fn set_peak_latency_us(&self, us: u16) {
        self.peak_latency_us.store(us, Ordering::Relaxed);
    }

    pub fn overruns(&self) -> u16 {
        self.overruns.load(Ordering::Relaxed)
    }

    /// Worst iteration latency over the last reporting window, saturating at `u16::MAX` us.
    pub fn peak_latency_us(&self) -> u16 {
        self.peak_latency_us.load(Ordering::Relaxed)
    }
}

impl Default for LoopHealth {
    fn default() -> Self {
        Self::new()
    }
}

pub static LOOP_HEALTH: LoopHealth = LoopHealth::new();
