//! CPU utilization by idle-time accounting.
//!
//! Thread mode is the only context that ever sleeps: the two interrupt-mode executors return from
//! their handlers and every ISR runs to completion, so all the slack in the system ends up in the
//! `wfe` of the thread executor's poll loop. Timing that `wfe` and subtracting from wall-clock time
//! therefore yields whole-system utilization, ISRs and both interrupt executors included, without
//! instrumenting any of them.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_time::{Duration, Instant};

use mission::cpu::{CPU_LOAD, LOOP_HEALTH};

/// `SCB_SCR.SEVONPEND`.
const SCR_SEVONPEND: u32 = 1 << 4;

/// One measurement window.
const WINDOW: Duration = Duration::from_millis(100);

/// Windows per published value.
const WINDOWS_PER_REPORT: u32 = 5;

/// The main loop's period. An iteration taking longer than this has missed its deadline.
const TICK_PERIOD: Duration = Duration::from_micros(1000);

/// Sleep accumulated by [`sleep`] since the last window rollover, in embassy ticks.
static SLEEP_TICKS: AtomicU32 = AtomicU32::new(0);

/// Set SEVONPEND, without which [`sleep`]'s `wfe` would hang.
///
/// `wfe` only treats an interrupt as a wake-up event if that interrupt could actually preempt, and
/// [`sleep`] runs with PRIMASK set so nothing can. SEVONPEND widens the condition to *any* interrupt
/// entering the pending state, masked or not, which is exactly the case we sleep in. It also leaves
/// the event register semantics intact, so the pender's `sev` still closes the race where a task is
/// woken between the last poll and the `wfe`.
///
/// Must run before the first [`sleep`].
pub fn init() {
    // SAFETY: single core, called once at boot; no other context touches the SCB.
    unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        scb.scr.modify(|scr| scr | SCR_SEVONPEND);
    }
}

/// Sleep until the executor has work again, charging the time to the idle account.
pub fn sleep() {
    cortex_m::interrupt::free(|_| {
        let start = Instant::now();
        cortex_m::asm::wfe();
        let slept = u32::try_from(start.elapsed().as_ticks()).unwrap_or(u32::MAX);
        SLEEP_TICKS.fetch_add(slept, Ordering::Relaxed);
    });
}

/// Rolls the measurement window and publishes to [`CPU_LOAD`]. Owned by the main loop.
pub struct CpuMonitor {
    window_start: Instant,
    windows: u32,
    peak_permille: u16,
    peak_latency_us: u16,
}

impl CpuMonitor {
    pub fn new() -> Self {
        Self {
            window_start: Instant::now(),
            windows: 0,
            peak_permille: 0,
            peak_latency_us: 0,
        }
    }

    /// Call once per main loop iteration, passing how long this iteration's work took.
    pub fn update(&mut self, latency: Duration) {
        if latency > TICK_PERIOD {
            LOOP_HEALTH.record_overrun();
        }

        let latency_us = u16::try_from(latency.as_micros()).unwrap_or(u16::MAX);
        self.peak_latency_us = self.peak_latency_us.max(latency_us);

        let elapsed = self.window_start.elapsed();
        if elapsed < WINDOW {
            return;
        }

        let slept = u64::from(SLEEP_TICKS.swap(0, Ordering::Relaxed));

        // Advance by `elapsed` rather than to `now()` so windows tile exactly with no lost sliver.
        #[allow(
            clippy::arithmetic_side_effects,
            reason = "Instant + Duration, both derived from this same monotonic clock"
        )]
        {
            self.window_start += elapsed;
        }

        let total = elapsed.as_ticks();
        let busy = total.saturating_sub(slept);

        #[allow(
            clippy::arithmetic_side_effects,
            reason = "total is at least one WINDOW of 1 MHz ticks so never zero, and busy <= total \
                      keeps the product far inside u64"
        )]
        let permille = (busy * 1000 / total) as u16;

        self.peak_permille = self.peak_permille.max(permille);
        self.windows = self.windows.saturating_add(1);

        if self.windows >= WINDOWS_PER_REPORT {
            CPU_LOAD.set_permille(self.peak_permille);
            LOOP_HEALTH.set_peak_latency_us(self.peak_latency_us);
            self.peak_permille = 0;
            self.peak_latency_us = 0;
            self.windows = 0;
        }
    }
}

impl Default for CpuMonitor {
    fn default() -> Self {
        Self::new()
    }
}
