//! Shared test harness for sitl integration tests.
//!
//! Drives a real `Vehicle` + `FlightSimulation` in a tight loop without any
//! real-time waits. Each `Harness` owns its own `RecoveryFlags`, so multiple
//! tests may run in parallel.

#![allow(dead_code)]

use mission::{Settings, Storage, Vehicle as MissionVehicle};
use rapid_dialect::FlightMode;
use sitl::{FlightSimulation, RecoveryFlags, StdOutputs, StdSensors};

pub type Vehicle = MissionVehicle<StdSensors, StdOutputs, MemoryStorage>;

/// Test `Storage` double that hands out a fixed `Settings` (or None).
pub struct MemoryStorage {
    stored: Option<Settings>,
}

impl MemoryStorage {
    pub fn new(stored: Option<Settings>) -> Self {
        Self { stored }
    }
}

impl Storage for MemoryStorage {
    async fn read_settings(&mut self) -> Option<Settings> {
        self.stored.clone()
    }

    async fn write_settings(&mut self, settings: &Settings) {
        self.stored = Some(settings.clone());
    }
}

pub struct Harness {
    pub vehicle: Vehicle,
    pub flags: RecoveryFlags,
}

impl Harness {
    /// Construct a harness. If `settings` is Some, the vehicle reads it from
    /// storage on construction (exercising the real `Vehicle::new` code
    /// path). If None, storage returns None and `Settings::default()` is
    /// used.
    pub async fn new(settings: Option<Settings>) -> Self {
        let flags = RecoveryFlags::default();
        let sensors = StdSensors::new(flags.clone());
        let outputs = StdOutputs::new(flags.clone());
        let storage = MemoryStorage::new(settings);
        let vehicle = MissionVehicle::new(sensors, outputs, storage).await;
        Self { vehicle, flags }
    }

    pub fn arm(&mut self) {
        self.vehicle.set_mode(FlightMode::Armed);
        // Notify sim directly so the 5s ignition timer starts even though
        // run_until/run_ticks hasn't seen the mode change yet.
        self.vehicle.sensors.set_flight_mode(FlightMode::Armed);
    }

    pub fn mode(&self) -> FlightMode {
        self.vehicle.mode()
    }

    pub fn sim(&self) -> &FlightSimulation {
        self.vehicle.sensors.sim()
    }

    pub fn altitude_agl(&self) -> f32 {
        self.sim().altitude_agl()
    }

    pub fn drogue_active(&self) -> bool {
        self.flags.drogue.load(std::sync::atomic::Ordering::Relaxed)
    }

    pub fn main_active(&self) -> bool {
        self.flags.main.load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Drive `vehicle.tick()` N times. Mirrors sitl::main_loop's
    /// mode-change-> sim notification so arming transitions into launch.
    pub async fn run_ticks(&mut self, n: u32) {
        let mut last_mode = self.vehicle.mode();
        for _ in 0..n {
            self.vehicle.tick().await;
            let mode = self.vehicle.mode();
            if mode != last_mode {
                self.vehicle.sensors.set_flight_mode(mode);
                last_mode = mode;
            }
        }
    }

    /// Tick until `pred(&harness)` returns true, or `max_ticks` elapses.
    /// Returns the tick count at which it held, or Err on timeout.
    /// The predicate is also called once before the first tick.
    pub async fn run_until(
        &mut self,
        max_ticks: u32,
        mut pred: impl FnMut(&Self) -> bool,
    ) -> Result<u32, &'static str> {
        if pred(self) {
            return Ok(0);
        }
        let mut last_mode = self.vehicle.mode();
        for i in 1..=max_ticks {
            self.vehicle.tick().await;
            let mode = self.vehicle.mode();
            if mode != last_mode {
                self.vehicle.sensors.set_flight_mode(mode);
                last_mode = mode;
            }
            if pred(self) {
                return Ok(i);
            }
        }
        Err("run_until: max_ticks exceeded")
    }
}

/// Convenience: run an async block on the current thread to completion.
pub fn block_on<F: core::future::Future>(f: F) -> F::Output {
    embassy_futures::block_on(f)
}
