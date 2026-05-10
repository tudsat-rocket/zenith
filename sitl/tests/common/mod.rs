//! Shared test harness for sitl integration tests.

#![allow(dead_code)]

use std::sync::{Arc, Mutex};

use mission::{Settings, Storage, Vehicle as MissionVehicle};
use rapid_dialect::FlightMode;
use sitl::{RecoveryFlags, SharedSimulation, Simulation, StdOutputs, StdSensors};

#[cfg(not(feature = "hybrid"))]
use mission::NoPropulsion;

#[cfg(feature = "hybrid")]
use sitl::simulation::hybrid::SitlPropulsion;

#[cfg(not(feature = "hybrid"))]
pub type Vehicle = MissionVehicle<StdSensors, StdOutputs, MemoryStorage, NoPropulsion>;

#[cfg(feature = "hybrid")]
pub type Vehicle = MissionVehicle<StdSensors, StdOutputs, MemoryStorage, SitlPropulsion>;

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
    pub sim: SharedSimulation,
}

impl Harness {
    pub async fn new(settings: Option<Settings>) -> Self {
        let flags = RecoveryFlags::default();
        let sim: SharedSimulation = Arc::new(Mutex::new(Simulation::new(flags.clone())));
        let sensors = StdSensors::new(sim.clone());
        let outputs = StdOutputs::new(flags);
        let storage = MemoryStorage::new(settings);

        #[cfg(not(feature = "hybrid"))]
        let vehicle = MissionVehicle::new(sensors, outputs, storage, NoPropulsion).await;

        #[cfg(feature = "hybrid")]
        let vehicle = {
            let propulsion = SitlPropulsion::new(sim.clone());
            MissionVehicle::new(sensors, outputs, storage, propulsion).await
        };

        Self { vehicle, sim }
    }

    pub fn arm(&mut self) {
        self.vehicle.set_mode(FlightMode::Armed);
        // Notify sim directly so the 5s ignition timer starts even though
        // run_until/run_ticks hasn't seen the mode change yet.
        self.sim.lock().unwrap().set_flight_mode(FlightMode::Armed);
    }

    pub fn mode(&self) -> FlightMode {
        self.vehicle.mode()
    }

    pub fn altitude_agl(&self) -> f32 {
        self.sim.lock().unwrap().physics.altitude_agl()
    }

    pub fn drogue_active(&self) -> bool {
        self.sim
            .lock()
            .unwrap()
            .physics
            .flags
            .drogue
            .load(std::sync::atomic::Ordering::Relaxed)
    }

    pub fn main_active(&self) -> bool {
        self.sim
            .lock()
            .unwrap()
            .physics
            .flags
            .main
            .load(std::sync::atomic::Ordering::Relaxed)
    }

    /// Fast-forward through the boot-time pressurant fill. The hybrid sim
    /// fills pressurant from ambient to nominal over 30s while in `Idle`;
    /// most tests want pressurant ready before exercising propellant ops.
    #[cfg(feature = "hybrid")]
    pub async fn fill_pressurant(&mut self) {
        self.run_ticks(31_000).await;
    }

    fn tick_sim(&mut self) {
        let mut s = self.sim.lock().unwrap();
        s.set_flight_mode(self.vehicle.mode());
        s.tick();
    }

    pub async fn run_ticks(&mut self, n: u32) {
        for _ in 0..n {
            self.tick_sim();
            self.vehicle.tick().await;
        }
    }

    pub async fn run_until(
        &mut self,
        max_ticks: u32,
        mut pred: impl FnMut(&Self) -> bool,
    ) -> Result<u32, &'static str> {
        if pred(self) {
            return Ok(0);
        }

        for i in 1..=max_ticks {
            self.tick_sim();
            self.vehicle.tick().await;
            if pred(self) {
                return Ok(i);
            }
        }

        Err("run_until: max_ticks exceeded")
    }
}

pub fn block_on<F: core::future::Future>(f: F) -> F::Output {
    embassy_futures::block_on(f)
}
