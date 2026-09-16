use std::sync::{Arc, Mutex};

use rapid_dialect::FlightMode;

pub mod battery;
pub mod faults;
#[cfg(feature = "hybrid")]
pub mod hybrid;
mod outputs;
mod physics;
pub mod sensors;
pub mod storage;

pub use battery::Battery;
pub use faults::Faults;
pub use outputs::StdOutputs;
pub use physics::RecoveryFlags;
pub use physics::{DT, FlightPhase, FlightPhysics};
pub use sensors::StdSensors;
pub use storage::MemoryStorage;

#[cfg(feature = "hybrid")]
use hybrid::HybridSimulation;

pub struct Simulation {
    pub physics: FlightPhysics,
    pub battery: Battery,
    #[cfg(feature = "hybrid")]
    pub hybrid: HybridSimulation,
    pub faults: Faults,
}

pub type SharedSimulation = Arc<Mutex<Simulation>>;

impl Simulation {
    /// A simulation where everything works.
    pub fn new(flags: RecoveryFlags) -> Self {
        Self::with_faults(flags, Faults::nominal())
    }

    /// A simulation running the given off-nominal scenario.
    pub fn with_faults(flags: RecoveryFlags, faults: Faults) -> Self {
        let mut physics = FlightPhysics::new(flags);
        physics.config.mass_flow_factor = faults.mass_flow_factor();
        physics.config.drogue_fails = faults.drogue_failure;

        Self {
            physics,
            battery: Battery::new(),
            #[cfg(feature = "hybrid")]
            hybrid: HybridSimulation::new(faults.mass_flow_factor()),
            faults,
        }
    }

    pub fn set_flight_mode(&mut self, mode: FlightMode) {
        let prev = self.physics.mode;
        self.physics.set_flight_mode(mode);
        #[cfg(feature = "hybrid")]
        self.hybrid.set_flight_mode(mode);

        if mode == FlightMode::Idle && prev != FlightMode::Idle {
            self.battery = Battery::new();
            #[cfg(feature = "hybrid")]
            {
                self.hybrid = HybridSimulation::new(self.faults.mass_flow_factor());
            }
        }
    }

    pub fn tick(&mut self) {
        self.physics.tick();
        self.battery.tick(DT, self.physics.mode);

        #[cfg(feature = "hybrid")]
        self.hybrid.tick(DT);
        #[cfg(feature = "hybrid")]
        self.physics
            .set_chamber_pressure(self.hybrid.chamber_pressure);
    }
}
