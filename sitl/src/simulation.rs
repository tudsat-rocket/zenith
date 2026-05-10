use std::sync::{Arc, Mutex};

use rapid_dialect::FlightMode;

pub mod battery;
#[cfg(feature = "hybrid")]
pub mod hybrid;
mod outputs;
mod physics;
pub mod sensors;

pub use battery::Battery;
pub use outputs::StdOutputs;
pub use physics::RecoveryFlags;
pub use physics::{DT, FlightPhysics};
pub use sensors::StdSensors;

#[cfg(feature = "hybrid")]
use hybrid::HybridSimulation;

pub struct Simulation {
    pub physics: FlightPhysics,
    pub battery: Battery,
    #[cfg(feature = "hybrid")]
    pub hybrid: HybridSimulation,
}

pub type SharedSimulation = Arc<Mutex<Simulation>>;

impl Simulation {
    pub fn new(flags: RecoveryFlags) -> Self {
        Self {
            physics: FlightPhysics::new(flags),
            battery: Battery::new(),
            #[cfg(feature = "hybrid")]
            hybrid: HybridSimulation::new(),
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
                self.hybrid = HybridSimulation::new();
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
