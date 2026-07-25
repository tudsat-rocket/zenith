//! Crate containing code specific to the software-in-the-loop (SITL) execution mode, i.e. being
//! run on a normal operating system (currently assumed to be Linux due to the network device
//! setup).
//!
//! This includes some basic hybrid rocket and flight simulation code.

pub mod simulation;

pub use simulation::{
    MemoryStorage, RecoveryFlags, SharedSimulation, Simulation, StdOutputs, StdSensors,
};

#[cfg(not(feature = "hybrid"))]
pub type Vehicle<F = MemoryStorage> =
    mission::Vehicle<StdSensors, StdOutputs, F, mission::bus::NoBus>;

#[cfg(feature = "hybrid")]
pub type Vehicle<F = MemoryStorage> =
    mission::Vehicle<StdSensors, StdOutputs, F, simulation::hybrid::SitlBus>;
