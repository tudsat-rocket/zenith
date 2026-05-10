//! Crate containing code specific to the software-in-the-loop (SITL) execution mode, i.e. being
//! run on a normal operating system (currently assumed to be Linux due to the network device
//! setup).
//!
//! This includes some basic hybrid rocket and flight simulation code.

pub mod simulation;

pub use simulation::{RecoveryFlags, SharedSimulation, Simulation, StdOutputs, StdSensors};

#[cfg(not(feature = "hybrid"))]
pub type Vehicle<F = mission::NoStorage> =
    mission::Vehicle<StdSensors, StdOutputs, F, mission::NoPropulsion>;

#[cfg(feature = "hybrid")]
pub type Vehicle<F = mission::NoStorage> =
    mission::Vehicle<StdSensors, StdOutputs, F, simulation::hybrid::SitlPropulsion>;
