pub mod sensors;
pub mod simulation;

pub use sensors::{StdOutputs, StdSensors};
pub use simulation::{FlightSimulation, RecoveryFlags};

pub type Vehicle<F = mission::NoStorage> = mission::Vehicle<StdSensors, StdOutputs, F>;
