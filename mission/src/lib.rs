#![no_std]
#![allow(async_fn_in_trait)]

pub mod flight_logic;
mod mavlink;
pub mod propulsion;
mod settings;
mod traits;
mod vehicle;

pub use settings::*;
pub use traits::*;
pub use vehicle::Vehicle;

pub use propulsion::{
    NoPropulsion, Propulsion, PropulsionError, TankId, TankReading, ValveCommand, ValveReading,
};
