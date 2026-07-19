#![no_std]
#![allow(async_fn_in_trait)]

pub mod bus;
pub mod flight_logic;
pub mod inventory;
mod mavlink;
mod settings;
mod traits;
pub mod valves;
mod vehicle;

pub use settings::*;
pub use traits::*;
pub use vehicle::Vehicle;

pub use inventory::TankId;
pub use valves::ValveError;
