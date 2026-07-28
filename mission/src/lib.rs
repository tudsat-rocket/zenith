#![no_std]
#![allow(async_fn_in_trait)]

pub mod bus;
pub mod flight_logic;
pub mod inventory;
mod mavlink;
pub mod params;
mod schedule;
mod traits;
pub mod valves;
mod vehicle;

pub use params::{Params, RecoveryParams};
pub use traits::*;
pub use vehicle::Vehicle;

pub use inventory::TankId;
pub use valves::ValveError;
