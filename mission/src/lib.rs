#![no_std]
#![allow(async_fn_in_trait)]

pub mod flight_logic;
mod mavlink;
mod telemetry;
mod traits;
mod vehicle;

pub use telemetry::TelemetryLink;
pub use traits::*;
pub use vehicle::Vehicle;
