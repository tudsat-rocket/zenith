use core::{cmp::*, num::Wrapping, usize};
pub use rapid_dialect::ValveCommand;
use rapid_dialect::rapid::enums::FluidType;
pub use rapid_dialect::rapid::enums::ValveId;

use crate::{foreign_io::DataWithTime, vehicle::VehicleSnapshot};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum TankId {
    Pressurant,
    Oxidizer,
    CombustionChamber,
}

/// Every foreign temperature sensor.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum TempSensId {
    OxTankUpper,
    OxTankLower,
}

/// Every foreign pressure sensor.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum PressSensId {
    Nosecone,
    PressurantTank,
    PReg1,
    PReg2,
    OxTankUpper,
    OxTankLower,
    CombustionChamber,
    ExternalPressurant,
    ExternalOxidizer,
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum BinaryOutputId {
    Igniter1,
    Igniter2,
    Camera1,
    Camera2,
}

pub type TempSensArray = [TempSensId; 2];
pub const ALL_TEMP_SENS: TempSensArray = [TempSensId::OxTankUpper, TempSensId::OxTankLower];

pub type PressSensArray = [PressSensId; 9];
pub const ALL_PRESS_SENS: PressSensArray = [
    PressSensId::Nosecone,
    PressSensId::PressurantTank,
    PressSensId::PReg1,
    PressSensId::PReg2,
    PressSensId::OxTankUpper,
    PressSensId::OxTankLower,
    PressSensId::CombustionChamber,
    PressSensId::ExternalPressurant,
    PressSensId::ExternalOxidizer,
];

pub const ALL_BINARY_OUTPUTS: [BinaryOutputId; 4] = [
    BinaryOutputId::Igniter1,
    BinaryOutputId::Igniter2,
    BinaryOutputId::Camera1,
    BinaryOutputId::Camera2,
];

pub const ALL_TANKS: [TankId; 3] = [
    TankId::Pressurant,
    TankId::Oxidizer,
    TankId::CombustionChamber,
];

pub type ValveArray = [ValveId; 5];
pub const ALL_VALVES: ValveArray = [
    ValveId::PressurantVent,
    ValveId::Pressurization,
    ValveId::OxidizerVent,
    ValveId::OxidizerFill,
    ValveId::Main,
];

#[derive(Clone, PartialEq)]
pub struct TankReading {
    // TODO: (felix) document Units
    pub pressure1: Option<f32>,
    pub pressure2: Option<f32>,
    pub temperature1: Option<f32>,
    pub temperature2: Option<f32>,
    pub level: Option<f32>,
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum PropulsionError {
    NotPermittedInMode,
    Inhibited,
    TransportFailed,
}

pub fn tank_reading(tank: TankId, snapshot: &VehicleSnapshot) -> TankReading {
    use PressSensId as P;
    use TempSensId as T;

    let readings = snapshot.input_image;

    match tank {
        TankId::Oxidizer => TankReading {
            pressure1: readings.press_sens[P::OxTankUpper as usize].map(|d| d.data),
            pressure2: readings.press_sens[P::OxTankLower as usize].map(|d| d.data),
            temperature1: readings.temp_sens[T::OxTankUpper as usize].map(|d| d.data),
            temperature2: readings.temp_sens[T::OxTankLower as usize].map(|d| d.data),
            level: None,
        },
        TankId::Pressurant => TankReading {
            pressure1: readings.press_sens[P::PressurantTank as usize].map(|d| d.data),
            pressure2: readings.press_sens[P::PressurantTank as usize].map(|d| d.data),
            temperature1: None,
            temperature2: None,
            level: None,
        },
        TankId::CombustionChamber => TankReading {
            pressure1: readings.press_sens[P::CombustionChamber as usize].map(|d| d.data),
            pressure2: readings.press_sens[P::CombustionChamber as usize].map(|d| d.data),
            temperature1: None,
            temperature2: None,
            level: None,
        },
    }
}

// pub trait Propulsion {
//     const PROPULSION_TYPE: PropulsionType = PropulsionType::Solid;
//
//     fn tank_state(&self, id: TankId) -> Option<TankReading>;
//     fn valve_state(&self, id: ValveId) -> Option<ValveReading>;
//     fn command_valve(&mut self, id: ValveId, cmd: ValveCommand) -> Result<(), PropulsionError>;
//     fn fire_igniter(&mut self) -> Result<(), PropulsionError>;
// }
//
// #[derive(Default, Copy, Clone, Debug)]
// pub struct NoPropulsion;
//
// impl Propulsion for NoPropulsion {
//     fn tank_state(&self, _id: TankId) -> Option<TankReading> {
//         None
//     }
//
//     fn valve_state(&self, _id: ValveId) -> Option<ValveReading> {
//         None
//     }
//
//     fn command_valve(&mut self, _id: ValveId, _cmd: ValveCommand) -> Result<(), PropulsionError> {
//         Err(PropulsionError::Inhibited)
//     }
//
//     fn fire_igniter(&mut self) -> Result<(), PropulsionError> {
//         Err(PropulsionError::Inhibited)
//     }
// }
