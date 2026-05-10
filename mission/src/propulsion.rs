pub use rapid_dialect::ValveCommand;
use rapid_dialect::rapid::enums::{PropulsionType, ValveId};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum TankId {
    Pressurant,
    Oxidizer,
    CombustionChamber,
}

pub const ALL_TANKS: [TankId; 3] = [
    TankId::Pressurant,
    TankId::Oxidizer,
    TankId::CombustionChamber,
];

pub const ALL_VALVES: [ValveId; 5] = [
    ValveId::PressurantVent,
    ValveId::Pressurization,
    ValveId::OxidizerVent,
    ValveId::OxidizerFill,
    ValveId::Main,
];

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct TankReading {
    pub pressure1: Option<f32>,
    pub pressure2: Option<f32>,
    pub temperature1: Option<f32>,
    pub temperature2: Option<f32>,
    pub level: Option<f32>,
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct ValveReading {
    pub commanded_state: Option<f32>,
    pub measured_state: Option<f32>,
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum PropulsionError {
    NotPermittedInMode,
    Inhibited,
    TransportFailed,
}

pub trait Propulsion {
    const PROPULSION_TYPE: PropulsionType = PropulsionType::Solid;

    fn tank_state(&self, id: TankId) -> Option<TankReading>;
    fn valve_state(&self, id: ValveId) -> Option<ValveReading>;
    fn command_valve(&mut self, id: ValveId, cmd: ValveCommand) -> Result<(), PropulsionError>;
    fn fire_igniter(&mut self) -> Result<(), PropulsionError>;
}

#[derive(Default, Copy, Clone, Debug)]
pub struct NoPropulsion;

impl Propulsion for NoPropulsion {
    fn tank_state(&self, _id: TankId) -> Option<TankReading> {
        None
    }

    fn valve_state(&self, _id: ValveId) -> Option<ValveReading> {
        None
    }

    fn command_valve(&mut self, _id: ValveId, _cmd: ValveCommand) -> Result<(), PropulsionError> {
        Err(PropulsionError::Inhibited)
    }

    fn fire_igniter(&mut self) -> Result<(), PropulsionError> {
        Err(PropulsionError::Inhibited)
    }
}
