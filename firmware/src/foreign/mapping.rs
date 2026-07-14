use mission::{
    foreign_io::IoAddr,
    propulsion::{ALL_BINARY_OUTPUTS, ALL_PRESS_SENS, ALL_VALVES, ValveId},
};

// FIXME: valves are 1 indexed
pub struct ValveIdMap(pub [IoAddr; ALL_VALVES.len() + 1]);
pub struct PressureSensIdMap(pub [IoAddr; ALL_PRESS_SENS.len()]);
pub struct BinaryOutputIdMap(pub [IoAddr; ALL_BINARY_OUTPUTS.len()]);

// Io-board store index
pub const VALVE_IDX: u16 = 0x2005;
pub const HC_OUTPUT_IDX: u16 = 0x2006;

pub const VALVE_ID_MAP: ValveIdMap = ValveIdMap([
    IoAddr::new(0, 0, 0),         // FIXME: valves are 1 indexed
    IoAddr::new(5, VALVE_IDX, 3), // ValveId::PressurantVent
    IoAddr::new(5, VALVE_IDX, 3), // ValveId::Pressurization
    IoAddr::new(4, VALVE_IDX, 1), // ValveId::OxidizerVent
    IoAddr::new(6, VALVE_IDX, 3), // ValveId::OxidizerFill
    IoAddr::new(6, VALVE_IDX, 1), // ValveId::Main
]);

impl ValveIdMap {
    pub fn get_io_addr(&self, valve_id: ValveId) -> &IoAddr {
        // FIXME: valves are 1 indexed
        &self.0[valve_id as usize - 1]
    }
}

pub const BINARY_OUTPUT_ID_MAP: BinaryOutputIdMap = BinaryOutputIdMap([
    IoAddr::new(7, HC_OUTPUT_IDX, 2), // BinaryOutputId::Igniter1,
    IoAddr::new(7, HC_OUTPUT_IDX, 3), // BinaryOutputId::Igniter2,
    IoAddr::new(2, HC_OUTPUT_IDX, 0), // BinaryOutputId::Camera1,
    IoAddr::new(2, HC_OUTPUT_IDX, 0), // BinaryOutputId::Camera2,,
]);
