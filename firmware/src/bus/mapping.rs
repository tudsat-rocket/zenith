use mission::bus::IoAddr;
use mission::inventory::{BinaryOutputMap, ValveMap};

// Io-board store index
pub const VALVE_IDX: u16 = 0x2005;
pub const HC_OUTPUT_IDX: u16 = 0x2006;

// TODO
pub const VALVE_ID_MAP: ValveMap<IoAddr> = ValveMap::new([
    IoAddr::new(5, VALVE_IDX, 3),       // ValveId::PressurantVent
    IoAddr::new(5, VALVE_IDX, 3),       // ValveId::Pressurization
    IoAddr::new(4, VALVE_IDX, 1),       // ValveId::OxidizerVent
    IoAddr::new(6, VALVE_IDX, 3),       // ValveId::OxidizerFill
    IoAddr::new(6, VALVE_IDX, 1),       // ValveId::Main
    IoAddr::new(0xff, VALVE_IDX, 0xff), // ValveId::ExternalPressurantFill
    IoAddr::new(0xff, VALVE_IDX, 0xff), // ValveId::ExternalOxidizerFill
    IoAddr::new(0xff, VALVE_IDX, 0xff), // ValveId::ExternalPressurantVent
    IoAddr::new(0xff, VALVE_IDX, 0xff), // ValveId::ExternalOxidizerVent
]);

// TODO
pub const BINARY_OUTPUT_ID_MAP: BinaryOutputMap<IoAddr> = BinaryOutputMap::new([
    IoAddr::new(7, HC_OUTPUT_IDX, 2), // BinaryOutputId::Igniter1,
    IoAddr::new(7, HC_OUTPUT_IDX, 3), // BinaryOutputId::Igniter2,
    IoAddr::new(2, HC_OUTPUT_IDX, 0), // BinaryOutputId::Camera1,
    IoAddr::new(2, HC_OUTPUT_IDX, 0), // BinaryOutputId::Camera2,
]);
