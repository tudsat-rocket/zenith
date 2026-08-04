use crate::bus::pdo_mapping::{PdMessageKind, SensorAddr};
use mission::bus::IoAddr;
use mission::inventory::{BinaryOutputMap, PressureSensorMap, TemperatureSensorMap, ValveMap};

// Io-board store index
pub const VALVE_IDX: u16 = 0x2005;
pub const HC_OUTPUT_IDX: u16 = 0x2006;

#[allow(dead_code)]
const HCO1: u8 = 0;
#[allow(dead_code)]
const HCO2: u8 = 1;
#[allow(dead_code)]
const HCO3: u8 = 2;
#[allow(dead_code)]
const HCO4: u8 = 3;

pub const VALVE_ID_MAP: ValveMap<IoAddr> = ValveMap::new([
    // TODO: external io boards
    // PressurantVent = 0,
    IoAddr::new(5, VALVE_IDX, HCO4),
    // Pressurization = 1,
    IoAddr::new(5, VALVE_IDX, HCO2),
    // OxidizerVent = 2,
    IoAddr::new(4, VALVE_IDX, HCO1),
    // OxidizerFill = 3,
    IoAddr::new(6, VALVE_IDX, HCO4),
    // Main = 4,
    IoAddr::new(6, VALVE_IDX, HCO2),
    // ExternalPressurantFill = 5,
    IoAddr::new(0xff, VALVE_IDX, 0xff),
    // ExternalOxidizerFill = 6,
    IoAddr::new(0xff, VALVE_IDX, 0xff),
    // ExternalPressurantVent = 7,
    IoAddr::new(0xff, VALVE_IDX, 0xff),
    // ExternalOxidizerVent = 8,
    IoAddr::new(0xff, VALVE_IDX, 0xff),
]);

pub const BINARY_OUTPUT_ID_MAP: BinaryOutputMap<IoAddr> = BinaryOutputMap::new([
    IoAddr::new(7, HC_OUTPUT_IDX, 0), // BinaryOutputId::Igniter1,
    IoAddr::new(7, HC_OUTPUT_IDX, 2), // BinaryOutputId::Igniter2,
    IoAddr::new(2, HC_OUTPUT_IDX, 0), // BinaryOutputId::Camera1,
    IoAddr::new(2, HC_OUTPUT_IDX, 2), // BinaryOutputId::Camera2,
    IoAddr::new(3, HC_OUTPUT_IDX, 0), // BinaryOutputId::Camera3,
]);

pub const TEMP_SENSOR_ID_MAP: TemperatureSensorMap<SensorAddr> = TemperatureSensorMap::new([
    // node_id, sensor_msg_num, slot
    SensorAddr::from_sensor_idx(5, 0).unwrap(), // TempSensId::OxTankUpper
    SensorAddr::from_sensor_idx(6, 2).unwrap(), // TempSensId::OxTankLower
]);

pub const PRESS_SENSOR_ID_MAP: PressureSensorMap<SensorAddr> = PressureSensorMap::new([
    // node_id, sensor_msg_num, slot
    SensorAddr::from_sensor_idx(2, 0).unwrap(), // PressSensId::Nosecone
    SensorAddr::from_sensor_idx(5, 4).unwrap(), // PressSensId::PressurantTank
    SensorAddr::from_sensor_idx(5, 1).unwrap(), // PressSensId::PReg1
    SensorAddr::from_sensor_idx(5, 2).unwrap(), // PressSensId::PReg2
    SensorAddr::from_sensor_idx(5, 3).unwrap(), // PressSensId::OxTankUpper
    SensorAddr::from_sensor_idx(6, 1).unwrap(), // PressSensId::OxTankLower
    SensorAddr::from_sensor_idx(6, 0).unwrap(), // PressSensId::CombustionChamber
    // TODO: external io board
    SensorAddr::from_sensor_idx(8, 0).unwrap(), // PressSensId::ExternalPressurant
    SensorAddr::from_sensor_idx(8, 0).unwrap(), // PressSensId::ExternalOxidizer
]);
