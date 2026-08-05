use crate::bus::pdo_mapping::SensorAddr;
use mission::bus::IoAddr;
use mission::inventory::{BinaryOutputMap, PressureSensorMap, TemperatureSensorMap, ValveMap};

// Io-board store index
/// "valve commanded state, position word", promille per valve.
pub const VALVE_STORE_IDX: u16 = 0x2010;
/// "high current outputs, digital level", one byte per output.
pub const HC_OUTPUT_STORE_IDX: u16 = 0x2020;

#[allow(dead_code)]
const HCO1: u8 = 1;
#[allow(dead_code)]
const HCO2: u8 = 2;
#[allow(dead_code)]
const HCO3: u8 = 3;
#[allow(dead_code)]
const HCO4: u8 = 4;

// These are CANopen sub-indices, so they are 1-based: sub 0 of an array object is the entry
// count, the elements start at sub 1. The TPDO frames carry the same elements 0-indexed, so
// decoding converts (see `pdo_mapping::subindex_to_slot`) rather than these being written twice.

// TODO: change this map construction
pub const VALVE_ID_MAP: ValveMap<IoAddr> = ValveMap::new([
    // TODO: external io boards
    // PressurantVent = 0,
    IoAddr::new(5, VALVE_STORE_IDX, 2),
    // Pressurization = 1,
    IoAddr::new(5, VALVE_STORE_IDX, 1),
    // OxidizerVent = 2,
    IoAddr::new(4, VALVE_STORE_IDX, 1),
    // OxidizerFill = 3,
    IoAddr::new(6, VALVE_STORE_IDX, 2),
    // Main = 4,
    IoAddr::new(6, VALVE_STORE_IDX, 1),
    // ExternalPressurantFill = 5,
    IoAddr::new(0xff, VALVE_STORE_IDX, 0xff),
    // ExternalOxidizerFill = 6,
    IoAddr::new(0xff, VALVE_STORE_IDX, 0xff),
    // ExternalPressurantVent = 7,
    IoAddr::new(0xff, VALVE_STORE_IDX, 0xff),
    // ExternalOxidizerVent = 8,
    IoAddr::new(0xff, VALVE_STORE_IDX, 0xff),
]);

// TODO: change this map construction
pub const BINARY_OUTPUT_ID_MAP: BinaryOutputMap<IoAddr> = BinaryOutputMap::new([
    IoAddr::new(7, HC_OUTPUT_STORE_IDX, HCO1), // BinaryOutputId::Igniter1,
    IoAddr::new(7, HC_OUTPUT_STORE_IDX, HCO3), // BinaryOutputId::Igniter2,
    IoAddr::new(2, HC_OUTPUT_STORE_IDX, HCO1), // BinaryOutputId::Camera1,
    IoAddr::new(2, HC_OUTPUT_STORE_IDX, HCO3), // BinaryOutputId::Camera2,
    IoAddr::new(3, HC_OUTPUT_STORE_IDX, HCO1), // BinaryOutputId::Camera3,
]);

// Sensor slots, unlike the sub-indices above, are the protocol's own 0-based slot numbering:
// readings only ever arrive over TPDO, never over an SDO read.

/// 0-indexed, since it arrives by tpdo
pub const TEMP_SENSOR_ID_MAP: TemperatureSensorMap<SensorAddr> = TemperatureSensorMap::new([
    // node_id, sensor slot
    SensorAddr::from_sensor_idx(5, 0).unwrap(), // TempSensId::OxTankUpper
    SensorAddr::from_sensor_idx(6, 2).unwrap(), // TempSensId::OxTankLower
]);

/// 0-indexed, since it arrives by tpdo
pub const PRESS_SENSOR_ID_MAP: PressureSensorMap<SensorAddr> = PressureSensorMap::new([
    // node_id, sensor slot
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
