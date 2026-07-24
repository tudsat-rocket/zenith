#![allow(
    clippy::arithmetic_side_effects,
    reason = "CANopen PDO bit/byte offset math, bounded by frame layout"
)]
#![allow(
    clippy::indexing_slicing,
    reason = "indexing into fixed-size CAN frame buffers"
)]

use defmt::Debug2Format;
use mission::{
    bus::ValveState,
    inventory::{BinaryOutputId, PressSensId, TempSensId, ValveId},
};

use crate::bus::mapping::{
    BINARY_OUTPUT_ID_MAP, PRESS_SENSOR_ID_MAP, TEMP_SENSOR_ID_MAP, VALVE_ID_MAP,
};

/// every pdo message has 8 bytes payload
/// every entry is encoded as litle endian
/// ProcessDataMessageKind

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum PdMessageKind {
    // NOTE: maybe have a look at num_enum: TryFromPrimitive
    /// holds 4 ValveStates (promille open) as le u16
    Valves,
    /// holds 4 bools as le u16
    BinaryOutpus,
    /// holds 4 pwm microseconds entries as le u16
    PwmUs,
    /// holds fist 4 raw adc measurements of i2c bus 0
    RawBus0a,
    /// holds second 4 raw adc measurements of i2c bus 0
    RawBus0b,
    /// holds first 4 raw adc measurements of i2c bus 1
    RawBus1a,
    /// holds second 4 raw adc measurements of i2c bus 1
    RawBus1b,
    /// holds first 4 preprocessed sensor values as i16 or u16
    /// temp(i16): centi celcius, pressure(u16): kilo pascal (=centi bar)
    Sensor0,
    /// holds second 4 preprocessed sensor values as i16 or u16
    /// temp(i16): centi celcius, pressure(u16): kilo pascal (=centi bar)
    Sensor1,
}
impl TryFrom<u8> for PdMessageKind {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, ()> {
        use PdMessageKind as K;
        match value {
            0 => Ok(K::Valves),
            1 => Ok(K::BinaryOutpus),
            2 => Ok(K::PwmUs),
            3 => Ok(K::RawBus0a),
            4 => Ok(K::RawBus0b),
            5 => Ok(K::RawBus1a),
            6 => Ok(K::RawBus1b),
            7 => Ok(K::Sensor0),
            8 => Ok(K::Sensor1),
            _ => Err(()),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub struct ProcessDataCanId {
    pub node_id: u8,
    pub kind: PdMessageKind,
}

impl TryFrom<u16> for ProcessDataCanId {
    type Error = ();
    fn try_from(value: u16) -> Result<Self, ()> {
        // 0d512 = 2^9
        if !(0x200..(0x200 + 512)).contains(&value) {
            defmt::warn!("cob id (0x{:x}) not contained in range", value);
            return Err(());
        }
        let identifier: u16 = (value >> 4) & 0b1_1111;
        let kind = PdMessageKind::try_from(identifier as u8);
        let Ok(kind) = kind else {
            defmt::warn!("kind does not exist: {}", Debug2Format(&kind));
            return Err(());
        };

        Ok(Self {
            node_id: (value & 0b1111) as u8,
            kind,
        })
    }
}

/// Where a single reading lives on the bus: which node, which of the two
/// Sensor messages, and which of the 4 slots within that message.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct SensorAddr {
    pub node_id: u8,
    pub kind: PdMessageKind,
    pub slot: u8,
}

impl SensorAddr {
    pub const fn from_sensor_idx(node_id: u8, sensor_idx: usize) -> Option<Self> {
        if sensor_idx >= 8 {
            return None;
        }
        if node_id >= 16 {
            return None;
        }
        let kind = match sensor_idx / 4 {
            0 => PdMessageKind::Sensor0,
            1 => PdMessageKind::Sensor1,
            _ => return None,
        };
        let slot = (sensor_idx % 4) as u8;
        Some(Self {
            node_id,
            kind,
            slot,
        })
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum SensorReading {
    Temperature(TempSensId, f32),
    Pressure(PressSensId, f32),
}

pub fn valve_msg_to_valve(node_id: u16, data: &[u8]) -> heapless::Vec<(ValveId, ValveState), 4> {
    let mut out = heapless::Vec::new();
    let Some(words) = decode_le_u16x4(data) else {
        return out;
    };
    for (i, &word) in words.iter().enumerate() {
        if let Some(id) = valve_id_for(node_id, i) {
            let _ = out.push((id, ValveState::from_promille_clamped(word)));
        }
    }
    out
}

fn valve_id_for(node_id: u16, subindex: usize) -> Option<ValveId> {
    VALVE_ID_MAP
        .iter()
        .find(|(_, addr)| addr.node_id as u16 == node_id && addr.subindex as usize == subindex)
        .map(|(id, _)| id)
}

pub fn binary_output_msg_to_bo(
    node_id: u16,
    data: &[u8],
) -> heapless::Vec<(BinaryOutputId, bool), 4> {
    let mut out = heapless::Vec::new();
    let Some(words) = decode_le_u16x4(data) else {
        return out;
    };
    for (i, &word) in words.iter().enumerate() {
        if let Some(id) = bo_id_for(node_id, i) {
            let _ = out.push((id, word != 0));
        }
    }
    out
}

fn bo_id_for(node_id: u16, subindex: usize) -> Option<BinaryOutputId> {
    BINARY_OUTPUT_ID_MAP
        .iter()
        .find(|(_, addr)| addr.node_id as u16 == node_id && addr.subindex as usize == subindex)
        .map(|(id, _)| id)
}

pub fn sensor_msg_to_readings(
    node_id: u16,
    kind: PdMessageKind,
    data: &[u8],
) -> heapless::Vec<SensorReading, 4> {
    let mut out = heapless::Vec::new();
    let Some(words) = decode_le_u16x4(data) else {
        return out;
    };

    for (slot, &word) in words.iter().enumerate() {
        if let Some(id) = temp_sensor_id_for(node_id, kind, slot) {
            // centi-celsius, signed
            let raw = word as i16;
            let _ = out.push(SensorReading::Temperature(id, f32::from(raw) / 100.0));
        } else if let Some(id) = press_sensor_id_for(node_id, kind, slot) {
            // kPa == centi-bar, unsigned
            let _ = out.push(SensorReading::Pressure(id, f32::from(word) / 100.0));
        }
    }

    out
}

fn temp_sensor_id_for(node_id: u16, kind: PdMessageKind, slot: usize) -> Option<TempSensId> {
    TEMP_SENSOR_ID_MAP
        .iter()
        .find(|(_, addr)| {
            addr.node_id as u16 == node_id && addr.kind == kind && addr.slot as usize == slot
        })
        .map(|(id, _)| id)
}

fn press_sensor_id_for(node_id: u16, kind: PdMessageKind, slot: usize) -> Option<PressSensId> {
    PRESS_SENSOR_ID_MAP
        .iter()
        .find(|(_, addr)| {
            addr.node_id as u16 == node_id && addr.kind == kind && addr.slot as usize == slot
        })
        .map(|(id, _)| id)
}

fn decode_le_u16x4(data: &[u8]) -> Option<[u16; 4]> {
    let data: &[u8; 8] = data.try_into().ok()?;
    Some(core::array::from_fn(|i| {
        u16::from_le_bytes([data[i * 2], data[i * 2 + 1]])
    }))
}
