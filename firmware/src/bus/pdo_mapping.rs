use mission::{
    bus::ValveState,
    inventory::{BinaryOutputId, ValveId},
};

use crate::bus::mapping::{BINARY_OUTPUT_ID_MAP, VALVE_ID_MAP};

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
    /// holds first 4 preprocessed sensor values as u16
    /// temp: centi celcius, pressure: kilo pascal
    Sensor0,
    /// holds second 4 preprocessed sensor values as u16
    /// temp: centi celcius, pressure: kilo pascal
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
        if !(0x1800..0x1A00).contains(&value) {
            return Err(());
        }
        let identifier: u16 = (value >> 4) & 0b111_1111;
        let kind = PdMessageKind::try_from(identifier as u8);
        let Ok(kind) = kind else {
            return Err(());
        };

        Ok(Self {
            node_id: (value & 0b1111) as u8,
            kind,
        })
    }
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

fn decode_le_u16x4(data: &[u8]) -> Option<[u16; 4]> {
    let data: &[u8; 8] = data.try_into().ok()?;
    Some(core::array::from_fn(|i| {
        u16::from_le_bytes([data[i * 2], data[i * 2 + 1]])
    }))
}
