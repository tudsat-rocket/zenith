//! Mapping between the ioCan bus protocol and this vehicle's inventory ids.
//!
//! The wire format itself — identifier layout, which fields sit in which of the 8 bytes, how
//! they are packed — lives in [`iocan_proto`]; this module only answers "node 5 slot 2 is which
//! sensor on this rocket", and converts the protocol's raw counts into the units the rest of the
//! firmware works in.

use iocan_proto::tpdo::NUM_PROTOCOL_SENSOR_SLOTS;
use iocan_proto::{HcoOutput, TpdoKind};

use mission::{
    bus::ValveState,
    inventory::{BinaryOutputId, PressSensId, TempSensId, ValveId},
};

use crate::bus::mapping::{
    BINARY_OUTPUT_ID_MAP, PRESS_SENSOR_ID_MAP, TEMP_SENSOR_ID_MAP, VALVE_ID_MAP,
};

// TODO: this is very error prone
// CANopen array objects keep the entry count at sub 0 and the elements at subs `1..=N`, but a
// TPDO frame carries those same elements 0-indexed. An [`IoAddr`](mission::bus::IoAddr)'s
// subindex is the store's, so it has to come back by one to name a slot in a frame.
/// Convert 0-indexed subindex to 1-indexed slot
const fn subindex_to_slot(subindex: u8) -> Option<usize> {
    match subindex.checked_sub(1) {
        Some(slot) => Some(slot as usize),
        // Sub 0 is the entry count, not an element, so it names no slot.
        None => None,
    }
}

const SENSOR_INVALID: i16 = i16::MIN;

/// Where a single reading lives on the bus: which node, and which of the protocol's sensor
/// slots. Unlike an [`IoAddr`](mission::bus::IoAddr)'s subindex this is a 0-indexed protocol
/// slot (`0..NUM_PROTOCOL_SENSOR_SLOTS`), because sensor readings only ever arrive over TPDO —
/// which of the three sensor frames carries one follows from the slot.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct SensorAddr {
    pub node_id: u8,
    pub slot: u8,
}

impl SensorAddr {
    pub const fn from_sensor_idx(node_id: u8, sensor_idx: usize) -> Option<Self> {
        if sensor_idx >= NUM_PROTOCOL_SENSOR_SLOTS {
            return None;
        }
        if node_id >= 16 {
            return None;
        }
        Some(Self {
            node_id,
            slot: sensor_idx as u8,
        })
    }
}

/// The protocol-wide slot index that the first entry of a sensor frame carries, or `None` for a
/// kind that carries no sensor values at all.
pub const fn sensor_slot_base(kind: TpdoKind) -> Option<u8> {
    match kind {
        TpdoKind::Sensor0 => Some(0),
        TpdoKind::Sensor1 => Some(4),
        TpdoKind::Sensor3 => Some(8),
        _ => None,
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum SensorReading {
    Temperature(TempSensId, f32),
    Pressure(PressSensId, f32),
}

/// Valve positions in promille, in the frame's slot order.
pub fn valve_msg_to_valve(
    node_id: u8,
    positions: [u16; 4],
) -> heapless::Vec<(ValveId, ValveState), 4> {
    let mut out = heapless::Vec::new();
    for (slot, &promille) in positions.iter().enumerate() {
        if let Some(id) = valve_id_for(node_id, slot) {
            let _ = out.push((id, ValveState::from_promille_clamped(promille)));
        }
    }
    out
}

fn valve_id_for(node_id: u8, slot: usize) -> Option<ValveId> {
    VALVE_ID_MAP
        .iter()
        .find(|(_, addr)| addr.node_id == node_id && subindex_to_slot(addr.subindex) == Some(slot))
        .map(|(id, _)| id)
}

/// The high current outputs, in the frame's slot order. Digital and PWM outputs share one frame
/// now, so an output we drive as a binary output is energised unless it reports
/// [`HcoOutput::DigitalOff`] — which is also what a zero pulse width decodes to.
pub fn hco_msg_to_binary_outputs(
    node_id: u8,
    outputs: [HcoOutput; 4],
) -> heapless::Vec<(BinaryOutputId, bool), 4> {
    let mut out = heapless::Vec::new();
    for (slot, &output) in outputs.iter().enumerate() {
        if let Some(id) = bo_id_for(node_id, slot) {
            let _ = out.push((id, output != HcoOutput::DigitalOff));
        }
    }
    out
}

fn bo_id_for(node_id: u8, slot: usize) -> Option<BinaryOutputId> {
    BINARY_OUTPUT_ID_MAP
        .iter()
        .find(|(_, addr)| addr.node_id == node_id && subindex_to_slot(addr.subindex) == Some(slot))
        .map(|(id, _)| id)
}

/// Calibrated sensor values, in the frame's slot order. Both temperature and pressure are signed
/// counts of a hundredth of their unit: centi-celsius, and kilopascal (= centi-bar).
pub fn sensor_msg_to_readings(
    node_id: u8,
    kind: TpdoKind,
    values: [i16; 4],
) -> heapless::Vec<SensorReading, 4> {
    let mut out = heapless::Vec::new();
    let Some(base) = sensor_slot_base(kind) else {
        return out;
    };

    for (offset, &raw) in values.iter().enumerate() {
        if raw == SENSOR_INVALID {
            continue;
        }
        let Ok(offset) = u8::try_from(offset) else {
            continue;
        };
        let Some(slot) = base.checked_add(offset) else {
            continue;
        };

        if let Some(id) = temp_sensor_id_for(node_id, slot) {
            let _ = out.push(SensorReading::Temperature(id, f32::from(raw) / 100.0));
        } else if let Some(id) = press_sensor_id_for(node_id, slot) {
            let _ = out.push(SensorReading::Pressure(id, f32::from(raw) / 100.0));
        }
    }

    out
}

fn temp_sensor_id_for(node_id: u8, slot: u8) -> Option<TempSensId> {
    TEMP_SENSOR_ID_MAP
        .iter()
        .find(|(_, addr)| addr.node_id == node_id && addr.slot == slot)
        .map(|(id, _)| id)
}

fn press_sensor_id_for(node_id: u8, slot: u8) -> Option<PressSensId> {
    PRESS_SENSOR_ID_MAP
        .iter()
        .find(|(_, addr)| addr.node_id == node_id && addr.slot == slot)
        .map(|(id, _)| id)
}
