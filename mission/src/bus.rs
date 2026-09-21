use core::num::Wrapping;

use crate::inventory::{BinaryOutputMap, PressureSensorMap, TemperatureSensorMap, ValveMap};

pub trait Bus {
    fn get_input_image(&mut self) -> BusInputImage;
    fn set_output_image(&mut self, outputs: BusOutputImage);
}

/// A `Bus` with nothing attached: inputs stay unknown, outputs go nowhere.
/// Stand-in for builds without IO boards (e.g. the solid-rocket SITL).
pub struct NoBus;

impl Bus for NoBus {
    fn get_input_image(&mut self) -> BusInputImage {
        BusInputImage::default()
    }

    fn set_output_image(&mut self, _outputs: BusOutputImage) {}
}

#[derive(Clone, Copy)]
pub struct DataWithTime<T> {
    pub data: T,
    pub time: Wrapping<u32>,
}

#[derive(Clone)]
pub struct BusInputImage {
    pub temp_sens: TemperatureSensorMap<Option<DataWithTime<f32>>>,
    pub press_sens: PressureSensorMap<Option<DataWithTime<f32>>>,
    pub valve_state: ValveMap<Option<DataWithTime<ValveState>>>,
    pub binary_outputs: BinaryOutputMap<Option<DataWithTime<bool>>>,
    pub ox_tank_level: Option<DataWithTime<f32>>,
    pub nodes: NodeSet,
    /// A subset of `nodes`: a board we cannot hear from tells us nothing.
    pub nodes_armed: NodeSet,
}

/// The IO board protocol's node id field is four bits wide.
pub const NODE_ID_COUNT: usize = 16;

const _: () = assert!(
    links::SELF_COMPONENT_ID == 1,
    "IO node ids start at 2 because 0 is MAVLink's \"all components\" and 1 is zenith itself"
);

/// The node ids zenith reports as MAVLink components.
#[expect(
    clippy::indexing_slicing,
    reason = "const-evaluated fill of a fixed-length array, bounded by the array's own length"
)]
pub const IO_NODE_IDS: [u8; NODE_ID_COUNT - 2] = {
    let mut ids = [0; NODE_ID_COUNT - 2];
    let mut i = 0;
    while i < ids.len() {
        ids[i] = (i + 2) as u8;
        i += 1;
    }
    ids
};

/// A set of IO board node ids. Bit n is node id n, which is also the LoRa downlink's encoding,
/// so [`Self::bits`] goes straight onto the wire.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NodeSet(u16);

const _: () = assert!(NODE_ID_COUNT == u16::BITS as usize);

impl NodeSet {
    pub const NONE: Self = Self(0);

    pub const fn from_bits(bits: u16) -> Self {
        Self(bits)
    }

    pub const fn bits(self) -> u16 {
        self.0
    }

    pub fn contains(self, node_id: u8) -> bool {
        Self::bit(node_id).is_some_and(|bit| self.0 & bit != 0)
    }

    pub fn set(&mut self, node_id: u8, member: bool) {
        let Some(bit) = Self::bit(node_id) else {
            return;
        };

        if member {
            self.0 |= bit;
        } else {
            self.0 &= !bit;
        }
    }

    #[must_use]
    pub const fn intersection(self, other: Self) -> Self {
        Self(self.0 & other.0)
    }

    fn bit(node_id: u8) -> Option<u16> {
        1u16.checked_shl(u32::from(node_id))
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub struct BusOutputImage {
    pub valve: ValveMap<ValveState>,
    pub binary_output: BinaryOutputMap<bool>,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub struct ValveState {
    /// 0 = fully closed, 1000 = fully open
    promille: u16,
}

/// Address for CanOpen sdo request
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct IoAddr {
    /// id of the io board
    pub node_id: u8,
    /// index into the CanOpen store
    pub index: u16,
    /// subindex for array objects in the store
    pub subindex: u8,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum BusDataError {
    UnexpectedFormat,
    IoAddrNotMapped,
    OutOfRange,
}

impl<T: Copy> DataWithTime<T> {
    pub fn new(data: T, time: Wrapping<u32>) -> Self {
        DataWithTime { data, time }
    }

    pub fn injest(&mut self, data: T, time: Wrapping<u32>) {
        self.data = data;
        self.time = time;
    }
}

impl BusInputImage {
    pub const fn default() -> Self {
        Self {
            temp_sens: TemperatureSensorMap::splat(None),
            press_sens: PressureSensorMap::splat(None),
            valve_state: ValveMap::splat(None),
            binary_outputs: BinaryOutputMap::splat(None),
            ox_tank_level: None,
            nodes: NodeSet::NONE,
            nodes_armed: NodeSet::NONE,
        }
    }
}

impl BusOutputImage {
    pub const fn default() -> Self {
        Self {
            valve: ValveMap::splat(ValveState::fully_closed()),
            binary_output: BinaryOutputMap::splat(false),
        }
    }
}

impl ValveState {
    /// 0 = fully closed, 100 = fully open
    #[allow(
        clippy::arithmetic_side_effects,
        reason = "value is clamped to <= 100, so value * 10 <= 1000 fits u16"
    )]
    pub const fn from_percent_open(value: u16) -> Self {
        // clamp ist not const yet
        let value = { if value > 100 { 100 } else { value } };
        Self {
            promille: value * 10,
        }
    }
    pub const fn fully_open() -> Self {
        Self { promille: 1000 }
    }
    pub const fn fully_closed() -> Self {
        Self { promille: 0 }
    }
    pub const fn set_fully_open(&mut self) {
        self.promille = 1000;
    }
    pub const fn set_fully_closed(&mut self) {
        self.promille = 0;
    }
    pub const fn from_promille_clamped(promille: u16) -> Self {
        // clamp is not const
        let promille = if promille > 1000 { 1000 } else { promille };
        Self { promille }
    }
    /// getter
    /// 0 = fully closed, 1000 = fully open
    pub fn promille(&self) -> u16 {
        self.promille
    }
}

impl IoAddr {
    pub const fn new(board_id: u8, index: u16, subindex: u8) -> Self {
        IoAddr {
            node_id: board_id,
            index,
            subindex,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A bit landing next door would move a board.
    #[test]
    fn presence_bits_do_not_overlap() {
        for subject in 0..NODE_ID_COUNT as u8 {
            let mut nodes = NodeSet::NONE;
            nodes.set(subject, true);

            for other in 0..NODE_ID_COUNT as u8 {
                assert_eq!(
                    nodes.contains(other),
                    other == subject,
                    "node {subject} leaked into {other}"
                );
            }

            nodes.set(subject, false);
            assert_eq!(nodes, NodeSet::NONE, "node {subject} would not clear");
        }
    }

    #[test]
    fn ids_off_the_bus_are_never_present() {
        let mut nodes = NodeSet::from_bits(u16::MAX);

        for node_id in [NODE_ID_COUNT as u8, 100, u8::MAX] {
            assert!(!nodes.contains(node_id));

            nodes.set(node_id, true);
            assert_eq!(nodes.bits(), u16::MAX, "setting {node_id} moved a real bit");
        }
    }
}
