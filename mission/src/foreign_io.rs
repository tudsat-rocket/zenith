use crate::propulsion::{
    ALL_BINARY_OUTPUTS, ALL_PRESS_SENS, ALL_TEMP_SENS, ALL_VALVES, BinaryOutputId, PressSensId,
    TempSensId, ValveId,
};
use core::num::Wrapping;

#[derive(Clone)]
pub struct ForeignInputImage {
    pub temp_sens: [Option<DataWithTime<f32>>; ALL_TEMP_SENS.len()],
    pub press_sens: [Option<DataWithTime<f32>>; ALL_PRESS_SENS.len()],
    pub valve_state: [Option<DataWithTime<ValveState>>; ALL_VALVES.len()],
    pub binary_outputs: [Option<DataWithTime<bool>>; ALL_BINARY_OUTPUTS.len()],
    pub ox_tank_level: Option<DataWithTime<f32>>,
}
impl ForeignInputImage {
    pub const fn default() -> Self {
        Self {
            temp_sens: [None; ALL_TEMP_SENS.len()],
            press_sens: [None; ALL_PRESS_SENS.len()],
            valve_state: [None; ALL_VALVES.len()],
            binary_outputs: [None; ALL_BINARY_OUTPUTS.len()],
            ox_tank_level: None,
        }
    }
    /// getter
    pub fn valve_state(&self, valve: ValveId) -> &Option<DataWithTime<ValveState>> {
        // FIXME: valve_id is 1 indexed
        &self.valve_state[valve as usize - 1]
    }
    // pub fn set_valve(&mut self, valve: ValveId) {
    //     // FIXME: valve_id is 1 indexed
    //     self.valve_state[valve as usize - 1]
    // }
}

// NOTE: not sure if deriving Copy is correct
#[derive(Clone, Copy)]
pub struct DataWithTime<T> {
    pub data: T,
    pub time: Wrapping<u32>,
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

#[derive(Copy, Clone, PartialEq, Eq)]
pub struct ForeignOutputImage {
    pub dirty: bool,
    pub valve: [ValveState; ALL_VALVES.len()],
    pub binary_output: [bool; ALL_BINARY_OUTPUTS.len()],
}
impl ForeignOutputImage {
    // NOTE: adjust defaults
    pub const fn default() -> Self {
        Self {
            dirty: true,
            valve: [ValveState::fully_closed(); ALL_VALVES.len()],
            binary_output: [false; ALL_BINARY_OUTPUTS.len()],
        }
    }
    /// helper
    pub fn set_valve(&mut self, id: ValveId, state: ValveState) {
        self.dirty = true;
        // FIXME: valve_id is 1 indexed currently
        self.valve[id as usize - 1] = state;
    }
    pub fn get_valve(&self, id: ValveId) -> ValveState {
        // FIXME: valve_id is 1 indexed currently
        self.valve[id as usize - 1]
    }

    /// helper
    pub fn set_binary_output(&mut self, id: BinaryOutputId, state: bool) {
        self.dirty = true;
        self.binary_output[id as usize] = state;
    }
    /// getter
    pub const fn valves(&self) -> &[ValveState; ALL_VALVES.len()] {
        &self.valve
    }
    pub const fn binary_outpus(&self) -> &[bool; ALL_BINARY_OUTPUTS.len()] {
        &self.binary_output
    }
}

pub trait ForeignIo {
    fn tick(&mut self);
    fn get_input_image(&mut self) -> ForeignInputImage;
    fn set_output_image(&mut self, outputs: ForeignOutputImage);
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub struct ValveState {
    /// 0 = fully closed, 1000 = fully open
    promille: u16,
}

impl ValveState {
    /// 0 = fully closed, 100 = fully open
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

impl IoAddr {
    pub const fn new(board_id: u8, index: u16, subindex: u8) -> Self {
        IoAddr {
            node_id: board_id,
            index,
            subindex,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum ForeignDataError {
    UnexpectedFormat,
    IoAddrNotMapped,
    OutOfRange,
}
