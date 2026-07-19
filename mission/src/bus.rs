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
