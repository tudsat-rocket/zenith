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
    /// What the flight computer reports about *itself* on the vehicle bus. Not
    /// an output in the sense the other two are — nothing acts on it — but it
    /// leaves through the same path, and putting it here is what lets the bus
    /// layer stay the only thing that knows the wire format.
    pub temperature: FcTemperature,
}

/// The flight computer's own temperatures, in millidegrees Celsius.
///
/// Millidegrees in `i32` rather than degrees in `f32` for two reasons: it is the
/// unit the vehicle bus carries, and `BusOutputImage` is compared with `Eq` to
/// decide whether a value is worth a CAN frame, which a float cannot do.
#[derive(Copy, Clone, PartialEq, Eq, Default, Debug)]
pub struct FcTemperature {
    /// Ambient, taken from a barometer's own die sensor — the flight computer
    /// has no dedicated board thermistor the way an IO board does.
    pub baro_milli_c: Option<i32>,
    /// The MCU die sensor.
    pub mcu_milli_c: Option<i32>,
}

/// Degrees Celsius to the millidegrees the bus carries.
///
/// Saturating rather than wrapping: a barometer that reports a nonsense
/// temperature should pin the reading, not wrap it around to the other end of
/// the scale where it looks plausible again.
pub fn celsius_to_milli_c(celsius: f32) -> i32 {
    let milli = celsius * 1000.0;
    if milli >= i32::MAX as f32 {
        i32::MAX
    } else if milli <= i32::MIN as f32 {
        i32::MIN
    } else {
        milli as i32
    }
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
            // `Default` is not const, so this is spelled out.
            temperature: FcTemperature {
                baro_milli_c: None,
                mcu_milli_c: None,
            },
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
