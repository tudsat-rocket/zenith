#![allow(
    clippy::unwrap_used,
    reason = "boot-time baro setup / non-empty sample buffers"
)]
#![allow(
    clippy::arithmetic_side_effects,
    reason = "datasheet fixed-point pressure/temperature compensation math"
)]
#![allow(
    clippy::indexing_slicing,
    reason = "fixed-size PROM/sample buffer access"
)]

use heapless::{Deque, Vec};

use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::SpiDevice;

use num_traits::float::Float;

use defmt::*;

const BARO_MEDIAN_FILTER_LENGTH: usize = 20;

/// How long the PROM takes to reload after a reset (2.8 ms in the datasheet). Reading it any
/// earlier returns partially loaded coefficients, which can look perfectly plausible.
const RESET_TIME: Duration = Duration::from_millis(3);

const INIT_ATTEMPTS: usize = 3;

/// Highest pressure, in 0.01mbar, that we expect to see from an MS5607 that is
/// (incorrectly) evaluated as an MS5611, which halves the reported value.
const MS5607_MAX_PRESSURE: i32 = 54_000;

#[derive(PartialEq, Clone, Copy, defmt::Format)]
pub enum MS56Variant {
    MS5607,
    MS5611,
}

struct MS56CalibrationData {
    pressure_sensitivity: u16,
    pressure_offset: u16,
    temp_coef_pressure_sensitivity: u16,
    temp_coef_pressure_offset: u16,
    reference_temperature: u16,
    temp_coef_temperature: u16,
}

impl MS56CalibrationData {
    /// The coefficients in the 8 PROM words, if they pass the CRC4 in the low nibble of word 7
    /// (application note AN520) and none of them is stuck at 0x0000 or 0xffff, which the CRC
    /// alone would not catch for an all-zero PROM.
    fn from_prom(prom: [u16; 8]) -> Option<Self> {
        let mut words = prom;
        words[7] &= 0xff00;

        let mut remainder: u16 = 0;
        for byte in words.iter().flat_map(|word| word.to_be_bytes()) {
            remainder ^= u16::from(byte);
            for _ in 0..8 {
                remainder = if remainder & 0x8000 == 0 {
                    remainder << 1
                } else {
                    (remainder << 1) ^ 0x3000
                };
            }
        }
        if remainder >> 12 != prom[7] & 0x000f {
            return None;
        }

        let data = Self {
            pressure_sensitivity: prom[1],
            pressure_offset: prom[2],
            temp_coef_pressure_sensitivity: prom[3],
            temp_coef_pressure_offset: prom[4],
            reference_temperature: prom[5],
            temp_coef_temperature: prom[6],
        };
        data.valid().then_some(data)
    }

    fn valid(&self) -> bool {
        // We assume that every value needs to be non-zero and non-0xffff.
        self.pressure_sensitivity != 0x0000
            && self.pressure_offset != 0x0000
            && self.temp_coef_pressure_sensitivity != 0x0000
            && self.temp_coef_pressure_offset != 0x0000
            && self.reference_temperature != 0x0000
            && self.temp_coef_temperature != 0x0000
            && self.pressure_sensitivity != 0xffff
            && self.pressure_offset != 0xffff
            && self.temp_coef_pressure_sensitivity != 0xffff
            && self.temp_coef_pressure_offset != 0xffff
            && self.reference_temperature != 0xffff
            && self.temp_coef_temperature != 0xffff
    }
}

pub struct MS56<SPI: SpiDevice<u8>> {
    variant: MS56Variant,
    spi: SPI,
    calibration_data: Option<MS56CalibrationData>,
    read_temp: bool,
    dt: Option<i32>,
    temp: Option<i32>,
    raw_pressure: Option<i32>,
    pressure: Option<i32>,
    baro_filter: BaroFilter,
}

impl<SPI: SpiDevice<u8>> MS56<SPI> {
    pub async fn init(spi: SPI) -> Result<Self, SPI::Error> {
        let mut baro = Self {
            variant: MS56Variant::MS5611, // fallback if detection fails
            spi,
            calibration_data: None,
            read_temp: true,
            dt: None,
            temp: None,
            raw_pressure: None,
            pressure: None,
            baro_filter: BaroFilter::default(),
        };

        for _i in 0..INIT_ATTEMPTS {
            baro.reset().await?;
            Timer::after(RESET_TIME).await;

            let prom = baro.read_prom().await?;
            baro.calibration_data = MS56CalibrationData::from_prom(prom);
            if baro.calibration_data.is_some() {
                break;
            }
            warn!("MS56xx PROM failed its CRC: {:#06x}", prom);
        }

        if baro.calibration_data.is_some() {
            baro.detect_variant().await?;
            info!("MS56xx initialized ({})", baro.variant);
        } else {
            error!("Failed to initialize MS56xx, it will not report");
        }

        Ok(baro)
    }

    /// Distinguishes the two variants by taking a sample. Since we evaluate it
    /// as an MS5611, an MS5607 will report roughly half the true pressure.
    async fn detect_variant(&mut self) -> Result<(), SPI::Error> {
        for _i in 0..3 {
            for read_temp in [true, false] {
                self.read_temp = read_temp;
                self.start_next_conversion().await?;
                Timer::after(Duration::from_millis(1)).await;
                self.read_sensor_data().await?;
            }

            if let Some(pressure) = self.pressure {
                if pressure <= MS5607_MAX_PRESSURE {
                    self.variant = MS56Variant::MS5607;
                }
                return Ok(());
            }
        }

        error!("Failed to detect MS56xx variant, assuming {}", self.variant);
        Ok(())
    }

    async fn command(
        &mut self,
        command: MS56Command,
        response_len: usize,
    ) -> Result<Vec<u8, 32>, SPI::Error> {
        let mut payload = [0x00; 32];
        payload[0] = command.into();
        self.spi
            .transfer_in_place(&mut payload[..=response_len])
            .await?;
        Ok(Vec::from_slice(&payload[1..=response_len]).unwrap_or_default())
    }

    async fn reset(&mut self) -> Result<(), SPI::Error> {
        self.command(MS56Command::Reset, 0).await?;
        Ok(())
    }

    async fn read_prom(&mut self) -> Result<[u16; 8], SPI::Error> {
        let mut prom = [0; 8];
        for (address, word) in (0..).zip(prom.iter_mut()) {
            let bytes = self.command(MS56Command::ReadProm(address), 2).await?;
            *word = u16::from_be_bytes([bytes[0], bytes[1]]);
        }
        Ok(prom)
    }

    async fn read_sensor_data(&mut self) -> Result<(), SPI::Error> {
        if self.calibration_data.is_none() {
            return Ok(());
        }

        let response = self.command(MS56Command::ReadAdc, 3).await?;
        // when read to fast after starting conversion, conversion is corrupted and data all 0
        if response.iter().all(|&b| b == 0) {
            debug!("invalid barometer response, polled to soon");
            return Ok(());
        }
        let value =
            ((response[0] as i32) << 16) | ((response[1] as i32) << 8) | (response[2] as i32);
        let Some(cal) = self.calibration_data.as_ref() else {
            return Ok(());
        };

        if self.read_temp {
            let dt = (value as i32) - ((cal.reference_temperature as i32) << 8);
            let filtered = self.baro_filter.filter(dt);
            self.dt = Some(filtered);
        } else {
            self.raw_pressure = Some(value);
        }

        if let Some((dt, raw_pressure)) = self.dt.zip(self.raw_pressure) {
            let mut temp = 2000 + (((dt as i64) * (cal.temp_coef_temperature as i64)) >> 23);

            let mut offset = ((cal.pressure_offset as i64) << 16)
                + ((cal.temp_coef_pressure_offset as i64 * dt as i64) >> 7);
            let mut sens = ((cal.pressure_sensitivity as i64) << 15)
                + (((cal.temp_coef_pressure_sensitivity as i64) * (dt as i64)) >> 8);

            // Second order temp compensation. Like OFF and SENS above, the
            // MS5607 coefficients are halved compared to its datasheet, since
            // we calculate at the MS5611's scale and shift once less below.
            if temp < 2000 {
                let t2 = ((dt as i64) * (dt as i64)) >> 31;
                let temp_offset = temp - 2000;
                let (mut off2, mut sens2) = match self.variant {
                    MS56Variant::MS5607 => (
                        (61 * temp_offset * temp_offset) >> 5,
                        temp_offset * temp_offset,
                    ),
                    MS56Variant::MS5611 => {
                        let off2 = (5 * temp_offset * temp_offset) >> 1;
                        (off2, off2 >> 1)
                    }
                };

                if temp < -1500 {
                    // brrrr
                    let temp_offset = temp + 1500;
                    let (off2_extra, sens2_extra) = match self.variant {
                        MS56Variant::MS5607 => (
                            (15 * temp_offset * temp_offset) >> 1,
                            4 * temp_offset * temp_offset,
                        ),
                        MS56Variant::MS5611 => (
                            7 * temp_offset * temp_offset,
                            (11 * temp_offset * temp_offset) >> 1,
                        ),
                    };
                    off2 += off2_extra;
                    sens2 += sens2_extra;
                }

                temp -= t2;
                offset -= off2;
                sens -= sens2;
            }

            self.temp = Some(temp as i32);

            let shift = match self.variant {
                MS56Variant::MS5607 => 14,
                MS56Variant::MS5611 => 15,
            };
            let p = (((raw_pressure as i64 * sens) >> 21) - offset) >> shift;
            self.pressure = Some(p as i32);
        }

        Ok(())
    }

    async fn start_next_conversion(&mut self) -> Result<(), SPI::Error> {
        let osr = MS56OSR::OSR256;
        if self.read_temp {
            self.command(MS56Command::StartTempConversion(osr), 0)
                .await?;
        } else {
            self.command(MS56Command::StartPressureConversion(osr), 0)
                .await?;
        }
        Ok(())
    }

    pub async fn tick(&mut self) {
        if let Err(_) = self.read_sensor_data().await {
            mission::health::SENSOR_ERRORS.record();
            error!("baro spi error, read_sensor_data");
            self.dt = None;
            self.temp = None;
            self.raw_pressure = None;
            self.pressure = None;
            self.read_temp = true;
        } else {
            self.read_temp = !self.read_temp;
        }

        if let Err(_) = self.start_next_conversion().await {
            mission::health::SENSOR_ERRORS.record();
            error!("baro spi error, start_next_conversion");
            self.dt = None;
            self.temp = None;
            self.raw_pressure = None;
            self.pressure = None;
            self.read_temp = true;
        }
    }

    pub fn temperature(&self) -> Option<f32> {
        self.temp.map(|t| (t as f32) / 100.0)
    }

    pub fn pressure(&self) -> Option<f32> {
        self.pressure.map(|p| (p as f32) / 100.0)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum MS56Command {
    Reset,
    StartPressureConversion(MS56OSR),
    StartTempConversion(MS56OSR),
    ReadAdc,
    ReadProm(u8),
}

impl From<MS56Command> for u8 {
    fn from(cmd: MS56Command) -> u8 {
        match cmd {
            MS56Command::Reset => 0x1e,
            MS56Command::StartPressureConversion(osr) => 0x40 + ((osr as u8) << 1),
            MS56Command::StartTempConversion(osr) => 0x50 + ((osr as u8) << 1),
            MS56Command::ReadAdc => 0x00,
            MS56Command::ReadProm(adr) => 0xa0 + (adr << 1),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum MS56OSR {
    OSR256 = 0b000,
    OSR512 = 0b001,
    OSR1024 = 0b010,
    OSR2048 = 0b011,
    OSR4096 = 0b100,
}

#[derive(Default)]
pub struct BaroFilter {
    previous_raw_values: Deque<i32, BARO_MEDIAN_FILTER_LENGTH>,
    last_spike_warning_counter: u32,
}

impl BaroFilter {
    //median filter
    pub fn filter(&mut self, input_value: i32) -> i32 {
        const SPIKE_WARNING_THRESHOLD: i32 = 8_000_000;

        while self.previous_raw_values.len() > (BARO_MEDIAN_FILTER_LENGTH - 1) {
            let _ = self.previous_raw_values.pop_front();
        }

        let _ = self.previous_raw_values.push_back(input_value);

        let mut sorted: Vec<_, BARO_MEDIAN_FILTER_LENGTH> =
            self.previous_raw_values.iter().collect();
        sorted.sort_unstable();

        if self.last_spike_warning_counter <= 100 {
            self.last_spike_warning_counter += 1;
        }

        let diff = *sorted.last().unwrap() - *sorted.first().unwrap();
        if diff > SPIKE_WARNING_THRESHOLD && self.last_spike_warning_counter > 100 {
            defmt::warn!("Baro temp spike: {}", diff);
            self.last_spike_warning_counter = 0;
        }

        *sorted[sorted.len() / 2]
    }
}
