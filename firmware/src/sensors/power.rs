#![allow(
    clippy::unwrap_used,
    reason = "boot-time ADC setup; panic-on-failure is the embedded model"
)]
#![allow(
    clippy::arithmetic_side_effects,
    reason = "fixed-point ADC-to-engineering-unit conversion math"
)]
#![allow(clippy::indexing_slicing, reason = "fixed ADC channel arrays")]

use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, AdcChannel, Instance, SampleTime, Temperature, VrefInt};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker, Timer};

use mission::AdcData;

use crate::BoardAdc;

const VSENSE_DIVIDER: u64 = (100 + 10) / 10;

const SAMPLING_RATE_HZ: u64 = 100;

/// Factory temperature-sensor calibration, programmed into system memory during
/// production. Both are raw 16-bit ADC readings of the die sensor taken at a
/// known temperature with VDDA = 3.3 V, which is what makes the STM32H7's
/// internal sensor worth reading at all — unlike the uncalibrated one on the IO
/// boards' STM32F1, this one is good to a couple of degrees.
///
/// Addresses are from the STM32H743 datasheet ("Temperature sensor
/// characteristics"). [`ts_cal`] refuses to use them unless they look like real
/// calibration data, so a wrong address here reports "no reading" rather than a
/// confident wrong temperature.
const TS_CAL1_ADDR: *const u16 = 0x1FF1_E820 as *const u16;
const TS_CAL2_ADDR: *const u16 = 0x1FF1_E840 as *const u16;
/// Raw VREFINT reading taken during the same factory calibration, used to
/// correct this board's readings back to the VDDA the calibration assumed.
const VREFINT_CAL_ADDR: *const u16 = 0x1FF1_E860 as *const u16;

/// The temperatures [`TS_CAL1_ADDR`] and [`TS_CAL2_ADDR`] were measured at.
const TS_CAL1_TEMP_C: i32 = 30;
const TS_CAL2_TEMP_C: i32 = 110;

/// The three factory calibration words, or `None` if they do not look
/// programmed.
///
/// An erased or unprogrammed word reads `0xFFFF`, and a valid pair must have
/// `TS_CAL2 > TS_CAL1` because the sensor's output rises with temperature. Both
/// checks exist so that reading the wrong address — a different package, a
/// different part, a transcription error — produces nothing rather than a
/// number that looks like a temperature.
fn ts_cal() -> Option<(i32, i32, i32)> {
    // SAFETY: all three are read-only words in the system memory region, mapped
    // and valid on every STM32H743, and read as plain `u16`.
    let (cal1, cal2, vref_cal) = unsafe {
        (
            TS_CAL1_ADDR.read_volatile(),
            TS_CAL2_ADDR.read_volatile(),
            VREFINT_CAL_ADDR.read_volatile(),
        )
    };

    if cal1 == 0xFFFF || cal2 == 0xFFFF || vref_cal == 0xFFFF || cal2 <= cal1 || vref_cal == 0 {
        return None;
    }

    Some((i32::from(cal1), i32::from(cal2), i32::from(vref_cal)))
}

/// Die temperature in millidegrees Celsius from this tick's raw sensor and
/// VREFINT readings.
///
/// The datasheet's two-point form, `T = (110 - 30) / (CAL2 - CAL1) * (raw -
/// CAL1) + 30`, with one correction in front of it: the calibration was taken at
/// VDDA = 3.3 V and this board's VDDA is whatever its regulator actually
/// produces. An ADC reading is a ratio against VDDA, so scaling the raw count by
/// `VREFINT_CAL / vrefint` puts it back on the calibration's scale. VREFINT is
/// already being sampled in the same sequence, so this costs nothing.
///
/// Both readings are 16-bit because the ADC is left at its reset resolution,
/// which is also the resolution the calibration words were taken at. The
/// voltage maths below divides by 65536 on the same assumption.
///
/// Widened to `i64` throughout, not for precision but because the obvious `i32`
/// version overflows: the slope numerator alone is 80_000 millidegrees times a
/// count difference that can reach 65_535, which is 5.2e9. An overflow here
/// would be a wrong temperature rather than a crash, so the width is the only
/// thing preventing it.
fn die_temperature_milli_c(raw: u16, vrefint: u16) -> Option<i32> {
    let (cal1, cal2, vref_cal) = ts_cal()?;
    if vrefint == 0 {
        return None;
    }

    let corrected = (i64::from(raw) * i64::from(vref_cal)) / i64::from(vrefint);
    let span_milli_c = i64::from(TS_CAL2_TEMP_C - TS_CAL1_TEMP_C) * 1000;
    let slope_divisor = i64::from(cal2 - cal1);

    let milli_c = i64::from(TS_CAL1_TEMP_C) * 1000
        + (span_milli_c * (corrected - i64::from(cal1))) / slope_divisor;

    // A reading this far outside the part's rated range is a broken measurement,
    // not a hot board, and saying nothing is more useful than clamping it to
    // something that reads as real.
    (-60_000..=200_000)
        .contains(&milli_c)
        .then_some(milli_c as i32)
}

static SIGNAL: Signal<CriticalSectionRawMutex, AdcData> = Signal::new();

#[derive(Default)]
pub struct PowerMonitor {
    history: heapless::Deque<AdcData, 20>,
    filtered: AdcData,
}

pub fn spawn(adc: BoardAdc, spawner: Spawner) -> PowerMonitor {
    spawner.spawn(run(adc)).unwrap();

    PowerMonitor::default()
}

#[unsafe(link_section = ".ram_d3")]
static mut DMA_BUF: [u16; 7] = [0; 7];

#[embassy_executor::task]
async fn run(mut adc: BoardAdc) -> ! {
    let read_buffer = unsafe { &mut DMA_BUF[..] };

    let mut vrefint = adc.adc1.enable_vrefint().degrade_adc();
    let mut temperature = adc.adc1.enable_temperature().degrade_adc();

    let mut ticker = Ticker::every(Duration::from_millis(1000 / SAMPLING_RATE_HZ));

    loop {
        adc.adc1
            .read(
                adc.dma.reborrow(),
                [
                    (&mut vrefint, SampleTime::CYCLES64_5),
                    (&mut temperature, SampleTime::CYCLES64_5),
                    (&mut adc.main_voltage, SampleTime::CYCLES64_5),
                    (&mut adc.supply_voltage, SampleTime::CYCLES64_5),
                    (&mut adc.main_current, SampleTime::CYCLES810_5),
                    (&mut adc.recovery_voltage, SampleTime::CYCLES64_5),
                    (&mut adc.recovery_current, SampleTime::CYCLES810_5),
                ]
                .into_iter(),
                read_buffer,
            )
            .await;

        let temperature_milli_c = die_temperature_milli_c(read_buffer[1], read_buffer[0]);

        let bus_main_voltage = VSENSE_DIVIDER * 3300 * (read_buffer[2] as u64) / 65536;
        let bus_supply_voltage = VSENSE_DIVIDER * 3300 * (read_buffer[3] as u64) / 65536;
        let fc_current = (33000 * (read_buffer[4] as u64)) / 65536;

        let recovery_voltage = VSENSE_DIVIDER * 3300 * (read_buffer[5] as u64) / 65536;
        let recovery_current = (33000 * (read_buffer[6] as u64)) / 65536;

        let data = AdcData {
            bus_main_voltage: bus_main_voltage as u16,
            bus_supply_voltage: bus_supply_voltage as u16,
            fc_current: fc_current as i32,
            //fc_current: read_buffer[4] as i32,
            recovery_voltage: recovery_voltage as u16,
            recovery_current: recovery_current as i32,
            //recovery_current: read_buffer[6] as i32,
            temperature_milli_c,
        };
        SIGNAL.signal(data);

        ticker.next().await;
    }
}

impl PowerMonitor {
    pub fn tick(&mut self) {
        if let Some(data) = SIGNAL.try_take() {
            if self.history.is_full() {
                let _ = self.history.pop_front();
            }
            let _ = self.history.push_back(data);
        } else if self.history.is_empty() {
            return;
        }

        let mut bus_main_voltage: u64 = 0;
        let mut bus_supply_voltage: u64 = 0;
        let mut fc_current: i64 = 0;
        let mut recovery_voltage: u64 = 0;
        let mut recovery_current: i64 = 0;
        // Averaged over however many samples actually produced a temperature,
        // rather than over the whole window: without the calibration words there
        // is no reading at all, and counting those as zero would report a board
        // sitting at 0 C.
        let mut temperature_sum: i64 = 0;
        let mut temperature_count: i64 = 0;
        for data in &self.history {
            bus_main_voltage += data.bus_main_voltage as u64;
            bus_supply_voltage += data.bus_supply_voltage as u64;
            fc_current += data.fc_current as i64;
            recovery_voltage += data.recovery_voltage as u64;
            recovery_current += data.recovery_current as i64;
            if let Some(t) = data.temperature_milli_c {
                temperature_sum += t as i64;
                temperature_count += 1;
            }
        }

        let len = self.history.len() as u64;
        self.filtered = AdcData {
            bus_main_voltage: (bus_main_voltage / len) as u16,
            bus_supply_voltage: (bus_supply_voltage / len) as u16,
            fc_current: (fc_current / (len as i64)) as i32,
            recovery_voltage: (recovery_voltage / len) as u16,
            recovery_current: (recovery_current / (len as i64)) as i32,
            temperature_milli_c: (temperature_count > 0)
                .then(|| (temperature_sum / temperature_count) as i32),
        }
    }

    pub fn adc(&self) -> Option<AdcData> {
        Some(self.filtered.clone())
    }
}
