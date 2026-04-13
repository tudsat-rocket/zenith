use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, AdcChannel, Instance, SampleTime, Temperature, VrefInt};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker, Timer};

use mission::AdcData;

use crate::BoardAdc;

const VSENSE_DIVIDER: u64 = (100 + 10) / 10;

const SAMPLING_RATE_HZ: u64 = 100;

static SIGNAL: Signal<CriticalSectionRawMutex, AdcData> = Signal::new();

#[derive(Default)]
pub struct PowerMonitor {
    history: heapless::Deque<AdcData, 20>,
    filtered: AdcData,
}

pub fn init(adc: BoardAdc, spawner: Spawner) -> PowerMonitor {
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

        let temperature = read_buffer[1] as i32;

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
            temperature,
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
        for data in &self.history {
            bus_main_voltage += data.bus_main_voltage as u64;
            bus_supply_voltage += data.bus_supply_voltage as u64;
            fc_current += data.fc_current as i64;
            recovery_voltage += data.recovery_voltage as u64;
            recovery_current += data.recovery_current as i64;
        }

        let len = self.history.len() as u64;
        self.filtered = AdcData {
            bus_main_voltage: (bus_main_voltage / len) as u16,
            bus_supply_voltage: (bus_supply_voltage / len) as u16,
            fc_current: (fc_current / (len as i64)) as i32,
            recovery_voltage: (recovery_voltage / len) as u16,
            recovery_current: (recovery_current / (len as i64)) as i32,
            temperature: 0,
        }
    }

    pub fn adc(&self) -> Option<AdcData> {
        Some(self.filtered.clone())
    }
}
