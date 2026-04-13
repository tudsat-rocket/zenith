#![no_std]
#![allow(unused_variables)]
#![allow(unused_imports)]

use embassy_stm32::adc::{Adc, AnyAdcChannel};
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::Spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_futures::join::{join3, join5};

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{Sx126x, Sx1262};

use mission::{BaroReading, NoStorage, Outputs, SensorReadings, Sensors};

use sensors::*;

pub mod board;
pub mod can;
pub mod links;
pub mod sensors;

bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => embassy_stm32::can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => embassy_stm32::can::IT1InterruptHandler<FDCAN1>;
    FDCAN2_IT0 => embassy_stm32::can::IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => embassy_stm32::can::IT1InterruptHandler<FDCAN2>;
    OTG_FS => embassy_stm32::usb::InterruptHandler<USB_OTG_FS>;
    ETH => embassy_stm32::eth::InterruptHandler;
    RNG => embassy_stm32::rng::InterruptHandler<embassy_stm32::peripherals::RNG>;
});

pub type OurSpiDevice<'a> = SpiDevice<'a, CriticalSectionRawMutex, Spi<'static, Async>, Output<'a>>;
pub type LoraVariant = GenericSx126xInterfaceVariant<Output<'static>, ExtiInput<'static>>;
pub type LoraTransceiver = Sx126x<OurSpiDevice<'static>, LoraVariant, Sx1262>;

pub type Vehicle = mission::Vehicle<BoardSensors, BoardOutputs, NoStorage>;

pub struct BoardSensors {
    pub imu1: LSM6<OurSpiDevice<'static>>,
    pub imu2: ICM42688P<OurSpiDevice<'static>>,
    pub imu3: ICM42670P<OurSpiDevice<'static>>,
    pub highg: H3LIS331DL<OurSpiDevice<'static>>,
    pub mag: LIS3MDL<OurSpiDevice<'static>>,
    pub baro1: MS56<OurSpiDevice<'static>>,
    pub baro2: LPS22<OurSpiDevice<'static>>,
    pub baro3: BMP580<OurSpiDevice<'static>>,
    pub power: PowerMonitor,
}

pub struct BoardOutputs {
    pub leds: (Output<'static>, Output<'static>, Output<'static>),
    pub recovery_high: Output<'static>,
    pub recovery_lows: (
        Output<'static>,
        Output<'static>,
        Output<'static>,
        Output<'static>,
    ),
}

pub struct BoardAdc {
    pub dma: embassy_stm32::Peri<'static, DMA2_CH7>,
    pub adc1: Adc<'static, ADC1>,
    pub adc2: Adc<'static, ADC2>,
    pub adc3: Adc<'static, ADC3>,
    pub main_voltage: AnyAdcChannel<ADC1>,
    pub supply_voltage: AnyAdcChannel<ADC1>,
    pub recovery_voltage: AnyAdcChannel<ADC1>,
    pub main_current: AnyAdcChannel<ADC1>,
    pub recovery_current: AnyAdcChannel<ADC1>,
    pub continuity_check: AnyAdcChannel<ADC1>,
}

impl Sensors for BoardSensors {
    async fn tick(&mut self) -> SensorReadings {
        join5(
            self.imu1.tick(),
            self.imu2.tick(),
            self.imu3.tick(),
            self.highg.tick(),
            self.mag.tick(),
        )
        .await;

        join3(self.baro1.tick(), self.baro2.tick(), self.baro3.tick()).await;

        self.power.tick();

        SensorReadings {
            imu1_gyro: self.imu1.gyroscope(),
            imu1_accel: self.imu1.accelerometer(),
            imu2_gyro: self.imu2.gyroscope(),
            imu2_accel: self.imu2.accelerometer(),
            imu3_gyro: self.imu3.gyroscope(),
            imu3_accel: self.imu3.accelerometer(),
            highg_accel: self.highg.accelerometer(),
            mag: self.mag.magnetometer(),
            baro1: BaroReading {
                pressure: self.baro1.pressure(),
                temperature: self.baro1.temperature(),
                altitude: self.baro1.altitude(),
            },
            baro2: BaroReading {
                pressure: self.baro2.pressure(),
                temperature: self.baro2.temperature(),
                altitude: self.baro2.altitude(),
            },
            baro3: BaroReading {
                pressure: self.baro3.pressure(),
                temperature: self.baro3.temperature(),
                altitude: self.baro3.altitude(),
            },
            power: self.power.adc(),
        }
    }
}

impl Outputs for BoardOutputs {
    fn set_recovery_armed(&mut self, armed: bool) {
        self.recovery_high.set_level(armed.into());
    }

    fn set_drogue(&mut self, high: bool) {
        self.recovery_lows.0.set_level(high.into());
        self.recovery_lows.1.set_level(high.into());
    }

    fn set_main(&mut self, high: bool) {
        self.recovery_lows.2.set_level(high.into());
        self.recovery_lows.3.set_level(high.into());
    }
}
