use std::sync::atomic::Ordering;

use mission::{AdcData, BaroReading, Outputs, SensorReadings, Sensors};
use rapid_dialect::FlightMode;

use crate::simulation::{self, FlightSimulation};

pub struct StdSensors {
    sim: FlightSimulation,
}

impl Default for StdSensors {
    fn default() -> Self {
        Self {
            sim: FlightSimulation::new(),
        }
    }
}

impl StdSensors {
    pub fn set_flight_mode(&mut self, mode: FlightMode) {
        self.sim.set_flight_mode(mode);
    }
}

impl Sensors for StdSensors {
    async fn tick(&mut self) -> SensorReadings {
        self.sim.tick();

        let gyro = Some(self.sim.gyroscope());
        let accel = Some(self.sim.accelerometer());
        let mag = Some(self.sim.magnetometer());

        let baro = BaroReading {
            pressure: Some(self.sim.pressure()),
            temperature: Some(self.sim.temperature()),
            altitude: Some(self.sim.baro_altitude()),
        };

        SensorReadings {
            imu1_gyro: gyro,
            imu1_accel: accel,
            imu2_gyro: gyro,
            imu2_accel: accel,
            imu3_gyro: gyro,
            imu3_accel: accel,
            highg_accel: accel,
            mag,
            baro1: baro.clone(),
            baro2: baro.clone(),
            baro3: baro,
            power: Some(AdcData::default()),
        }
    }
}

#[derive(Default)]
pub struct StdOutputs {
    #[allow(dead_code)]
    recovery_armed: bool,
    #[allow(dead_code)]
    drogue_high: bool,
    #[allow(dead_code)]
    main_high: bool,
}

impl Outputs for StdOutputs {
    fn set_recovery_armed(&mut self, armed: bool) {
        self.recovery_armed = armed;
    }

    fn set_drogue(&mut self, high: bool) {
        self.drogue_high = high;
        simulation::DROGUE_ACTIVE.store(high, Ordering::Relaxed);
    }

    fn set_main(&mut self, high: bool) {
        self.main_high = high;
        simulation::MAIN_ACTIVE.store(high, Ordering::Relaxed);
    }
}
