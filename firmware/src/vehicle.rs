use core::num::Wrapping;

use embassy_executor::Spawner;
use embassy_futures::join::{join3, join5};

use rapid_dialect::FlightMode;
use state_estimator::{StateEstimator, StateEstimatorSettings};

use crate::links::UplinkCommand;
use crate::sensors::PowerMonitor;
use crate::{BoardAdc, BoardOutputs, BoardSensors};

pub struct Vehicle {
    pub time: Wrapping<u32>,
    mode: FlightMode,
    pub sensors: BoardSensors,
    // TODO: move this out of here again?
    pub outputs: BoardOutputs,
    pub power: PowerMonitor,
    pub state_estimator: StateEstimator,
}

impl Vehicle {
    pub async fn init(
        sensors: BoardSensors,
        outputs: BoardOutputs,
        adc: BoardAdc,
        low_priority_spawner: Spawner,
    ) -> Self {
        let power = crate::sensors::power::init(adc, low_priority_spawner);

        Self {
            time: Wrapping(0),
            mode: FlightMode::default(),
            sensors,
            outputs,
            power,
            state_estimator: StateEstimator::new(1000.0, StateEstimatorSettings::default()),
        }
    }

    pub async fn tick(&mut self) {
        join5(
            self.sensors.imu1.tick(),
            self.sensors.imu2.tick(),
            self.sensors.imu3.tick(),
            self.sensors.highg.tick(),
            self.sensors.mag.tick(),
        )
        .await;

        join3(
            self.sensors.baro1.tick(),
            self.sensors.baro2.tick(),
            self.sensors.baro3.tick(),
        )
        .await;

        self.power.tick();

        self.state_estimator.update(
            self.time,
            self.sensors.imu1.gyroscope(),
            self.sensors.imu1.accelerometer(),
            self.sensors.highg.accelerometer(),
            self.sensors.mag.magnetometer(),
            self.sensors.baro1.altitude(),
            None,
        );

        let recovery_armed = self.mode >= FlightMode::Armed;
        self.outputs.recovery_high.set_level(recovery_armed.into());
        // TODO: timing params
        let drogue_high = self.mode == FlightMode::RecoveryDrogue;
        let main_high = self.mode == FlightMode::RecoveryMain;
        self.outputs.recovery_lows.0.set_level(drogue_high.into());
        self.outputs.recovery_lows.1.set_level(drogue_high.into());
        self.outputs.recovery_lows.2.set_level(main_high.into());
        self.outputs.recovery_lows.3.set_level(main_high.into());

        self.time += 1;
    }

    pub fn mode(&self) -> FlightMode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: FlightMode) {
        if mode == self.mode {
            return;
        }

        self.mode = mode;
    }
}
