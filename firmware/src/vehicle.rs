use core::num::Wrapping;

use embassy_futures::join::{join3, join5};

use crate::BoardSensors;

pub struct Vehicle {
    pub time: Wrapping<u32>,
    pub sensors: BoardSensors,
}

impl Vehicle {
    pub async fn init(sensors: BoardSensors) -> Self {
        Self {
            time: Wrapping(0),
            sensors,
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

        self.time += 1;
    }
}
