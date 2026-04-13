use core::num::Wrapping;

use rapid_dialect::rapid::messages::{
    Attitude, BatteryStatus, Heartbeat, LocalPositionNed, ScaledImu, ScaledImu2, ScaledImu3,
    ScaledPressure, ScaledPressure2, ScaledPressure3,
};
use rapid_dialect::{FlightMode, Rapid};
use state_estimator::StateEstimator;

use crate::flight_logic::FlightLogic;
use crate::telemetry;
use crate::traits::{Outputs, RecoverySettings, SensorReadings, Sensors, Settings, Storage};

pub struct Vehicle<S: Sensors, O: Outputs, F: Storage> {
    pub time: Wrapping<u32>,
    mode: FlightMode,
    flight_logic: FlightLogic,
    recovery_settings: RecoverySettings,
    pub sensors: S,
    pub outputs: O,
    pub storage: F,
    pub readings: SensorReadings,
    pub state_estimator: StateEstimator,
}

impl<S: Sensors, O: Outputs, F: Storage> Vehicle<S, O, F> {
    pub async fn new(sensors: S, outputs: O, mut storage: F) -> Self {
        let settings = storage.read_settings().await.unwrap_or_else(|| {
            log::info!("No settings stored in flash, reverting to defaults.");
            Settings::default()
        });

        Self {
            time: Wrapping(0),
            mode: FlightMode::default(),
            flight_logic: FlightLogic::new(),
            recovery_settings: settings.recovery,
            sensors,
            outputs,
            storage,
            readings: SensorReadings::default(),
            state_estimator: StateEstimator::new(1000.0, settings.state_estimator),
        }
    }

    pub fn new_with_settings(sensors: S, outputs: O, storage: F, settings: Settings) -> Self {
        Self {
            time: Wrapping(0),
            mode: FlightMode::default(),
            flight_logic: FlightLogic::new(),
            recovery_settings: settings.recovery,
            sensors,
            outputs,
            storage,
            readings: SensorReadings::default(),
            state_estimator: StateEstimator::new(1000.0, settings.state_estimator),
        }
    }

    pub async fn tick(&mut self) {
        self.readings = self.sensors.tick().await;

        self.state_estimator.update(
            self.time,
            self.mode,
            self.readings.imu1_gyro,
            self.readings.imu1_accel,
            self.readings.highg_accel,
            self.readings.mag,
            self.readings.baro1.altitude,
            None,
        );

        // Run automatic flight logic (Armed -> Burn -> Coast -> Recovery -> Landed)
        if let Some(new_mode) = self.flight_logic.update(
            self.time,
            self.mode,
            &self.state_estimator,
            &self.recovery_settings,
        ) {
            self.set_mode(new_mode);
        }

        let recovery_armed = self.mode >= FlightMode::Armed;
        self.outputs.set_recovery_armed(recovery_armed);
        self.outputs
            .set_drogue(self.mode == FlightMode::RecoveryDrogue);
        self.outputs.set_main(self.mode == FlightMode::RecoveryMain);

        self.time += 1;
    }

    pub fn mode(&self) -> FlightMode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: FlightMode) {
        if mode == self.mode {
            return;
        }

        log::info!("Mode change: {:?} -> {:?}", self.mode, mode);
        self.flight_logic.on_mode_change(self.time, mode);
        self.mode = mode;
    }

    fn send_msg<M: mavio::Message + Into<Rapid>>(&self, link: &mut impl telemetry::TelemetryLink)
    where
        for<'a> &'a Self: Into<M>,
    {
        let m: M = self.into();
        link.send_message(m.into());
    }

    pub fn send_telemetry(&self, link: &mut impl telemetry::TelemetryLink) {
        if self.time.0 % telemetry::HEARTBEAT_INTERVAL_MS == 0 {
            self.send_msg::<Heartbeat>(link);
        }

        if self.time.0 % telemetry::SENSOR_INTERVAL_MS == 0 {
            self.send_msg::<Attitude>(link);
            self.send_msg::<LocalPositionNed>(link);
            self.send_msg::<ScaledImu>(link);
            self.send_msg::<ScaledImu2>(link);
            self.send_msg::<ScaledImu3>(link);
        }

        if self.time.0 % telemetry::SENSOR_INTERVAL_MS == telemetry::SENSOR_INTERVAL_MS / 2 {
            self.send_msg::<ScaledPressure>(link);
            self.send_msg::<ScaledPressure2>(link);
            self.send_msg::<ScaledPressure3>(link);
        }

        if self.time.0 % telemetry::BATTERY_INTERVAL_MS == 0 {
            self.send_msg::<BatteryStatus>(link);
        }
    }
}
