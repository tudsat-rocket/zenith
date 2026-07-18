use core::num::Wrapping;

use rapid_dialect::rapid::enums::{FluidType, PressureVesselFlag, ValveId};
use rapid_dialect::rapid::messages::{
    Attitude, BatteryStatus, GlobalPositionInt, GpsRawInt, Heartbeat, LocalPositionNed,
    PressureVessel, RocketInfo, ScaledImu, ScaledImu2, ScaledImu3, ScaledPressure, ScaledPressure2,
    ScaledPressure3, SysStatus, Valve, VfrHud,
};
use rapid_dialect::{FlightMode, Rapid};

use state_estimator::StateEstimator;

use crate::TelemetryLink;
use crate::flight_logic::FlightLogic;
use crate::foreign_io::{ForeignInputImage, ForeignIo, ForeignOutputImage, ValveState};
use crate::propulsion::{
    ALL_TANKS, ALL_VALVES, BinaryOutputId, PropulsionError, TankId, ValveCommand, tank_reading,
};
use crate::settings::{RecoverySettings, Settings};
use crate::traits::{Outputs, SensorReadings, Sensors, Storage};

pub const HEARTBEAT_INTERVAL_MS: u32 = 500;
pub const SENSOR_INTERVAL_MS: u32 = 100;
pub const BATTERY_INTERVAL_MS: u32 = 200;
pub const GPS_INTERVAL_MS: u32 = 500;
pub const VEHICLE_INFO_INTERVAL_MS: u32 = 1000;
pub const PROPULSION_INTERVAL_MS: u32 = 200;

pub struct Vehicle<S: Sensors, O: Outputs, F: Storage, E: ForeignIo> {
    pub time: Wrapping<u32>,
    mode: FlightMode,
    flight_logic: FlightLogic,
    recovery_settings: RecoverySettings,
    pub sensors: S,
    pub outputs: O,
    pub storage: F,
    pub readings: SensorReadings,
    pub state_estimator: StateEstimator,
    pub foreign_io: E,
    pub input_image: ForeignInputImage,
    pub output_image: ForeignOutputImage,
}

pub struct VehicleSnapshot<'a> {
    pub time: Wrapping<u32>,
    pub mode: FlightMode,
    pub recovery_settings: &'a RecoverySettings,
    pub readings: &'a SensorReadings,
    pub state_estimator: &'a StateEstimator,
    pub input_image: &'a ForeignInputImage,
    pub output_image: &'a ForeignOutputImage,
}

impl<S: Sensors, O: Outputs, F: Storage, E: ForeignIo> Vehicle<S, O, F, E> {
    pub async fn new(sensors: S, outputs: O, mut storage: F, foreign_io: E) -> Self {
        let settings = storage.read_settings().await.unwrap_or_else(|| {
            log::info!("No settings stored in flash, reverting to defaults.");
            Settings::default()
        });

        Self::new_with_settings(sensors, outputs, storage, settings, foreign_io)
    }

    pub fn new_with_settings(
        sensors: S,
        outputs: O,
        storage: F,
        settings: Settings,
        foreign_io: E,
    ) -> Self {
        Self {
            time: Wrapping(0),
            mode: FlightMode::default(),
            flight_logic: FlightLogic::default(),
            recovery_settings: settings.recovery,
            sensors,
            outputs,
            storage,
            readings: SensorReadings::default(),
            state_estimator: StateEstimator::new(1000.0, settings.state_estimator),
            foreign_io,
            input_image: ForeignInputImage::default(),
            output_image: ForeignOutputImage::default(),
        }
    }
    pub fn into_snapshot(&self) -> VehicleSnapshot<'_> {
        VehicleSnapshot {
            time: self.time,
            mode: self.mode,
            recovery_settings: &self.recovery_settings,
            readings: &self.readings,
            input_image: &self.input_image,
            state_estimator: &self.state_estimator,
            output_image: &self.output_image,
        }
    }

    pub async fn tick(&mut self) {
        self.readings = self.sensors.tick().await;

        self.input_image = self.foreign_io.get_input_image();

        self.state_estimator.update(
            self.time,
            self.mode,
            self.readings.imu1_gyro,
            self.readings.imu1_accel,
            self.readings.highg_accel,
            self.readings.mag,
            self.readings.baro1.altitude,
            self.readings.gps.clone(),
        );

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

        self.foreign_io.set_output_image(self.output_image);
        self.foreign_io.tick();

        self.time += 1;
    }

    pub fn mode(&self) -> FlightMode {
        self.mode
    }

    // TODO: document
    pub fn set_mode(&mut self, mode: FlightMode) {
        if mode == self.mode {
            return;
        }

        log::info!("Mode change: {:?} -> {:?}", self.mode, mode);
        self.flight_logic.set_mode(self.time, mode);
        self.mode = mode;

        for valve in ALL_VALVES {
            if let Some(cmd) = Self::default_valve_policy(mode, valve) {
                self.output_image.set_valve(valve, cmd);
            }
        }

        if mode == FlightMode::Ignition {
            // Igniter inhibit lives next to the call site: this is the one
            // place fire_igniter() is invoked. NoPropulsion returns Err and
            // the call is harmlessly ignored.
            // FIXME: turn off igniter at some point

            self.output_image
                .set_binary_output(BinaryOutputId::Igniter1, true);
            self.output_image
                .set_binary_output(BinaryOutputId::Igniter2, true);
        }
    }

    /// Try to manually drive a valve from an uplink command. Returns
    /// `NotPermittedInMode` if the current mode doesn't grant operator
    /// authority over that valve.
    pub fn try_command_valve(
        &mut self,
        valve: ValveId,
        cmd: ValveCommand,
    ) -> Result<(), PropulsionError> {
        if !Self::manual_valve_allowed(self.mode, valve) {
            return Err(crate::propulsion::PropulsionError::NotPermittedInMode);
        }
        let state = match cmd {
            // TODO: implement pulseOpen
            ValveCommand::Open => ValveState::fully_open(),
            ValveCommand::Close => ValveState::fully_closed(),
            // FIXME: document partial ValveCommand
            ValveCommand::Partial(p) => ValveState::from_promille_clamped((p * 1000.0) as u16),
            // TODO: implement pulse open
            ValveCommand::PulseOpen(_) => ValveState::fully_open(),
        };
        self.output_image.set_valve(valve, state);
        Ok(())
    }

    // NOTE: this wrapper is probably not required
    fn send_msg<M: mavio::Message + Into<Rapid>>(
        snaphot: &VehicleSnapshot,
        link: &mut impl TelemetryLink,
    ) where
        for<'a> &'a VehicleSnapshot<'a>: Into<M>,
    {
        let m: M = snaphot.into();
        link.send_message(m.into());
    }

    /// This determines the pattern of data the flight computer sends via MAVlink for all of the
    /// non-RF telemetry paths (primarily ethernet)
    pub fn send_telemetry(&self, link: &mut impl TelemetryLink) {
        if self.time.0 % HEARTBEAT_INTERVAL_MS == 0 {
            let snap = self.into_snapshot();
            link.send_message(Heartbeat::from(&snap).into());
            Self::send_msg::<Heartbeat>(&snap, link);
        }

        if self.time.0 % HEARTBEAT_INTERVAL_MS == HEARTBEAT_INTERVAL_MS / 2 {
            let snap = self.into_snapshot();
            Self::send_msg::<SysStatus>(&snap, link);
        }

        if self.time.0 % SENSOR_INTERVAL_MS == 0 {
            let snap = self.into_snapshot();

            Self::send_msg::<Attitude>(&snap, link);
            Self::send_msg::<LocalPositionNed>(&snap, link);
            Self::send_msg::<VfrHud>(&snap, link);
            Self::send_msg::<ScaledImu>(&snap, link);
            Self::send_msg::<ScaledImu2>(&snap, link);
            Self::send_msg::<ScaledImu3>(&snap, link);
        }

        if self.time.0 % GPS_INTERVAL_MS == 0 {
            let snap = self.into_snapshot();
            Self::send_msg::<GlobalPositionInt>(&snap, link);
            Self::send_msg::<GpsRawInt>(&snap, link);
        }

        if self.time.0 % SENSOR_INTERVAL_MS == SENSOR_INTERVAL_MS / 2 {
            let snap = self.into_snapshot();
            Self::send_msg::<ScaledPressure>(&snap, link);
            Self::send_msg::<ScaledPressure2>(&snap, link);
            Self::send_msg::<ScaledPressure3>(&snap, link);
        }

        if self.time.0 % BATTERY_INTERVAL_MS == 0 {
            let snap = self.into_snapshot();
            Self::send_msg::<BatteryStatus>(&snap, link);
        }

        if self.time.0 % VEHICLE_INFO_INTERVAL_MS == 0 {
            let snap = self.into_snapshot();
            Self::send_msg::<RocketInfo>(&snap, link);
        }

        // these are instance messages, so the generic send_msg is not enough here
        if self.time.0 % PROPULSION_INTERVAL_MS == 0 {
            self.send_pressure_vessels(link);
            self.send_valve_states(link);
        }
    }

    fn send_pressure_vessels(&self, link: &mut impl TelemetryLink) {
        let snapshot = self.into_snapshot();
        for tank in ALL_TANKS {
            let reading = tank_reading(tank, &snapshot);

            let (fluid, rated) = match tank {
                TankId::Pressurant => (FluidType::Nitrogen, 30_000u16),
                TankId::Oxidizer => (FluidType::NitrousOxide, 7_000u16),
                TankId::CombustionChamber => (FluidType::Combustion, 7_000u16),
            };

            let pressure1_kpa = reading
                .pressure1
                .map(|p| (p * 100.0).clamp(0.0, f32::from(u16::MAX)) as u16)
                .unwrap_or(0);
            let pressure2_kpa = reading
                .pressure2
                .map(|p| (p * 100.0).clamp(0.0, f32::from(u16::MAX)) as u16)
                .unwrap_or(0);
            let temp1_cdegc = reading
                .temperature1
                .map(|t| (t * 100.0).clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16)
                .unwrap_or(i16::MAX);
            let temp2_cdegc = reading
                .temperature2
                .map(|t| (t * 100.0).clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16)
                .unwrap_or(i16::MAX);

            let flags = PressureVesselFlag::empty();

            let level_c_percent = reading
                .level
                .map(|l| (l * 10000.0).clamp(0.0, f32::from(u16::MAX)) as u16)
                .unwrap_or(0);

            let msg = PressureVessel {
                id: tank as u8,
                flags,
                fluid,
                pressure1: pressure1_kpa,
                pressure2: pressure2_kpa,
                rated_pressure: rated,
                temperature1: temp1_cdegc,
                temperature2: temp2_cdegc,
                volume: 0,
                level: level_c_percent,
            };
            link.send_message(msg.into());
        }
    }

    fn send_valve_states(&self, link: &mut impl TelemetryLink) {
        for valve in ALL_VALVES {
            // TODO: send only fresh data?
            // FIXME: document what f32 means in terms of valve state
            // 0 = closed, 1 = open ??
            let state = self
                .input_image
                .valve_state(valve)
                .map(|state| f32::from(state.data.promille()) / 1000.0)
                .unwrap_or(f32::NAN);
            let commanded = f32::from(self.output_image.get_valve(valve).promille()) / 1000.0;
            let msg = Valve {
                id: valve,
                state,
                commanded,
            };
            link.send_message(msg.into());
        }
    }

    fn manual_valve_allowed(mode: FlightMode, valve: ValveId) -> bool {
        match mode {
            // Hold: every valve commandable
            FlightMode::Hold => true,
            // Vent valves pulsable for relief during fill / pressurize
            FlightMode::Filling | FlightMode::Pressurizing => {
                matches!(
                    valve,
                    ValveId::PressurantVent | ValveId::OxidizerVent | ValveId::OxidizerFill
                )
            }
            // No manual overrides anywhere else
            _ => false,
        }
    }

    // TODO: implement procedures
    fn default_valve_policy(mode: FlightMode, valve: ValveId) -> Option<ValveState> {
        use ValveState as V;

        match (mode, valve) {
            (FlightMode::Hold, _) => None,
            (FlightMode::Filling, ValveId::OxidizerFill) => Some(V::fully_open()),
            (FlightMode::Pressurizing, ValveId::Pressurization) => Some(V::fully_open()),
            (FlightMode::Venting, ValveId::PressurantVent | ValveId::OxidizerVent) => {
                Some(V::fully_open())
            }
            (
                FlightMode::Coast | FlightMode::Ignition | FlightMode::Burn,
                ValveId::Main | ValveId::Pressurization,
            ) => Some(V::fully_open()),
            (
                FlightMode::RecoveryDrogue | FlightMode::RecoveryMain,
                ValveId::PressurantVent | ValveId::OxidizerVent,
            ) => Some(V::fully_open()),
            _ => Some(V::fully_closed()),
        }
    }
}
