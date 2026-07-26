use core::f32;
use core::num::Wrapping;

use rapid_dialect::rapid::enums::ValveId;
use rapid_dialect::rapid::messages::{
    Attitude, AutopilotVersion, BatteryStatus, GlobalPositionInt, GpsRawInt, Heartbeat,
    LocalPositionNed, PressureVessel, RocketInfo, ScaledImu, ScaledImu2, ScaledImu3,
    ScaledPressure, ScaledPressure2, ScaledPressure3, SysStatus, Valve, VfrHud,
};
use rapid_dialect::{FlightMode, Rapid};

use state_estimator::StateEstimator;

use crate::TelemetryLink;
use crate::bus::{Bus, BusInputImage, BusOutputImage};
use crate::flight_logic::FlightLogic;
use crate::inventory::{BinaryOutputId, InventoryId, TankId};
use crate::params::{Params, RecoveryParams};
use crate::traits::{Outputs, SensorReadings, Sensors, Storage};
use crate::valves::{ValveCommand, ValveController, ValveError};

pub const HEARTBEAT_INTERVAL_MS: u32 = 500;
pub const SENSOR_INTERVAL_MS: u32 = 100;
pub const BATTERY_INTERVAL_MS: u32 = 200;
pub const GPS_INTERVAL_MS: u32 = 500;
pub const VEHICLE_INFO_INTERVAL_MS: u32 = 1000;
pub const PROPULSION_INTERVAL_MS: u32 = 200;

pub const IGNITER_ON_DURATION_MS: u32 = 3000;

pub struct Vehicle<S: Sensors, O: Outputs, F: Storage, B: Bus> {
    pub time: Wrapping<u32>,
    mode: FlightMode,
    /// Vehicle time at which the current mode was entered.
    mode_entered_at: Wrapping<u32>,
    flight_logic: FlightLogic,
    recovery_params: RecoveryParams,
    pub sensors: S,
    pub outputs: O,
    pub storage: F,
    pub bus: B,
    pub readings: SensorReadings,
    pub state_estimator: StateEstimator,
    pub bus_inputs: BusInputImage,
    pub bus_outputs: BusOutputImage,
    pub valves: ValveController,
}

pub struct VehicleSnapshot<'a> {
    pub time: Wrapping<u32>,
    pub mode: FlightMode,
    pub recovery_params: &'a RecoveryParams,
    pub readings: &'a SensorReadings,
    pub state_estimator: &'a StateEstimator,
    pub input_image: &'a BusInputImage,
    pub output_image: &'a BusOutputImage,
}

impl<S: Sensors, O: Outputs, F: Storage, B: Bus> Vehicle<S, O, F, B> {
    pub async fn new(sensors: S, outputs: O, mut storage: F, bus: B) -> Self {
        let params = storage.read_params().unwrap_or_else(|| {
            log::info!("No params stored in flash, reverting to defaults.");
            Params::default()
        });

        Self::new_with_params(sensors, outputs, storage, params, bus)
    }

    pub fn new_with_params(sensors: S, outputs: O, storage: F, params: Params, bus: B) -> Self {
        Self {
            time: Wrapping(0),
            mode: FlightMode::default(),
            mode_entered_at: Wrapping(0),
            flight_logic: FlightLogic::default(),
            recovery_params: params.recovery,
            sensors,
            outputs,
            storage,
            bus,
            readings: SensorReadings::default(),
            state_estimator: StateEstimator::new(1000.0, params.state_estimator),
            bus_inputs: BusInputImage::default(),
            bus_outputs: BusOutputImage::default(),
            valves: ValveController::new(),
        }
    }

    pub async fn tick(&mut self) {
        use FlightMode as FM;

        // Read our on-board sensors and the vehicle bus inputs first, so
        // everything below acts on this tick's data.
        // TODO: incorporate all IMUs, baros into the state estimator.
        self.readings = self.sensors.tick().await;
        self.bus_inputs = self.bus.get_input_image();
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

        // Check if we need to auto-transition to a new flight mode based on the new state.
        if let Some(new_mode) = self.flight_logic.update(
            self.time,
            self.mode,
            &self.state_estimator,
            &self.recovery_params,
        ) {
            self.set_mode(new_mode);
        }

        // Set our on-board recovery outputs based on flight mode.
        self.outputs.set_recovery_armed(self.mode >= FM::Armed);
        self.outputs.set_drogue(self.mode == FM::RecoveryDrogue);
        self.outputs.set_main(self.mode == FM::RecoveryMain);

        // The igniters are energized for the first IGNITER_ON_DURATION_MS of Ignition.
        let igniting = self.mode == FM::Ignition
            && (self.time - self.mode_entered_at).0 < IGNITER_ON_DURATION_MS;
        self.bus_outputs.binary_output[BinaryOutputId::Igniter1] = igniting;
        self.bus_outputs.binary_output[BinaryOutputId::Igniter2] = igniting;

        // Determine the intended state of all valves and push it out on the vehicle bus.
        self.bus_outputs.valve = self.valves.resolve(self.time);
        self.bus.set_output_image(self.bus_outputs);

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
        self.mode = mode;
        self.mode_entered_at = self.time;

        self.flight_logic.set_mode(self.time, mode);
        self.valves.set_mode(mode);

        // Camera outputs are turned on automatically, but are not automatically turned
        // back off.
        if mode >= FlightMode::Armed {
            self.bus_outputs.binary_output[BinaryOutputId::Camera1] = true;
            self.bus_outputs.binary_output[BinaryOutputId::Camera2] = true;
        }
    }

    pub fn try_command_valve(
        &mut self,
        valve: ValveId,
        cmd: ValveCommand,
    ) -> Result<(), ValveError> {
        self.valves.try_command(valve, cmd, self.time)
    }

    pub async fn set_param(&mut self, id: u16, raw: u32) {
        use crate::params::ParameterGroup;

        let Some(descriptor) = Params::by_id(id) else {
            log::warn!("Ignoring set_param for unknown param id {id:#x}");
            return;
        };

        let value = descriptor.ty.decode_raw(raw);

        let mut params = Params {
            state_estimator: self.state_estimator.params().clone(),
            recovery: self.recovery_params.clone(),
        };

        params.set(descriptor.id, value);

        log::info!("Applying param {} (id {id:#x})", descriptor.name);
        self.recovery_params = params.recovery;
        self.state_estimator.update_params(params.state_estimator);

        self.storage.write_param(descriptor.id, value);
    }

    pub fn into_snapshot(&self) -> VehicleSnapshot<'_> {
        VehicleSnapshot {
            time: self.time,
            mode: self.mode,
            recovery_params: &self.recovery_params,
            readings: &self.readings,
            input_image: &self.bus_inputs,
            state_estimator: &self.state_estimator,
            output_image: &self.bus_outputs,
        }
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
            Self::send_msg::<AutopilotVersion>(&snap, link);
        }

        // these are instance messages, so the generic send_msg is not enough here
        if self.time.0 % PROPULSION_INTERVAL_MS == 0 {
            self.send_pressure_vessels(link);
            self.send_valve_states(link);
        }
    }

    fn send_pressure_vessels(&self, link: &mut impl TelemetryLink) {
        for tank in TankId::ALL {
            let p_ids = tank.pressure_sensors();
            let t_ids = tank.temperature_sensors();

            let pressure1 = p_ids[0]
                .map(|id| self.bus_inputs.press_sens[id])
                .and_then(|o| o.map(|d| d.data))
                .map(|bar| (bar * 100.0).clamp(0.0, f32::from(u16::MAX)) as u16)
                .unwrap_or(u16::MAX);

            let pressure2 = p_ids[1]
                .map(|id| self.bus_inputs.press_sens[id])
                .and_then(|o| o.map(|d| d.data))
                .map(|bar| (bar * 100.0).clamp(0.0, f32::from(u16::MAX)) as u16)
                .unwrap_or(u16::MAX);

            let temperature1 = t_ids[0]
                .map(|id| self.bus_inputs.temp_sens[id])
                .and_then(|o| o.map(|d| d.data))
                .map(|celsius| {
                    (celsius * 100.0).clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16
                })
                .unwrap_or(i16::MAX);

            let temperature2 = t_ids[1]
                .map(|id| self.bus_inputs.temp_sens[id])
                .and_then(|o| o.map(|d| d.data))
                .map(|celsius| {
                    (celsius * 100.0).clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16
                })
                .unwrap_or(i16::MAX);

            let level = (tank == TankId::Oxidizer)
                .then_some(self.bus_inputs.ox_tank_level.map(|d| d.data))
                .flatten()
                .map(|l| (l * 10000.0).clamp(0.0, f32::from(u16::MAX)) as u16)
                .unwrap_or(u16::MAX);

            let msg = PressureVessel {
                id: tank as u8,
                flags: tank.flags(),
                fluid: tank.fluid(),
                pressure1,
                pressure2,
                rated_pressure: (tank.pressure_rating_bar() * 100.0) as u16,
                temperature1,
                temperature2,
                volume: (tank.volume_l() * 1000.0) as u16,
                level,
            };
            link.send_message(msg.into());
        }
    }

    fn send_valve_states(&self, link: &mut impl TelemetryLink) {
        for valve in ValveId::ALL {
            // Both fields use 0.0 = fully closed, 1.0 = fully open; NAN = unknown.
            let state = self.bus_inputs.valve_state[valve]
                .map(|state| f32::from(state.data.promille()) / 1000.0)
                .unwrap_or(f32::NAN);
            let commanded = f32::from(self.bus_outputs.valve[valve].promille()) / 1000.0;

            let msg = Valve {
                id: valve,
                state,
                commanded,
            };

            link.send_message(msg.into());
        }
    }
}
