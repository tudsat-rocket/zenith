use core::num::Wrapping;

use rapid_dialect::FlightMode;
use rapid_dialect::rapid::enums::ValveId;

use state_estimator::StateEstimator;

use crate::bus::{Bus, BusInputImage, BusOutputImage};
use crate::flight_logic::FlightLogic;
use crate::inventory::BinaryOutputId;
use crate::mavlink::VehicleSnapshot;
use crate::params::{Params, PropulsionParams, RecoveryParams};
use crate::traits::{Outputs, SensorReadings, Sensors, Storage};
use crate::valves::{ValveCommand, ValveController, ValveError};

pub struct Vehicle<S: Sensors, O: Outputs, F: Storage, B: Bus> {
    pub time: Wrapping<u32>,
    mode: FlightMode,
    /// Vehicle time at which the current mode was entered.
    mode_entered_at: Wrapping<u32>,
    flight_logic: FlightLogic,
    recovery_params: RecoveryParams,
    propulsion_params: PropulsionParams,
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
            propulsion_params: params.propulsion,
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
        self.outputs
            .set_recovery_armed(self.mode >= FM::DetectLaunch);
        self.outputs.set_drogue(self.mode == FM::DeployDrogue);
        self.outputs.set_main(self.mode == FM::DeployMain);

        // The igniters are energized for the first PROP_IGNTR_TIME milliseconds of Ignition.
        let igniting = self.mode == FM::Ignite
            && (self.time - self.mode_entered_at).0 < self.propulsion_params.igniter_on_time;
        self.bus_outputs.binary_output[BinaryOutputId::Igniter1] = igniting;
        self.bus_outputs.binary_output[BinaryOutputId::Igniter2] = igniting;

        // Determine the intended state of all valves and push it out on the vehicle bus.
        self.bus_outputs.valve = self.valves.resolve(self.time, &self.propulsion_params);
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
        self.valves.set_mode(mode, self.time);

        // Camera outputs are turned on automatically, but are not automatically turned
        // back off.
        if mode >= FlightMode::DetectLaunch {
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
            propulsion: self.propulsion_params.clone(),
        };

        params.set(descriptor.id, value);

        log::info!("Applying param {} (id {id:#x})", descriptor.name);
        self.recovery_params = params.recovery;
        self.propulsion_params = params.propulsion;
        self.state_estimator.update_params(params.state_estimator);

        self.storage.write_param(descriptor.id, value);
    }

    pub fn snapshot(&self) -> VehicleSnapshot<'_> {
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
}
