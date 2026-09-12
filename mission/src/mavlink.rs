//! The vehicle's state at one tick, and the MAVLink built from it.

use core::f32;
use core::num::Wrapping;

use nalgebra::{UnitQuaternion, Vector3};

#[cfg(target_os = "none")]
use num_traits::Float as _;

use rapid_dialect::FlightMode;
use rapid_dialect::rapid::enums::{
    GpsFixType, MavAutopilot, MavBatteryChargeState, MavBatteryFault, MavBatteryFunction,
    MavBatteryMode, MavBatteryType, MavModeFlag, MavProtocolCapability, MavSysStatusSensor,
    MavSysStatusSensorExtended, MavType, RocketCapability,
};
use rapid_dialect::rapid::messages::{
    Attitude, AutopilotVersion, BatteryStatus, GlobalPositionInt, GpsRawInt, Heartbeat,
    LocalPositionNed, PressureVessel, RocketInfo, ScaledImu, ScaledImu2, ScaledImu3,
    ScaledPressure, ScaledPressure2, ScaledPressure3, SysStatus, Valve, VfrHud,
};

use state_estimator::StateEstimator;

use crate::TelemetryLink;
use crate::bus::{BusInputImage, BusOutputImage};
use crate::inventory::{InventoryId, TankId, ValveId};
use crate::params::StateMachineParams;
use crate::schedule::downlink_schedule;
use crate::traits::SensorReadings;

/// Everything the vehicle exposes about one tick, borrowed rather than copied. Built by
/// `Vehicle::snapshot`; the conversions in this module read nothing else.
pub struct VehicleSnapshot<'a> {
    pub time: Wrapping<u32>,
    pub mode: FlightMode,
    pub state_machine_params: &'a StateMachineParams,
    pub readings: &'a SensorReadings,
    pub state_estimator: &'a StateEstimator,
    pub input_image: &'a BusInputImage,
    pub output_image: &'a BusOutputImage,
}

impl VehicleSnapshot<'_> {
    /// This determines the pattern of data the flight computer sends via MAVlink for all of the
    /// non-RF telemetry paths (primarily ethernet): which messages go out unprompted, and how often.
    ///
    /// At most one message goes out per tick, the phases coming from [`crate::schedule`].
    pub fn send_telemetry(&self, link: &mut impl TelemetryLink) {
        downlink_schedule! { self.time.0, self, link:
            every 100 ms => Attitude, VfrHud, ScaledImu, ScaledImu2, ScaledImu3;
            every 200 ms => BatteryStatus, LocalPositionNed,
                ScaledPressure, ScaledPressure2, ScaledPressure3;
            every 500 ms => Heartbeat, SysStatus, GlobalPositionInt, GpsRawInt;
            every 2000 ms => RocketInfo, AutopilotVersion;
            // One message per component
            every 200 ms => PressureVessel[TankId::ALL], Valve[ValveId::ALL];
        }
    }

    /// Whether the vehicle is *physically* armed, i.e. the arming pins/switches are thrown. This is
    /// orthogonal to the flight mode and is what MAVLink SAFETY_ARMED reflects.
    fn is_physically_armed(&self) -> bool {
        const RECOVERY_ARMED_THRESHOLD_MV: u16 = 6000;

        let recovery_hot = self
            .readings
            .power
            .as_ref()
            .map(|p| p.recovery_voltage > RECOVERY_ARMED_THRESHOLD_MV)
            .unwrap_or(false);

        // TODO: replace with a real IO-board armed flag once available on the bus.
        #[cfg(feature = "hybrid")]
        let io_armed = false;
        #[cfg(not(feature = "hybrid"))]
        let io_armed = false;

        recovery_hot || io_armed
    }
}

/// Attitude as (roll, pitch, yaw) in radians, in MAVLink's aircraft convention.
fn euler_angles(orientation: &UnitQuaternion<f32>) -> (f32, f32, f32) {
    let nose = orientation.transform_vector(&Vector3::z());
    let board_normal = orientation.transform_vector(&Vector3::y());
    let across_board = orientation.transform_vector(&Vector3::x());

    let roll = across_board.z.atan2(board_normal.z);
    let pitch = nose.z.clamp(-1.0, 1.0).asin();
    // yaw is compass heading; the estimator is east-north-up
    let yaw = nose.x.atan2(nose.y);

    (roll, pitch, yaw)
}

impl From<&VehicleSnapshot<'_>> for Heartbeat {
    fn from(snap: &VehicleSnapshot<'_>) -> Self {
        Heartbeat {
            type_: MavType::Rocket,
            autopilot: MavAutopilot::Generic,
            base_mode: if snap.is_physically_armed() {
                MavModeFlag::CUSTOM_MODE_ENABLED | MavModeFlag::SAFETY_ARMED
            } else {
                MavModeFlag::CUSTOM_MODE_ENABLED
            },
            custom_mode: snap.mode as u32,
            system_status: snap.mode.into(),
            mavlink_version: 2,
        }
    }
}

impl Into<Attitude> for &VehicleSnapshot<'_> {
    fn into(self) -> Attitude {
        let (roll, pitch, yaw) =
            euler_angles(&self.state_estimator.orientation.unwrap_or_default());
        let gyro = self.readings.imu1_gyro.unwrap_or_default();

        Attitude {
            time_boot_ms: self.time.0,
            roll,
            pitch,
            yaw,
            // Body rates about the same aircraft axes: forward, right and down.
            rollspeed: gyro.z.to_radians(),
            pitchspeed: -gyro.x.to_radians(),
            yawspeed: -gyro.y.to_radians(),
        }
    }
}

impl Into<LocalPositionNed> for &VehicleSnapshot<'_> {
    fn into(self) -> LocalPositionNed {
        let pos = self.state_estimator.position_local();
        let vel = self.state_estimator.velocity();

        LocalPositionNed {
            time_boot_ms: self.time.0,
            x: pos.y,
            y: pos.x,
            z: -self.state_estimator.altitude_agl(),
            vx: vel.y,
            vy: vel.x,
            vz: -vel.z,
        }
    }
}

impl Into<GlobalPositionInt> for &VehicleSnapshot<'_> {
    fn into(self) -> GlobalPositionInt {
        let vel = self.state_estimator.velocity();

        let (_roll, _pitch, yaw) =
            euler_angles(&self.state_estimator.orientation.unwrap_or_default());

        GlobalPositionInt {
            time_boot_ms: self.time.0,
            lat: (f64::from(self.state_estimator.latitude().unwrap_or(0.0)) * 1e7) as i32,
            lon: (f64::from(self.state_estimator.longitude().unwrap_or(0.0)) * 1e7) as i32,
            alt: (self.state_estimator.altitude_asl() * 1000.0) as i32,
            relative_alt: (self.state_estimator.altitude_agl() * 1000.0) as i32,
            vx: (vel.y * 100.0) as i16,
            vy: (vel.x * 100.0) as i16,
            vz: (-vel.z * 100.0) as i16,
            hdg: ({
                let d = yaw.to_degrees();
                (d - (d / 360.0).floor() * 360.0) * 100.0
            }) as u16,
        }
    }
}

impl Into<GpsRawInt> for &VehicleSnapshot<'_> {
    #[allow(
        clippy::arithmetic_side_effects,
        reason = "time.0 is u32 widened to u64 before *1000, cannot overflow u64"
    )]
    fn into(self) -> GpsRawInt {
        let gps = self.readings.gps.as_ref();

        let fix_type = match gps {
            Some(g) if g.latitude.is_some() && g.longitude.is_some() && g.altitude.is_some() => {
                GpsFixType::_3dFix
            }
            Some(_) => GpsFixType::NoFix,
            None => GpsFixType::NoGps,
        };

        GpsRawInt {
            time_usec: u64::from(self.time.0) * 1000,
            fix_type,
            lat: gps
                .and_then(|g| g.latitude)
                .map(|v| (f64::from(v) * 1e7) as i32)
                .unwrap_or(i32::MAX),
            lon: gps
                .and_then(|g| g.longitude)
                .map(|v| (f64::from(v) * 1e7) as i32)
                .unwrap_or(i32::MAX),
            alt: gps
                .and_then(|g| g.altitude)
                .map(|v| (v * 1000.0) as i32)
                .unwrap_or(i32::MAX),
            eph: gps.map(|g| g.hdop).unwrap_or(u16::MAX),
            epv: u16::MAX,
            vel: u16::MAX,
            cog: u16::MAX,
            satellites_visible: gps.map(|g| g.num_satellites).unwrap_or(u8::MAX),
            alt_ellipsoid: 0,
            h_acc: 0,
            v_acc: 0,
            vel_acc: 0,
            hdg_acc: 0,
            yaw: 0,
        }
    }
}

impl Into<ScaledImu> for &VehicleSnapshot<'_> {
    fn into(self) -> ScaledImu {
        let acc1 = self.readings.imu1_accel;
        let gyro1 = self.readings.imu1_gyro;
        let mag1 = self.readings.mag;

        ScaledImu {
            time_boot_ms: self.time.0,
            xacc: (acc1.map(|v| v.x).unwrap_or_default() * 101.972) as i16,
            yacc: (acc1.map(|v| v.y).unwrap_or_default() * 101.972) as i16,
            zacc: (acc1.map(|v| v.z).unwrap_or_default() * 101.972) as i16,
            xgyro: (gyro1.map(|v| v.x).unwrap_or_default() * 17.45329) as i16,
            ygyro: (gyro1.map(|v| v.y).unwrap_or_default() * 17.45329) as i16,
            zgyro: (gyro1.map(|v| v.z).unwrap_or_default() * 17.45329) as i16,
            xmag: (mag1.map(|v| v.x).unwrap_or_default() * 10.0) as i16,
            ymag: (mag1.map(|v| v.y).unwrap_or_default() * 10.0) as i16,
            zmag: (mag1.map(|v| v.z).unwrap_or_default() * 10.0) as i16,
            temperature: 0, // TODO
        }
    }
}

impl Into<ScaledImu2> for &VehicleSnapshot<'_> {
    fn into(self) -> ScaledImu2 {
        let acc2 = self.readings.imu2_accel;
        let gyro2 = self.readings.imu2_gyro;

        ScaledImu2 {
            time_boot_ms: self.time.0,
            xacc: (acc2.map(|v| v.x).unwrap_or_default() * 101.972) as i16,
            yacc: (acc2.map(|v| v.y).unwrap_or_default() * 101.972) as i16,
            zacc: (acc2.map(|v| v.z).unwrap_or_default() * 101.972) as i16,
            xgyro: (gyro2.map(|v| v.x).unwrap_or_default() * 17.45329) as i16,
            ygyro: (gyro2.map(|v| v.y).unwrap_or_default() * 17.45329) as i16,
            zgyro: (gyro2.map(|v| v.z).unwrap_or_default() * 17.45329) as i16,
            ..Default::default()
        }
    }
}

impl Into<ScaledImu3> for &VehicleSnapshot<'_> {
    fn into(self) -> ScaledImu3 {
        let acc3 = self.readings.imu3_accel;
        let gyro3 = self.readings.imu3_gyro;

        ScaledImu3 {
            time_boot_ms: self.time.0,
            xacc: (acc3.map(|v| v.x).unwrap_or_default() * 101.972) as i16,
            yacc: (acc3.map(|v| v.y).unwrap_or_default() * 101.972) as i16,
            zacc: (acc3.map(|v| v.z).unwrap_or_default() * 101.972) as i16,
            xgyro: (gyro3.map(|v| v.x).unwrap_or_default() * 17.45329) as i16,
            ygyro: (gyro3.map(|v| v.y).unwrap_or_default() * 17.45329) as i16,
            zgyro: (gyro3.map(|v| v.z).unwrap_or_default() * 17.45329) as i16,
            ..Default::default()
        }
    }
}

impl Into<ScaledPressure> for &VehicleSnapshot<'_> {
    fn into(self) -> ScaledPressure {
        ScaledPressure {
            time_boot_ms: self.time.0,
            press_abs: self.readings.baro1.pressure.unwrap_or_default(),
            press_diff: 0.0,
            temperature: (self.readings.baro1.temperature.unwrap_or_default() * 10.0) as i16,
            temperature_press_diff: 0,
        }
    }
}

impl Into<ScaledPressure2> for &VehicleSnapshot<'_> {
    fn into(self) -> ScaledPressure2 {
        ScaledPressure2 {
            time_boot_ms: self.time.0,
            press_abs: self.readings.baro2.pressure.unwrap_or_default(),
            press_diff: 0.0,
            temperature: (self.readings.baro2.temperature.unwrap_or_default() * 10.0) as i16,
            temperature_press_diff: 0,
        }
    }
}

impl Into<ScaledPressure3> for &VehicleSnapshot<'_> {
    fn into(self) -> ScaledPressure3 {
        ScaledPressure3 {
            time_boot_ms: self.time.0,
            press_abs: self.readings.baro3.pressure.unwrap_or_default(),
            press_diff: 0.0,
            temperature: (self.readings.baro3.temperature.unwrap_or_default() * 10.0) as i16,
            temperature_press_diff: 0,
        }
    }
}

/// The throttle percentage VFR_HUD reports for a flight mode. Also used by the telemetry receiver.
pub fn throttle_percent(mode: FlightMode) -> u16 {
    if mode == FlightMode::Burn { 100 } else { 0 }
}

impl Into<VfrHud> for &VehicleSnapshot<'_> {
    fn into(self) -> VfrHud {
        let (_roll, _pitch, yaw) =
            euler_angles(&self.state_estimator.orientation.unwrap_or_default());
        let vel = self.state_estimator.velocity();

        VfrHud {
            // No airspeed sensor on this vehicle.
            airspeed: f32::NAN,
            groundspeed: vel.xy().magnitude(),
            heading: ({
                let d = yaw.to_degrees();
                d - (d / 360.0).floor() * 360.0
            }) as i16,
            throttle: throttle_percent(self.mode),
            alt: self.state_estimator.altitude_asl(),
            climb: vel.z,
        }
    }
}

impl Into<SysStatus> for &VehicleSnapshot<'_> {
    #[allow(
        clippy::arithmetic_side_effects,
        reason = "`enabled -= FLAG` is MavSysStatusSensor bitflag set-difference, not integer arithmetic"
    )]
    fn into(self) -> SysStatus {
        let r = &self.readings;

        let hw_armed = self.is_physically_armed();
        let armed = self.mode >= FlightMode::DetectLaunch;

        // All sensors/subsystems physically present on the board.
        let present = MavSysStatusSensor::_3D_GYRO
            | MavSysStatusSensor::_3D_ACCEL
            | MavSysStatusSensor::_3D_MAG
            | MavSysStatusSensor::ABSOLUTE_PRESSURE
            | MavSysStatusSensor::GPS
            | MavSysStatusSensor::MAV_SYS_STATUS_AHRS
            | MavSysStatusSensor::BATTERY
            | MavSysStatusSensor::MOTOR_OUTPUTS
            | MavSysStatusSensor::MAV_SYS_STATUS_LOGGING
            | MavSysStatusSensor::MAV_SYS_STATUS_PREARM_CHECK
            | MavSysStatusSensor::PROPULSION
            | MavSysStatusSensor::MAV_SYS_STATUS_EXTENSION_USED;

        let mut enabled = present;
        if !armed {
            enabled -= MavSysStatusSensor::MAV_SYS_STATUS_LOGGING;
        }
        if !hw_armed {
            enabled -= MavSysStatusSensor::MOTOR_OUTPUTS;
        }
        if self.mode != FlightMode::Burn && self.mode != FlightMode::Ignite {
            enabled -= MavSysStatusSensor::PROPULSION;
        }

        let mut health = MavSysStatusSensor::empty();
        if r.imu1_gyro.is_some() && r.imu2_gyro.is_some() && r.imu3_gyro.is_some() {
            health |= MavSysStatusSensor::_3D_GYRO;
        }
        if r.imu1_accel.is_some() && r.imu2_accel.is_some() && r.imu3_accel.is_some() {
            health |= MavSysStatusSensor::_3D_ACCEL;
        }
        if r.mag.is_some() {
            health |= MavSysStatusSensor::_3D_MAG;
        }
        if r.baro1.pressure.is_some() {
            health |= MavSysStatusSensor::ABSOLUTE_PRESSURE;
        }
        if r.gps.is_some() {
            health |= MavSysStatusSensor::GPS;
        }
        if self.state_estimator.orientation.is_some() {
            health |= MavSysStatusSensor::MAV_SYS_STATUS_AHRS;
        }
        if r.power.is_some() {
            health |= MavSysStatusSensor::BATTERY;
        }
        if armed {
            health |= MavSysStatusSensor::MAV_SYS_STATUS_LOGGING;
        }
        if hw_armed {
            health |= MavSysStatusSensor::MOTOR_OUTPUTS;
        }
        if hw_armed
            && let Some(g) = &r.gps
            && self.state_estimator.gps_reliable(g)
        {
            health |= MavSysStatusSensor::MAV_SYS_STATUS_PREARM_CHECK;
        }
        health |= MavSysStatusSensor::MAV_SYS_STATUS_EXTENSION_USED;

        let recovery_present = MavSysStatusSensorExtended::MAV_SYS_STATUS_RECOVERY_SYSTEM;
        let recovery_enabled = if armed {
            recovery_present
        } else {
            MavSysStatusSensorExtended::empty()
        };
        let recovery_health = if armed {
            recovery_present
        } else {
            MavSysStatusSensorExtended::empty()
        };

        SysStatus {
            onboard_control_sensors_present: present,
            onboard_control_sensors_enabled: enabled,
            onboard_control_sensors_health: health,
            load: 0,
            voltage_battery: r
                .power
                .as_ref()
                .map(|d| d.bus_main_voltage)
                .unwrap_or(u16::MAX),
            current_battery: r
                .power
                .as_ref()
                .map(|d| (d.fc_current / 10).clamp(i16::MIN as i32, i16::MAX as i32) as i16)
                .unwrap_or(-1),
            battery_remaining: -1,
            drop_rate_comm: 0,
            errors_comm: 0,
            errors_count1: 0,
            errors_count2: 0,
            errors_count3: 0,
            errors_count4: 0,
            onboard_control_sensors_present_extended: recovery_present,
            onboard_control_sensors_enabled_extended: recovery_enabled,
            onboard_control_sensors_health_extended: recovery_health,
        }
    }
}

impl Into<AutopilotVersion> for &VehicleSnapshot<'_> {
    fn into(self) -> AutopilotVersion {
        AutopilotVersion {
            // Ground stations only enable their parameter UI once one of the PARAM_ENCODE_* flags
            // says how param_value is encoded. We encode bytewise (see links::protocols::params),
            // so the C-cast flag must stay clear.
            capabilities: MavProtocolCapability::PARAM_ENCODE_BYTEWISE,
            ..Default::default()
        }
    }
}

impl Into<RocketInfo> for &VehicleSnapshot<'_> {
    fn into(self) -> RocketInfo {
        RocketInfo {
            // FIXME:
            // propulsion_type: P::PROPULSION_TYPE,
            propulsion_type: rapid_dialect::rapid::enums::PropulsionType::Hybrid,
            capability_flags: RocketCapability::default(),
        }
    }
}

impl Into<BatteryStatus> for &VehicleSnapshot<'_> {
    #[allow(
        clippy::arithmetic_side_effects,
        reason = "bounded i32 sensor math with nonzero constant divisors, cannot over/underflow or divide by zero"
    )]
    fn into(self) -> BatteryStatus {
        const CELLS: usize = 3;

        let adc = self.readings.power.as_ref();

        let mut voltages: [u16; 10] = [u16::MAX; 10];
        if let Some(pack_mv) = adc.map(|d| d.bus_main_voltage) {
            voltages[0] = pack_mv;
        }

        let current_battery = adc
            .map(|d| (d.fc_current / 10).clamp(i16::MIN as i32, i16::MAX as i32) as i16)
            .unwrap_or(-1);

        let battery_remaining = adc
            .map(|d| {
                let cell_mv = i32::from(d.bus_main_voltage) / CELLS as i32;
                (((cell_mv - 3300) * 100) / (4200 - 3300)).clamp(0, 100) as i8
            })
            .unwrap_or(-1);

        BatteryStatus {
            id: 0x01,
            type_: MavBatteryType::Lion,
            battery_function: MavBatteryFunction::Avionics,
            temperature: i16::MAX,
            voltages,
            current_battery,
            current_consumed: -1,
            energy_consumed: -1,
            battery_remaining,
            time_remaining: 0,
            charge_state: MavBatteryChargeState::Undefined,
            voltages_ext: [u16::MAX; 4],
            mode: MavBatteryMode::Unknown,
            fault_bitmask: MavBatteryFault::default(),
        }
    }
}

/// A message the flight computer sends one of per component.
trait InstanceMessage<I> {
    fn build(snapshot: &VehicleSnapshot<'_>, id: I) -> Self;
}

impl InstanceMessage<TankId> for PressureVessel {
    fn build(snap: &VehicleSnapshot<'_>, tank: TankId) -> Self {
        let p_ids = tank.pressure_sensors();
        let t_ids = tank.temperature_sensors();

        let pressure1 = p_ids[0]
            .map(|id| snap.input_image.press_sens[id])
            .and_then(|o| o.map(|d| d.data))
            .map(|bar| (bar * 100.0).clamp(0.0, f32::from(u16::MAX)) as u16)
            .unwrap_or(u16::MAX);

        let pressure2 = p_ids[1]
            .map(|id| snap.input_image.press_sens[id])
            .and_then(|o| o.map(|d| d.data))
            .map(|bar| (bar * 100.0).clamp(0.0, f32::from(u16::MAX)) as u16)
            .unwrap_or(u16::MAX);

        let temperature1 = t_ids[0]
            .map(|id| snap.input_image.temp_sens[id])
            .and_then(|o| o.map(|d| d.data))
            .map(|celsius| (celsius * 100.0).clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16)
            .unwrap_or(i16::MAX);

        let temperature2 = t_ids[1]
            .map(|id| snap.input_image.temp_sens[id])
            .and_then(|o| o.map(|d| d.data))
            .map(|celsius| (celsius * 100.0).clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16)
            .unwrap_or(i16::MAX);

        let level = (tank == TankId::Oxidizer)
            .then_some(snap.input_image.ox_tank_level.map(|d| d.data))
            .flatten()
            .map(|l| (l * 10000.0).clamp(0.0, f32::from(u16::MAX)) as u16)
            .unwrap_or(u16::MAX);

        PressureVessel {
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
        }
    }
}

impl InstanceMessage<ValveId> for Valve {
    fn build(snap: &VehicleSnapshot<'_>, valve: ValveId) -> Self {
        // Both fields use 0.0 = fully closed, 1.0 = fully open; NAN = unknown.
        let state = snap.input_image.valve_state[valve]
            .map(|state| f32::from(state.data.promille()) / 1000.0)
            .unwrap_or(f32::NAN);
        let commanded = f32::from(snap.output_image.valve[valve].promille()) / 1000.0;

        Valve {
            id: valve,
            state,
            commanded,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use core::f32::consts::FRAC_PI_2;

    use nalgebra::Matrix3;

    /// The board lying on a bench, components up, nosecone end facing north: body z points north,
    /// body y (the board's own up) points up, and body x is what is left.
    fn level_facing_north() -> UnitQuaternion<f32> {
        UnitQuaternion::from_matrix(&Matrix3::from_columns(&[
            -Vector3::x(),
            Vector3::z(),
            Vector3::y(),
        ]))
    }

    /// Nosecone end at compass `heading`, raised `elevation` above the horizon, the board turned
    /// `bank` about the airframe from having its components face up.
    fn attitude(heading: f32, elevation: f32, bank: f32) -> UnitQuaternion<f32> {
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -heading)
            * level_facing_north()
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -elevation)
            * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), bank)
    }

    /// The bench check: a board lying flat with its components up is level, whichever way round it
    /// happens to be lying.
    fn assert_level(orientation: &UnitQuaternion<f32>, expected_yaw: f32) {
        let (roll, pitch, yaw) = euler_angles(orientation);
        assert!(roll.abs() < 1e-3, "roll {roll}");
        assert!(pitch.abs() < 1e-3, "pitch {pitch}");
        assert!((yaw - expected_yaw).abs() < 1e-3, "yaw {yaw}");
    }

    #[test]
    fn a_flat_board_is_level_at_any_heading() {
        for heading in [0.0, 0.7, FRAC_PI_2, -2.5] {
            assert_level(&attitude(heading, 0.0, 0.0), heading);
        }
    }

    /// Yaw is a compass heading: zero north, positive east. Built by hand rather than through
    /// `attitude`, so the helper's own convention is not what is being checked.
    #[test]
    fn yaw_is_a_compass_heading() {
        let level_facing_east = UnitQuaternion::from_matrix(&Matrix3::from_columns(&[
            Vector3::y(),
            Vector3::z(),
            Vector3::x(),
        ]));
        assert_level(&level_facing_east, FRAC_PI_2);
    }

    /// Roll is the board turning about the airframe and nothing else - not the heading, which is
    /// what it tracked while the decomposition put its reference in the world frame.
    #[test]
    fn roll_is_the_board_turning_about_the_airframe() {
        for bank in [-2.0, -0.5, 0.0, 0.5, 2.0] {
            for heading in [0.0, 2.5] {
                for elevation in [0.0, 0.6] {
                    let (roll, pitch, yaw) = euler_angles(&attitude(heading, elevation, bank));

                    assert!((roll - bank).abs() < 1e-3, "roll {roll}, bank {bank}");
                    assert!((pitch - elevation).abs() < 1e-3, "pitch {pitch}");
                    assert!((yaw - heading).abs() < 1e-3, "yaw {yaw}");
                }
            }
        }
    }

    /// Positive roll is right wing down, so with the nose north the board's components turn east.
    #[test]
    fn positive_roll_drops_the_right_hand_side() {
        let banked = attitude(0.0, 0.0, FRAC_PI_2);
        let board_up = banked.transform_vector(&Vector3::y());

        assert!(board_up.x > 0.9, "components face {board_up:?}");
    }

    #[test]
    fn pitch_is_the_nose_above_the_horizon() {
        for elevation in [-1.4, -0.3, 0.0, 0.9, 1.4] {
            let (_, pitch, _) = euler_angles(&attitude(1.0, elevation, 0.8));
            assert!((pitch - elevation).abs() < 1e-3, "pitch {pitch}");
        }
    }
}
