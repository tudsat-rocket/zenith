#[cfg(target_os = "none")]
use num_traits::Float as _;
use rapid_dialect::FlightMode;
use rapid_dialect::rapid::enums::{
    GpsFixType, MavAutopilot, MavBatteryChargeState, MavBatteryFault, MavBatteryFunction,
    MavBatteryMode, MavBatteryType, MavModeFlag, MavSysStatusSensor, MavSysStatusSensorExtended,
    MavType, RocketCapability,
};
use rapid_dialect::rapid::messages::{
    Attitude, BatteryStatus, GlobalPositionInt, GpsRawInt, Heartbeat, LocalPositionNed, RocketInfo,
    ScaledImu, ScaledImu2, ScaledImu3, ScaledPressure, ScaledPressure2, ScaledPressure3, SysStatus,
    VfrHud,
};

use crate::propulsion::Propulsion;
use crate::traits::{Outputs, Sensors, Storage};
use crate::vehicle::Vehicle;

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<Heartbeat> for &Vehicle<S, O, F, P> {
    fn into(self) -> Heartbeat {
        Heartbeat {
            type_: MavType::Rocket,
            autopilot: MavAutopilot::Generic,
            // TODO: rethink how we want to use the "armed" term
            base_mode: if self.mode() >= FlightMode::Armed {
                MavModeFlag::CUSTOM_MODE_ENABLED | MavModeFlag::SAFETY_ARMED
            } else {
                MavModeFlag::CUSTOM_MODE_ENABLED
            },
            custom_mode: self.mode() as u32,
            system_status: self.mode().into(),
            mavlink_version: 2,
        }
    }
}

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<Attitude> for &Vehicle<S, O, F, P> {
    fn into(self) -> Attitude {
        let q = self.state_estimator.orientation.unwrap_or_default();
        let body_z_world = q.transform_vector(&nalgebra::Vector3::new(0.0, 0.0, 1.0));
        let pitch = body_z_world.z.clamp(-1.0, 1.0).asin();
        let yaw = (-body_z_world.y).atan2(body_z_world.x);
        let gyro = self.readings.imu1_gyro.unwrap_or_default();

        Attitude {
            time_boot_ms: self.time.0,
            roll: 0.0,
            pitch,
            yaw,
            rollspeed: gyro.z.to_radians(),
            pitchspeed: gyro.x.to_radians(),
            yawspeed: gyro.y.to_radians(),
        }
    }
}

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<LocalPositionNed>
    for &Vehicle<S, O, F, P>
{
    fn into(self) -> LocalPositionNed {
        let pos = self.state_estimator.position_local();
        let vel = self.state_estimator.velocity();

        LocalPositionNed {
            time_boot_ms: self.time.0,
            x: pos.y,
            y: pos.x,
            z: -pos.z,
            vx: vel.y,
            vy: vel.x,
            vz: -vel.z,
        }
    }
}

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<GlobalPositionInt>
    for &Vehicle<S, O, F, P>
{
    fn into(self) -> GlobalPositionInt {
        let vel = self.state_estimator.velocity();

        let q = self.state_estimator.orientation.unwrap_or_default();
        let (_roll, _pitch, yaw) = q.euler_angles();

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

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<GpsRawInt> for &Vehicle<S, O, F, P> {
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
                .unwrap_or(0),
            lon: gps
                .and_then(|g| g.longitude)
                .map(|v| (f64::from(v) * 1e7) as i32)
                .unwrap_or(0),
            alt: gps
                .and_then(|g| g.altitude)
                .map(|v| (v * 1000.0) as i32)
                .unwrap_or(0),
            eph: gps.map(|g| g.hdop).unwrap_or(u16::MAX),
            epv: u16::MAX,
            vel: u16::MAX,
            cog: u16::MAX,
            satellites_visible: u8::MAX,
            alt_ellipsoid: 0,
            h_acc: 0,
            v_acc: 0,
            vel_acc: 0,
            hdg_acc: 0,
            yaw: 0,
        }
    }
}

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<ScaledImu> for &Vehicle<S, O, F, P> {
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

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<ScaledImu2> for &Vehicle<S, O, F, P> {
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

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<ScaledImu3> for &Vehicle<S, O, F, P> {
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

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<ScaledPressure>
    for &Vehicle<S, O, F, P>
{
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

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<ScaledPressure2>
    for &Vehicle<S, O, F, P>
{
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

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<ScaledPressure3>
    for &Vehicle<S, O, F, P>
{
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

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<VfrHud> for &Vehicle<S, O, F, P> {
    fn into(self) -> VfrHud {
        let q = self.state_estimator.orientation.unwrap_or_default();
        let (_roll, _pitch, yaw) = q.euler_angles();
        let vel = self.state_estimator.velocity();

        VfrHud {
            airspeed: 0.0,
            groundspeed: vel.xy().magnitude(),
            heading: ({
                let d = yaw.to_degrees();
                d - (d / 360.0).floor() * 360.0
            }) as i16,
            throttle: if self.mode() == FlightMode::Burn {
                100
            } else {
                0
            },
            alt: self.state_estimator.altitude_asl(),
            climb: vel.z,
        }
    }
}

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<SysStatus> for &Vehicle<S, O, F, P> {
    fn into(self) -> SysStatus {
        let r = &self.readings;

        let hw_armed = self.mode() >= FlightMode::HardwareArmed;
        let armed = self.mode() >= FlightMode::Armed;

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
        if self.mode() != FlightMode::Burn && self.mode() != FlightMode::Ignition {
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
        if self.mode() >= FlightMode::HardwareArmed {
            health |= MavSysStatusSensor::MOTOR_OUTPUTS;
        }
        if self.mode() >= FlightMode::HardwareArmed
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

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<RocketInfo> for &Vehicle<S, O, F, P> {
    fn into(self) -> RocketInfo {
        RocketInfo {
            propulsion_type: P::PROPULSION_TYPE,
            capability_flags: RocketCapability::default(),
        }
    }
}

impl<S: Sensors, O: Outputs, F: Storage, P: Propulsion> Into<BatteryStatus>
    for &Vehicle<S, O, F, P>
{
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
