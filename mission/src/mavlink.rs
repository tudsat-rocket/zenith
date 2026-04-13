use rapid_dialect::rapid::enums::{
    MavAutopilot, MavBatteryChargeState, MavBatteryFault, MavBatteryFunction, MavBatteryMode,
    MavBatteryType, MavModeFlag, MavType,
};
use rapid_dialect::FlightMode;
use rapid_dialect::rapid::messages::{
    Attitude, BatteryStatus, Heartbeat, LocalPositionNed, ScaledImu, ScaledImu2, ScaledImu3,
    ScaledPressure, ScaledPressure2, ScaledPressure3,
};

use crate::traits::{Outputs, Sensors, Storage};
use crate::vehicle::Vehicle;

impl<S: Sensors, O: Outputs, F: Storage> Into<Heartbeat> for &Vehicle<S, O, F> {
    fn into(self) -> Heartbeat {
        Heartbeat {
            type_: MavType::Rocket,
            autopilot: MavAutopilot::Generic,
            base_mode: if self.mode() >= FlightMode::HardwareArmed {
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

impl<S: Sensors, O: Outputs, F: Storage> Into<Attitude> for &Vehicle<S, O, F> {
    fn into(self) -> Attitude {
        let q = self.state_estimator.orientation.unwrap_or_default();
        let (roll, pitch, yaw) = q.euler_angles();

        Attitude {
            time_boot_ms: self.time.0,
            roll: pitch, // TODO
            pitch: roll,
            yaw,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
        }
    }
}

impl<S: Sensors, O: Outputs, F: Storage> Into<LocalPositionNed> for &Vehicle<S, O, F> {
    fn into(self) -> LocalPositionNed {
        LocalPositionNed {
            time_boot_ms: self.time.0,
            x: self.state_estimator.position_local().x,
            y: self.state_estimator.position_local().y,
            z: -self.state_estimator.position_local().z,
            vx: self.state_estimator.velocity().x,
            vy: self.state_estimator.velocity().y,
            vz: -self.state_estimator.velocity().z,
        }
    }
}

impl<S: Sensors, O: Outputs, F: Storage> Into<ScaledImu> for &Vehicle<S, O, F> {
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

impl<S: Sensors, O: Outputs, F: Storage> Into<ScaledImu2> for &Vehicle<S, O, F> {
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

impl<S: Sensors, O: Outputs, F: Storage> Into<ScaledImu3> for &Vehicle<S, O, F> {
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

impl<S: Sensors, O: Outputs, F: Storage> Into<ScaledPressure> for &Vehicle<S, O, F> {
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

impl<S: Sensors, O: Outputs, F: Storage> Into<ScaledPressure2> for &Vehicle<S, O, F> {
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

impl<S: Sensors, O: Outputs, F: Storage> Into<ScaledPressure3> for &Vehicle<S, O, F> {
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

impl<S: Sensors, O: Outputs, F: Storage> Into<BatteryStatus> for &Vehicle<S, O, F> {
    fn into(self) -> BatteryStatus {
        let adc = &self.readings.power;

        let mut voltages: [u16; 10] = [u16::MAX; 10];
        voltages[0] = adc.as_ref().map(|d| d.bus_main_voltage).unwrap_or(u16::MAX);

        BatteryStatus {
            id: 0x01,
            type_: MavBatteryType::Lion,
            battery_function: MavBatteryFunction::Avionics,
            temperature: i16::MAX,
            voltages,
            current_battery: (adc.as_ref().map(|d| d.fc_current).unwrap_or(0)) as i16,
            current_consumed: (adc.as_ref().map(|d| d.recovery_current).unwrap_or(0)),
            energy_consumed: (adc.as_ref().map(|d| d.recovery_voltage).unwrap_or(0)) as i32,
            battery_remaining: -1,
            time_remaining: 0,
            charge_state: MavBatteryChargeState::Undefined,
            voltages_ext: [u16::MAX; 4],
            mode: MavBatteryMode::Unknown,
            fault_bitmask: MavBatteryFault::default(),
        }
    }
}
