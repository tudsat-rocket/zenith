use core::f32::consts::PI;

use embassy_executor::Spawner;
use embassy_stm32::eth::{Ethernet, GenericPhy};
use embassy_stm32::peripherals::*;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_sync::pubsub::PubSubChannel;

use mavio::prelude::V2;
use mavio::{Frame, Message};
use nalgebra::{Quaternion, Unit};

use rapid_dialect::rapid::enums::{
    MavAutopilot, MavBatteryChargeState, MavBatteryFault, MavBatteryFunction, MavBatteryMode,
    MavBatteryType, MavCmd, MavModeFlag, MavResult, MavType,
};
use rapid_dialect::rapid::messages::{
    Attitude, AvailableModes, BatteryStatus, CommandAck, Heartbeat, LocalPositionNed, ScaledImu,
    ScaledImu2, ScaledImu3, ScaledPressure, ScaledPressure2, ScaledPressure3,
};
use rapid_dialect::{FlightMode, Rapid};

use crate::vehicle::Vehicle;

impl Into<Heartbeat> for &Vehicle {
    fn into(self) -> Heartbeat {
        Heartbeat {
            type_: MavType::Rocket,
            autopilot: MavAutopilot::Generic,
            base_mode: MavModeFlag::CUSTOM_MODE_ENABLED,
            custom_mode: self.mode() as u32,
            system_status: self.mode().into(),
            mavlink_version: 2,
        }
    }
}

impl Into<Attitude> for &Vehicle {
    fn into(self) -> Attitude {
        use num_traits::Float;

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

impl Into<LocalPositionNed> for &Vehicle {
    fn into(self) -> LocalPositionNed {
        // TODO: check x/y components
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

// TODO: where do we put high-g accelerometer?
impl Into<ScaledImu> for &Vehicle {
    fn into(self) -> ScaledImu {
        let acc1 = self.sensors.imu1.accelerometer();
        let gyro1 = self.sensors.imu1.gyroscope();
        let mag1 = self.sensors.mag.magnetometer();

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

impl Into<ScaledImu2> for &Vehicle {
    fn into(self) -> ScaledImu2 {
        let acc2 = self.sensors.imu2.accelerometer();
        let gyro2 = self.sensors.imu2.gyroscope();

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

impl Into<ScaledImu3> for &Vehicle {
    fn into(self) -> ScaledImu3 {
        let acc3 = self.sensors.imu3.accelerometer();
        let gyro3 = self.sensors.imu3.gyroscope();

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

impl Into<ScaledPressure> for &Vehicle {
    fn into(self) -> ScaledPressure {
        ScaledPressure {
            time_boot_ms: self.time.0,
            press_abs: self.sensors.baro1.pressure().unwrap_or_default(),
            press_diff: 0.0,
            temperature: (self.sensors.baro1.temperature().unwrap_or_default() * 10.0) as i16,
            temperature_press_diff: 0,
        }
    }
}

impl Into<ScaledPressure2> for &Vehicle {
    fn into(self) -> ScaledPressure2 {
        ScaledPressure2 {
            time_boot_ms: self.time.0,
            press_abs: self.sensors.baro2.pressure().unwrap_or_default(),
            press_diff: 0.0,
            temperature: (self.sensors.baro2.temperature().unwrap_or_default() * 10.0) as i16,
            temperature_press_diff: 0,
        }
    }
}

impl Into<ScaledPressure3> for &Vehicle {
    fn into(self) -> ScaledPressure3 {
        ScaledPressure3 {
            time_boot_ms: self.time.0,
            press_abs: self.sensors.baro3.pressure().unwrap_or_default(),
            press_diff: 0.0,
            temperature: (self.sensors.baro3.temperature().unwrap_or_default() * 10.0) as i16,
            temperature_press_diff: 0,
        }
    }
}

// TODO: battery status for other batteries
impl Into<BatteryStatus> for &Vehicle {
    fn into(self) -> BatteryStatus {
        let adc = self.power.adc();

        let mut voltages: [u16; 10] = [u16::MAX; 10];
        voltages[0] = adc.as_ref().map(|d| d.bus_main_voltage).unwrap_or(u16::MAX);

        BatteryStatus {
            id: 0x01,
            type_: MavBatteryType::Lion,
            battery_function: MavBatteryFunction::Avionics,
            temperature: i16::MAX,
            voltages,
            // TODO: proper units
            current_battery: (adc.as_ref().map(|d| d.fc_current).unwrap_or(0)) as i16,
            // TODO: remove/find somewhere else to put this
            current_consumed: (adc.as_ref().map(|d| d.recovery_current).unwrap_or(0)),
            // TODO: remove/find somewhere else to put this
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
