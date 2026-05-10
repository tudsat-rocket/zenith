use nalgebra::Vector3;

use rapid_dialect::Rapid;

use links::UplinkCommand;
use state_estimator::GpsDatum;

use crate::Settings;

#[derive(Clone, Default)]
pub struct AdcData {
    pub bus_main_voltage: u16,
    pub bus_supply_voltage: u16,
    pub fc_current: i32,
    pub recovery_voltage: u16,
    pub recovery_current: i32,
    pub temperature: i32,
}

#[derive(Clone, Default)]
pub struct BaroReading {
    pub pressure: Option<f32>,
    pub temperature: Option<f32>,
    pub altitude: Option<f32>,
}

#[derive(Clone, Default)]
pub struct SensorReadings {
    /// IMU1 angular rate [deg/s]
    pub imu1_gyro: Option<Vector3<f32>>,
    /// IMU1 acceleration [m/s^2]
    pub imu1_accel: Option<Vector3<f32>>,
    /// IMU2 angular rate [deg/s]
    pub imu2_gyro: Option<Vector3<f32>>,
    /// IMU2 acceleration [m/s^2]
    pub imu2_accel: Option<Vector3<f32>>,
    /// IMU3 angular rate [deg/s]
    pub imu3_gyro: Option<Vector3<f32>>,
    /// IMU3 acceleration [m/s^2]
    pub imu3_accel: Option<Vector3<f32>>,
    /// High-G accel. acceleration [m/s^2]
    pub highg_accel: Option<Vector3<f32>>,
    /// Magnetometer reading [µT]
    pub mag: Option<Vector3<f32>>,
    /// Barometer 1 reading
    pub baro1: BaroReading,
    /// Barometer 2 reading
    pub baro2: BaroReading,
    /// Barometer 3 reading
    pub baro3: BaroReading,
    /// ADC / Power data
    pub power: Option<AdcData>,
    /// GPS reading
    pub gps: Option<GpsDatum>,
}

pub trait Sensors {
    async fn tick(&mut self) -> SensorReadings;
}

pub trait Outputs {
    fn set_recovery_armed(&mut self, armed: bool);
    fn set_drogue(&mut self, high: bool);
    fn set_main(&mut self, high: bool);
}

pub trait TelemetryLink {
    fn send_message(&mut self, message: Rapid);
    fn try_recv_command(&mut self) -> Option<UplinkCommand>;
}

pub trait Storage {
    async fn read_settings(&mut self) -> Option<Settings>;
    async fn write_settings(&mut self, settings: &Settings);
}

// This is only here because the firmware doesn't have a storage impl yet. once that's in, this
// moves to SITL.
#[derive(Default)]
pub struct NoStorage;

impl Storage for NoStorage {
    async fn read_settings(&mut self) -> Option<Settings> {
        None
    }

    async fn write_settings(&mut self, _settings: &Settings) {}
}
