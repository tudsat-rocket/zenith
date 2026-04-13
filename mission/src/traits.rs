use nalgebra::Vector3;
use state_estimator::StateEstimatorSettings;

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
    pub imu1_gyro: Option<Vector3<f32>>,
    pub imu1_accel: Option<Vector3<f32>>,
    pub imu2_gyro: Option<Vector3<f32>>,
    pub imu2_accel: Option<Vector3<f32>>,
    pub imu3_gyro: Option<Vector3<f32>>,
    pub imu3_accel: Option<Vector3<f32>>,
    pub highg_accel: Option<Vector3<f32>>,
    pub mag: Option<Vector3<f32>>,
    pub baro1: BaroReading,
    pub baro2: BaroReading,
    pub baro3: BaroReading,
    pub power: Option<AdcData>,
}

pub trait Sensors {
    async fn tick(&mut self) -> SensorReadings;
}

pub trait Outputs {
    fn set_recovery_armed(&mut self, armed: bool);
    fn set_drogue(&mut self, high: bool);
    fn set_main(&mut self, high: bool);
}

#[derive(Debug, Default)]
pub struct Settings {
    pub state_estimator: StateEstimatorSettings,
    pub recovery: RecoverySettings,
}

#[derive(Debug)]
pub struct RecoverySettings {
    /// Altitude AGL (meters) at which to deploy the main parachute
    pub main_deploy_altitude: f32,
    /// Minimum time (ms) after launch before allowing drogue deployment
    pub min_time_to_drogue: u32,
    /// Minimum time (ms) after drogue before allowing main deployment
    pub min_time_to_main: u32,
}

impl Default for RecoverySettings {
    fn default() -> Self {
        Self {
            main_deploy_altitude: 450.0,
            min_time_to_drogue: 1000,
            min_time_to_main: 3000,
        }
    }
}

pub trait Storage {
    async fn read_settings(&mut self) -> Option<Settings>;
    async fn write_settings(&mut self, settings: &Settings);
}

#[derive(Default)]
pub struct NoStorage;

impl Storage for NoStorage {
    async fn read_settings(&mut self) -> Option<Settings> {
        None
    }

    async fn write_settings(&mut self, _settings: &Settings) {}
}
