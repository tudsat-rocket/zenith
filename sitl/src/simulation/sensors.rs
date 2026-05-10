//! Generates noisy sensor readings from physical state

use nalgebra::Vector3;
use rand::Rng;

use mission::{AdcData, BaroReading, SensorReadings, Sensors};
use state_estimator::GpsDatum;

use super::SharedSimulation;
use super::battery::Battery;
use super::physics::{FlightPhysics, body_x_from_body_z};

const MACH_M_PER_S: f32 = 343.2;
const GRAVITY: f32 = 9.80665;

/// Sea level pressure [Pa]
const P0: f32 = 101_325.0;
/// Sea level temperature [K]
const T0: f32 = 288.15;
/// Temperature lapse rate [K/m]
const L: f32 = 0.0065;

const METERS_PER_DEGREE_LAT: f32 = 111_111.0;

pub(crate) struct SensorConfig {
    /// Accelerometer sensor standard deviation
    pub accel_noise: f32,
    /// Gyroscope sensor standard deviation
    pub gyro_noise: f32,
    /// Barometer standard deviation
    pub baro_noise: f32,
    /// GPS origin latitude [degrees, decimal]
    pub gps_origin_lat: f32,
    /// GPS origin longitude [degrees, decimal]
    pub gps_origin_lon: f32,
    /// GPS horizontal noise std dev [m]
    pub gps_horizontal_noise: f32,
    /// GPS vertical noise std dev [m]
    pub gps_vertical_noise: f32,
    /// GPS output period [s]
    pub gps_update_period: f32,
}

impl Default for SensorConfig {
    fn default() -> Self {
        const DEFAULT_GPS_ORIGIN_LAT: f32 = 49.854_182;
        const DEFAULT_GPS_ORIGIN_LON: f32 = 8.592_405;

        Self {
            accel_noise: 0.05,
            gyro_noise: 0.01,
            baro_noise: 0.5,
            gps_origin_lat: DEFAULT_GPS_ORIGIN_LAT,
            gps_origin_lon: DEFAULT_GPS_ORIGIN_LON,
            gps_horizontal_noise: 2.5,
            gps_vertical_noise: 5.0,
            gps_update_period: 0.1,
        }
    }
}

struct SensorModel {
    rng: rand::rngs::ThreadRng,
    config: SensorConfig,
    /// Time since the last emitted GPS fix [s]
    gps_time_since_update: f32,
}

impl SensorModel {
    fn new() -> Self {
        Self {
            rng: rand::thread_rng(),
            config: SensorConfig::default(),
            gps_time_since_update: f32::INFINITY,
        }
    }

    /// Read all sensors from the current physics state, adding noise.
    fn sample(&mut self, physics: &FlightPhysics, battery: &Battery) -> SensorReadings {
        self.gps_time_since_update += super::physics::DT;

        let gyro = Some(self.gyroscope(physics));
        let accel = Some(self.accelerometer(physics));
        let mag = Some(self.magnetometer(physics));

        let baro = BaroReading {
            pressure: Some(self.pressure(physics)),
            temperature: Some(self.temperature(physics)),
            altitude: Some(self.baro_altitude(physics)),
        };

        let gps = self.gps(physics);

        let pack_mv = (battery.voltage * 1000.0) as u16;
        let current_ma = (battery.current * 1000.0) as i32;
        let power = AdcData {
            bus_main_voltage: pack_mv,
            bus_supply_voltage: 24000,
            fc_current: current_ma,
            recovery_voltage: pack_mv,
            recovery_current: 0,
            temperature: 0,
        };

        SensorReadings {
            imu1_gyro: gyro,
            imu1_accel: accel,
            imu2_gyro: gyro,
            imu2_accel: accel,
            imu3_gyro: gyro,
            imu3_accel: accel,
            highg_accel: accel,
            mag,
            baro1: baro.clone(),
            baro2: baro.clone(),
            baro3: baro,
            power: Some(power),
            gps,
        }
    }

    /// Body-frame accelerometer reading [m/s^2]
    fn accelerometer(&mut self, physics: &FlightPhysics) -> Vector3<f32> {
        let sf_world = physics.acceleration + Vector3::new(0.0, 0.0, GRAVITY);
        let body = project_to_body(physics, &sf_world);

        let n = self.config.accel_noise;
        let noise = Vector3::new(
            self.rng.gen_range(-n..n),
            self.rng.gen_range(-n..n),
            self.rng.gen_range(-n..n),
        );

        body + noise
    }

    /// Gyroscope reading [deg/s]
    fn gyroscope(&mut self, physics: &FlightPhysics) -> Vector3<f32> {
        let n = self.config.gyro_noise;
        let noise = Vector3::new(
            self.rng.gen_range(-n..n),
            self.rng.gen_range(-n..n),
            self.rng.gen_range(-n..n),
        );
        Vector3::new(
            physics.omega_body.x.to_degrees(),
            physics.omega_body.y.to_degrees(),
            physics.omega_body.z.to_degrees(),
        ) + noise
    }

    /// Magnetometer reading [µT]: earth's field projected onto the body axes
    fn magnetometer(&mut self, physics: &FlightPhysics) -> Vector3<f32> {
        let world = Vector3::new(1.0, 20.0, -43.0);
        let body = project_to_body(physics, &world);

        body + Vector3::new(
            self.rng.gen_range(-0.2..0.2),
            self.rng.gen_range(-0.2..0.2),
            self.rng.gen_range(-0.2..0.2),
        )
    }

    /// Barometric pressure [hPa] from simulated altitude
    fn pressure(&mut self, physics: &FlightPhysics) -> f32 {
        let n = self.config.baro_noise;
        let alt = physics.position.z + self.rng.gen_range(-n..n);
        altitude_to_pressure(alt)
    }

    /// Temperature at altitude [C]
    fn temperature(&self, physics: &FlightPhysics) -> f32 {
        let t_kelvin = T0 - L * physics.position.z;
        t_kelvin - 273.15
    }

    /// Barometric altitude [m ASL]
    fn baro_altitude(&mut self, physics: &FlightPhysics) -> f32 {
        let n = self.config.baro_noise;
        physics.position.z + self.rng.gen_range(-n..n)
    }

    fn gps(&mut self, physics: &FlightPhysics) -> Option<GpsDatum> {
        /// Below this mach number the GPS fix is always kept; above ~1.0 it is always dropped.
        const GPS_FIX_LOSS_MACH_LOW: f32 = 0.6;
        const GPS_FIX_LOSS_MACH_HIGH: f32 = 1.0;

        if self.gps_time_since_update < self.config.gps_update_period {
            return None;
        }

        self.gps_time_since_update = 0.0;

        let mach = physics.velocity.magnitude() / MACH_M_PER_S;
        let loss_prob = ((mach - GPS_FIX_LOSS_MACH_LOW)
            / (GPS_FIX_LOSS_MACH_HIGH - GPS_FIX_LOSS_MACH_LOW))
            .clamp(0.0, 1.0);
        if self.rng.r#gen::<f32>() < loss_prob {
            return None;
        }

        let horiz = self.config.gps_horizontal_noise;
        let vert = self.config.gps_vertical_noise;
        let noise_x = self.rng.gen_range(-horiz..horiz);
        let noise_y = self.rng.gen_range(-horiz..horiz);
        let noise_z = self.rng.gen_range(-vert..vert);

        let origin_lat = self.config.gps_origin_lat;
        let origin_lon = self.config.gps_origin_lon;
        let meters_per_degree_lon = METERS_PER_DEGREE_LAT * origin_lat.to_radians().cos();

        let lat = origin_lat + (physics.position.y + noise_y) / METERS_PER_DEGREE_LAT;
        let lon = origin_lon + (physics.position.x + noise_x) / meters_per_degree_lon;
        let alt = physics.position.z + noise_z;

        let hdop = self.rng.gen_range(80..150);

        Some(GpsDatum {
            latitude: Some(lat),
            longitude: Some(lon),
            altitude: Some(alt),
            hdop,
        })
    }
}

/// Express a world-frame vector in the body frame.
fn project_to_body(physics: &FlightPhysics, world: &Vector3<f32>) -> Vector3<f32> {
    let body_x = body_x_from_body_z(&physics.body_z);
    let body_y = physics.body_z.cross(&body_x);
    Vector3::new(
        world.dot(&body_x),
        world.dot(&body_y),
        world.dot(&physics.body_z),
    )
}

/// ISA pressure model: P = P0 * (1 - L*h/T0)^(g/(L*R))
/// Returns pressure in hPa.
fn altitude_to_pressure(altitude_m: f32) -> f32 {
    let ratio = 1.0 - (L * altitude_m) / T0;
    let pressure_pa = P0 * ratio.powf(GRAVITY / (L * 287.05));
    pressure_pa / 100.0
}

pub struct StdSensors {
    sim: SharedSimulation,
    sensor_model: SensorModel,
}

impl StdSensors {
    pub fn new(sim: SharedSimulation) -> Self {
        Self {
            sim,
            sensor_model: SensorModel::new(),
        }
    }
}

impl Sensors for StdSensors {
    async fn tick(&mut self) -> SensorReadings {
        let sim = self.sim.lock().unwrap();
        self.sensor_model.sample(&sim.physics, &sim.battery)
    }
}
