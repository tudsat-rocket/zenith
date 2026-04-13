use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use nalgebra::Vector3;
use rand::Rng;
use rapid_dialect::FlightMode;

/// Shared handle between `StdOutputs` (writer) and `FlightSimulation` (reader)
/// for recovery output state. One handle per vehicle instance so multiple
/// simulations can run in parallel (required for integration tests).
#[derive(Clone, Default)]
pub struct RecoveryFlags {
    pub drogue: Arc<AtomicBool>,
    pub main: Arc<AtomicBool>,
}

const GRAVITY: f32 = 9.80665;
const DT: f32 = 0.001; // 1kHz tick rate

// ISA pressure model constants
const P0: f32 = 101_325.0; // sea level pressure (Pa)
const T0: f32 = 288.15; // sea level temperature (K)
const L: f32 = 0.0065; // temperature lapse rate (K/m)

/// Simple rocket flight simulation using Euler integration.
pub struct FlightSimulation {
    /// Current simulation time (seconds)
    pub time: f32,
    /// World-frame position (m), Z is up
    pub position: Vector3<f32>,
    /// World-frame velocity (m/s)
    pub velocity: Vector3<f32>,

    phase: Phase,
    phase_time: f32,
    /// Time at which the vehicle was armed (None = not yet armed)
    armed_time: Option<f32>,

    config: SimConfig,
    rng: rand::rngs::ThreadRng,
    flags: RecoveryFlags,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    Pad,
    Burn,
    Coast,
    Drogue,
    Main,
    Landed,
}

struct SimConfig {
    /// Ground altitude ASL (m)
    ground_altitude: f32,
    /// Thrust acceleration (m/s^2), applied along Z
    thrust_accel: f32,
    /// Burn duration (s)
    burn_time: f32,
    /// Simple drag coefficient (1/s), force = -coeff * v^2 * sign(v)
    drag_coeff: f32,
    /// Drogue descent rate (m/s), negative = down
    drogue_descent_rate: f32,
    /// Main chute descent rate (m/s), negative = down
    main_descent_rate: f32,
    /// Sensor noise standard deviations
    accel_noise: f32,
    gyro_noise: f32,
    baro_noise: f32,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            ground_altitude: 100.0,
            thrust_accel: 120.0, // ~12G
            burn_time: 3.0,
            drag_coeff: 0.001,
            drogue_descent_rate: -15.0,
            main_descent_rate: -5.0,
            accel_noise: 0.05,
            gyro_noise: 0.01,
            baro_noise: 0.5,
        }
    }
}

impl FlightSimulation {
    pub fn new(flags: RecoveryFlags) -> Self {
        let config = SimConfig::default();
        Self {
            time: 0.0,
            position: Vector3::new(0.0, 0.0, config.ground_altitude),
            velocity: Vector3::zeros(),
            phase: Phase::Pad,
            phase_time: 0.0,
            armed_time: None,
            config,
            rng: rand::thread_rng(),
            flags,
        }
    }

    /// Notify the simulation of the current vehicle flight mode so it knows
    /// when arming happens.
    pub fn set_flight_mode(&mut self, mode: FlightMode) {
        if mode >= FlightMode::Armed && self.armed_time.is_none() {
            log::info!(
                "[SIM] Vehicle armed at t={:.2}s, launching in 5s",
                self.time
            );
            self.armed_time = Some(self.time);
        }
    }

    /// Advance simulation by one tick. Call at 1kHz.
    pub fn tick(&mut self) {
        self.time += DT;
        self.phase_time += DT;

        match self.phase {
            // Stay on pad until armed, then wait 5 seconds before ignition
            Phase::Pad => {
                if let Some(armed_t) = self.armed_time
                    && self.time - armed_t >= 5.0
                {
                    self.transition(Phase::Burn);
                }
            }
            Phase::Burn => {
                let accel_z = self.config.thrust_accel - GRAVITY;
                self.velocity.z += accel_z * DT;
                self.position.z += self.velocity.z * DT;

                if self.phase_time > self.config.burn_time {
                    self.transition(Phase::Coast);
                }
            }
            Phase::Coast => {
                // Freefall with drag until recovery outputs fire
                let drag = -self.config.drag_coeff * self.velocity.z * self.velocity.z.abs();
                let accel_z = -GRAVITY + drag;
                self.velocity.z += accel_z * DT;
                self.position.z += self.velocity.z * DT;

                if self.flags.drogue.load(Ordering::Relaxed) {
                    self.transition(Phase::Drogue);
                }
            }
            Phase::Drogue => {
                // Drogue chute: approach drogue descent rate
                let target = self.config.drogue_descent_rate;
                self.velocity.z += (target - self.velocity.z) * 2.0 * DT;
                self.position.z += self.velocity.z * DT;

                if self.flags.main.load(Ordering::Relaxed) {
                    self.transition(Phase::Main);
                }
            }
            Phase::Main => {
                let target = self.config.main_descent_rate;
                self.velocity.z += (target - self.velocity.z) * 2.0 * DT;
                self.position.z += self.velocity.z * DT;

                if self.altitude_agl() <= 0.0 {
                    self.position.z = self.config.ground_altitude;
                    self.velocity = Vector3::zeros();
                    self.transition(Phase::Landed);
                }
            }
            Phase::Landed => {}
        }
    }

    pub fn altitude_agl(&self) -> f32 {
        self.position.z - self.config.ground_altitude
    }

    /// Body-frame accelerometer reading.
    /// Measures specific force (all non-gravitational forces).
    /// Rocket body Z axis points up.
    pub fn accelerometer(&mut self) -> Vector3<f32> {
        let specific_force_z = match self.phase {
            Phase::Pad | Phase::Landed => GRAVITY, // normal force
            Phase::Burn => self.config.thrust_accel,
            Phase::Coast => -self.config.drag_coeff * self.velocity.z * self.velocity.z.abs(),
            Phase::Drogue | Phase::Main => {
                // Under chute: drag counters most of gravity
                // specific_force = total_accel + g (since accel = drag - g)
                // When at terminal velocity, accel = 0, so specific_force = g
                let target = match self.phase {
                    Phase::Drogue => self.config.drogue_descent_rate,
                    _ => self.config.main_descent_rate,
                };
                let diff = target - self.velocity.z;
                // Drag provides deceleration toward terminal velocity
                GRAVITY + diff * 2.0
            }
        };

        let noise = Vector3::new(
            self.rng
                .gen_range(-self.config.accel_noise..self.config.accel_noise),
            self.rng
                .gen_range(-self.config.accel_noise..self.config.accel_noise),
            self.rng
                .gen_range(-self.config.accel_noise..self.config.accel_noise),
        );

        Vector3::new(0.0, 0.0, specific_force_z) + noise
    }

    /// Gyroscope reading (deg/s). Rocket doesn't rotate much in this sim.
    pub fn gyroscope(&mut self) -> Vector3<f32> {
        Vector3::new(
            self.rng
                .gen_range(-self.config.gyro_noise..self.config.gyro_noise),
            self.rng
                .gen_range(-self.config.gyro_noise..self.config.gyro_noise),
            self.rng
                .gen_range(-self.config.gyro_noise..self.config.gyro_noise),
        )
    }

    /// Magnetometer reading (gauss). Roughly earth's field, north + down.
    pub fn magnetometer(&mut self) -> Vector3<f32> {
        // Approximate earth field for central Europe, body frame (rocket pointing up)
        // X = north, Y = east, Z = down in NED; but body Z = up, so flip
        Vector3::new(
            0.20 + self.rng.gen_range(-0.002..0.002),
            0.01 + self.rng.gen_range(-0.002..0.002),
            -0.43 + self.rng.gen_range(-0.002..0.002),
        )
    }

    /// Barometric pressure (hPa) from simulated altitude.
    pub fn pressure(&mut self) -> f32 {
        let alt = self.position.z
            + self
                .rng
                .gen_range(-self.config.baro_noise..self.config.baro_noise);
        altitude_to_pressure(alt)
    }

    /// Temperature at altitude (C).
    pub fn temperature(&self) -> f32 {
        let t_kelvin = T0 - L * self.position.z;
        t_kelvin - 273.15
    }

    /// Barometric altitude (m ASL).
    pub fn baro_altitude(&mut self) -> f32 {
        self.position.z
            + self
                .rng
                .gen_range(-self.config.baro_noise..self.config.baro_noise)
    }

    fn transition(&mut self, new_phase: Phase) {
        log::info!(
            "[SIM] Phase transition: {:?} -> {:?} (t={:.2}s, alt_agl={:.1}m, vz={:.1}m/s)",
            self.phase,
            new_phase,
            self.time,
            self.altitude_agl(),
            self.velocity.z,
        );
        self.phase = new_phase;
        self.phase_time = 0.0;
    }
}

/// ISA pressure model: P = P0 * (1 - L*h/T0)^(g/(L*R))
/// Returns pressure in hPa.
fn altitude_to_pressure(altitude_m: f32) -> f32 {
    let ratio = 1.0 - (L * altitude_m) / T0;
    let pressure_pa = P0 * ratio.powf(GRAVITY / (L * 287.05));
    pressure_pa / 100.0 // Pa -> hPa
}
