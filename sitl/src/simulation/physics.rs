use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use nalgebra::Vector3;

use rapid_dialect::FlightMode;

pub const DT: f32 = 0.001;
const GRAVITY: f32 = 9.80665;

const T0: f32 = 288.15;
const L: f32 = 0.0065;
const RHO_0: f32 = 1.225;

#[derive(Clone, Default)]
pub struct RecoveryFlags {
    pub drogue: Arc<AtomicBool>,
    pub main: Arc<AtomicBool>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlightPhase {
    Pad,
    Burn,
    Coast,
    Drogue,
    Main,
    Landed,
}

pub struct DragConfig {
    /// Drag coefficient
    pub cd: f32,
    /// Reference area [m^2]
    pub area: f32,
}

pub struct PhysicsConfig {
    /// Ground altitude ASL [m]
    pub ground_altitude: f32,
    /// Dry mass (without propellant) [kg]
    pub dry_mass: f32,
    /// Propellant mass [kg]
    pub propellant_mass: f32,
    /// Motor thrust [N]
    pub thrust_force: f32,
    /// Burn duration for solids [s]
    #[cfg_attr(feature = "hybrid", allow(dead_code))]
    pub burn_time: f32,
    /// Rocket body aerodynamics
    pub body_drag: DragConfig,
    /// Drogue parachute aerodynamics
    pub drogue_drag: DragConfig,
    /// Main parachute aerodynamics
    pub main_drag: DragConfig,
    /// Launch elevation angle above the horizon [deg]
    pub launch_elevation_deg: f32,
    /// Launch heading [deg]
    pub launch_heading_deg: f32,
    /// Wind speed [m/s]
    pub wind_speed_mps: f32,
    /// Wind heading [deg]
    pub wind_heading_deg: f32,
}

pub struct FlightPhysics {
    /// Simulation config
    pub config: PhysicsConfig,
    /// Phase of flight simulation, driven by this struct
    pub phase: FlightPhase,
    /// Firmware's flight mode, driven by mission::flight_logic
    pub mode: FlightMode,
    /// Current simulation time [s]
    pub time: f32,
    /// Current vehicle mass [kg], decreases during burn
    pub mass: f32,
    /// World-frame position [m], Z is up
    pub position: Vector3<f32>,
    /// World-frame velocity [m/s]
    pub velocity: Vector3<f32>,
    /// Body Z (long axis) in world frame
    pub body_z: Vector3<f32>,
    /// Body-frame angular velocity [rad/s] (pitch,yaw,roll)
    pub omega_body: Vector3<f32>,
    /// World-frame acceleration of the vehicle [m/s^2]
    pub acceleration: Vector3<f32>,
    /// Time in current phase [s]
    phase_time: f32,
    /// Time at which the vehicle was armed [s]
    armed_time: Option<f32>,
    /// Tracks state of parachutes, triggered by mission:flight_logic
    pub flags: RecoveryFlags,
    /// On hybrid builds, the `Burn`-phase thrust is derived from chamber
    /// pressure set each tick by `Simulation`.
    #[cfg(feature = "hybrid")]
    chamber_pressure: f32,
}

impl PhysicsConfig {
    pub(crate) fn launch_axis(&self) -> Vector3<f32> {
        let elev = self.launch_elevation_deg.to_radians();
        let hdg = self.launch_heading_deg.to_radians();
        let horizontal = elev.cos();
        Vector3::new(horizontal * hdg.sin(), horizontal * hdg.cos(), elev.sin())
    }
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            ground_altitude: 100.0,
            dry_mass: 22.0,
            propellant_mass: 8.0,
            thrust_force: 2000.0,
            burn_time: 12.0,
            body_drag: DragConfig {
                cd: 0.8,
                area: 0.017_67,
            },
            drogue_drag: DragConfig {
                cd: 1.5,
                area: 0.200,
            },
            main_drag: DragConfig {
                cd: 2.2,
                area: 2.695,
            },
            launch_elevation_deg: 84.0,
            launch_heading_deg: 250.0, // west-southwest
            wind_speed_mps: 3.0,
            wind_heading_deg: 135.0, // blowing toward southeast
        }
    }
}

impl FlightPhysics {
    pub fn new(flags: RecoveryFlags) -> Self {
        let config = PhysicsConfig::default();
        let body_z = config.launch_axis();
        let mass = config.dry_mass + config.propellant_mass;
        Self {
            time: 0.0,
            mass,
            position: Vector3::new(0.0, 0.0, config.ground_altitude),
            velocity: Vector3::zeros(),
            body_z,
            omega_body: Vector3::zeros(),
            acceleration: Vector3::zeros(),
            phase: FlightPhase::Pad,
            phase_time: 0.0,
            armed_time: None,
            config,
            flags,
            mode: FlightMode::Idle,
            #[cfg(feature = "hybrid")]
            chamber_pressure: 0.0,
        }
    }

    #[cfg(feature = "hybrid")]
    pub fn set_chamber_pressure(&mut self, pressure: f32) {
        self.chamber_pressure = pressure;
    }

    pub fn set_flight_mode(&mut self, mode: FlightMode) {
        if mode == self.mode {
            return;
        }

        self.mode = mode;

        if mode == FlightMode::Idle {
            log::info!("[SIM] Vehicle mode set to Idle, resetting simulation state");
            self.time = 0.0;
            self.mass = self.config.dry_mass + self.config.propellant_mass;
            self.position = Vector3::new(0.0, 0.0, self.config.ground_altitude);
            self.velocity = Vector3::zeros();
            self.body_z = self.config.launch_axis();
            self.omega_body = Vector3::zeros();
            self.acceleration = Vector3::zeros();
            self.phase = FlightPhase::Pad;
            self.phase_time = 0.0;
            self.armed_time = None;
        } else if mode >= FlightMode::Armed && self.armed_time.is_none() {
            log::info!(
                "[SIM] Vehicle armed at t={:.2}s, launching in 5s",
                self.time
            );
            self.armed_time = Some(self.time);
        }
    }

    /// Advance physics by one tick (1 ms).
    pub fn tick(&mut self) {
        self.time += DT;
        self.phase_time += DT;

        if matches!(self.phase, FlightPhase::Pad | FlightPhase::Landed) {
            self.velocity = Vector3::zeros();
            self.acceleration = Vector3::zeros();
        } else {
            let thrust = match self.phase {
                FlightPhase::Burn => {
                    #[cfg(not(feature = "hybrid"))]
                    let thrust_accel = {
                        self.mass -= (self.config.propellant_mass / self.config.burn_time) * DT;
                        self.mass = self.mass.max(self.config.dry_mass);
                        self.config.thrust_force / self.mass
                    };
                    #[cfg(feature = "hybrid")]
                    let thrust_accel = {
                        let normalized = (self.chamber_pressure / 17.0).clamp(0.0, 1.0);
                        self.mass -=
                            normalized * (self.config.propellant_mass / self.config.burn_time) * DT;
                        self.mass = self.mass.max(self.config.dry_mass);
                        normalized * self.config.thrust_force / self.mass
                    };
                    thrust_accel * self.body_z
                }
                _ => Vector3::zeros(),
            };

            let v_rel = self.velocity - self.wind_world();
            let d = match self.phase {
                FlightPhase::Burn | FlightPhase::Coast => &self.config.body_drag,
                FlightPhase::Drogue => &self.config.drogue_drag,
                _ => &self.config.main_drag,
            };
            let rho = RHO_0 * density_ratio(self.position.z);
            let drag = -0.5 * rho * d.cd * d.area / self.mass * v_rel.magnitude() * v_rel;

            self.acceleration = thrust + drag - Vector3::new(0.0, 0.0, GRAVITY);
            self.velocity += self.acceleration * DT;
            self.position += self.velocity * DT;
        }

        match self.phase {
            FlightPhase::Pad => {
                #[cfg(not(feature = "hybrid"))]
                {
                    if let Some(armed_t) = self.armed_time
                        && self.time - armed_t >= 5.0
                    {
                        self.transition(FlightPhase::Burn);
                    }
                }
                #[cfg(feature = "hybrid")]
                {
                    if self.mode == FlightMode::Ignition {
                        self.transition(FlightPhase::Burn);
                    }
                }
            }
            FlightPhase::Burn => {
                #[cfg(not(feature = "hybrid"))]
                if self.phase_time > self.config.burn_time {
                    self.transition(FlightPhase::Coast);
                }
                #[cfg(feature = "hybrid")]
                {
                    let normalized = (self.chamber_pressure / 17.0).clamp(0.0, 1.0);
                    let thrust_accel = normalized * self.config.thrust_force / self.mass;
                    if thrust_accel < 1.0 && self.phase_time > 0.5 {
                        self.transition(FlightPhase::Coast);
                    }
                }
            }
            FlightPhase::Coast => {
                if self.flags.drogue.load(Ordering::Relaxed) {
                    self.transition(FlightPhase::Drogue);
                }
            }
            FlightPhase::Drogue => {
                if self.flags.main.load(Ordering::Relaxed) {
                    self.transition(FlightPhase::Main);
                }
            }
            FlightPhase::Main => {
                if self.altitude_agl() <= 0.0 {
                    self.position.z = self.config.ground_altitude;
                    self.velocity = Vector3::zeros();
                    self.transition(FlightPhase::Landed);
                }
            }
            FlightPhase::Landed => {}
        }

        self.update_orientation();
    }

    /// Refresh body_z to follow velocity (AoA=0) and derive the body-frame
    /// angular velocity from the rotation between ticks. No roll for now.
    fn update_orientation(&mut self) {
        // Hold body_z at the launch axis on the pad / ground; once airborne,
        // align with velocity. A 5 m/s threshold avoids singular behaviour
        // through apogee, where total speed dips through zero on near-vertical
        // launches.
        let new_body_z = if matches!(self.phase, FlightPhase::Pad | FlightPhase::Landed) {
            self.config.launch_axis()
        } else if self.velocity.magnitude() > 5.0 {
            self.velocity.normalize()
        } else {
            self.body_z
        };

        // Small-angle approximation: ω_world ≈ (u × v) / dt for unit vectors.
        let omega_world = self.body_z.cross(&new_body_z) / DT;

        let body_x = body_x_from_body_z(&self.body_z);
        let body_y = self.body_z.cross(&body_x);

        self.omega_body = Vector3::new(omega_world.dot(&body_x), omega_world.dot(&body_y), 0.0);
        self.body_z = new_body_z;
    }

    pub fn altitude_agl(&self) -> f32 {
        self.position.z - self.config.ground_altitude
    }

    /// World-frame wind velocity vector [m/s], horizontal only
    fn wind_world(&self) -> Vector3<f32> {
        let hdg = self.config.wind_heading_deg.to_radians();
        let speed = self.config.wind_speed_mps;
        Vector3::new(speed * hdg.sin(), speed * hdg.cos(), 0.0)
    }

    fn transition(&mut self, new_phase: FlightPhase) {
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

pub(crate) fn body_x_from_body_z(body_z: &Vector3<f32>) -> Vector3<f32> {
    let world_up = Vector3::new(0.0, 0.0, 1.0);
    let cross = world_up.cross(body_z);
    if cross.norm_squared() > 1e-6 {
        cross.normalize()
    } else {
        Vector3::new(1.0, 0.0, 0.0)
    }
}

fn density_ratio(altitude_m: f32) -> f32 {
    let base = 1.0 - (L * altitude_m) / T0;
    base.powf(GRAVITY / (L * 287.05) - 1.0)
}
