#![cfg_attr(target_os = "none", no_std)] // this is imported by the firmware, so no standard library

#[cfg(target_os = "none")]
use core::num::Wrapping;
#[cfg(not(target_os = "none"))]
use std::num::Wrapping;

use ahrs::Ahrs;
use filter::kalman::kalman_filter::KalmanFilter;
use nalgebra::*;
use rapid_dialect::FlightMode;

pub const GRAVITY: f32 = 9.80665;
const GPS_NO_FIX_STD_DEV: f32 = 999_999.0;

#[derive(Debug, Clone, Default)]
pub struct GpsDatum {
    // TODO: include fix enum here?
    pub latitude: Option<f32>,
    pub longitude: Option<f32>,
    pub altitude: Option<f32>,
    /// Horizontal dilution of precision * 100
    pub hdop: u16,
}

#[derive(Debug, Clone)]
pub struct StateEstimatorSettings {
    /// proportional filter gain for Mahony attitude estimator
    pub mahony_kp: f32,
    /// integral filter gain for Mahony attitude estimator
    pub mahony_ki: f32,
    /// proportional filter gain for Mahony attitude estimator
    pub mahony_kp_ascent: f32,
    /// integral filter gain for Mahony attitude estimator
    pub mahony_ki_ascent: f32,
    /// accelerometer standard deviation for kalman filter
    pub std_dev_accelerometer: f32,
    /// barometer standard deviation for kalman filter
    pub std_dev_barometer: f32,
    /// barometer standard deviation for kalman filter when in the transsonic region
    pub std_dev_barometer_transsonic: f32,
    /// process standard deviation for kalman filter
    pub std_dev_process: f32,
}

impl Default for StateEstimatorSettings {
    fn default() -> Self {
        Self {
            mahony_kp: 0.1,
            mahony_ki: 0.0,
            mahony_kp_ascent: 0.1,
            mahony_ki_ascent: 0.0,
            std_dev_accelerometer: 0.5,
            std_dev_barometer: 10.0,
            std_dev_barometer_transsonic: 5000.0,
            std_dev_process: 0.5,
        }
    }
}

#[derive(Debug)]
pub struct StateEstimator {
    /// current time
    time: Wrapping<u32>,
    /// current flight mode
    mode: FlightMode,
    /// time current flight mode was entered
    mode_time: Wrapping<u32>,
    /// time of takeoff (entering Burn)
    takeoff_time: Wrapping<u32>,
    /// settings
    settings: StateEstimatorSettings,
    /// orientation
    ahrs: ahrs::Mahony<f32>,
    /// main Kalman filter
    pub kalman: KalmanFilter<f32, U9, U6, U0>,
    /// current orientation
    pub orientation: Option<Unit<Quaternion<f32>>>,
    /// current vehicle-space acceleration, switched between low- and high-G accelerometer
    acceleration: Option<Vector3<f32>>,
    /// world-space acceleration, rotated using estimated orientation
    acceleration_world: Option<Vector3<f32>>,
    /// altitude (ASL) at ground level, to allow calculating AGL altitude. locked in when armed.
    pub altitude_ground: f32,
    /// apogee (ASL)
    pub altitude_max: f32,
    /// GPS origin (lat, lng)
    // TODO: move altitude_ground into here?
    gps_origin: Option<Vector3<f32>>,
    last_covariance_update: Wrapping<u32>,
    #[cfg(not(target_os = "none"))]
    pub last_apogee_error: f32,
}

impl StateEstimator {
    pub fn new(main_loop_freq_hertz: f32, settings: StateEstimatorSettings) -> Self {
        Self::new_with_quat(main_loop_freq_hertz, settings, UnitQuaternion::default())
    }

    pub fn new_with_quat(
        main_loop_freq_hertz: f32,
        settings: StateEstimatorSettings,
        initial_orientation: UnitQuaternion<f32>,
    ) -> Self {
        let dt = 1.0 / main_loop_freq_hertz;
        let ahrs = ahrs::Mahony::new_with_quat(
            dt,
            settings.mahony_kp,
            settings.mahony_ki,
            initial_orientation,
        );

        let kalman = KalmanFilter {
            // State Vector
            x: vector![
                0.0, 0.0, 0.0, // XYZ position (m)
                0.0, 0.0, 0.0, // XYZ velocity (m/s)
                0.0, 0.0, 0.0 // XYZ acceleration (m/s^2)
            ],
            // State Transition Matrix
            F: matrix![
                    1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.5 * dt * dt, 0.0, 0.0;
                    0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.5 * dt * dt, 0.0;
                    0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.5 * dt * dt;
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0;
                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0;
                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt;
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
            ],
            // Measurement Matrix
            H: matrix![
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // barometer measures Z pos
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0; // acceleration X
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0; // acceleration Y
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0; // acceleration Z
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // GPS X pos
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // GPS Y pos
            ],
            // State Covariance Matrix (initialized to a high value)
            P: Matrix::<f32, U9, U9, _>::identity() * 999.0,
            // Process Covariance Matrix
            Q: matrix![
                0.25f32 * dt.powi(4), 0.0, 0.0, 0.5f32 * dt.powi(3), 0.0, 0.0, 0.5f32 * dt.powi(2), 0.0, 0.0;
                0.0, 0.25f32 * dt.powi(4), 0.0, 0.0, 0.5f32 * dt.powi(3), 0.0, 0.0, 0.5f32 * dt.powi(2), 0.0;
                0.0, 0.0, 0.25f32 * dt.powi(4), 0.0, 0.0, 0.5f32 * dt.powi(3), 0.0, 0.0, 0.5f32 * dt.powi(2);
                0.5f32 * dt.powi(3), 0.0, 0.0, dt.powi(2), 0.0, 0.0, dt, 0.0, 0.0;
                0.0, 0.5f32 * dt.powi(3), 0.0, 0.0, dt.powi(2), 0.0, 0.0, dt, 0.0;
                0.0, 0.0, 0.5f32 * dt.powi(3), 0.0, 0.0, dt.powi(2), 0.0, 0.0, dt;
                0.5f32 * dt.powi(2), 0.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, 0.0;
                0.0, 0.5f32 * dt.powi(2), 0.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0;
                0.0, 0.0, 0.5f32 * dt.powi(2), 0.0, 0.0, dt, 0.0, 0.0, 1.0;
            ] * settings.std_dev_process.powi(2),
            // Measurement Covariance Matrix
            R: matrix!(
                settings.std_dev_barometer.powi(2), 0.0, 0.0, 0.0, 0.0, 0.0;
                0.0, settings.std_dev_accelerometer.powi(2), 0.0, 0.0, 0.0, 0.0;
                0.0, 0.0, settings.std_dev_accelerometer.powi(2), 0.0, 0.0, 0.0;
                0.0, 0.0, 0.0, settings.std_dev_accelerometer.powi(2), 0.0, 0.0;
                0.0, 0.0, 0.0, 0.0, GPS_NO_FIX_STD_DEV.powi(2), 0.0;
                0.0, 0.0, 0.0, 0.0, 0.0, GPS_NO_FIX_STD_DEV.powi(2);
            ),
            ..Default::default()
        };

        Self {
            time: Wrapping(0),
            mode: FlightMode::default(),
            mode_time: Wrapping(0),
            takeoff_time: Wrapping(0),
            settings,
            ahrs,
            kalman,
            orientation: None,
            acceleration: None,
            acceleration_world: None,
            altitude_ground: 0.0,
            altitude_max: -10_000.0,
            gps_origin: None,
            last_covariance_update: Wrapping(0),
            #[cfg(not(target_os = "none"))]
            last_apogee_error: 0.0,
        }
    }

    fn apply_measurements(
        &mut self,
        altitude_baro: f32,
        accel: Vector3<f32>,
        gps: Option<&GpsDatum>,
    ) {
        // Update GPS measurement noise
        let std_dev = self.hdop_to_std_dev(gps.as_ref().map(|gps| gps.hdop));
        self.kalman.R[(4, 4)] = std_dev;
        self.kalman.R[(5, 5)] = std_dev;

        let pos = if let Some(gps) = &gps {
            let global_pos = Vector3::new(
                gps.latitude.unwrap_or_default(),
                gps.longitude.unwrap_or_default(),
                gps.altitude.unwrap_or_default(),
            );
            if self.gps_origin.is_none() {
                self.gps_origin = Some(global_pos);
            }
            self.global_to_local(global_pos)
        } else {
            self.position_local()
        };

        self.kalman.predict(None, None, None, None);
        let z = Vector6::new(altitude_baro, accel.x, accel.y, accel.z, pos.x, pos.y);

        // Updating the state covariance is pretty expensive, so we just don't
        // do it every time.
        // TODO: try and get rid of this, at least make configurable
        if gps.is_some() || (self.time - self.last_covariance_update).0 > 10 {
            self.kalman.update(&z, None, None);
            self.last_covariance_update = self.time;
        } else {
            self.kalman.update_steadystate(&z);
        }
    }

    pub fn update(
        &mut self,
        time: Wrapping<u32>,
        mode: FlightMode,
        gyroscope: Option<Vector3<f32>>,
        accelerometer1: Option<Vector3<f32>>,
        accelerometer2: Option<Vector3<f32>>,
        magnetometer: Option<Vector3<f32>>,
        barometer: Option<f32>,
        gps_datum: Option<GpsDatum>,
    ) {
        self.time = time;

        if mode != self.mode {
            if mode == FlightMode::Burn {
                self.takeoff_time = self.time;
            }

            self.mode = mode;
            self.mode_time = self.time;

            // In the free-fall flight modes we ignore the accelerometer data
            // for orientation estimation.
            (
                *self.ahrs.acc_gain_mut(),
                *self.ahrs.kp_mut(),
                *self.ahrs.ki_mut(),
            ) = match self.mode {
                FlightMode::Burn | FlightMode::Coast => (
                    0.0,
                    self.settings.mahony_kp_ascent,
                    self.settings.mahony_ki_ascent,
                ),
                _ => (1.0, self.settings.mahony_kp, self.settings.mahony_ki),
            };
        }

        // Determine accelerometer to use. We prefer the primary because it is less noisy,
        // but have to switch to the secondary if we exceed +-16G (or get close enough) on any axis.
        let acc = match (accelerometer1, accelerometer2) {
            (Some(acc1), Some(acc2))
                if acc1.amax() > 14.0 * GRAVITY && acc2.amax() > 14.0 * GRAVITY =>
            {
                Some(acc2)
            }
            (Some(acc1), _) => Some(acc1),
            (None, Some(acc2)) => Some(acc2),
            (None, None) => None,
        };
        self.acceleration = acc.map(|a| self.correct_orientation(&a));

        if let (Some(gyro), Some(acc), Some(mag)) = (&gyroscope, &self.acceleration, &magnetometer)
        {
            let gyro = self.correct_orientation(gyro);
            let mag = self.correct_orientation(mag);

            // During burn, skip orientation updates entirely - assume the rocket
            // flies straight. Less wrong than AHRS with launch rail vibrations.
            if self.mode != FlightMode::Burn {
                self.orientation = self
                    .ahrs
                    .update(
                        &(gyro * <f32 as num_traits::FloatConst>::PI() / 180.0),
                        acc,
                        &mag,
                    )
                    .ok()
                    .copied();
            }

            // Rotate acceleration vector to get world-space acceleration
            // (where Z is straight up) and subtract gravity.
            self.acceleration_world = self
                .orientation
                .map(|quat| quat.transform_vector(acc) - Vector3::new(0.0, 0.0, GRAVITY));
        } else {
            self.orientation = None;
            self.acceleration_world = None;
        }

        // In the trans-/supersonic region, barometer readings become unreliable.
        let mach = if self.mode == FlightMode::Burn || self.mode == FlightMode::Coast {
            self.mach()
        } else {
            0.0
        };
        let f = ((mach.clamp(0.1, 1.0) - 0.1) / 0.9).powi(1);
        self.kalman.R[0] =
            self.settings.std_dev_barometer + f * self.settings.std_dev_barometer_transsonic;

        // Update the Kalman filter with barometric altitude and world-space acceleration
        let altitude_baro = barometer
            .and_then(|a| (!a.is_nan()).then_some(a)) // NaN is not a valid altitude
            .and_then(|a| (a > -100.0 && a < 12_000.0).then_some(a)); // neither is -13000
        let accel = self
            .acceleration_world
            .and_then(|a| (!(a.x.is_nan() || a.y.is_nan() || a.z.is_nan())).then_some(a));
        let gps = gps_datum.and_then(|d| self.gps_reliable(&d).then_some(d));

        match (accel, altitude_baro) {
            (Some(accel), Some(altitude_baro)) => {
                self.apply_measurements(altitude_baro, accel, gps.as_ref());
            }
            (Some(accel), None) => {
                // Use predicted altitude values, basically attempting to do inertial navigation.
                self.apply_measurements(self.altitude_asl(), accel, gps.as_ref());
            }
            (None, Some(altitude_baro)) => {
                // Just assume acceleration is zero.
                self.apply_measurements(altitude_baro, Vector3::new(0.0, 0.0, 0.0), gps.as_ref());
            }
            (None, None) => {
                // Do nothing, as long as this gap isn't too big and barometer values come back,
                // the Kalman filter should be able to recover from this.
            }
        }

        // Continuously reset ground altitude before arming.
        if mode < FlightMode::Armed {
            self.altitude_ground = self.altitude_asl();
        }

        // Only track maximum height during flight
        self.altitude_max = match mode {
            FlightMode::Idle
            | FlightMode::HardwareArmed
            | FlightMode::Filling
            | FlightMode::Venting
            | FlightMode::Pressurizing
            | FlightMode::Hold
            | FlightMode::Armed
            | FlightMode::Ignition => self.altitude_asl(),
            FlightMode::Burn
            | FlightMode::Coast
            | FlightMode::RecoveryDrogue
            | FlightMode::RecoveryMain => f32::max(self.altitude_max, self.altitude_asl()),
            FlightMode::Landed => self.altitude_max,
        };
    }

    /// Vehicle-frame acceleration (body axes), after orientation correction and
    /// accelerometer switching.
    pub fn acceleration_vehicle(&self) -> Option<&Vector3<f32>> {
        self.acceleration.as_ref()
    }

    pub fn acceleration_world_raw(&self) -> Option<&Vector3<f32>> {
        self.acceleration_world.as_ref()
    }

    pub fn position_local(&self) -> Vector3<f32> {
        Vector3::new(self.kalman.x[0], self.kalman.x[1], self.kalman.x[2])
    }

    pub fn latitude(&self) -> Option<f32> {
        if self.gps_origin.is_some() {
            let pos_global = self.local_to_global(self.position_local());
            Some(pos_global.x)
        } else {
            None
        }
    }

    pub fn longitude(&self) -> Option<f32> {
        if self.gps_origin.is_some() {
            let pos_global = self.local_to_global(self.position_local());
            Some(pos_global.y)
        } else {
            None
        }
    }

    pub fn velocity(&self) -> Vector3<f32> {
        Vector3::new(self.kalman.x[3], self.kalman.x[4], self.kalman.x[5])
    }

    pub fn acceleration_world(&self) -> Vector3<f32> {
        Vector3::new(self.kalman.x[6], self.kalman.x[7], self.kalman.x[8])
    }

    pub fn altitude_asl(&self) -> f32 {
        self.position_local().z
    }

    pub fn altitude_agl(&self) -> f32 {
        self.altitude_asl() - self.altitude_ground
    }

    pub fn apogee_asl(&self) -> Option<f32> {
        None
        // TODO
        //match self.mode {
        //    FlightMode::Coast => {
        //        // First, figure out the current drag force
        //        let drag = self.acceleration_world() + Vector3::new(0.0, 0.0, GRAVITY);

        //        // By how much should we reduce our drag estimate to roughly match the
        //        // average over the remaining flight?
        //        let reduction = 1.0
        //            + self.settings.drag_reduction_factor
        //                * self.mach().powf(self.settings.drag_reduction_exp);

        //        // This gives 0.5 * air density * drag coefficient * area
        //        let drag_over_vel_squared =
        //            (drag.magnitude() / self.velocity().magnitude_squared()) / reduction;

        //        // Calculate terminal velocity and remaining vertical distance
        //        let terminal_vel_squared = GRAVITY / drag_over_vel_squared;
        //        let remaining_height = (terminal_vel_squared / (2.0 * GRAVITY))
        //            * ((self.vertical_speed().powi(2) + terminal_vel_squared)
        //                / terminal_vel_squared)
        //                .ln();

        //        Some(self.altitude_asl() + remaining_height)
        //    }
        //    FlightMode::RecoveryDrogue | FlightMode::RecoveryMain | FlightMode::Landed => {
        //        Some(self.altitude_max)
        //    }
        //    _ => None,
        //}
    }

    pub fn apogee_agl(&self) -> Option<f32> {
        self.apogee_asl().map(|alt| alt - self.altitude_ground)
    }

    pub fn ground_speed(&self) -> f32 {
        (self.velocity().x.powi(2) + self.velocity().y.powi(2)).sqrt()
    }

    pub fn vertical_speed(&self) -> f32 {
        self.velocity().z
    }

    pub fn vertical_acceleration(&self) -> f32 {
        self.acceleration_world().z
    }

    pub fn mach(&self) -> f32 {
        self.velocity().magnitude() / 343.2
    }

    pub fn time_since_takeoff(&self) -> u32 {
        (self.time - self.takeoff_time).0
    }

    pub fn time_in_mode(&self) -> u32 {
        (self.time - self.mode_time).0
    }

    #[allow(clippy::unused_self)]
    fn correct_orientation(&self, raw: &Vector3<f32>) -> Vector3<f32> {
        *raw
        // TODO
        //match self.settings.orientation {
        //    Orientation::ZUp => *raw,
        //    Orientation::ZDown => Vector3::new(-raw.x, raw.y, -raw.z),
        //}
    }

    #[allow(clippy::unused_self)]
    pub fn gps_reliable(&self, datum: &GpsDatum) -> bool {
        // TODO: include fix enum here?
        datum.latitude.is_some()
            && datum.longitude.is_some()
            && datum.altitude.is_some()
            && datum.hdop > 0
            && datum.hdop < 300
    }

    #[allow(clippy::unused_self)]
    fn hdop_to_std_dev(&self, hdop: Option<u16>) -> f32 {
        hdop.map(|hdop| (hdop as f32 / 100.0) * 0.003)
            .unwrap_or(GPS_NO_FIX_STD_DEV)
    }

    fn global_to_local(&self, global: Vector3<f32>) -> Vector3<f32> {
        let offset = global - self.gps_origin.unwrap_or_default();
        let (lat, lng) = (offset.x, offset.y);
        Vector3::new(
            lng * 111_111.0 * self.gps_origin.unwrap_or_default().x.to_radians().cos(),
            lat * 111_111.0,
            global.z,
        )
    }

    fn local_to_global(&self, local: Vector3<f32>) -> Vector3<f32> {
        let offset = Vector3::new(
            local.y / 111_111.0,
            local.x / (111_111.0 * self.gps_origin.unwrap_or_default().x.to_radians().cos()),
            local.z,
        );
        self.gps_origin.unwrap_or_default() + offset
    }
}
