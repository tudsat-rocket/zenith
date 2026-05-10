use core::num::Wrapping;

use rapid_dialect::FlightMode;
use state_estimator::{GRAVITY, StateEstimator};

use crate::RecoverySettings;

/// Automatic flight mode transitions based on state estimator data.
///
/// All conditions must be true for a debounce duration to avoid spurious
/// transitions from sensor noise/glitches.
pub struct FlightLogic {
    /// Time since which the current transition condition has been true
    condition_true_since: Option<Wrapping<u32>>,
    /// Time at which the current mode was entered
    mode_time: Wrapping<u32>,
    /// Time at which takeoff (Burn) was entered
    takeoff_time: Wrapping<u32>,
}

impl Default for FlightLogic {
    fn default() -> Self {
        Self {
            condition_true_since: None,
            mode_time: Wrapping(0),
            takeoff_time: Wrapping(0),
        }
    }
}

impl FlightLogic {
    /// Evaluate whether a mode transition should happen. Returns the new mode
    /// if a transition is warranted, or None to stay in the current mode.
    pub fn update(
        &mut self,
        time: Wrapping<u32>,
        mode: FlightMode,
        estimator: &StateEstimator,
        settings: &RecoverySettings,
    ) -> Option<FlightMode> {
        let t_in_mode = (time - self.mode_time).0;
        let t_since_takeoff = (time - self.takeoff_time).0;

        match mode {
            // Takeoff detection: sustained high acceleration along body Z axis
            FlightMode::Armed | FlightMode::Ignition => {
                let accel_z = estimator.acceleration_vehicle().map(|a| a.z).unwrap_or(0.0);
                // ~3G threshold for 50ms
                let high_accel = accel_z > 3.0 * GRAVITY;
                self.true_since(time, high_accel, 50)
                    .then_some(FlightMode::Burn)
            }

            // Wait for motor burnout (negative vehicle-frame Z accel = deceleration)
            FlightMode::Burn => {
                let accel_z = estimator.acceleration_vehicle().map(|a| a.z).unwrap_or(0.0);
                let burnout = self.true_since(time, accel_z < 0.0, 50);
                let min_exceeded = t_since_takeoff > 15_000; // safety timeout
                (burnout || min_exceeded).then_some(FlightMode::Coast)
            }

            // Apogee detection: sustained negative vertical speed
            FlightMode::Coast => {
                let falling = self.true_since(time, estimator.vertical_speed() < 0.0, 500);
                let min_exceeded = t_since_takeoff > settings.min_time_to_drogue;
                let max_exceeded = t_since_takeoff > 30_000; // safety: 30s max coast
                ((min_exceeded && falling) || max_exceeded).then_some(FlightMode::RecoveryDrogue)
            }

            // Main chute deployment: below altitude threshold
            FlightMode::RecoveryDrogue => {
                let below_alt = self.true_since(
                    time,
                    estimator.altitude_agl() < settings.main_deploy_altitude,
                    100,
                );
                let min_time = settings.min_time_to_main;
                (t_in_mode > min_time && below_alt).then_some(FlightMode::RecoveryMain)
            }

            // Landing detection: near-zero vertical speed with ~1G present
            FlightMode::RecoveryMain => {
                let gravity_present = estimator
                    .acceleration_vehicle()
                    .map(|acc| (GRAVITY * 0.9..GRAVITY * 1.1).contains(&acc.magnitude()))
                    .unwrap_or(true);
                let landed = self.true_since(
                    time,
                    gravity_present && estimator.vertical_speed().abs() < 1.0,
                    1000,
                );
                (t_in_mode > 3000 && landed).then_some(FlightMode::Landed)
            }

            // No autonomous transition out of these.
            FlightMode::Idle
            | FlightMode::HardwareArmed
            | FlightMode::Landed
            | FlightMode::Filling
            | FlightMode::Venting
            | FlightMode::Pressurizing
            | FlightMode::Hold => None,
        }
    }

    /// Must be called when the mode actually changes, to track mode timing.
    pub fn set_mode(&mut self, time: Wrapping<u32>, new_mode: FlightMode) {
        self.mode_time = time;
        self.condition_true_since = None;
        if new_mode == FlightMode::Burn {
            self.takeoff_time = time;
        }
    }

    /// Returns true if `cond` has been continuously true for at least `duration` ms.
    fn true_since(&mut self, time: Wrapping<u32>, cond: bool, duration: u32) -> bool {
        self.condition_true_since = match (cond, self.condition_true_since) {
            (true, None) => Some(time),
            (true, Some(t)) => Some(t),
            (false, _) => None,
        };

        self.condition_true_since
            .map(|t| (time - t).0 > duration)
            .unwrap_or(false)
    }
}
