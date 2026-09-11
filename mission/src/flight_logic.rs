use core::num::Wrapping;

use rapid_dialect::FlightMode;
use state_estimator::{GRAVITY, StateEstimator};

use crate::{FailsafeParams, RecoveryParams};

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
    /// Time of the last ground station contact on any link.
    last_uplink: Wrapping<u32>,
}

impl Default for FlightLogic {
    fn default() -> Self {
        Self {
            condition_true_since: None,
            mode_time: Wrapping(0),
            takeoff_time: Wrapping(0),
            last_uplink: Wrapping(0),
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
        params: &RecoveryParams,
        failsafe: &FailsafeParams,
    ) -> Option<FlightMode> {
        let t_in_mode = (time - self.mode_time).0;
        let t_since_takeoff = (time - self.takeoff_time).0;

        if mode < FlightMode::DetectLaunch
            && let Some(new_mode) = self.uplink_failsafe(time, mode, failsafe)
        {
            log::warn!(
                "Uplink lost for {}ms, falling back to {new_mode:?}",
                (time - self.last_uplink).0
            );
            return Some(new_mode);
        }

        match mode {
            // Takeoff detection: sustained high acceleration along body Z axis
            FlightMode::DetectLaunch | FlightMode::Ignite => {
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
                let min_exceeded = t_since_takeoff > params.min_time_to_drogue;
                let max_exceeded = t_since_takeoff > 30_000; // safety: 30s max coast
                ((min_exceeded && falling) || max_exceeded).then_some(FlightMode::DeployDrogue)
            }

            // Main chute deployment: below altitude threshold
            FlightMode::DeployDrogue => {
                let below_alt = self.true_since(
                    time,
                    estimator.altitude_agl() < params.main_deploy_altitude,
                    100,
                );
                let min_time = params.min_time_to_main;
                (t_in_mode > min_time && below_alt).then_some(FlightMode::DeployMain)
            }

            // Landing detection: near-zero vertical speed with ~1G present
            FlightMode::DeployMain => {
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
            | FlightMode::Landed
            | FlightMode::FillPressurant
            | FlightMode::FillOxidizer
            | FlightMode::Vent
            | FlightMode::Pressurize
            | FlightMode::Hold => None,
        }
    }

    /// Must be called whenever ground station traffic is received on any link.
    pub fn note_uplink(&mut self, time: Wrapping<u32>) {
        self.last_uplink = time;
    }

    /// Two-stage fallback to Idle (closing all valves), then Vent for a lost uplink on the ground
    fn uplink_failsafe(
        &self,
        time: Wrapping<u32>,
        mode: FlightMode,
        params: &FailsafeParams,
    ) -> Option<FlightMode> {
        let age = (time - self.last_uplink).0;
        let idle_due = params.uplink_idle_timeout != 0 && age >= params.uplink_idle_timeout;
        let vent_due = params.uplink_vent_timeout != 0 && age >= params.uplink_vent_timeout;

        // Vent obviously beats Idle
        match (idle_due, vent_due) {
            (_, true) => (mode != FlightMode::Vent).then_some(FlightMode::Vent),
            (true, false) => (mode != FlightMode::Idle).then_some(FlightMode::Idle),
            (false, false) => None,
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

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, reason = "unwrap is idiomatic in test assertions")]

    use super::*;
    use FlightMode as M;
    use state_estimator::StateEstimatorParams;

    const IDLE_TIMEOUT: u32 = 10_000;
    const VENT_TIMEOUT: u32 = 120_000;

    fn estimator() -> StateEstimator {
        StateEstimator::new(1000.0, StateEstimatorParams::default())
    }

    fn failsafe_params() -> FailsafeParams {
        FailsafeParams {
            uplink_idle_timeout: IDLE_TIMEOUT,
            uplink_vent_timeout: VENT_TIMEOUT,
        }
    }

    /// The ground modes the failsafe applies in.
    fn ground_modes() -> impl Iterator<Item = FlightMode> {
        (0u8..)
            .map_while(|m| FlightMode::try_from(m).ok())
            .filter(|m| *m < M::DetectLaunch)
    }

    /// `update` for tests that only care about the failsafe.
    fn update(
        logic: &mut FlightLogic,
        now: u32,
        mode: FlightMode,
        params: &FailsafeParams,
    ) -> Option<FlightMode> {
        logic.update(
            Wrapping(now),
            mode,
            &estimator(),
            &RecoveryParams::default(),
            params,
        )
    }

    #[test]
    fn ground_modes_are_held_while_the_uplink_is_alive() {
        let params = failsafe_params();

        for mode in ground_modes() {
            let mut logic = FlightLogic::default();
            for t in (0..10 * VENT_TIMEOUT).step_by(1000) {
                logic.note_uplink(Wrapping(t));
                assert_eq!(
                    update(&mut logic, t, mode, &params),
                    None,
                    "transitioned out of {mode:?} at t={t} despite a live uplink"
                );
            }
        }
    }

    #[test]
    fn uplink_loss_falls_back_to_idle_after_the_configured_timeout() {
        let params = failsafe_params();

        for mode in ground_modes().filter(|m| *m != M::Idle) {
            let mut logic = FlightLogic::default();
            logic.note_uplink(Wrapping(5000));

            assert_eq!(
                update(&mut logic, 5000 + IDLE_TIMEOUT - 1, mode, &params),
                None
            );
            assert_eq!(
                update(&mut logic, 5000 + IDLE_TIMEOUT, mode, &params),
                Some(M::Idle),
                "{mode:?} did not fall back to Idle"
            );
        }
    }

    #[test]
    fn idle_is_stable_until_the_vent_timeout() {
        let params = failsafe_params();
        let mut logic = FlightLogic::default();

        for t in (IDLE_TIMEOUT..VENT_TIMEOUT).step_by(1000) {
            assert_eq!(update(&mut logic, t, M::Idle, &params), None, "at t={t}");
        }
        assert_eq!(
            update(&mut logic, VENT_TIMEOUT, M::Idle, &params),
            Some(M::Vent)
        );
    }

    #[test]
    fn sustained_uplink_loss_escalates_to_vent_and_stays_there() {
        let params = failsafe_params();
        let mut logic = FlightLogic::default();

        assert_eq!(update(&mut logic, VENT_TIMEOUT - 1, M::Idle, &params), None);
        assert_eq!(
            update(&mut logic, VENT_TIMEOUT, M::Idle, &params),
            Some(M::Vent)
        );

        // Once vented, the idle stage must not pull the vehicle back out of Vent.
        for t in (VENT_TIMEOUT..10 * VENT_TIMEOUT).step_by(1000) {
            assert_eq!(update(&mut logic, t, M::Vent, &params), None, "at t={t}");
        }
    }

    #[test]
    fn the_timeouts_run_from_boot_without_any_contact() {
        let params = failsafe_params();
        let mut logic = FlightLogic::default();

        assert_eq!(
            update(&mut logic, IDLE_TIMEOUT, M::Hold, &params),
            Some(M::Idle)
        );
    }

    #[test]
    fn contact_resets_the_timeouts() {
        let params = failsafe_params();
        let mut logic = FlightLogic::default();

        assert_eq!(
            update(&mut logic, IDLE_TIMEOUT - 1, M::Pressurize, &params),
            None
        );
        logic.note_uplink(Wrapping(IDLE_TIMEOUT - 1));
        assert_eq!(
            update(&mut logic, 2 * IDLE_TIMEOUT - 2, M::Pressurize, &params),
            None
        );
        assert_eq!(
            update(&mut logic, 2 * IDLE_TIMEOUT - 1, M::Pressurize, &params),
            Some(M::Idle)
        );
    }

    #[test]
    fn a_zero_timeout_disables_its_stage() {
        let mut logic = FlightLogic::default();

        let vent_only = FailsafeParams {
            uplink_idle_timeout: 0,
            uplink_vent_timeout: VENT_TIMEOUT,
        };
        assert_eq!(
            update(&mut logic, VENT_TIMEOUT - 1, M::FillOxidizer, &vent_only),
            None
        );
        assert_eq!(
            update(&mut logic, VENT_TIMEOUT, M::FillOxidizer, &vent_only),
            Some(M::Vent)
        );

        let idle_only = FailsafeParams {
            uplink_idle_timeout: IDLE_TIMEOUT,
            uplink_vent_timeout: 0,
        };
        assert_eq!(
            update(&mut logic, IDLE_TIMEOUT, M::FillOxidizer, &idle_only),
            Some(M::Idle)
        );
        assert_eq!(
            update(&mut logic, 10 * VENT_TIMEOUT, M::Idle, &idle_only),
            None
        );

        let disabled = FailsafeParams {
            uplink_idle_timeout: 0,
            uplink_vent_timeout: 0,
        };
        for mode in ground_modes() {
            assert_eq!(
                update(&mut logic, 10 * VENT_TIMEOUT, mode, &disabled),
                None,
                "{mode:?}"
            );
        }
    }

    #[test]
    fn the_failsafe_does_not_apply_once_armed() {
        let params = failsafe_params();
        let mut logic = FlightLogic::default();

        // Landed sits above DetectLaunch as well, and already vents via the valve controller.
        // The in-flight modes have their own timeouts, so only the failsafe targets are ruled out.
        for mode in [M::DetectLaunch, M::Ignite, M::Burn, M::Coast, M::Landed] {
            let new_mode = update(&mut logic, 10 * VENT_TIMEOUT, mode, &params);
            assert!(
                !matches!(new_mode, Some(M::Idle | M::Vent)),
                "{mode:?} was overridden by the uplink failsafe: {new_mode:?}"
            );
        }
    }

    #[test]
    fn the_failsafe_does_not_disturb_the_launch_debounce() {
        let params = failsafe_params();
        let mut logic = FlightLogic::default();
        let mut estimator = estimator();
        let accel = nalgebra::Vector3::new(0.0, 0.0, 5.0 * GRAVITY);

        // No uplink at all, and the vehicle is armed: launch detection has to work regardless.
        for t in 0..100u32 {
            estimator.update(
                Wrapping(t),
                M::DetectLaunch,
                Some(accel),
                Some(accel),
                Some(accel),
                None,
                None,
                None,
            );
            let new_mode = logic.update(
                Wrapping(t),
                M::DetectLaunch,
                &estimator,
                &RecoveryParams::default(),
                &params,
            );
            assert_eq!(new_mode, (t > 50).then_some(M::Burn), "at t={t}");
        }
    }
}
