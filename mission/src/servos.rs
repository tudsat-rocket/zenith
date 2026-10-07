//! Reconciles the current flight mode with operator commands into the position of each servo.
//!
//! The commanded position of each servo is decided as follows (highest wins):
//!
//! 1. manual command
//! 2. `Hold` baseline (the positions in effect at the moment `Hold` was entered)
//! 3. mode-based position
//!
//! Only `Disconnect` and `Retract` drive the servos; every other mode releases them, i.e. the IO
//! board stops driving them and they hold (or do not hold) mechanically. A manual command always
//! drives.
//!
//! Manual commands are reset upon flight mode changes, except for the Hold mode.

use core::num::Wrapping;

use rapid_dialect::FlightMode;

use crate::bus::ValveState;
use crate::inventory::{ServoId, ServoMap};
use crate::params::QdParams;

pub struct ServoController {
    mode: FlightMode,
    commands: ServoMap<Option<ValveState>>,
    /// Positions from the most recent [`Self::resolve`], which entering Hold freezes. `None` is
    /// released.
    last: ServoMap<Option<ValveState>>,
    hold_baseline: ServoMap<Option<ValveState>>,
    /// Vehicle time [ms] at which [`Self::mode`] was entered.
    mode_entered_at: Wrapping<u32>,
}

impl Default for ServoController {
    fn default() -> Self {
        Self::new()
    }
}

impl ServoController {
    pub const fn new() -> Self {
        Self {
            mode: FlightMode::Idle,
            commands: ServoMap::splat(None),
            last: ServoMap::splat(None),
            hold_baseline: ServoMap::splat(None),
            mode_entered_at: Wrapping(0),
        }
    }

    /// Replaces any previous command for the same servo.
    pub fn command(&mut self, servo: ServoId, state: ValveState) {
        self.commands[servo] = Some(state);
    }

    /// Must be called on every mode change with the mode being entered.
    pub fn set_mode(&mut self, entered: FlightMode, now: Wrapping<u32>) {
        if entered == FlightMode::Hold {
            self.hold_baseline = self.last;
        } else {
            self.commands = ServoMap::splat(None);
        }

        self.mode = entered;
        self.mode_entered_at = now;
    }

    /// `None` releases the servo. Hold has no opinion of its own and is resolved by the caller.
    fn mode_servo_state(
        &self,
        servo: ServoId,
        time_in_mode: u32,
        params: &QdParams,
    ) -> Option<ValveState> {
        use FlightMode as M;
        use ServoId as S;

        let open = ValveState::fully_open();
        let closed = ValveState::fully_closed();
        let percent = |p: u32| ValveState::from_percent_open(u16::try_from(p).unwrap_or(u16::MAX));
        let retract_at = params.disconnect_hold_time;
        let release_at = retract_at.saturating_add(params.disconnect_retract_time);

        let state = match (self.mode, servo) {
            (M::Disconnect, S::PressurantDisconnect | S::OxidizerDisconnect) => {
                if time_in_mode < release_at {
                    open
                } else {
                    closed
                }
            }
            (M::Disconnect, _) if time_in_mode < retract_at => closed,
            (M::Disconnect, S::PressurantRetract) => percent(params.disconnect_retract_pressurant),
            (M::Disconnect, S::OxidizerRetract) => percent(params.disconnect_retract_oxidizer),

            (M::Retract, S::PressurantDisconnect | S::OxidizerDisconnect) => closed,
            (M::Retract, S::PressurantRetract | S::OxidizerRetract) => open,

            (
                M::Hold
                | M::Idle
                | M::FillPressurant
                | M::FillOxidizer
                | M::Vent
                | M::Pressurize
                | M::DetectLaunch
                | M::Ignite
                | M::Burn
                | M::Coast
                | M::DeployDrogue
                | M::DeployMain
                | M::Landed,
                _,
            ) => return None,
        };

        Some(state)
    }

    /// Resolve the commanded position of every servo for this tick, `None` being released.
    pub fn resolve(
        &mut self,
        now: Wrapping<u32>,
        params: &QdParams,
    ) -> ServoMap<Option<ValveState>> {
        let time_in_mode = (now - self.mode_entered_at).0;

        self.last = ServoMap::from_fn(|servo| {
            self.commands[servo].or_else(|| {
                if self.mode == FlightMode::Hold {
                    self.hold_baseline[servo]
                } else {
                    self.mode_servo_state(servo, time_in_mode, params)
                }
            })
        });

        self.last
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::inventory::InventoryId;

    use FlightMode as M;
    use ServoId as S;

    const OPEN: Option<ValveState> = Some(ValveState::fully_open());
    const CLOSED: Option<ValveState> = Some(ValveState::fully_closed());

    fn percent(p: u16) -> Option<ValveState> {
        Some(ValveState::from_percent_open(p))
    }

    fn params() -> QdParams {
        QdParams {
            disconnect_hold_time: 1000,
            disconnect_retract_pressurant: 10,
            disconnect_retract_oxidizer: 20,
            disconnect_retract_time: 500,
        }
    }

    fn resolve(c: &mut ServoController, now: u32) -> ServoMap<Option<ValveState>> {
        c.resolve(Wrapping(now), &params())
    }

    fn all_modes() -> impl Iterator<Item = FlightMode> {
        (0u8..).map_while(|m| FlightMode::try_from(m).ok())
    }

    #[test]
    fn only_disconnect_and_retract_drive_the_servos() {
        let mut c = ServoController::new();
        for mode in all_modes() {
            c.set_mode(mode, Wrapping(0));
            for servo in ServoId::ALL {
                assert_eq!(
                    c.mode_servo_state(servo, 0, &params()).is_some(),
                    matches!(mode, M::Disconnect | M::Retract),
                    "mode {mode:?} servo {servo:?}"
                );
            }
        }
    }

    #[test]
    fn servos_start_released() {
        assert_eq!(resolve(&mut ServoController::new(), 0).values(), &[None; 4]);
    }

    #[test]
    fn disconnect_runs_its_sequence() {
        let mut c = ServoController::new();
        c.set_mode(M::Disconnect, Wrapping(5000));

        let (pr, ox) = (percent(10), percent(20));
        for (now, disconnect, pressurant_retract, oxidizer_retract) in [
            (5000, OPEN, CLOSED, CLOSED),
            (5999, OPEN, CLOSED, CLOSED),
            (6000, OPEN, pr, ox),
            (6499, OPEN, pr, ox),
            (6500, CLOSED, pr, ox),
            (60_000, CLOSED, pr, ox),
        ] {
            let servos = resolve(&mut c, now);
            assert_eq!(servos[S::PressurantDisconnect], disconnect, "t={now}");
            assert_eq!(servos[S::OxidizerDisconnect], disconnect, "t={now}");
            assert_eq!(servos[S::PressurantRetract], pressurant_retract, "t={now}");
            assert_eq!(servos[S::OxidizerRetract], oxidizer_retract, "t={now}");
        }
    }

    #[test]
    fn retract_holds_the_final_position_and_later_modes_release() {
        let mut c = ServoController::new();
        c.set_mode(M::Retract, Wrapping(0));
        assert_eq!(resolve(&mut c, 10).values(), &[CLOSED, CLOSED, OPEN, OPEN]);

        for mode in [M::Pressurize, M::Ignite, M::Landed] {
            c.set_mode(mode, Wrapping(0));
            assert_eq!(resolve(&mut c, 10).values(), &[None; 4], "{mode:?}");
        }
    }

    #[test]
    fn a_manual_command_beats_the_mode_until_the_mode_changes() {
        let mut c = ServoController::new();
        c.set_mode(M::Retract, Wrapping(0));

        c.command(S::OxidizerRetract, ValveState::from_percent_open(30));
        let servos = resolve(&mut c, 1);
        assert_eq!(servos[S::OxidizerRetract], percent(30));
        assert_eq!(servos[S::PressurantRetract], OPEN);

        c.set_mode(M::Pressurize, Wrapping(2));
        c.command(S::PressurantRetract, ValveState::fully_closed());
        let servos = resolve(&mut c, 3);
        assert_eq!(servos[S::PressurantRetract], CLOSED);
        assert_eq!(servos[S::OxidizerRetract], None);
    }

    #[test]
    fn hold_freezes_the_positions_and_commands_from_entry() {
        let mut c = ServoController::new();
        c.set_mode(M::Disconnect, Wrapping(0));
        c.command(S::OxidizerRetract, ValveState::from_percent_open(30));
        let entered = resolve(&mut c, 1200);

        c.set_mode(M::Hold, Wrapping(1200));
        assert_eq!(resolve(&mut c, 60_000).values(), entered.values());

        c.command(S::PressurantDisconnect, ValveState::fully_closed());
        assert_eq!(resolve(&mut c, 60_001)[S::PressurantDisconnect], CLOSED);
        assert_eq!(resolve(&mut c, 60_001)[S::OxidizerDisconnect], OPEN);

        c.set_mode(M::Idle, Wrapping(60_002));
        assert_eq!(resolve(&mut c, 60_003).values(), &[None; 4]);
    }

    #[test]
    fn hold_from_a_released_mode_stays_released() {
        let mut c = ServoController::new();
        resolve(&mut c, 0);
        c.set_mode(M::Hold, Wrapping(1));
        assert_eq!(resolve(&mut c, 2).values(), &[None; 4]);
    }
}
