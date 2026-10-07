//! Turns operator commands into the position of each servo.
//!
//! The servos form two quick disconnects, each a disconnect servo and a retract servo. Every servo
//! starts released (the IO board does not drive it) and otherwise holds its last commanded
//! position, independently of the flight mode.
//!
//! Releasing a quick disconnect runs a sequence on both of its servos, timed from the command:
//!
//! 1. disconnect servo open, retract servo closed
//! 2. after `QD_DISC_HOLD_T`: retract servo to `QD_DISC_RETR_PR` / `QD_DISC_RETR_OX`
//! 3. after a further `QD_DISC_RETR_T`: disconnect servo closed
//!
//! A command for either servo of a quick disconnect aborts its running sequence, leaving the other
//! servo where the sequence had put it.

use core::num::Wrapping;

use crate::bus::ValveState;
use crate::inventory::{ServoId, ServoMap};
use crate::params::QdParams;

/// The (disconnect, retract) servos of each quick disconnect, by 0-based instance.
const QUICK_DISCONNECTS: [(ServoId, ServoId); 2] = [
    (ServoId::PressurantDisconnect, ServoId::PressurantRetract),
    (ServoId::OxidizerDisconnect, ServoId::OxidizerRetract),
];

pub struct ServoController {
    /// `None` is released.
    positions: ServoMap<Option<ValveState>>,
    /// Vehicle time [ms] at which each quick disconnect's running release sequence started.
    releases: [Option<Wrapping<u32>>; 2],
}

impl Default for ServoController {
    fn default() -> Self {
        Self::new()
    }
}

impl ServoController {
    pub const fn new() -> Self {
        Self {
            positions: ServoMap::splat(None),
            releases: [None; 2],
        }
    }

    /// Replaces any previous command for the same servo.
    pub fn command(&mut self, servo: ServoId, state: ValveState) {
        for (release, (disconnect, retract)) in self.releases.iter_mut().zip(QUICK_DISCONNECTS) {
            if servo == disconnect || servo == retract {
                *release = None;
            }
        }

        self.positions[servo] = Some(state);
    }

    /// Starts the release sequence of quick disconnect `instance` (0-based), or closes its
    /// disconnect servo if `grab`. Returns `false` if there is no such quick disconnect.
    pub fn gripper(&mut self, instance: u8, grab: bool, now: Wrapping<u32>) -> bool {
        let Some(&(disconnect, _)) = QUICK_DISCONNECTS.get(usize::from(instance)) else {
            return false;
        };

        if grab {
            self.command(disconnect, ValveState::fully_closed());
        } else if let Some(release) = self.releases.get_mut(usize::from(instance)) {
            *release = Some(now);
        }

        true
    }

    /// Resolve the commanded position of every servo for this tick, `None` being released.
    pub fn resolve(
        &mut self,
        now: Wrapping<u32>,
        params: &QdParams,
    ) -> ServoMap<Option<ValveState>> {
        let open = ValveState::fully_open();
        let closed = ValveState::fully_closed();
        let percent = |p: u32| ValveState::from_percent_open(u16::try_from(p).unwrap_or(u16::MAX));
        let retract_at = params.disconnect_hold_time;
        let release_at = retract_at.saturating_add(params.disconnect_retract_time);
        let retracted = [
            percent(params.disconnect_retract_pressurant),
            percent(params.disconnect_retract_oxidizer),
        ];

        for ((release, (disconnect, retract)), retracted) in self
            .releases
            .iter_mut()
            .zip(QUICK_DISCONNECTS)
            .zip(retracted)
        {
            let Some(started) = *release else { continue };
            let elapsed = (now - started).0;

            self.positions[disconnect] = Some(if elapsed < release_at { open } else { closed });
            self.positions[retract] = Some(if elapsed < retract_at {
                closed
            } else {
                retracted
            });

            if elapsed >= release_at {
                *release = None;
            }
        }

        self.positions
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn servos_start_released() {
        assert_eq!(resolve(&mut ServoController::new(), 0).values(), &[None; 4]);
    }

    #[test]
    fn release_runs_its_sequence_on_one_quick_disconnect() {
        let mut c = ServoController::new();
        assert!(c.gripper(1, false, Wrapping(5000)));

        let ox = percent(20);
        for (now, disconnect, retract) in [
            (5000, OPEN, CLOSED),
            (5999, OPEN, CLOSED),
            (6000, OPEN, ox),
            (6499, OPEN, ox),
            (6500, CLOSED, ox),
            (60_000, CLOSED, ox),
        ] {
            let servos = resolve(&mut c, now);
            assert_eq!(servos[S::OxidizerDisconnect], disconnect, "t={now}");
            assert_eq!(servos[S::OxidizerRetract], retract, "t={now}");
            assert_eq!(servos[S::PressurantDisconnect], None, "t={now}");
            assert_eq!(servos[S::PressurantRetract], None, "t={now}");
        }
    }

    #[test]
    fn quick_disconnects_release_independently() {
        let mut c = ServoController::new();
        assert!(c.gripper(0, false, Wrapping(0)));
        assert!(c.gripper(1, false, Wrapping(1000)));

        assert_eq!(
            resolve(&mut c, 1200).values(),
            &[OPEN, OPEN, percent(10), CLOSED]
        );
        assert_eq!(
            resolve(&mut c, 1600).values(),
            &[CLOSED, OPEN, percent(10), CLOSED]
        );
        assert_eq!(
            resolve(&mut c, 2600).values(),
            &[CLOSED, CLOSED, percent(10), percent(20)]
        );
    }

    #[test]
    fn a_command_aborts_the_release_of_its_quick_disconnect() {
        let mut c = ServoController::new();
        assert!(c.gripper(0, false, Wrapping(0)));
        assert!(c.gripper(1, false, Wrapping(0)));
        resolve(&mut c, 1200);

        c.command(S::PressurantRetract, ValveState::from_percent_open(50));
        assert!(c.gripper(1, true, Wrapping(1200)));

        let servos = resolve(&mut c, 60_000);
        assert_eq!(servos[S::PressurantDisconnect], OPEN);
        assert_eq!(servos[S::PressurantRetract], percent(50));
        assert_eq!(servos[S::OxidizerDisconnect], CLOSED);
        assert_eq!(servos[S::OxidizerRetract], percent(20));
    }

    #[test]
    fn grab_closes_only_the_disconnect_servo() {
        let mut c = ServoController::new();
        assert!(c.gripper(0, true, Wrapping(0)));
        assert_eq!(resolve(&mut c, 1).values(), &[CLOSED, None, None, None]);
    }

    #[test]
    fn a_missing_quick_disconnect_changes_nothing() {
        let mut c = ServoController::new();
        assert!(!c.gripper(2, false, Wrapping(0)));
        assert!(!c.gripper(2, true, Wrapping(0)));
        assert_eq!(resolve(&mut c, 1).values(), &[None; 4]);
    }
}
