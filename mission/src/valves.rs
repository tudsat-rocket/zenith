//! Reconciles the current flight mode and the required valve state that implies with any operator
//! commands, and determines the flight computer's final word on what each valve should be doing.
//!
//! # Assumptions
//!
//! - The safe default for all valves on initialization is `Closed`.
//! - Any critical safety functions (e.g. emergency venting) are out-of-scope here and the
//!   responsibility of the actuator controlling the valve (e.g. if a tank needs to be vented
//!   automatically, that should be controlled by the same PCB reading the required sensors).
//!   In such a case, the commands emitted here may be ignored.
//!
//! # Valve Priorities
//!
//! The commanded state of each valve at any given time is decided as follows (highest wins):
//!
//! 1. manual command (only if allowed for mode and valve)
//! 2. mode-based state
//! 3. `Hold` baseline (the setpoints frozen at the moment `Hold` was entered)
//!
//! # Manual Valve Commands
//!
//! Manual commands may be attempted at any point by an operator. Depending on the active flight
//! mode, manual valve overrides may or may not be allowed. `Hold` allows all commands, most modes
//! allow only some or none.
//!
//! - Open: open a valve and keep it opened
//! - Close: close a valve and keep it closed
//! - Partial: drive a valve to a specific setpoint and keep it there
//! - PulseOpen: pulse the valve open for a certain time, then return to previous state
//!
//! Manual commands are reset upon flight mode changes (both for manual and automatic changes),
//! except for the Hold mode.
//!
//! PulseOpen does nothing (noticable) if the valve is already open, and will not close the valve
//! after the pulse is over.
//!
//! Entering `Hold` during an active `PulseOpen` command does not freeze the valve open. The pulse
//! will finish, and the valve will return to the last, non-pulse setpoint from before the `Hold`
//! transition.

use core::num::Wrapping;
use core::time::Duration;

use rapid_dialect::FlightMode;
pub use rapid_dialect::ValveCommand;

use crate::bus::ValveState;
use crate::inventory::{InventoryId, ValveId, ValveMap};

pub const MAX_PULSE_DURATION: Duration = Duration::from_secs(30);

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum ValveError {
    NotPermittedInMode,
    InvalidCommand,
    Inhibited,
    TransportFailed,
}

#[derive(Copy, Clone)]
struct ManualCommand {
    /// Vehicle time [ms] at which the command was accepted.
    issued_at: Wrapping<u32>,
    cmd: ValveCommand,
    /// The non-pulse command this pulse replaced, restored when the pulse expires. Always None for
    /// non-pulse commands.
    replaced: Option<ValveCommand>,
}

pub struct ValveController {
    mode: FlightMode,
    commands: ValveMap<Option<ManualCommand>>,
    /// Setpoints from the most recent [`Self::resolve`], computed as if no pulse were running.
    /// This is what entering Hold freezes, so an active pulse cannot leak its temporary "open"
    /// into the Hold baseline.
    nonpulse_setpoints: ValveMap<ValveState>,
    /// What Hold holds: the non-pulse setpoints frozen when Hold was last
    /// entered.
    hold_baseline: ValveMap<ValveState>,
}

impl Default for ValveController {
    fn default() -> Self {
        Self::new()
    }
}

impl ValveController {
    pub const fn new() -> Self {
        Self {
            mode: FlightMode::Idle,
            commands: ValveMap::splat(None),
            nonpulse_setpoints: ValveMap::splat(ValveState::fully_closed()),
            hold_baseline: ValveMap::splat(ValveState::fully_closed()),
        }
    }

    /// May the operator manually command this valve in this mode?
    fn manual_valve_allowed(&self, valve: ValveId) -> bool {
        use FlightMode as M;
        use ValveId as V;

        // TODO
        match self.mode {
            // Hold: every valve commandable
            M::Hold => true,

            // Vent valves pulsable for relief during fill / pressurize
            M::Filling | M::Pressurizing => {
                matches!(valve, V::PressurantVent | V::OxidizerVent | V::OxidizerFill)
            }

            // No manual overrides anywhere else
            M::Idle
            | M::HardwareArmed
            | M::Venting
            | M::Armed
            | M::Ignition
            | M::Burn
            | M::Coast
            | M::RecoveryDrogue
            | M::RecoveryMain
            | M::Landed => false,
        }
    }

    /// Accept or reject a manual valve command. A newly accepted command replaces any previous
    /// command for the same valve.
    pub fn try_command(
        &mut self,
        valve: ValveId,
        cmd: ValveCommand,
        now: Wrapping<u32>,
    ) -> Result<(), ValveError> {
        if !self.manual_valve_allowed(valve) {
            return Err(ValveError::NotPermittedInMode);
        }

        let valid = match cmd {
            ValveCommand::PulseOpen(dur) => dur <= MAX_PULSE_DURATION,
            // NaN fails every comparison and is rejected too
            ValveCommand::Partial(p) => (0.0..=1.0).contains(&p),
            ValveCommand::Open | ValveCommand::Close => true,
        };
        if !valid {
            return Err(ValveError::InvalidCommand);
        }

        // A pulse remembers the command it replaced so it can restore it on expiry. A pulse
        // replacing a pulse inherits the original target, so repeated pulses can't ratchet the
        // restore state towards "open".
        let replaced = match (cmd, self.commands[valve]) {
            (ValveCommand::PulseOpen(_), Some(prev)) => match prev.cmd {
                ValveCommand::PulseOpen(_) => prev.replaced,
                non_pulse => Some(non_pulse),
            },
            _ => None,
        };

        self.commands[valve] = Some(ManualCommand {
            issued_at: now,
            cmd,
            replaced,
        });

        Ok(())
    }

    /// Must be called on every mode change with the mode being entered.
    ///
    /// Clears all manual commands, except when entering Hold, which instead freezes the current
    /// non-pulse setpoints as the baseline it maintains and lets running commands (e.g. an active
    /// pulse) finish on top.
    pub fn set_mode(&mut self, entered: FlightMode) {
        if entered == FlightMode::Hold {
            self.hold_baseline = self.nonpulse_setpoints;
        } else {
            self.commands = ValveMap::splat(None);
        }

        self.mode = entered;
    }

    /// The per-mode valve truth table. `None` only for Hold, which has no opinion of its own;
    /// every other mode asserts a state for every valve.
    fn mode_valve_state(&self, valve: ValveId) -> Option<ValveState> {
        use FlightMode as M;
        use ValveId as V;

        let open = ValveState::fully_open();
        let closed = ValveState::fully_closed();

        // TODO
        let state = match self.mode {
            // Hold asserts nothing; the arbiter substitutes the setpoints frozen when Hold was
            // entered.
            M::Hold => return None,

            // Inert ground modes: everything closed.
            M::Idle | M::HardwareArmed | M::Armed => closed,

            // On-pad modes
            M::Filling => match valve {
                V::OxidizerFill => open,
                V::PressurantVent | V::Pressurization | V::OxidizerVent | V::Main => closed,
            },
            M::Pressurizing => match valve {
                V::Pressurization => open,
                V::PressurantVent | V::OxidizerVent | V::OxidizerFill | V::Main => closed,
            },
            M::Venting => match valve {
                V::PressurantVent | V::OxidizerVent => open,
                V::Pressurization | V::OxidizerFill | V::Main => closed,
            },

            // In-flight modes
            M::Ignition | M::Burn | M::Coast => match valve {
                V::Main | V::Pressurization => open,
                V::PressurantVent | V::OxidizerVent | V::OxidizerFill => closed,
            },
            M::RecoveryDrogue | M::RecoveryMain | M::Landed => match valve {
                V::PressurantVent | V::OxidizerVent => open,
                V::Pressurization | V::OxidizerFill | V::Main => closed,
            },
        };

        Some(state)
    }

    /// Resolve the commanded state of every valve for this tick.
    pub fn resolve(&mut self, now: Wrapping<u32>) -> ValveMap<ValveState> {
        // Expired pulses hand their slot back to the command they replaced, or clear it so the
        // mode / baseline takes over again.
        for valve in ValveId::ALL {
            let Some(c) = self.commands[valve] else {
                continue;
            };

            let expired = match c.cmd {
                ValveCommand::PulseOpen(dur) => (now - c.issued_at).0 > dur.as_millis() as u32,
                ValveCommand::Open | ValveCommand::Close | ValveCommand::Partial(_) => false,
            };

            if expired {
                self.commands[valve] = c.replaced.map(|cmd| ManualCommand {
                    issued_at: now,
                    cmd,
                    replaced: None,
                });
            }
        }

        // Resolve final output state for each valve, in the order: command, mode, baseline
        let resolved = ValveMap::from_fn(|valve| {
            self.commands[valve]
                .map(|c| ValveState::from(c.cmd))
                .or_else(|| self.mode_valve_state(valve))
                .unwrap_or(self.hold_baseline[valve])
        });

        // Imagine a world without pulses (a running pulse counts as the command it replaced), so
        // we can keep track of what to return to
        self.nonpulse_setpoints = ValveMap::from_fn(|valve| {
            self.commands[valve]
                .and_then(|c| match c.cmd {
                    ValveCommand::PulseOpen(_) => c.replaced,
                    cmd => Some(cmd),
                })
                .map(ValveState::from)
                .or_else(|| self.mode_valve_state(valve))
                .unwrap_or(self.hold_baseline[valve])
        });

        resolved
    }
}

impl From<ValveCommand> for ValveState {
    fn from(cmd: ValveCommand) -> Self {
        // The state a command drives its valve towards. A pulse maps to open, the expiry is the
        // controller's job.
        match cmd {
            ValveCommand::Close => ValveState::fully_closed(),
            ValveCommand::Partial(p) => ValveState::from_promille_clamped((p * 1000.0) as u16),
            ValveCommand::Open | ValveCommand::PulseOpen(_) => ValveState::fully_open(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::time::Duration;

    use FlightMode as M;
    use ValveId as V;

    const OPEN: ValveState = ValveState::fully_open();
    const CLOSED: ValveState = ValveState::fully_closed();

    fn all_modes() -> impl Iterator<Item = FlightMode> {
        (0u8..).map_while(|m| FlightMode::try_from(m).ok())
    }

    #[test]
    fn every_mode_except_hold_asserts_every_valve() {
        let mut c = ValveController::new();
        for mode in all_modes() {
            c.set_mode(mode);
            for valve in ValveId::ALL {
                assert_eq!(
                    c.mode_valve_state(valve).is_none(),
                    mode == M::Hold,
                    "mode {mode:?} valve {valve:?}"
                );
            }
        }
    }

    #[test]
    fn pulse_returns_control_to_mode_policy_after_expiry() {
        let mut c = ValveController::new();
        c.set_mode(M::FillOxidizer);

        c.try_command(
            V::OxidizerVent,
            ValveCommand::PulseOpen(Duration::from_millis(500)),
            Wrapping(1000),
        )
        .unwrap();

        // During the pulse the vent is open, afterwards FillOxidizer's policy
        // (closed) resumes - a future in-mode auto controller would regain
        // control the same way.
        assert!(c.resolve(Wrapping(1400))[V::OxidizerVent] == OPEN);
        assert!(c.resolve(Wrapping(1600))[V::OxidizerVent] == CLOSED);
        // The mode policy is untouched by all this.
        assert!(c.resolve(Wrapping(1600))[V::OxidizerFill] == OPEN);
    }

    #[test]
    fn pulse_in_hold_restores_the_setpoints_from_entry() {
        let mut c = ValveController::new();

        // FillOxidizer, with a manually opened pressurant vent.
        c.set_mode(M::FillOxidizer);
        c.try_command(V::PressurantVent, ValveCommand::Open, Wrapping(0))
            .unwrap();
        c.resolve(Wrapping(1));

        // Entering Hold freezes exactly that picture...
        c.set_mode(M::Hold);
        let held = c.resolve(Wrapping(2));
        assert!(held[V::OxidizerFill] == OPEN);
        assert!(held[V::PressurantVent] == OPEN);
        assert!(held[V::Main] == CLOSED);

        // ...and a pulse deviates from it only for its duration.
        c.try_command(
            V::OxidizerVent,
            ValveCommand::PulseOpen(Duration::from_millis(500)),
            Wrapping(100),
        )
        .unwrap();
        assert!(c.resolve(Wrapping(300))[V::OxidizerVent] == OPEN);

        let after = c.resolve(Wrapping(700));
        assert!(after[V::OxidizerVent] == CLOSED);
        assert!(after[V::OxidizerFill] == OPEN);
        assert!(after[V::PressurantVent] == OPEN);
    }

    #[test]
    fn pulse_survives_hold_entry_and_ends_at_non_pulse_setpoint() {
        let mut c = ValveController::new();
        c.set_mode(M::FillOxidizer);

        c.try_command(
            V::OxidizerVent,
            ValveCommand::PulseOpen(Duration::from_millis(500)),
            Wrapping(0),
        )
        .unwrap();
        assert!(c.resolve(Wrapping(100))[V::OxidizerVent] == OPEN);

        // Entering Hold mid-pulse: the pulse keeps running...
        c.set_mode(M::Hold);
        assert!(c.resolve(Wrapping(300))[V::OxidizerVent] == OPEN);

        // ...and finishes into the pre-pulse setpoint (closed under FillOxidizer's
        // policy), not the pulsed-open state.
        let after = c.resolve(Wrapping(600));
        assert!(after[V::OxidizerVent] == CLOSED);
        // The rest of the frozen FillOxidizer picture is untouched.
        assert!(after[V::OxidizerFill] == OPEN);
    }

    #[test]
    fn pulse_with_unknown_prior_ends_closed() {
        // Fresh state, e.g. right after an FC reboot into Hold: no setpoint
        // was ever resolved, so the pulse must fall back to closed.
        let mut c = ValveController::new();
        c.set_mode(M::Hold);

        c.try_command(
            V::OxidizerVent,
            ValveCommand::PulseOpen(Duration::from_millis(500)),
            Wrapping(0),
        )
        .unwrap();

        assert!(c.resolve(Wrapping(100))[V::OxidizerVent] == OPEN);
        assert!(c.resolve(Wrapping(600))[V::OxidizerVent] == CLOSED);
    }

    #[test]
    fn mode_change_clears_manual_commands() {
        let mut c = ValveController::new();
        c.set_mode(M::Hold);

        c.try_command(V::Main, ValveCommand::Open, Wrapping(0))
            .unwrap();
        assert!(c.resolve(Wrapping(1))[V::Main] == OPEN);

        c.set_mode(M::FillOxidizer);
        assert!(c.resolve(Wrapping(2))[V::Main] == CLOSED);
    }

    #[test]
    fn commands_rejected_without_permission() {
        let mut c = ValveController::new();

        c.set_mode(M::Burn);
        assert!(matches!(
            c.try_command(V::Main, ValveCommand::Close, Wrapping(0)),
            Err(ValveError::NotPermittedInMode)
        ));

        c.set_mode(M::FillOxidizer);
        assert!(matches!(
            c.try_command(V::Main, ValveCommand::Open, Wrapping(0)),
            Err(ValveError::NotPermittedInMode)
        ));
    }

    #[test]
    fn pulse_restores_the_latched_command_it_replaced() {
        let mut c = ValveController::new();
        c.set_mode(M::Hold); // baseline: everything closed

        // A latched half-open setpoint...
        c.try_command(V::OxidizerVent, ValveCommand::Partial(0.5), Wrapping(0))
            .unwrap();
        c.resolve(Wrapping(1));

        // ...is temporarily overridden by a pulse...
        c.try_command(
            V::OxidizerVent,
            ValveCommand::PulseOpen(Duration::from_millis(500)),
            Wrapping(100),
        )
        .unwrap();
        assert!(c.resolve(Wrapping(300))[V::OxidizerVent] == OPEN);

        // ...and restored on expiry, instead of the (closed) Hold baseline.
        let after = c.resolve(Wrapping(700));
        assert!(after[V::OxidizerVent] == ValveState::from_promille_clamped(500));
    }

    #[test]
    fn repeated_pulses_inherit_the_original_restore_target() {
        let mut c = ValveController::new();
        c.set_mode(M::Hold);

        c.try_command(V::OxidizerVent, ValveCommand::Partial(0.5), Wrapping(0))
            .unwrap();
        c.resolve(Wrapping(1));

        // A second pulse replaces the first mid-flight; the restore target
        // must stay the latched command, not become "open".
        for t in [100, 300] {
            c.try_command(
                V::OxidizerVent,
                ValveCommand::PulseOpen(Duration::from_millis(500)),
                Wrapping(t),
            )
            .unwrap();
        }

        assert!(c.resolve(Wrapping(500))[V::OxidizerVent] == OPEN);
        let after = c.resolve(Wrapping(900));
        assert!(after[V::OxidizerVent] == ValveState::from_promille_clamped(500));
    }

    #[test]
    fn invalid_commands_rejected() {
        let mut c = ValveController::new();
        c.set_mode(M::Hold);

        let overlong = MAX_PULSE_DURATION + Duration::from_millis(1);
        assert!(matches!(
            c.try_command(
                V::OxidizerVent,
                ValveCommand::PulseOpen(overlong),
                Wrapping(0)
            ),
            Err(ValveError::InvalidCommand)
        ));
        assert!(matches!(
            c.try_command(
                V::OxidizerVent,
                ValveCommand::Partial(f32::NAN),
                Wrapping(0)
            ),
            Err(ValveError::InvalidCommand)
        ));
        assert!(matches!(
            c.try_command(V::OxidizerVent, ValveCommand::Partial(1.5), Wrapping(0)),
            Err(ValveError::InvalidCommand)
        ));
        assert!(matches!(
            c.try_command(V::OxidizerVent, ValveCommand::Partial(-0.1), Wrapping(0)),
            Err(ValveError::InvalidCommand)
        ));

        // The boundaries themselves are accepted.
        c.try_command(
            V::OxidizerVent,
            ValveCommand::PulseOpen(MAX_PULSE_DURATION),
            Wrapping(0),
        )
        .unwrap();
        c.try_command(V::OxidizerVent, ValveCommand::Partial(1.0), Wrapping(0))
            .unwrap();
        c.try_command(V::OxidizerVent, ValveCommand::Partial(0.0), Wrapping(0))
            .unwrap();
    }

    #[test]
    fn pulse_expiry_is_wrapping_safe() {
        let mut c = ValveController::new();
        c.set_mode(M::Hold);

        // Command issued just before the 32-bit millisecond clock wraps.
        c.try_command(
            V::OxidizerVent,
            ValveCommand::PulseOpen(Duration::from_millis(500)),
            Wrapping(u32::MAX - 100),
        )
        .unwrap();

        assert!(c.resolve(Wrapping(u32::MAX - 50))[V::OxidizerVent] == OPEN);
        assert!(c.resolve(Wrapping(200))[V::OxidizerVent] == OPEN);
        assert!(c.resolve(Wrapping(500))[V::OxidizerVent] == CLOSED);
    }
}
