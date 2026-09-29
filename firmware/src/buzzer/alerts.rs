//! Decides what the buzzer plays, based on the flight mode and the battery
//! voltage.
//!
//! [`Alerts::update`] is called from the main loop on every tick. It emits
//! two kinds of requests:
//!
//! * A short chirp on every flight mode change ([`Sound::ModeChange`]).
//! * A continuous *alert loop*, for conditions that last longer than a single
//!   sound. Only one can be active at a time, in order of priority:
//!   1. [`Sound::Landed`] - locator beacon, so the rocket can be found.
//!   2. [`Sound::Pressurized`] - hazard warning, from the moment the vehicle is
//!      pressurized until it lifts off (or is vented again).
//!
//! Battery warnings are one-shots repeated at a fixed interval. They are muted
//! while an alert loop is running, because both of those are more important
//! than a battery that is running low, and after liftoff, where nobody can hear
//! or act on them anyway.

use defmt::*;
use embassy_time::{Duration, Instant};
use rapid_dialect::FlightMode;

use super::{Sound, request_loop, request_sound};

/// Bus voltage below which the battery is considered low [mV].
///
/// The flight computer runs off a 3S LiPo, i.e. 12.6V fully charged and 9.9V
/// empty. 11.5V is ~3.83V per cell, 10.0V is ~3.33V per cell.
const BATTERY_LOW_MV: u16 = 11_500;
/// Bus voltage below which the battery is considered nearly empty [mV].
const BATTERY_EXTREME_LOW_MV: u16 = 10_000;
/// Hysteresis applied when recovering from a warning [mV], so that a voltage
/// hovering around a threshold does not toggle the warning back and forth.
const BATTERY_HYSTERESIS_MV: u16 = 300;
/// Below this the main bus is assumed to be unpowered [mV], e.g. because the
/// board is being bench-tested without a battery, and warnings are muted.
const BATTERY_PRESENT_MIN_MV: u16 = 5_000;
/// How long the voltage has to stay beyond a threshold before we warn about it.
const BATTERY_DEBOUNCE: Duration = Duration::from_secs(2);
/// How often the low battery warning repeats.
const BATTERY_LOW_INTERVAL: Duration = Duration::from_secs(30);
/// How often the nearly empty battery warning repeats.
const BATTERY_EXTREME_LOW_INTERVAL: Duration = Duration::from_secs(10);

#[derive(Clone, Copy, Default, PartialEq, Eq, Format)]
enum BatteryState {
    #[default]
    Ok,
    Low,
    ExtremeLow,
}

/// Buzzer alert state machine, see the [module documentation](self).
///
/// Starts out in the flight mode the vehicle boots up in, so that no mode
/// change chirp is emitted for the initial mode.
#[derive(Default)]
pub struct Alerts {
    mode: FlightMode,
    /// Whether the vehicle is currently pressurized.
    pressurized: bool,
    battery: BatteryState,
    /// The battery state we are about to switch to, and since when the voltage
    /// has continuously called for it.
    battery_pending: Option<(BatteryState, Instant)>,
    /// When the current battery warning was last played.
    battery_warned_at: Option<Instant>,
    /// The alert loop currently requested, so it is only sent once.
    alert_loop: Option<Sound>,
}

impl Alerts {
    /// Update the buzzer with the current flight mode and main bus voltage
    /// (`None` if the ADC has no reading yet).
    pub fn update(&mut self, mode: FlightMode, battery_mv: Option<u16>) {
        if mode != self.mode {
            self.mode = mode;
            self.mode_changed(mode);
        }

        self.update_battery(battery_mv);
        self.warn_about_battery();
    }

    fn mode_changed(&mut self, mode: FlightMode) {
        info!("Buzzer: mode changed to {}", Debug2Format(&mode));

        // Track whether the vehicle is pressurized.
        self.pressurized = match mode {
            // Entering Pressurize opens the pressurization valve.
            FlightMode::Pressurize => true,
            // Reachable from both pressurized and unpressurized modes, so keep
            // whatever we were in: the tanks stay pressurized while the launch
            // is awaited and while the ignition sequence runs.
            FlightMode::Hold | FlightMode::DetectLaunch | FlightMode::Ignite => self.pressurized,
            // Either not pressurized (yet), or already lifted off - in both
            // cases nobody is standing next to a pressurized rocket.
            FlightMode::Idle
            | FlightMode::FillPressurant
            | FlightMode::FillOxidizer
            | FlightMode::Vent
            | FlightMode::Burn
            | FlightMode::Coast
            | FlightMode::DeployDrogue
            | FlightMode::DeployMain
            | FlightMode::Landed => false,
        };

        // The alert loop is updated before the chirp is requested, because a
        // one-shot takes precedence over the loop, but not the other way
        // around: doing it the other way around would cut the chirp short.
        self.set_alert_loop(if mode == FlightMode::Landed {
            Some(Sound::Landed)
        } else if self.pressurized {
            Some(Sound::Pressurized)
        } else {
            None
        });

        request_sound(Sound::ModeChange);
    }

    /// Whether a continuous alert is currently sounding.
    ///
    /// Used to keep lower-priority sounds, e.g. a tune requested from the
    /// ground, from silencing the hazard warning or the landing beacon.
    pub fn alert_active(&self) -> bool {
        self.alert_loop.is_some()
    }

    fn set_alert_loop(&mut self, sound: Option<Sound>) {
        if sound != self.alert_loop {
            self.alert_loop = sound;
            request_loop(sound);
        }
    }

    /// Classify the given voltage, with hysteresis around the thresholds, and
    /// commit the new state once it has been stable for [`BATTERY_DEBOUNCE`].
    fn update_battery(&mut self, battery_mv: Option<u16>) {
        let Some(battery_mv) = battery_mv else {
            return;
        };

        let candidate = self.classify_battery(battery_mv);
        if candidate == self.battery {
            self.battery_pending = None;
            return;
        }

        match self.battery_pending {
            Some((pending, since)) if pending == candidate => {
                if since.elapsed() >= BATTERY_DEBOUNCE {
                    info!(
                        "Battery state {} -> {} ({} mV)",
                        self.battery, candidate, battery_mv
                    );
                    self.battery = candidate;
                    self.battery_pending = None;
                    // Warn about the new state right away.
                    self.battery_warned_at = None;
                }
            }
            _ => self.battery_pending = Some((candidate, Instant::now())),
        }
    }

    fn classify_battery(&self, battery_mv: u16) -> BatteryState {
        // Recovering from a warning requires a little more voltage than
        // entering it did. Note that a battery under load sags, so the voltage
        // does jump around by a few hundred mV in normal operation.
        let hysteresis = |threshold: u16, state: BatteryState| {
            if self.battery == state {
                threshold.saturating_add(BATTERY_HYSTERESIS_MV)
            } else {
                threshold
            }
        };

        // A voltage this low means the main bus is not powered by a battery at
        // all, e.g. because the board is running off the debug probe, rather
        // than by a battery that is nearly empty. Once we have decided that
        // there is a battery and that it is running low, we keep warning about
        // it, no matter how far it sags.
        if battery_mv < BATTERY_PRESENT_MIN_MV && self.battery == BatteryState::Ok {
            BatteryState::Ok
        } else if battery_mv < hysteresis(BATTERY_EXTREME_LOW_MV, BatteryState::ExtremeLow) {
            BatteryState::ExtremeLow
        } else if battery_mv < hysteresis(BATTERY_LOW_MV, BatteryState::Low) {
            BatteryState::Low
        } else {
            BatteryState::Ok
        }
    }

    fn warn_about_battery(&mut self) {
        let (sound, interval) = match self.battery {
            BatteryState::Ok => return,
            BatteryState::Low => (Sound::BatteryLow, BATTERY_LOW_INTERVAL),
            BatteryState::ExtremeLow => (Sound::BatteryExtremeLow, BATTERY_EXTREME_LOW_INTERVAL),
        };

        // An active alert loop and the flight itself both take precedence. The
        // warning is not lost, it is played once the buzzer is free again.
        if self.alert_loop.is_some() || self.mode >= FlightMode::Burn {
            return;
        }

        let due = self
            .battery_warned_at
            .is_none_or(|warned_at| warned_at.elapsed() >= interval);
        if due {
            request_sound(sound);
            self.battery_warned_at = Some(Instant::now());
        }
    }
}
