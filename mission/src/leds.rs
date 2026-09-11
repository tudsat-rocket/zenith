use rapid_dialect::FlightMode;

/// Duration of a single slot of the repeating blink cycle, in milliseconds.
const SLOT_MS: u32 = 150;

/// Number of slots per blink cycle.
const SLOTS: u32 = 8;

/// State of the three on-board status LEDs.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct LedState {
    pub red: bool,
    pub yellow: bool,
    pub green: bool,
}

impl LedState {
    pub fn for_mode(mode: FlightMode, time_ms: u32) -> Self {
        let (red, yellow, green) = Self::masks(mode);
        let slot = (time_ms / SLOT_MS) % SLOTS;
        let bit = 1u8.wrapping_shl(slot);

        Self {
            red: red & bit != 0,
            yellow: yellow & bit != 0,
            green: green & bit != 0,
        }
    }

    /// The `(red, yellow, green)` blink masks of a mode, one bit per slot of the cycle, LSB first.
    const fn masks(mode: FlightMode) -> (u8, u8, u8) {
        match mode {
            FlightMode::Idle => (0, 0b0000_0001, 0xff),
            FlightMode::Landed => (0b0000_0001, 0, 0xff),
            FlightMode::FillPressurant => (0, 0xff, 0b0000_0001),
            FlightMode::FillOxidizer => (0, 0xff, 0b0000_0101),
            FlightMode::Vent => (0b0000_0100, 0xff, 0b0000_0001),
            FlightMode::Pressurize => (0b0000_0001, 0xff, 0),
            FlightMode::Hold => (0b0000_0001, 0xff, 0b0000_0001),
            FlightMode::DetectLaunch => (0xff, 0b0000_0001, 0),
            FlightMode::Ignite => (0xff, 0b0001_0101, 0),
            FlightMode::Burn => (0xff, 0b0000_0001, 0b0000_0100),
            FlightMode::Coast => (0xff, 0, 0b0000_0001),
            FlightMode::DeployDrogue => (0xff, 0, 0b0000_0101),
            FlightMode::DeployMain => (0xff, 0b0000_0001, 0b0000_0001),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_mode_has_one_solid_and_something_flashing() {
        for mode in FlightMode::ALL {
            let (red, yellow, green) = LedState::masks(mode);

            let solid = [red, yellow, green].iter().filter(|m| **m == 0xff).count();
            assert_eq!(solid, 1, "{mode:?} must light exactly one LED continuously");

            let flashing = [red, yellow, green]
                .iter()
                .filter(|m| **m != 0xff && **m != 0)
                .count();
            assert!(flashing > 0, "{mode:?} must flash at least one LED");
        }
    }

    #[test]
    fn modes_are_distinguishable() {
        for (i, a) in FlightMode::ALL.iter().enumerate() {
            for b in &FlightMode::ALL[i + 1..] {
                assert_ne!(
                    LedState::masks(*a),
                    LedState::masks(*b),
                    "{a:?} and {b:?} share a LED pattern"
                );
            }
        }
    }

    #[test]
    fn pattern_repeats_every_cycle() {
        let cycle = SLOT_MS * SLOTS;
        for mode in FlightMode::ALL {
            for t in 0..cycle {
                assert_eq!(
                    LedState::for_mode(mode, t),
                    LedState::for_mode(mode, t + cycle)
                );
            }
        }
    }
}
