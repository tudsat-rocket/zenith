use std::sync::atomic::Ordering;

use mission::Outputs;
use mission::leds::LedState;

use crate::simulation::physics::RecoveryFlags;

pub struct StdOutputs {
    flags: RecoveryFlags,
    #[allow(dead_code)]
    recovery_armed: bool,
    #[allow(dead_code)]
    leds: LedState,
}

impl StdOutputs {
    pub fn new(flags: RecoveryFlags) -> Self {
        Self {
            flags,
            recovery_armed: false,
            leds: LedState::default(),
        }
    }
}

impl Outputs for StdOutputs {
    fn set_recovery_armed(&mut self, armed: bool) {
        self.recovery_armed = armed;
    }

    fn set_drogue(&mut self, high: bool) {
        self.flags.drogue.store(high, Ordering::Relaxed);
    }

    fn set_main(&mut self, high: bool) {
        self.flags.main.store(high, Ordering::Relaxed);
    }

    fn set_leds(&mut self, leds: LedState) {
        self.leds = leds;
    }
}
