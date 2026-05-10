use std::sync::atomic::Ordering;

use mission::Outputs;

use crate::simulation::physics::RecoveryFlags;

pub struct StdOutputs {
    flags: RecoveryFlags,
    #[allow(dead_code)]
    recovery_armed: bool,
}

impl StdOutputs {
    pub fn new(flags: RecoveryFlags) -> Self {
        Self {
            flags,
            recovery_armed: false,
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
}
