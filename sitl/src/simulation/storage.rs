//! Stands in for the flight computer's NOR flash.

use mission::params::{ParamId, ParamValue, ParameterGroup, SharedParams};
use mission::{Params, Storage};

/// Live parameter mirror served to ground stations by the param protocol task.
pub static PARAM_STORE: SharedParams = SharedParams::new();

/// Persists params and parameter writes for the duration of a SITL session.
#[derive(Default)]
pub struct MemoryStorage {
    stored: Option<Params>,
}

pub fn init(stored: Option<Params>) -> MemoryStorage {
    PARAM_STORE.init(stored.clone().unwrap_or_default());

    MemoryStorage::new(stored)
}

impl MemoryStorage {
    pub fn new(stored: Option<Params>) -> Self {
        Self { stored }
    }

    pub fn stored(&self) -> Option<&Params> {
        self.stored.as_ref()
    }
}

impl Storage for MemoryStorage {
    fn read_params(&mut self) -> Option<Params> {
        self.stored.clone()
    }

    fn write_param(&mut self, id: ParamId, value: ParamValue) {
        let mut params = self.stored.take().unwrap_or_default();
        params.set(id, value);
        self.stored = Some(params);
    }
}
