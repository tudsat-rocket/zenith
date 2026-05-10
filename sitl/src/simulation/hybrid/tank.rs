//! Single-phase ideal-gas tank (e.g. N2 pressurant tank)

use crate::simulation::hybrid::fluid::{
    AMBIENT_TEMP, WALL_COOLING_TIME_CONSTANT, moles_to_pressure, pressure_to_moles,
};

pub struct Tank {
    /// Gas content [mol]
    pub moles: f32,
    /// Bulk gas temperature [K]
    pub temp: f32,
    /// Tank volume [L]
    volume: f32,
}

impl Tank {
    /// Construct a tank with the given volume [L], initial pressure [bar],
    /// and initial temperature [K].
    pub fn new(volume: f32, initial_pressure: f32, initial_temp: f32) -> Self {
        Self {
            moles: pressure_to_moles(initial_pressure, volume, initial_temp),
            temp: initial_temp,
            volume,
        }
    }

    /// Tank pressure [bar]
    pub fn pressure(&self) -> f32 {
        moles_to_pressure(self.moles, self.volume, self.temp)
    }

    /// Add gas at the given temperature [K]
    pub fn add_gas(&mut self, moles: f32, incoming_temp: f32) {
        let moles_before = self.moles;
        self.moles += moles;
        if self.moles > 0.0 {
            self.temp = (moles_before * self.temp + moles * incoming_temp) / self.moles;
        }
    }

    /// Remove gas with isentropic expansion cooling (gamma = 7/5 for N2)
    pub fn remove_gas(&mut self, moles: f32) {
        let moles_before = self.moles;
        self.moles -= moles.min(self.moles);
        if moles_before > 0.0 && self.moles > 0.0 {
            self.temp *= (self.moles / moles_before).powf(0.4);
        }
    }

    pub fn tick(&mut self, dt: f32) {
        let blend = (dt / WALL_COOLING_TIME_CONSTANT).min(1.0);
        self.temp += (AMBIENT_TEMP - self.temp) * blend;

        self.moles = self.moles.max(0.0);
        self.temp = self.temp.clamp(200.0, 320.0);
    }
}
