//! Two-phase tank containing liquid N2O and a mixed gas ullage (N2 + N2O vapor)

use crate::simulation::hybrid::fluid::{
    self, AMBIENT_TEMP, INTRA_TANK_TIME_CONSTANT, N2_HEAT_CAPACITY, N2_MOLAR_MASS, N2O_LATENT_HEAT,
    N2O_LIQUID_HEAT_CAPACITY, N2O_MOLAR_MASS, N2O_VAPOR_HEAT_CAPACITY, WALL_COOLING_TIME_CONSTANT,
};

const MAX_TEMP_RATE: f32 = 10.0; // [K/s]

pub struct TwoPhaseTank {
    /// Liquid N2O mass [kg].
    pub liquid_mass: f32,
    /// N2 (pressurant) in the ullage [mol].
    pub ullage_pressurant_moles: f32,
    /// N2O vapor in the ullage [mol].
    pub ullage_n2o_vapor_moles: f32,
    /// Liquid pool temperature [K].
    pub liquid_temp: f32,
    /// Ullage gas temperature [K].
    pub ullage_temp: f32,
    /// Tank volume [L].
    pub volume: f32,
    /// Floor on the ullage volume used in pressure calculations [L].
    pub min_ullage: f32,
}

impl TwoPhaseTank {
    /// Construct a tank with the given volume [L], minimum ullage volume [L],
    /// initial pressure [bar], and initial temperature [K]. Starts empty of
    /// liquid; ullage is initialized to the given pressure with N2
    pub fn new(volume: f32, min_ullage: f32, initial_pressure: f32, initial_temp: f32) -> Self {
        Self {
            liquid_mass: 0.0,
            ullage_pressurant_moles: fluid::pressure_to_moles(
                initial_pressure,
                volume,
                initial_temp,
            ),
            ullage_n2o_vapor_moles: 0.0,
            liquid_temp: initial_temp,
            ullage_temp: initial_temp,
            volume,
            min_ullage,
        }
    }

    /// Liquid volume [L]
    pub fn liquid_volume(&self) -> f32 {
        if self.liquid_mass <= 0.0 {
            0.0
        } else {
            self.liquid_mass / fluid::n2o_liquid_density(self.liquid_temp)
        }
    }

    /// Ullage volume [L], floored at `min_ullage`
    fn ullage_volume(&self) -> f32 {
        (self.volume - self.liquid_volume()).max(self.min_ullage)
    }

    /// Ullage pressure [bar]
    pub fn ullage_pressure(&self) -> f32 {
        let total_moles = self.ullage_pressurant_moles + self.ullage_n2o_vapor_moles;
        fluid::moles_to_pressure(total_moles, self.ullage_volume(), self.ullage_temp)
    }

    /// Liquid fill level as a fraction of total volume
    pub fn fill_level(&self) -> f32 {
        (self.liquid_volume() / self.volume).clamp(0.0, 1.0)
    }

    /// Tank temperature [K]
    /// TODO: multiple sensors
    pub fn temperature(&self) -> f32 {
        if self.liquid_mass > 0.0 {
            self.liquid_temp
        } else {
            self.ullage_temp
        }
    }

    /// Add pressurant gas (N2) to the ullage at the given temperature [K]
    pub fn add_pressurant(&mut self, moles: f32, incoming_temp: f32) {
        let capacity_before = self.ullage_heat_capacity();
        let incoming_capacity = moles * N2_MOLAR_MASS * N2_HEAT_CAPACITY;
        let capacity_after = capacity_before + incoming_capacity;
        if capacity_after > 0.0 {
            self.ullage_temp = (capacity_before * self.ullage_temp
                + incoming_capacity * incoming_temp)
                / capacity_after;
        }
        self.ullage_pressurant_moles += moles;
    }

    /// Vent ullage gas to atmosphere
    pub fn vent_ullage(&mut self, total_moles: f32) {
        let total = self.ullage_pressurant_moles + self.ullage_n2o_vapor_moles;
        let delta = total_moles.min(total);
        if total > 0.0 {
            let n2_fraction = self.ullage_pressurant_moles / total;
            self.ullage_pressurant_moles -= delta * n2_fraction;
            self.ullage_n2o_vapor_moles -= delta * (1.0 - n2_fraction);

            let remaining = total - delta;
            if remaining > 0.0 {
                // gamma_eff ~ 1.3 for N2/N2O mix (between 1.4 for N2 and 1.27 for N2O)
                self.ullage_temp *= (remaining / total).powf(0.3);
            }
        }
    }

    /// Add liquid N2O [kg] at the given temperature [K]
    pub fn add_liquid(&mut self, mass: f32, incoming_temp: f32) {
        let mass_after = self.liquid_mass + mass;
        if mass_after > 0.0 {
            self.liquid_temp =
                (self.liquid_mass * self.liquid_temp + mass * incoming_temp) / mass_after;
        }
        self.liquid_mass = mass_after;
    }

    /// Drain liquid [kg]. Returns the actual mass drained
    pub fn drain_liquid(&mut self, mass: f32) -> f32 {
        let drained = mass.min(self.liquid_mass);
        self.liquid_mass -= drained;
        drained
    }

    pub fn tick(&mut self, dt: f32) {
        // Equilibrate N2O vapor with the liquid pool. While liquid is present,
        // vapor evaporates or condenses each tick to track saturation at the
        // liquid temperature. Latent heat is debited/credited to the liquid pool.
        let liquid_volume = self.liquid_volume();
        let ullage_volume = (self.volume - liquid_volume).max(self.min_ullage);

        let saturation_pressure = fluid::n2o_saturation_pressure(self.liquid_temp);
        let moles_at_saturation =
            fluid::pressure_to_moles(saturation_pressure, ullage_volume, self.ullage_temp);

        if self.liquid_mass > 0.0 && self.ullage_n2o_vapor_moles < moles_at_saturation {
            self.evaporate(dt, moles_at_saturation);
        } else if self.ullage_n2o_vapor_moles > moles_at_saturation {
            self.condense(dt, moles_at_saturation, liquid_volume);
        }

        // equalize liquid and gas temperatures
        let liquid_capacity = self.liquid_heat_capacity();
        let ullage_capacity = self.ullage_heat_capacity();
        if liquid_capacity > 0.0 && ullage_capacity > 0.0 {
            let coupling = liquid_capacity.min(ullage_capacity) / INTRA_TANK_TIME_CONSTANT;
            let heat_flow = coupling * (self.ullage_temp - self.liquid_temp) * dt;
            self.liquid_temp += heat_flow / liquid_capacity;
            self.ullage_temp -= heat_flow / ullage_capacity;
        }

        // equalize temperature to atmosphere
        let alpha = (dt / WALL_COOLING_TIME_CONSTANT).min(1.0);
        self.liquid_temp += (AMBIENT_TEMP - self.liquid_temp) * alpha;
        self.ullage_temp += (AMBIENT_TEMP - self.ullage_temp) * alpha;

        // clamp values
        self.liquid_mass = self.liquid_mass.max(0.0);
        self.ullage_pressurant_moles = self.ullage_pressurant_moles.max(0.0);
        self.ullage_n2o_vapor_moles = self.ullage_n2o_vapor_moles.max(0.0);
        self.liquid_temp = self.liquid_temp.clamp(200.0, 320.0);
        self.ullage_temp = self.ullage_temp.clamp(200.0, 320.0);
    }

    fn evaporate(&mut self, dt: f32, moles_at_saturation: f32) {
        let liquid_capacity_before = self.liquid_heat_capacity();
        let ullage_capacity_before = self.ullage_heat_capacity();

        let mass_cap = if liquid_capacity_before > 0.0 {
            MAX_TEMP_RATE * dt * liquid_capacity_before / N2O_LATENT_HEAT
        } else {
            f32::MAX
        };

        let mass_needed = (moles_at_saturation - self.ullage_n2o_vapor_moles) * N2O_MOLAR_MASS;
        let delta_mass = mass_needed.min(self.liquid_mass).min(mass_cap);
        let delta_moles = delta_mass / N2O_MOLAR_MASS;

        let liquid_temp_before = self.liquid_temp;

        self.liquid_mass -= delta_mass;
        self.ullage_n2o_vapor_moles += delta_moles;

        if liquid_capacity_before > 0.0 {
            self.liquid_temp -= N2O_LATENT_HEAT * delta_mass / liquid_capacity_before;
        }

        if ullage_capacity_before > 0.0 {
            let incoming_capacity = delta_mass * N2O_VAPOR_HEAT_CAPACITY;
            let ullage_capacity_after = ullage_capacity_before + incoming_capacity;
            self.ullage_temp = (ullage_capacity_before * self.ullage_temp
                + incoming_capacity * liquid_temp_before)
                / ullage_capacity_after;
        }
    }

    fn condense(&mut self, dt: f32, moles_at_saturation: f32, liquid_volume: f32) {
        let liquid_capacity_before = self.liquid_heat_capacity();

        let mass_cap = MAX_TEMP_RATE * dt * liquid_capacity_before / N2O_LATENT_HEAT;
        let mass_needed = (self.ullage_n2o_vapor_moles - moles_at_saturation) * N2O_MOLAR_MASS;
        let volume_headroom =
            (self.volume - liquid_volume).max(0.0) * fluid::n2o_liquid_density(self.liquid_temp);
        let delta_mass = mass_needed.min(volume_headroom).min(mass_cap);
        let delta_moles = delta_mass / N2O_MOLAR_MASS;

        let ullage_temp_before = self.ullage_temp;

        self.liquid_mass += delta_mass;
        self.ullage_n2o_vapor_moles -= delta_moles;

        if liquid_capacity_before > 0.0 {
            let incoming_capacity = delta_mass * N2O_LIQUID_HEAT_CAPACITY;
            let liquid_capacity_after = liquid_capacity_before + incoming_capacity;
            self.liquid_temp = (liquid_capacity_before * self.liquid_temp
                + incoming_capacity * ullage_temp_before)
                / liquid_capacity_after
                + N2O_LATENT_HEAT * delta_mass / liquid_capacity_after;
        }
    }

    /// Heat capacity of the ullage gas mixture [J/K].
    fn ullage_heat_capacity(&self) -> f32 {
        let n2_mass = self.ullage_pressurant_moles * N2_MOLAR_MASS;
        let n2o_vapor_mass = self.ullage_n2o_vapor_moles * N2O_MOLAR_MASS;
        n2_mass * N2_HEAT_CAPACITY + n2o_vapor_mass * N2O_VAPOR_HEAT_CAPACITY
    }

    /// Heat capacity of the liquid pool [J/K].
    fn liquid_heat_capacity(&self) -> f32 {
        self.liquid_mass.max(1e-4) * N2O_LIQUID_HEAT_CAPACITY
    }
}
