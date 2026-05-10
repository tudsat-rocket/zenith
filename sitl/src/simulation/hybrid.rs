//! Hybrid-rocket propulsion simulation for the SITL

use rapid_dialect::FlightMode;
use rapid_dialect::rapid::enums::{PropulsionType, ValveId};

use mission::propulsion::{
    Propulsion, PropulsionError, TankId, TankReading, ValveCommand, ValveReading,
};

use crate::simulation::hybrid::fluid::{
    AMBIENT_PRESSURE, AMBIENT_TEMP, n2o_liquid_density, n2o_saturation_pressure,
};
use crate::simulation::hybrid::tank::Tank;
use crate::simulation::hybrid::two_phase::TwoPhaseTank;
use crate::simulation::hybrid::valves::Valve;

mod fluid;
mod tank;
mod two_phase;
mod valves;

pub struct HybridSimulation {
    /// Nitrogen pressurant tank
    pressurant: Tank,
    /// Nitrous oxidizer tank
    oxidizer: TwoPhaseTank,
    /// Vent valve for the pressurant tank
    press_vent_valve: Valve,
    /// Pressurization valve between pressure regulator and oxidizer tank
    press_valve: Valve,
    /// Fill & dump valve in the bottom of the oxidizer tank
    ox_fill_valve: Valve,
    /// Vent valve on top of the oxidizer tank
    ox_vent_valve: Valve,
    /// Main valve, connecting oxidizer tank to combustion chamber
    main_valve: Valve,
    /// Remaining liquid N2O in the external GSE supply cylinder [kg].
    /// Filling debits this; once empty, fill flow stops regardless of dp.
    pub supply_n2o_mass: f32,
    /// Combustion chamber pressure [bar]
    pub chamber_pressure: f32,
    /// Solid fuel grain mass [kg]
    pub fuel_mass: f32,
    /// Has the igniter been fired?
    igniter_fired: bool,
    /// Current firmware flight mode
    flight_mode: FlightMode,
}

impl Default for HybridSimulation {
    fn default() -> Self {
        Self {
            // (tank volume [L], initial pressure [bar], initial temp [C])
            pressurant: Tank::new(2.0, AMBIENT_PRESSURE, AMBIENT_TEMP),
            // (tank volume [L], minmum ullage [L], initial pressure [bar], initial temp [C])
            oxidizer: TwoPhaseTank::new(7.81, 0.01, AMBIENT_PRESSURE, AMBIENT_TEMP),
            // (conductance, travel time) for each valve
            press_vent_valve: Valve::new(0.07, 0.2),
            press_valve: Valve::new(0.1, 0.5),
            ox_fill_valve: Valve::new(0.05, 0.2),
            ox_vent_valve: Valve::new(0.03, 0.2),
            main_valve: Valve::new(0.027, 0.5),
            supply_n2o_mass: 30.0,
            chamber_pressure: 0.0,
            fuel_mass: 1.5,
            igniter_fired: false,
            flight_mode: FlightMode::Idle,
        }
    }
}

impl HybridSimulation {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_flight_mode(&mut self, mode: FlightMode) {
        self.flight_mode = mode;
    }

    pub fn tick(&mut self, dt: f32) {
        /// Conductance for GSE pressurant fill [mol/(s*bar)]
        const CONDUCTANCE_PRESSURANT_FILL: f32 = 0.07;
        /// GSE high-pressure pressurant supply [bar].
        const GSE_SUPPLY_PRESSURE: f32 = 280.0;

        /// Regulator setpoint for the pressurization line [bar].
        const REGULATOR_SETPOINT: f32 = 55.0;

        /// Fuel regression rate during burn [kg/s].
        const FUEL_BURN_RATE: f32 = 0.2;
        /// Chamber pressure produced per unit total mass flow [bar/(kg/s)].
        const CHAMBER_PRESSURE_PER_MASS_FLOW: f32 = 16.0;
        /// Time constant for chamber pressure response to mass flow [s].
        const CHAMBER_PRESSURE_TIME_CONSTANT: f32 = 0.05;

        self.press_vent_valve.tick(dt);
        self.press_valve.tick(dt);
        self.ox_fill_valve.tick(dt);
        self.ox_vent_valve.tick(dt);
        self.main_valve.tick(dt);

        let pressurant_pressure = self.pressurant.pressure();
        let ullage_pressure = self.oxidizer.ullage_pressure();

        // In Idle we simulate someone manually filling N2.
        if self.flight_mode == FlightMode::Idle {
            let pressure_diff = (GSE_SUPPLY_PRESSURE - pressurant_pressure).max(0.0);
            let delta_moles = CONDUCTANCE_PRESSURANT_FILL * pressure_diff * dt;
            self.pressurant.add_gas(delta_moles, AMBIENT_TEMP);
        }

        // Pressurant Vent Valve
        let press_vent_diff = (pressurant_pressure - AMBIENT_PRESSURE).max(0.0);
        let delta_moles = self.press_vent_valve.conductance() * press_vent_diff * dt;
        self.pressurant.remove_gas(delta_moles);

        // Pressurization Valve
        let regulated_pressure = pressurant_pressure.min(REGULATOR_SETPOINT);
        let press_pressure_diff = (regulated_pressure - ullage_pressure).max(0.0);
        let delta_moles =
            (self.press_valve.conductance() * press_pressure_diff * dt).min(self.pressurant.moles);
        self.oxidizer
            .add_pressurant(delta_moles, self.pressurant.temp);
        self.pressurant.remove_gas(delta_moles);

        // Oxidizer Vent Valve
        let ox_vent_diff = (ullage_pressure - AMBIENT_PRESSURE).max(0.0);
        let delta_moles = self.ox_vent_valve.conductance() * ox_vent_diff * dt;
        self.oxidizer.vent_ullage(delta_moles);

        // Ox fill from the external supply cylinder
        let current_ullage_volume =
            (self.oxidizer.volume - self.oxidizer.liquid_volume()).max(self.oxidizer.min_ullage);
        let pressure_diff = (n2o_saturation_pressure(AMBIENT_TEMP) - ullage_pressure).max(0.0);
        let inflow_density = n2o_liquid_density(AMBIENT_TEMP);

        let ullage_fraction = (current_ullage_volume / self.oxidizer.volume).clamp(0.0, 1.0);
        let volume_request =
            self.ox_fill_valve.conductance() * pressure_diff * ullage_fraction * dt;

        let ox_delta_mass = (volume_request * inflow_density).min(self.supply_n2o_mass);
        self.oxidizer.add_liquid(ox_delta_mass, AMBIENT_TEMP);
        self.supply_n2o_mass -= ox_delta_mass;

        // Drain liquid through the main valve against chamber back-pressure.
        let downstream = self.chamber_pressure.max(AMBIENT_PRESSURE);
        let main_pressure_diff = (ullage_pressure - downstream).max(0.0);
        let delta_volume = self.main_valve.conductance() * main_pressure_diff * dt;

        let liquid_density = n2o_liquid_density(self.oxidizer.liquid_temp);
        let ox_mass_flow = self.oxidizer.drain_liquid(delta_volume * liquid_density) / dt;

        let combustion = self.main_valve.state() > 0.2
            && self.igniter_fired
            && self.oxidizer.liquid_mass > 0.0
            && self.fuel_mass > 0.0;
        let c = (combustion as u32) as f32;

        let fuel_mass_flow = c * FUEL_BURN_RATE;
        self.fuel_mass = (self.fuel_mass - fuel_mass_flow * dt).max(0.0);

        let target_pressure = c * CHAMBER_PRESSURE_PER_MASS_FLOW * (ox_mass_flow + fuel_mass_flow);
        let alpha = (dt / CHAMBER_PRESSURE_TIME_CONSTANT).min(1.0);
        self.chamber_pressure += (target_pressure - self.chamber_pressure) * alpha;

        self.supply_n2o_mass = self.supply_n2o_mass.max(0.0);
        self.fuel_mass = self.fuel_mass.max(0.0);

        self.pressurant.tick(dt);
        self.oxidizer.tick(dt);
    }

    /// Liquid N2O volume [L]
    pub fn liquid_volume(&self) -> f32 {
        self.oxidizer.liquid_volume()
    }

    /// Pressure [bar] for the requested tank
    pub fn tank_pressure(&self, id: TankId) -> f32 {
        match id {
            TankId::Pressurant => self.pressurant.pressure(),
            TankId::Oxidizer => self.oxidizer.ullage_pressure(),
            TankId::CombustionChamber => self.chamber_pressure,
        }
    }

    /// Fill level (0..=1) for the requested tank
    pub fn tank_level(&self, id: TankId) -> f32 {
        match id {
            TankId::Oxidizer => self.oxidizer.fill_level(),
            _ => 0.0,
        }
    }

    /// Tank temperature [C], or None for the combustion chamber
    pub fn tank_temperature(&self, id: TankId) -> Option<f32> {
        match id {
            TankId::Pressurant => Some(self.pressurant.temp - 273.15),
            TankId::Oxidizer => Some(self.oxidizer.temperature() - 273.15),
            TankId::CombustionChamber => None,
        }
    }

    /// Override the external supply cylinder mass [kg]
    pub fn set_supply_n2o_mass(&mut self, mass: f32) {
        self.supply_n2o_mass = mass.max(0.0);
    }

    fn valve(&self, id: ValveId) -> &Valve {
        match id {
            ValveId::PressurantVent => &self.press_vent_valve,
            ValveId::Pressurization => &self.press_valve,
            ValveId::OxidizerVent => &self.ox_vent_valve,
            ValveId::OxidizerFill => &self.ox_fill_valve,
            ValveId::Main => &self.main_valve,
        }
    }

    fn valve_mut(&mut self, id: ValveId) -> &mut Valve {
        match id {
            ValveId::PressurantVent => &mut self.press_vent_valve,
            ValveId::Pressurization => &mut self.press_valve,
            ValveId::OxidizerVent => &mut self.ox_vent_valve,
            ValveId::OxidizerFill => &mut self.ox_fill_valve,
            ValveId::Main => &mut self.main_valve,
        }
    }

    pub fn command_valve(&mut self, id: ValveId, cmd: ValveCommand) {
        self.valve_mut(id).command(cmd);
    }

    pub fn valve_state(&self, id: ValveId) -> f32 {
        self.valve(id).state()
    }

    pub fn fire_igniter(&mut self) {
        self.igniter_fired = true;
    }
}

pub struct SitlPropulsion {
    sim: super::SharedSimulation,
}

impl SitlPropulsion {
    pub fn new(sim: super::SharedSimulation) -> Self {
        Self { sim }
    }
}

impl Propulsion for SitlPropulsion {
    const PROPULSION_TYPE: PropulsionType = PropulsionType::Hybrid;

    fn tank_state(&self, id: TankId) -> Option<TankReading> {
        let sim = self.sim.lock().ok()?;
        Some(TankReading {
            pressure1: Some(sim.hybrid.tank_pressure(id)),
            pressure2: None,
            temperature1: sim.hybrid.tank_temperature(id),
            temperature2: None,
            level: match id {
                TankId::Oxidizer => Some(sim.hybrid.tank_level(id)),
                _ => None,
            },
        })
    }

    fn valve_state(&self, id: ValveId) -> Option<ValveReading> {
        let sim = self.sim.lock().ok()?;
        Some(ValveReading {
            commanded_state: Some(sim.hybrid.valve(id).state()),
            measured_state: Some(sim.hybrid.valve(id).state()),
        })
    }

    fn command_valve(&mut self, id: ValveId, cmd: ValveCommand) -> Result<(), PropulsionError> {
        let mut sim = self
            .sim
            .lock()
            .map_err(|_| PropulsionError::TransportFailed)?;
        sim.hybrid.command_valve(id, cmd);
        Ok(())
    }

    fn fire_igniter(&mut self) -> Result<(), PropulsionError> {
        let mut sim = self
            .sim
            .lock()
            .map_err(|_| PropulsionError::TransportFailed)?;
        sim.hybrid.fire_igniter();
        Ok(())
    }
}
