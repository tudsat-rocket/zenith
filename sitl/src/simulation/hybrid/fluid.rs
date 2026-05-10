//! Gas law helpers and N2O material property functions

/// Ambient temperature [K]
pub const AMBIENT_TEMP: f32 = 298.15;
/// Ambient pressure [bar]
pub const AMBIENT_PRESSURE: f32 = 1.0;

/// Universal gas constant [J/(mol*K)]
const GAS_CONSTANT: f32 = 8.314;

// N2O phase-equilibrium model
const SAT_ANCHOR_PRESSURE: f32 = 42.5;
/// Anchor temperature for the N2O sat curve [K]
const SAT_ANCHOR_TEMP: f32 = 283.15;

/// Latent heat of vaporization for N2O [J/kg]
pub const N2O_LATENT_HEAT: f32 = 376_000.0;
/// Molar mass of N2O [kg/mol]
pub const N2O_MOLAR_MASS: f32 = 0.044;
/// Molar mass of N2 [kg/mol]
pub const N2_MOLAR_MASS: f32 = 0.028;

// Heat capacities [J/(kg*K)]
pub const N2O_LIQUID_HEAT_CAPACITY: f32 = 1900.0;
pub const N2O_VAPOR_HEAT_CAPACITY: f32 = 880.0;
pub const N2_HEAT_CAPACITY: f32 = 1040.0;

/// Time constant for tank-wall heat exchange with ambient [s]
pub const WALL_COOLING_TIME_CONSTANT: f32 = 1800.0;
/// Time constant for heat exchange between liquid pool and ullage gas [s]
pub const INTRA_TANK_TIME_CONSTANT: f32 = 90.0;

/// Ideal gas law, returns moles given pressure [bar], volume [L], temperature [K]
pub fn pressure_to_moles(pressure: f32, volume: f32, temp: f32) -> f32 {
    let pressure_pa = pressure * 1.0e5;
    let volume_m3 = volume * 1.0e-3;
    pressure_pa * volume_m3 / (GAS_CONSTANT * temp)
}

/// Ideal gas law, returns pressure [bar] given moles, volume [L], temperature [K]
pub fn moles_to_pressure(moles: f32, volume: f32, temp: f32) -> f32 {
    if volume <= 0.0 {
        return AMBIENT_PRESSURE;
    }
    let volume_m3 = volume * 1.0e-3;
    let pressure_pa = moles * GAS_CONSTANT * temp / volume_m3;
    pressure_pa * 1.0e-5
}

/// Saturation pressure of N2O [bar] at temperature [K]
pub fn n2o_saturation_pressure(temp: f32) -> f32 {
    // Clausius-Clapeyron, anchored at 10 C / 42.5 bar with a constant latent
    // heat. Accurate to ~5 % over 0-35 C; goes singular near the 36.4 C
    // critical point but the temperature clamps keep us well below that.
    let exponent =
        (N2O_LATENT_HEAT * N2O_MOLAR_MASS / GAS_CONSTANT) * (1.0 / SAT_ANCHOR_TEMP - 1.0 / temp);
    SAT_ANCHOR_PRESSURE * exponent.exp()
}

/// Liquid N2O density [kg/L] at temperature [K]
pub fn n2o_liquid_density(temp: f32) -> f32 {
    let temp_c = temp - 273.15;
    (0.91 - 0.0066 * temp_c).clamp(0.5, 1.0)
}
