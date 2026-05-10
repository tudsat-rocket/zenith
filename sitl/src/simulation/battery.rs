//! Basic 3S1P LiIon battery simulation

use rapid_dialect::FlightMode;

const CELLS_SERIES: u8 = 3;
const CAPACITY_AH: f32 = 3.0;
const CELL_IR_OHM: f32 = 0.04;
const OCV_CURVE: &[(f32, f32)] = &[
    (0.00, 3.20),
    (0.05, 3.40),
    (0.20, 3.60),
    (0.50, 3.75),
    (0.80, 3.95),
    (1.00, 4.20),
];

pub struct Battery {
    /// current state of charge, 0.0 - 1.0
    pub soc: f32,
    /// battery current [A], positive for discharge
    pub current: f32,
    /// voltage [V].
    pub voltage: f32,
}

impl Default for Battery {
    fn default() -> Self {
        Self::new()
    }
}

impl Battery {
    pub fn new() -> Self {
        let soc = 0.90;
        Self {
            soc,
            current: 0.0,
            voltage: cell_ocv(soc) * f32::from(CELLS_SERIES),
        }
    }

    pub fn tick(&mut self, dt: f32, mode: FlightMode) {
        self.current = match mode {
            FlightMode::Idle => -0.2,
            FlightMode::Filling
            | FlightMode::Pressurizing
            | FlightMode::Hold
            | FlightMode::Venting
            | FlightMode::HardwareArmed
            | FlightMode::Armed => 0.3,
            FlightMode::Ignition => 4.8,
            FlightMode::Burn
            | FlightMode::Coast
            | FlightMode::RecoveryDrogue
            | FlightMode::RecoveryMain => 2.4,
            FlightMode::Landed => 0.6,
        };

        let drawn_ah = self.current * dt / 3600.0;
        self.soc = (self.soc - drawn_ah / CAPACITY_AH).clamp(0.0, 1.0);

        let pack_ocv = cell_ocv(self.soc) * f32::from(CELLS_SERIES);
        let pack_ir = CELL_IR_OHM * f32::from(CELLS_SERIES);
        self.voltage = (pack_ocv - self.current * pack_ir).max(0.0);
    }
}

fn cell_ocv(soc: f32) -> f32 {
    let soc = soc.clamp(0.0, 1.0);
    for w in OCV_CURVE.windows(2) {
        let (s0, v0) = w[0];
        let (s1, v1) = w[1];
        if soc <= s1 {
            let t = (soc - s0) / (s1 - s0);
            return v0 + t * (v1 - v0);
        }
    }
    OCV_CURVE[OCV_CURVE.len() - 1].1
}
