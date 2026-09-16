//! Off-nominal scenarios for operator training.
//!
//! Every scenario is opt-in through a cargo feature of the `sitl` crate, e.g.
//! `just sitl-hybrid --features fault-drogue-failure`. Features can be combined. `fault-random`
//! rolls a scenario at startup on top of whatever was selected explicitly, and may well roll a
//! nominal flight.
//!
//! The scenario is fixed for the lifetime of the process, a reset to `Idle` does not reroll it.
#![allow(
    clippy::arithmetic_side_effects,
    reason = "simulator fault timing math"
)]

use rand::Rng;
use rand::seq::SliceRandom;

#[cfg(feature = "hybrid")]
use mission::inventory::{InventoryId, PressSensId, TempSensId};

use super::physics::FlightPhysics;

/// Fraction of the nominal propellant mass flow an underperforming engine achieves. Low enough to
/// be unmistakable, high enough to still trip the default 3 g liftoff detection (`SM_LODEC_ACC`).
pub const UNDERPERFORMING_MASS_FLOW_FACTOR: f32 = 0.55;
/// Probability of a single downlink packet being lost on a bad link.
pub const BAD_DOWNLINK_LOSS_PROBABILITY: f64 = 0.5;
/// Share of `fault-random` rolls that come out as a completely nominal flight.
const RANDOM_NOMINAL_PROBABILITY: f64 = 0.25;
/// How many sensors fail in the sensor dropout scenario.
const SENSOR_DROPOUT_COUNT: std::ops::RangeInclusive<usize> = 1..=3;
/// Window after arming [s] in which the dropouts happen: from the pad through ascent and apogee.
const SENSOR_DROPOUT_WINDOW: std::ops::Range<f32> = 0.0..60.0;

/// A simulated sensor that can drop out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimSensor {
    Imu1,
    Imu2,
    Imu3,
    HighG,
    Mag,
    Baro1,
    Baro2,
    Baro3,
    Gps,
    Power,
    #[cfg(feature = "hybrid")]
    Pressure(PressSensId),
    #[cfg(feature = "hybrid")]
    Temperature(TempSensId),
}

impl SimSensor {
    fn all() -> Vec<Self> {
        #[allow(unused_mut, reason = "only extended on hybrid builds")]
        let mut all = vec![
            Self::Imu1,
            Self::Imu2,
            Self::Imu3,
            Self::HighG,
            Self::Mag,
            Self::Baro1,
            Self::Baro2,
            Self::Baro3,
            Self::Gps,
            Self::Power,
        ];
        #[cfg(feature = "hybrid")]
        {
            all.extend(PressSensId::ALL.map(Self::Pressure));
            all.extend(TempSensId::ALL.map(Self::Temperature));
        }
        all
    }
}

/// A sensor that stops reporting for good at `time_after_arming` [s].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorDropout {
    pub sensor: SimSensor,
    pub time_after_arming: f32,
}

/// The off-nominal behaviour active in this simulation run.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Faults {
    /// Engine only reaches [`UNDERPERFORMING_MASS_FLOW_FACTOR`] of its nominal mass flow.
    pub engine_underperformance: bool,
    /// Sensors that fall out during the flight. Empty for a healthy sensor suite.
    pub sensor_dropouts: Vec<SensorDropout>,
    /// Downlink drops [`BAD_DOWNLINK_LOSS_PROBABILITY`] of all packets.
    pub bad_downlink: bool,
    /// Drogue deployment is commanded, but the hardware never deploys the chute.
    pub drogue_failure: bool,
}

impl Faults {
    /// Everything works.
    pub fn nominal() -> Self {
        Self::default()
    }

    /// The scenario selected through the `fault-*` cargo features.
    pub fn from_build_flags() -> Self {
        let mut rng = rand::thread_rng();

        let mut faults = Self {
            engine_underperformance: cfg!(feature = "fault-engine-underperformance"),
            sensor_dropouts: if cfg!(feature = "fault-sensor-dropout") {
                random_sensor_dropouts(&mut rng)
            } else {
                Vec::new()
            },
            bad_downlink: cfg!(feature = "fault-downlink-loss"),
            drogue_failure: cfg!(feature = "fault-drogue-failure"),
        };

        if cfg!(feature = "fault-random") {
            faults.merge(Self::random(&mut rng));
        }

        faults
    }

    /// A random combination of faults, including none at all.
    pub fn random(rng: &mut impl Rng) -> Self {
        if rng.gen_bool(RANDOM_NOMINAL_PROBABILITY) {
            return Self::nominal();
        }

        // Reroll until at least one fault is active, the nominal case was decided above.
        loop {
            let faults = Self {
                engine_underperformance: rng.r#gen(),
                sensor_dropouts: if rng.r#gen() {
                    random_sensor_dropouts(rng)
                } else {
                    Vec::new()
                },
                bad_downlink: rng.r#gen(),
                drogue_failure: rng.r#gen(),
            };
            if !faults.is_nominal() {
                return faults;
            }
        }
    }

    fn merge(&mut self, other: Self) {
        self.engine_underperformance |= other.engine_underperformance;
        self.bad_downlink |= other.bad_downlink;
        self.drogue_failure |= other.drogue_failure;
        if self.sensor_dropouts.is_empty() {
            self.sensor_dropouts = other.sensor_dropouts;
        }
    }

    pub fn is_nominal(&self) -> bool {
        *self == Self::nominal()
    }

    /// Multiplier on the nominal propellant mass flow.
    pub fn mass_flow_factor(&self) -> f32 {
        if self.engine_underperformance {
            UNDERPERFORMING_MASS_FLOW_FACTOR
        } else {
            1.0
        }
    }

    /// Probability of each downlink packet getting lost.
    pub fn downlink_loss_probability(&self) -> f64 {
        if self.bad_downlink {
            BAD_DOWNLINK_LOSS_PROBABILITY
        } else {
            0.0
        }
    }

    /// Whether `sensor` has dropped out by now.
    pub fn sensor_failed(&self, sensor: SimSensor, physics: &FlightPhysics) -> bool {
        let Some(since_armed) = physics.time_since_armed() else {
            return false;
        };
        self.sensor_dropouts
            .iter()
            .any(|d| d.sensor == sensor && since_armed >= d.time_after_arming)
    }

    pub fn log(&self) {
        if self.is_nominal() {
            log::info!("[SIM] Fault scenario: nominal");
            return;
        }

        log::warn!("[SIM] Fault scenario active:");
        if self.engine_underperformance {
            log::warn!(
                "[SIM]   engine underperformance: {:.0}% of nominal mass flow",
                UNDERPERFORMING_MASS_FLOW_FACTOR * 100.0
            );
        }
        for d in &self.sensor_dropouts {
            log::warn!(
                "[SIM]   sensor dropout: {:?} at {:.1}s after arming",
                d.sensor,
                d.time_after_arming
            );
        }
        if self.bad_downlink {
            log::warn!(
                "[SIM]   bad downlink: {:.0}% packet loss",
                BAD_DOWNLINK_LOSS_PROBABILITY * 100.0
            );
        }
        if self.drogue_failure {
            log::warn!("[SIM]   drogue recovery hardware failure");
        }
    }
}

fn random_sensor_dropouts(rng: &mut impl Rng) -> Vec<SensorDropout> {
    let count = rng.gen_range(SENSOR_DROPOUT_COUNT);
    SimSensor::all()
        .choose_multiple(rng, count)
        .map(|&sensor| SensorDropout {
            sensor,
            time_after_arming: rng.gen_range(SENSOR_DROPOUT_WINDOW),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn random_scenarios_cover_nominal_and_every_fault() {
        let mut rng = rand::thread_rng();
        let rolls: Vec<Faults> = (0..1000).map(|_| Faults::random(&mut rng)).collect();

        assert!(rolls.iter().any(Faults::is_nominal));
        assert!(rolls.iter().any(|f| f.engine_underperformance));
        assert!(rolls.iter().any(|f| !f.sensor_dropouts.is_empty()));
        assert!(rolls.iter().any(|f| f.bad_downlink));
        assert!(rolls.iter().any(|f| f.drogue_failure));
    }

    #[test]
    fn sensor_dropouts_are_distinct_and_in_window() {
        let mut rng = rand::thread_rng();
        for _ in 0..100 {
            let dropouts = random_sensor_dropouts(&mut rng);
            assert!(SENSOR_DROPOUT_COUNT.contains(&dropouts.len()));
            for (i, d) in dropouts.iter().enumerate() {
                assert!(SENSOR_DROPOUT_WINDOW.contains(&d.time_after_arming));
                assert!(dropouts.iter().skip(i + 1).all(|o| o.sensor != d.sensor));
            }
        }
    }
}
