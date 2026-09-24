//! Oxidizer fill level from the row of temperature probes up the tank wall.
//!
//! The wall is colder where it is wetted. Each probe's excess over the coldest probe plus
//! `deadband`, as a fraction of the spread between the coldest and the warmest less `deadband` but
//! of at least `min_scale`, is summed from the bottom up. The level is the height at which that
//! running sum reaches `threshold` of the largest value it could take. The probes are not
//! calibrated against each other.
//!
//! The tank reads empty while the ox tank pressure is below [`MIN_PRESSURE`]. A column spreading no
//! more than `deadband` reads full while the tank is pressurised, as a tank filled past the topmost
//! probe does, and unknown while no pressure is reported.

use core::num::Wrapping;

use crate::bus::{BusInputImage, DataWithTime};
use crate::inventory::{OxProbeMap, PressSensId};
use crate::params::TankLevelParams;

/// Bar, the higher of the two ox tank sensors counting. Ignored while neither reports.
const MIN_PRESSURE: f32 = 5.0;
/// Milliseconds after which a reading is ignored.
const STALE: u32 = 4000;

#[derive(Debug, Clone)]
pub struct TankLevelEstimator {
    params: TankLevelParams,
    level: Option<f32>,
}

impl TankLevelEstimator {
    pub const fn new(params: TankLevelParams) -> Self {
        Self {
            params,
            level: None,
        }
    }

    pub fn params(&self) -> &TankLevelParams {
        &self.params
    }

    pub fn update_params(&mut self, params: TankLevelParams) {
        self.params = params;
    }

    /// Fill level, 0 - 1, or `None` while it cannot be told.
    pub fn update(&mut self, time: Wrapping<u32>, inputs: &BusInputImage) -> Option<f32> {
        self.level = self.instant(time, inputs).map(|instant| {
            #[allow(
                clippy::cast_precision_loss,
                reason = "a time constant beyond f32's integer range is not a real setting"
            )]
            // `tau` is in milliseconds and the main loop ticks once a millisecond.
            let alpha = 1.0 / self.params.tau.max(1) as f32;
            self.level
                .map_or(instant, |previous| previous + (instant - previous) * alpha)
        });
        self.level
    }

    /// The unfiltered level.
    fn instant(&self, time: Wrapping<u32>, inputs: &BusInputImage) -> Option<f32> {
        let recent = |d: &Option<DataWithTime<f32>>| {
            d.filter(|d| (time - d.time).0 <= STALE).map(|d| d.data)
        };

        let probes: OxProbeMap<Option<f32>> =
            OxProbeMap::from_fn(|id| recent(&inputs.ox_probes[id]));
        let readings = || probes.iter().filter_map(|(id, t)| t.map(|t| (id, t)));

        let (count, coldest, warmest) = readings().fold(
            (0.0_f32, f32::MAX, f32::MIN),
            |(count, coldest, warmest), (_, t)| (count + 1.0, coldest.min(t), warmest.max(t)),
        );
        if count < 2.0 {
            return None;
        }
        let pressure = [PressSensId::OxTankUpper, PressSensId::OxTankLower]
            .into_iter()
            .filter_map(|id| recent(&inputs.press_sens[id]))
            .reduce(f32::max);
        if pressure.is_some_and(|bar| bar < MIN_PRESSURE) {
            return Some(0.0);
        }
        let spread = warmest - coldest;
        if spread <= self.params.deadband {
            return pressure.map(|_| 1.0);
        }

        let base = coldest + self.params.deadband;
        let scale = (spread - self.params.deadband).max(self.params.min_scale);
        let target = self.params.threshold * (count - 1.0);
        let (mut below_height, mut below_sum, mut sum) = (0.0, 0.0, 0.0);
        for (id, t) in readings() {
            sum += (t - base).max(0.0) / scale;
            if sum >= target {
                let step = sum - below_sum;
                let fraction = if step > 0.0 {
                    (target - below_sum) / step
                } else {
                    0.0
                };
                return Some(below_height + fraction * (id.height() - below_height));
            }
            below_height = id.height();
            below_sum = sum;
        }
        Some(1.0)
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "the gated cases return a literal 0.0, not the result of arithmetic"
)]
mod tests {
    use super::*;
    use crate::inventory::{InventoryId, OxProbeId};

    use PressSensId::{OxTankLower, OxTankUpper};

    const NOW: Wrapping<u32> = Wrapping(100_000);

    /// Recorded: a tank part way through being drained.
    const DRAINED: [f32; OxProbeId::COUNT] = [
        9.47, 9.95, 10.52, 10.60, 9.79, 10.92, 12.04, 14.21, 14.78, 17.91,
    ];

    /// The probe row as given, and no pressure sensors reporting.
    fn row(celsius: [f32; OxProbeId::COUNT]) -> BusInputImage {
        let mut inputs = BusInputImage::default();
        inputs.ox_probes =
            OxProbeMap::from_fn(|id| celsius.get(id.idx()).map(|c| DataWithTime::new(*c, NOW)));
        inputs
    }

    /// One update from a fresh estimator returns the unfiltered value.
    fn level_at(
        time: Wrapping<u32>,
        inputs: &BusInputImage,
        params: TankLevelParams,
    ) -> Option<f32> {
        TankLevelEstimator::new(params).update(time, inputs)
    }

    fn level_of(celsius: [f32; OxProbeId::COUNT]) -> f32 {
        level_at(NOW, &row(celsius), TankLevelParams::default()).unwrap()
    }

    /// Recorded: a tank at ambient temperature.
    const AMBIENT: [f32; OxProbeId::COUNT] = [
        33.10, 33.42, 33.83, 34.23, 33.42, 34.71, 34.39, 34.55, 33.99, 35.19,
    ];

    fn with_pressure(mut inputs: BusInputImage, bar: f32) -> BusInputImage {
        inputs.press_sens[OxTankUpper] = Some(DataWithTime::new(bar, NOW));
        inputs.press_sens[OxTankLower] = Some(DataWithTime::new(bar, NOW));
        inputs
    }

    #[test]
    fn a_part_drained_tank_reads_in_between() {
        let level = level_of(DRAINED);
        assert!((level - 0.625).abs() < 0.01, "{level}");
    }

    /// Scaled by the second-warmest probe this reads empty, and scaled by its own spread without
    /// `min_scale`, around half full.
    #[test]
    fn a_column_warm_only_at_the_top_reads_nearly_full() {
        let level = level_of([19.7, 19.9, 20.3, 20.6, 19.7, 20.8, 20.3, 20.1, 19.6, 23.5]);
        assert!(level > 0.8, "{level}");
    }

    #[test]
    fn a_larger_threshold_reads_higher() {
        let low = level_of(DRAINED);
        let params = TankLevelParams {
            threshold: TankLevelParams::default().threshold * 1.5,
            ..TankLevelParams::default()
        };
        let high = level_at(NOW, &row(DRAINED), params).unwrap();
        assert!(high > low + 0.02, "{low} -> {high}");
    }

    #[test]
    fn a_missing_probe_is_skipped() {
        let mut inputs = row(DRAINED);
        inputs.ox_probes[OxProbeId::Probe4] = None;
        let level = level_at(NOW, &inputs, TankLevelParams::default()).unwrap();
        assert!((level - 0.612).abs() < 0.01, "{level}");
    }

    #[test]
    fn one_probe_is_not_a_row() {
        let mut inputs = BusInputImage::default();
        inputs.ox_probes[OxProbeId::Probe0] = Some(DataWithTime::new(10.0, NOW));
        assert_eq!(level_at(NOW, &inputs, TankLevelParams::default()), None);
    }

    #[test]
    fn a_depressurised_tank_reads_empty() {
        let mut inputs = row(DRAINED);
        inputs.press_sens[OxTankUpper] = Some(DataWithTime::new(1.0, NOW));
        inputs.press_sens[OxTankLower] = Some(DataWithTime::new(1.2, NOW));
        assert_eq!(
            level_at(NOW, &inputs, TankLevelParams::default()),
            Some(0.0)
        );
    }

    #[test]
    fn one_pressurised_sensor_is_enough() {
        let mut inputs = row(DRAINED);
        inputs.press_sens[OxTankUpper] = Some(DataWithTime::new(1.0, NOW));
        inputs.press_sens[OxTankLower] = Some(DataWithTime::new(30.0, NOW));
        let level = level_at(NOW, &inputs, TankLevelParams::default()).unwrap();
        assert!((level - 0.625).abs() < 0.01, "{level}");
    }

    #[test]
    fn stale_readings_are_ignored() {
        let late = NOW + Wrapping(STALE) + Wrapping(1);
        assert_eq!(
            level_at(late, &row(DRAINED), TankLevelParams::default()),
            None
        );
    }

    #[test]
    fn a_tank_filled_past_the_top_probe_reads_full() {
        let inputs = with_pressure(row([-5.0; OxProbeId::COUNT]), 30.0);
        assert_eq!(
            level_at(NOW, &inputs, TankLevelParams::default()),
            Some(1.0)
        );
    }

    #[test]
    fn a_flat_column_without_pressure_is_unknown() {
        assert_eq!(
            level_at(
                NOW,
                &row([20.0; OxProbeId::COUNT]),
                TankLevelParams::default()
            ),
            None
        );
    }

    #[test]
    fn a_pressurised_column_at_ambient_reads_nearly_full() {
        let inputs = with_pressure(row(AMBIENT), 30.0);
        let level = level_at(NOW, &inputs, TankLevelParams::default()).unwrap();
        assert!(level > 0.8, "{level}");
    }

    #[test]
    fn the_low_pass_converges_over_tau() {
        let params = TankLevelParams::default();
        let ticks = params.tau;
        let mut estimator = TankLevelEstimator::new(params);
        let drained = row(DRAINED);

        assert_eq!(
            estimator.update(NOW, &with_pressure(row(DRAINED), 1.0)),
            Some(0.0)
        );
        let mut level = 0.0;
        for _ in 0..ticks {
            level = estimator.update(NOW, &drained).unwrap();
        }
        // One time constant of a step from 0 to 0.625.
        assert!((level - 0.625 * 0.632).abs() < 0.01, "{level}");
    }
}
