//! Verify settings from `Storage` drive vehicle behavior, and that defaults
//! apply when storage is empty. Uses the main-parachute deploy altitude as
//! a concrete, observable effect.

mod common;

use common::{Harness, block_on};
use mission::{RecoverySettings, Settings};
use rapid_dialect::FlightMode;

const MAX_TICKS: u32 = 400_000;

/// Full flight helper. Arms the vehicle and runs until it enters
/// `RecoveryMain` (or times out). Returns the state estimator's
/// AGL reading at that moment - this is the value the flight
/// logic actually compares against `main_deploy_altitude`.
async fn estimator_altitude_at_main_deploy(h: &mut Harness) -> f32 {
    h.arm();
    let result = h
        .run_until(MAX_TICKS, |h| h.mode() == FlightMode::RecoveryMain)
        .await;
    assert!(
        result.is_ok(),
        "vehicle did not reach RecoveryMain within {MAX_TICKS} ticks (mode={:?}, alt={:.1}m)",
        h.mode(),
        h.altitude_agl(),
    );
    h.vehicle.state_estimator.altitude_agl()
}

#[test]
fn main_deploys_at_configured_altitude() {
    block_on(async {
        let settings = Settings {
            recovery: RecoverySettings {
                main_deploy_altitude: 200.0,
                ..RecoverySettings::default()
            },
            ..Settings::default()
        };
        let mut h = Harness::new(Some(settings)).await;
        let alt = estimator_altitude_at_main_deploy(&mut h).await;
        // Flight logic fires main when estimator altitude falls below the
        // configured threshold; 100ms debounce at drogue rate (~15 m/s)
        // adds up to ~1.5m of undershoot.
        assert!(
            (180.0..=200.0).contains(&alt),
            "main deployed at estimator altitude {alt:.1}m AGL, expected just below 200m",
        );
    });
}

#[test]
fn main_deploys_at_default_altitude_when_not_configured() {
    block_on(async {
        // `None` -> MemoryStorage::read_settings returns None
        // -> Vehicle::new falls back to Settings::default()
        // -> default main_deploy_altitude = 450.0
        let mut h = Harness::new(None).await;
        let alt = estimator_altitude_at_main_deploy(&mut h).await;
        assert!(
            (430.0..=450.0).contains(&alt),
            "main deployed at estimator altitude {alt:.1}m AGL, expected just below default 450m",
        );
    });
}
