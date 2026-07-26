//! Verify params from `Storage` drive vehicle behavior, and that defaults
//! apply when storage is empty. Uses the main-parachute deploy altitude as
//! a concrete, observable effect.

#![cfg(not(feature = "hybrid"))]

use common::{Harness, block_on};
use mission::{Params, RecoveryParams};
use rapid_dialect::rapid::enums::MavProtocolCapability;
use rapid_dialect::{FlightMode, Rapid};

mod common;

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
        let params = Params {
            recovery: RecoveryParams {
                main_deploy_altitude: 200.0,
                ..RecoveryParams::default()
            },
            ..Params::default()
        };
        let mut h = Harness::new(Some(params)).await;
        let alt = estimator_altitude_at_main_deploy(&mut h).await;
        // Flight logic fires main when estimator altitude falls below the
        // configured threshold; 100ms debounce at drogue rate (~40 m/s)
        // adds up to ~4m of undershoot.
        assert!(
            (180.0..=200.0).contains(&alt),
            "main deployed at estimator altitude {alt:.1}m AGL, expected just below 200m",
        );
    });
}

#[test]
fn main_deploys_at_default_altitude_when_not_configured() {
    block_on(async {
        // `None` -> MemoryStorage::read_params returns None
        // -> Vehicle::new falls back to Params::default()
        // -> default main_deploy_altitude = 400.0
        let mut h = Harness::new(None).await;
        let alt = estimator_altitude_at_main_deploy(&mut h).await;
        assert!(
            (380.0..=400.0).contains(&alt),
            "main deployed at estimator altitude {alt:.1}m AGL, expected just below default 400m",
        );
    });
}

#[test]
fn set_param_hot_applies_and_persists() {
    const RC_MAIN_ALT: u16 = 0x0200;

    block_on(async {
        // Boot on defaults (main = 400m), then reconfigure RC_MAIN_ALT to 200m via the parameter
        // path before flight.
        let mut h = Harness::new(None).await;
        h.vehicle.set_param(RC_MAIN_ALT, 200.0f32.to_bits()).await;

        // The write reached storage.
        let stored = h
            .vehicle
            .storage
            .stored()
            .expect("set_param should have persisted a Params");

        // Exact comparison is the point: the value round-trips as the f32 bits that were sent.
        #[allow(clippy::float_cmp, reason = "asserting a bit-exact round-trip")]
        {
            assert_eq!(stored.recovery.main_deploy_altitude, 200.0);
        }

        // And the live vehicle now deploys main at the new altitude.
        let alt = estimator_altitude_at_main_deploy(&mut h).await;
        assert!(
            (180.0..=200.0).contains(&alt),
            "after set_param, main deployed at {alt:.1}m AGL, expected just below 200m",
        );
    });
}

#[test]
fn advertises_bytewise_param_encoding() {
    // Ground stations only enable their parameter UI once AUTOPILOT_VERSION tells them how
    // param_value is encoded, so losing this flag silently disables params on the GCS side.
    block_on(async {
        let mut h = Harness::new(None).await;
        let messages = h.collect_telemetry(1_001).await;

        let capabilities = messages
            .iter()
            .find_map(|m| match m {
                Rapid::AutopilotVersion(av) => Some(av.capabilities),
                _ => None,
            })
            .expect("AUTOPILOT_VERSION should be sent periodically");

        assert!(
            capabilities.contains(MavProtocolCapability::PARAM_ENCODE_BYTEWISE),
            "expected PARAM_ENCODE_BYTEWISE, got {capabilities:?}",
        );
        assert!(
            !capabilities.contains(MavProtocolCapability::PARAM_ENCODE_C_CAST),
            "we encode param values bytewise, so the C-cast flag must stay clear",
        );
    });
}
