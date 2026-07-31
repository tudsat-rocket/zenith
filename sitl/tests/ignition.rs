#![cfg(feature = "hybrid")]
//! The hybrid ignition sequence. The main valve opens `PROP_MAIN_DELAY` into `Ignite`, so between
//! entering the mode and the motor coming up there is a stretch where the igniters are burning and
//! nothing is pushing - the vehicle has to stay on the pad through it.

mod common;

use common::{Harness, block_on};
use rapid_dialect::FlightMode;
use rapid_dialect::rapid::enums::ValveId;

/// Load the oxidizer tank and bring it up to pressure, leaving the vehicle ready for `Ignite`.
async fn ready_to_ignite() -> Harness {
    let mut h = Harness::new(None).await;
    h.fill_pressurant().await;

    h.vehicle.set_mode(FlightMode::FillOxidizer);
    h.run_ticks(60_000).await;

    h.vehicle.set_mode(FlightMode::Pressurize);
    h.run_ticks(15_000).await;

    h
}

fn chamber_pressure(h: &Harness) -> f32 {
    h.sim.lock().unwrap().hybrid.chamber_pressure
}

fn main_valve(h: &Harness) -> f32 {
    h.sim.lock().unwrap().hybrid.valve_state(ValveId::Main)
}

#[test]
fn ignition_lifts_off_and_the_vehicle_stays_put_until_it_does() {
    block_on(async {
        let mut h = ready_to_ignite().await;
        assert!(
            main_valve(&h) == 0.0 && chamber_pressure(&h) < 1.0,
            "expected a quiet pad before ignition (main {:.2}, pc {:.1} bar)",
            main_valve(&h),
            chamber_pressure(&h),
        );

        h.vehicle.set_mode(FlightMode::Ignite);

        // The main valve is still shut for the first PROP_MAIN_DELAY milliseconds. The vehicle
        // must not move, and in particular must not sink through the ground.
        h.run_ticks(500).await;
        assert_eq!(main_valve(&h), 0.0, "main valve opened before its delay");
        assert_eq!(
            h.altitude_agl(),
            0.0,
            "vehicle left the pad without the motor running"
        );

        // Once the valve opens the motor comes up and the vehicle actually flies.
        let lifted = h.run_until(10_000, |h| h.altitude_agl() > 1.0).await;
        assert!(
            lifted.is_ok(),
            "no liftoff (main {:.2}, pc {:.1} bar, alt {:.2} m)",
            main_valve(&h),
            chamber_pressure(&h),
            h.altitude_agl(),
        );

        // And the flight logic follows the acceleration into Burn.
        let burning = h.run_until(5_000, |h| h.mode() == FlightMode::Burn).await;
        assert!(burning.is_ok(), "still in {:?} after liftoff", h.mode());

        // The burn has to sustain. The sim calls burnout on thrust alone, so anything that lets
        // the motor be declared dead before it is up shows here as a very low burnout altitude.
        let burnout = h.run_until(20_000, |h| h.mode() == FlightMode::Coast).await;
        assert!(burnout.is_ok(), "never burned out (mode {:?})", h.mode());
        assert!(
            h.altitude_agl() > 500.0,
            "burnout at only {:.0} m AGL",
            h.altitude_agl(),
        );
    });
}
