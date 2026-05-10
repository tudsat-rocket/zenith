#![cfg(not(feature = "hybrid"))]
//! Recovery output safety and correctness:
//! - drogue/main GPIO flags are actually asserted in their respective modes;
//! - Armed state does not auto-advance to Burn without the sim providing
//!   thrust;
//! - neither output is ever high while the vehicle is pre-recovery (Idle,
//!   Armed, Burn, Coast).

mod common;

use common::{Harness, block_on};
use rapid_dialect::FlightMode;

const MAX_TICKS: u32 = 400_000;

#[test]
fn recovery_outputs_fire_in_matching_modes() {
    block_on(async {
        let mut h = Harness::new(None).await;
        h.arm();

        let mut drogue_true_in_drogue = false;
        let mut main_true_in_main = false;

        let result = h
            .run_until(MAX_TICKS, |h| {
                match h.mode() {
                    FlightMode::RecoveryDrogue if h.drogue_active() => {
                        drogue_true_in_drogue = true;
                    }
                    FlightMode::RecoveryMain if h.main_active() => {
                        main_true_in_main = true;
                    }
                    _ => {}
                }
                h.mode() == FlightMode::Landed
            })
            .await;

        assert!(result.is_ok(), "flight did not reach Landed");
        assert!(
            drogue_true_in_drogue,
            "drogue output was never high while in RecoveryDrogue"
        );
        assert!(
            main_true_in_main,
            "main output was never high while in RecoveryMain"
        );
    });
}

#[test]
fn armed_does_not_auto_advance_without_thrust() {
    block_on(async {
        // 2s << sim's 5s arm-to-ignition gate, so thrust never happens.
        let mut h = Harness::new(None).await;
        h.arm();
        h.run_ticks(2_000).await;
        assert_eq!(
            h.mode(),
            FlightMode::Armed,
            "vehicle auto-advanced to {:?} without simulated thrust",
            h.mode()
        );
        assert!(!h.drogue_active(), "drogue fired while still in Armed");
        assert!(!h.main_active(), "main fired while still in Armed");
    });
}

#[test]
fn outputs_silent_before_drogue_phase() {
    block_on(async {
        let mut h = Harness::new(None).await;
        h.arm();

        // On every tick before entering RecoveryDrogue, both outputs must
        // be low. Stop the moment we see RecoveryDrogue.
        let result = h
            .run_until(MAX_TICKS, |h| {
                if h.mode() < FlightMode::RecoveryDrogue {
                    assert!(
                        !h.drogue_active(),
                        "drogue fired early in mode {:?} at alt {:.1}m",
                        h.mode(),
                        h.altitude_agl(),
                    );
                    assert!(
                        !h.main_active(),
                        "main fired early in mode {:?} at alt {:.1}m",
                        h.mode(),
                        h.altitude_agl(),
                    );
                }
                h.mode() >= FlightMode::RecoveryDrogue
            })
            .await;

        assert!(
            result.is_ok(),
            "vehicle never reached RecoveryDrogue within {MAX_TICKS} ticks"
        );
    });
}
