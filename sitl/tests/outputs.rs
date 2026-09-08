#![cfg(not(feature = "hybrid"))]
//! Recovery output safety and correctness:
//! - drogue/main GPIO flags are actually asserted in their respective modes;
//! - Armed state does not auto-advance to Burn without the sim providing
//!   thrust;
//! - neither output is ever high while the vehicle is pre-recovery (Idle,
//!   Armed, Burn, Coast);
//! - the main output fires as a pulse train, with the widths and pulse count
//!   the `REC_MAIN_*` params ask for.

mod common;

use common::{Harness, block_on};
use mission::Params;
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
                    FlightMode::DeployDrogue if h.drogue_active() => {
                        drogue_true_in_drogue = true;
                    }
                    FlightMode::DeployMain if h.main_active() => {
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
            "drogue output was never high while in DeployDrogue"
        );
        assert!(
            main_true_in_main,
            "main output was never high while in DeployMain"
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
            FlightMode::DetectLaunch,
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

        // On every tick before entering DeployDrogue, both outputs must
        // be low. Stop the moment we see DeployDrogue.
        let result = h
            .run_until(MAX_TICKS, |h| {
                if h.mode() < FlightMode::DeployDrogue {
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
                h.mode() >= FlightMode::DeployDrogue
            })
            .await;

        assert!(
            result.is_ok(),
            "vehicle never reached DeployDrogue within {MAX_TICKS} ticks"
        );
    });
}

#[test]
fn main_output_fires_a_two_pulse_train() {
    block_on(async {
        let mut h = Harness::new(None).await;
        h.arm();

        // Sample the main output on every tick from the moment DeployMain is
        // entered, so the samples are a contiguous picture of the train.
        let mut samples: Vec<bool> = Vec::new();

        let result = h
            .run_until(MAX_TICKS, |h| {
                if h.mode() == FlightMode::DeployMain {
                    samples.push(h.main_active());
                }
                h.mode() == FlightMode::Landed
            })
            .await;

        assert!(result.is_ok(), "flight did not reach Landed");

        // Run-length encode into (level, length) so the pulse train is easy to
        // assert on.
        let mut runs: Vec<(bool, u32)> = Vec::new();
        for level in samples {
            match runs.last_mut() {
                Some((last, count)) if *last == level => *count += 1,
                _ => runs.push((level, 1)),
            }
        }

        let params = Params::default().recovery;
        let on = params.main_on_time;
        let gap = params.main_pulse_gap;

        assert_eq!(
            runs.len(),
            4,
            "expected high/low/high/low, got {} runs: {:?}",
            runs.len(),
            runs
        );
        assert_eq!(runs[0], (true, on), "first pulse width, runs: {runs:?}");
        assert_eq!(runs[1], (false, gap), "inter-pulse gap, runs: {runs:?}");
        assert_eq!(runs[2], (true, on), "second pulse width, runs: {runs:?}");
        assert!(
            !runs[3].0,
            "main output did not stay low after the train, runs: {runs:?}"
        );
    });
}

#[test]
fn main_output_train_is_configurable() {
    block_on(async {
        let mut params = Params::default();
        params.recovery.main_on_time = 200;
        params.recovery.main_pulse_gap = 300;
        params.recovery.main_pulses = 3;

        let mut h = Harness::new(Some(params)).await;
        h.arm();

        let mut high_ticks = 0u32;
        let mut edges = 0u32;
        let mut was_high = false;

        let result = h
            .run_until(MAX_TICKS, |h| {
                if h.mode() == FlightMode::DeployMain {
                    let high = h.main_active();
                    if high {
                        high_ticks += 1;
                    }
                    if high && !was_high {
                        edges += 1;
                    }
                    was_high = high;
                }
                h.mode() == FlightMode::Landed
            })
            .await;

        assert!(result.is_ok(), "flight did not reach Landed");
        assert_eq!(edges, 3, "expected 3 pulses");
        assert_eq!(high_ticks, 3 * 200, "total energized time");
    });
}
