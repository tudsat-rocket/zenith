#![cfg(feature = "hybrid")]
//! The oxidizer level the vehicle derives from the tank wall probe row, against the fill level the
//! simulation actually has. Nothing on the path is calibrated: the simulated probes carry the same
//! per-probe disagreement as the real row.

mod common;

use common::{Harness, block_on};
use mission::TankId;
use mission::valves::ValveCommand;
use rapid_dialect::rapid::enums::ValveId;
use rapid_dialect::{FlightMode, Rapid};

const TOLERANCE: f32 = 0.15;

async fn harness() -> Harness {
    let mut h = Harness::new(None).await;
    h.fill_pressurant().await;
    h
}

fn actual(h: &Harness) -> f32 {
    h.sim.lock().unwrap().hybrid.tank_level(TankId::Oxidizer)
}

fn derived(h: &Harness) -> Option<f32> {
    h.vehicle.bus_inputs.ox_tank_level.map(|d| d.data)
}

/// Fill until the tank holds `target`, then let the low-pass catch up. Stays in
/// `FillOxidizer`: returning to `Idle` rebuilds the whole propulsion simulation and empties the
/// tank again.
async fn fill_to(h: &mut Harness, target: f32) {
    h.vehicle.set_mode(FlightMode::FillOxidizer);
    let filled = h.run_until(120_000, |h| actual(h) >= target).await;
    assert!(filled.is_ok(), "the sim only reached {:.2}", actual(h));
    h.run_ticks(4_000).await;
}

#[test]
fn the_derived_level_follows_the_fill() {
    block_on(async {
        let mut h = harness().await;
        assert_eq!(derived(&h), Some(0.0), "an empty tank should read empty");

        h.vehicle.set_mode(FlightMode::FillOxidizer);

        let mut checked = 0;
        let mut previous = 0.0;
        for _ in 0..30 {
            h.run_ticks(2_000).await;
            let (actual, derived) = (actual(&h), derived(&h).expect("probes went quiet"));
            let moved = actual - previous;
            previous = actual;
            // Early on the tank takes on propellant faster than the low-pass follows, and the
            // comparison would be measuring the filter.
            if actual < 0.6 || moved > 0.02 {
                continue;
            }
            assert!(
                (derived - actual).abs() < TOLERANCE,
                "derived {derived:.2} against an actual {actual:.2}"
            );
            checked += 1;
        }
        assert!(
            checked > 5,
            "the sim never filled the tank, so this proves nothing"
        );
    });
}

/// Returning to `Idle` rebuilds the simulation, emptying the tank and resetting its clock.
#[test]
fn the_level_survives_a_reset() {
    block_on(async {
        let mut h = harness().await;
        fill_to(&mut h, 0.6).await;
        assert!(derived(&h).is_some_and(|l| l > 0.3), "{:?}", derived(&h));

        h.vehicle.set_mode(FlightMode::Idle);
        let emptied = h
            .run_until(30_000, |h| derived(h).is_some_and(|l| l < 0.01))
            .await;
        assert!(emptied.is_ok(), "{:?}", derived(&h));
    });
}

/// Venting in between lets the fill carry on until the topmost probe is wetted.
#[test]
fn a_tank_filled_past_the_top_probe_reads_full() {
    block_on(async {
        let mut h = harness().await;
        h.vehicle.set_mode(FlightMode::FillOxidizer);
        for _ in 0..12 {
            h.run_ticks(10_000).await;
            let _ = h
                .vehicle
                .try_command_valve(ValveId::OxidizerVent, ValveCommand::Open);
            h.run_ticks(3_000).await;
            let _ = h
                .vehicle
                .try_command_valve(ValveId::OxidizerVent, ValveCommand::Close);
        }
        h.run_ticks(10_000).await;

        assert!(actual(&h) > 0.97, "the sim only reached {:.2}", actual(&h));
        let derived = derived(&h).expect("probes went quiet");
        assert!(derived > 0.95, "{derived}");
    });
}

#[test]
fn the_level_and_the_raw_row_both_reach_the_ground_station() {
    block_on(async {
        let mut h = harness().await;
        // High enough that the level no longer moves between the last message and the assertion.
        fill_to(&mut h, 0.8).await;

        let messages = h.collect_telemetry(1000).await;
        let expected = derived(&h).expect("probes went quiet");

        let level = messages
            .iter()
            .rev()
            .find_map(|m| match m {
                Rapid::PressureVessel(p) if p.id == TankId::Oxidizer as u8 => Some(p.level),
                _ => None,
            })
            .expect("no PRESSURE_VESSEL for the oxidizer tank");
        assert_ne!(level, u16::MAX, "the oxidizer tank reported no level");

        let level = f32::from(level) / 10000.0;
        assert!(
            (level - expected).abs() < 0.01,
            "downlinked {level:.3}, vehicle has {expected:.3}"
        );

        let probes = messages
            .iter()
            .find_map(|m| match m {
                Rapid::DebugFloatArray(d) if &d.name == b"TLI\0\0\0\0\0\0\0" => Some(d),
                _ => None,
            })
            .expect("no TLI debug array");
        assert!(
            probes.data[9] > probes.data[0],
            "a part-filled tank should be warmer at the top: {:?}",
            &probes.data[..10]
        );
    });
}
