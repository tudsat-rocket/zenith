//! Off-nominal training scenarios: each fault has to show up in the simulation the way the
//! operator would experience it, without breaking the rest of the flight.

mod common;

use common::{Harness, block_on};
use mission::Sensors;
#[cfg(feature = "hybrid")]
use rapid_dialect::FlightMode;
use sitl::Faults;
#[cfg(not(feature = "hybrid"))]
use sitl::simulation::FlightPhase;
use sitl::simulation::faults::{SensorDropout, SimSensor};

#[cfg(not(feature = "hybrid"))]
const MAX_TICKS: u32 = 400_000;

#[cfg(not(feature = "hybrid"))]
fn phase(h: &Harness) -> FlightPhase {
    h.sim.lock().unwrap().physics.phase
}

/// Flies a solid-motor flight to touchdown, returning the apogee [m AGL] and every sim phase seen.
#[cfg(not(feature = "hybrid"))]
async fn fly(faults: Faults) -> (f32, Vec<FlightPhase>, bool, Harness) {
    let mut h = Harness::with_faults(None, faults).await;
    h.arm();

    let mut drogue_fired = false;
    let mut apogee = 0.0_f32;
    let mut phases = vec![phase(&h)];
    let result = h
        .run_until(MAX_TICKS, |h| {
            apogee = apogee.max(h.altitude_agl());
            drogue_fired |= h.drogue_active();
            let p = phase(h);
            if *phases.last().unwrap() != p {
                phases.push(p);
            }
            p == FlightPhase::Landed
        })
        .await;
    assert!(result.is_ok(), "never landed, phases {phases:?}");

    (apogee, phases, drogue_fired, h)
}

#[cfg(not(feature = "hybrid"))]
#[test]
fn failed_drogue_falls_ballistic_until_main() {
    block_on(async {
        let faults = Faults {
            drogue_failure: true,
            ..Faults::nominal()
        };
        let (_, phases, drogue_fired, _) = fly(faults).await;

        assert!(drogue_fired, "firmware never commanded the drogue");
        assert!(
            !phases.contains(&FlightPhase::Drogue),
            "drogue deployed despite the failure: {phases:?}"
        );
        assert_eq!(
            phases,
            [
                FlightPhase::Pad,
                FlightPhase::Burn,
                FlightPhase::Coast,
                FlightPhase::Main,
                FlightPhase::Landed
            ]
        );
    });
}

#[cfg(not(feature = "hybrid"))]
#[test]
fn underperforming_engine_reaches_a_lower_apogee() {
    block_on(async {
        let (nominal, _, _, _) = fly(Faults::nominal()).await;
        let (weak, phases, drogue_fired, _) = fly(Faults {
            engine_underperformance: true,
            ..Faults::nominal()
        })
        .await;

        // Same propellant at a lower flow rate: total impulse is unchanged, the loss comes from
        // the longer burn fighting gravity.
        assert!(
            weak < 0.85 * nominal,
            "apogee {weak:.0} m is not clearly below nominal {nominal:.0} m"
        );
        // Still a flight the firmware recognises and recovers, just a poor one.
        assert!(drogue_fired, "firmware never commanded the drogue");
        assert!(phases.contains(&FlightPhase::Drogue), "phases {phases:?}");
    });
}

#[test]
fn dropped_out_sensor_stops_reporting_and_others_keep_going() {
    block_on(async {
        let faults = Faults {
            sensor_dropouts: vec![
                SensorDropout {
                    sensor: SimSensor::Imu2,
                    time_after_arming: 1.0,
                },
                SensorDropout {
                    sensor: SimSensor::Baro1,
                    time_after_arming: 1.0,
                },
            ],
            ..Faults::nominal()
        };
        let mut h = Harness::with_faults(None, faults).await;
        let mut sensors = sitl::StdSensors::new(std::sync::Arc::clone(&h.sim));

        // Not armed yet: nothing has failed.
        h.run_ticks(2_000).await;
        let r = sensors.tick().await;
        assert!(r.imu2_accel.is_some() && r.baro1.pressure.is_some());

        #[cfg(feature = "hybrid")]
        h.vehicle.set_mode(FlightMode::Ignite);
        #[cfg(not(feature = "hybrid"))]
        h.arm();

        h.run_ticks(500).await;
        let r = sensors.tick().await;
        assert!(r.imu2_accel.is_some() && r.baro1.pressure.is_some());

        h.run_ticks(1_000).await;
        let r = sensors.tick().await;
        assert!(r.imu2_gyro.is_none() && r.imu2_accel.is_none());
        assert!(r.baro1.pressure.is_none() && r.baro1.altitude.is_none());
        assert!(r.imu1_accel.is_some() && r.imu3_accel.is_some());
        assert!(r.baro2.pressure.is_some() && r.baro3.pressure.is_some());
    });
}

#[cfg(feature = "hybrid")]
#[test]
fn dropped_out_bus_pressure_sensor_goes_silent() {
    use mission::bus::Bus;
    use mission::inventory::PressSensId;

    block_on(async {
        let faults = Faults {
            sensor_dropouts: vec![SensorDropout {
                sensor: SimSensor::Pressure(PressSensId::CombustionChamber),
                time_after_arming: 0.0,
            }],
            ..Faults::nominal()
        };
        let mut h = Harness::with_faults(None, faults).await;
        let mut bus = sitl::simulation::hybrid::SitlBus::new(std::sync::Arc::clone(&h.sim));

        let image = bus.get_input_image();
        assert!(image.press_sens[PressSensId::CombustionChamber].is_some());

        h.vehicle.set_mode(FlightMode::Ignite);
        h.run_ticks(10).await;
        let image = bus.get_input_image();
        assert!(image.press_sens[PressSensId::CombustionChamber].is_none());
        assert!(image.press_sens[PressSensId::PressurantTank].is_some());
    });
}

#[cfg(feature = "hybrid")]
#[test]
fn underperforming_hybrid_engine_still_lifts_off_at_reduced_chamber_pressure() {
    /// Fill, pressurize and ignite; returns the peak chamber pressure [bar] of the burn.
    async fn burn(faults: Faults) -> (f32, Harness) {
        let mut h = Harness::with_faults(None, faults).await;
        h.fill_pressurant().await;
        h.vehicle.set_mode(FlightMode::FillOxidizer);
        h.run_ticks(60_000).await;
        h.vehicle.set_mode(FlightMode::Pressurize);
        h.run_ticks(15_000).await;
        h.vehicle.set_mode(FlightMode::Ignite);

        let mut peak = 0.0_f32;
        let burnout = h
            .run_until(60_000, |h| {
                peak = peak.max(h.sim.lock().unwrap().hybrid.chamber_pressure);
                h.mode() == FlightMode::Coast
            })
            .await;
        assert!(burnout.is_ok(), "never reached Coast (mode {:?})", h.mode());
        (peak, h)
    }

    block_on(async {
        let (nominal, _) = burn(Faults::nominal()).await;
        let (weak, h) = burn(Faults {
            engine_underperformance: true,
            ..Faults::nominal()
        })
        .await;

        assert!(
            weak < 0.7 * nominal,
            "peak chamber pressure {weak:.1} bar is not clearly below nominal {nominal:.1} bar"
        );
        assert!(
            h.altitude_agl() > 100.0,
            "burnout at {:.0} m",
            h.altitude_agl()
        );
    });
}
