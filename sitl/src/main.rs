mod networking;

use std::sync::{Arc, Mutex};

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};

use links::UplinkCommand;
use mission::TelemetryLink;

use networking::Links;
use sitl::{RecoveryFlags, SharedSimulation, Simulation, StdOutputs, StdSensors, Vehicle};

#[cfg(feature = "hybrid")]
use sitl::simulation::hybrid::SitlBus;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp_millis()
        .init();

    let flags = RecoveryFlags::default();
    let sim: SharedSimulation = Arc::new(Mutex::new(Simulation::new(flags.clone())));

    #[cfg(not(feature = "hybrid"))]
    {
        log::info!("Starting rocket-std (solid build)");
        let vehicle = Vehicle::new(
            StdSensors::new(Arc::clone(&sim)),
            StdOutputs::new(flags),
            mission::NoStorage,
            mission::bus::NoBus,
        )
        .await;
        let links = Links::init(spawner);
        #[allow(
            clippy::unwrap_used,
            reason = "task spawn at sim startup; failure just aborts the sim"
        )]
        spawner.spawn(main_loop(vehicle, links, sim)).unwrap();
    }

    #[cfg(feature = "hybrid")]
    {
        log::info!("Starting rocket-std (hybrid build)");
        let vehicle = Vehicle::new(
            StdSensors::new(Arc::clone(&sim)),
            StdOutputs::new(flags),
            mission::NoStorage,
            SitlBus::new(Arc::clone(&sim)),
        )
        .await;
        let links = Links::init(spawner);
        #[allow(
            clippy::unwrap_used,
            reason = "task spawn at sim startup; failure just aborts the sim"
        )]
        spawner.spawn(main_loop(vehicle, links, sim)).unwrap();
    }
}

#[embassy_executor::task]
async fn main_loop(mut vehicle: Vehicle, mut links: Links, sim: SharedSimulation) -> ! {
    let mut ticker = Ticker::every(Duration::from_micros(1000));

    loop {
        {
            #[allow(
                clippy::unwrap_used,
                reason = "sim mutex; a poisoned lock means the sim already panicked"
            )]
            let mut s = sim.lock().unwrap();
            s.set_flight_mode(vehicle.mode());
            s.tick();
        }

        vehicle.tick().await;

        if let Some(cmd) = links.try_recv_command() {
            // Collapses to a single arm without the `hybrid` feature; kept as a match since
            // CommandValve is a real arm with it.
            #[cfg_attr(not(feature = "hybrid"), allow(clippy::collapsible_match))]
            match cmd {
                UplinkCommand::SetFlightMode(fm) => {
                    vehicle.set_mode(fm);
                }
                #[cfg(feature = "hybrid")]
                UplinkCommand::CommandValve(valve, valve_cmd) => {
                    if let Err(_e) = vehicle.try_command_valve(valve, valve_cmd) {
                        log::warn!("CommandValve {valve:?} {valve_cmd:?} rejected.");
                    }
                }
                _ => {}
            }
        }

        links.send_telemetry_messages(&vehicle);
        ticker.next().await;
    }
}
