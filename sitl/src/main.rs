mod networking;

use std::sync::{Arc, Mutex};

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};

use links::UplinkCommand;
use mission::TelemetryLink;

use networking::Links;
use sitl::{RecoveryFlags, SharedSimulation, Simulation, StdOutputs, StdSensors, Vehicle};

#[cfg(feature = "hybrid")]
use sitl::simulation::hybrid::SitlPropulsion;

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
            StdSensors::new(sim.clone()),
            StdOutputs::new(flags),
            mission::NoStorage,
            mission::NoPropulsion,
        )
        .await;
        let links = Links::init(spawner);
        spawner.spawn(main_loop(vehicle, links, sim)).unwrap();
    }

    #[cfg(feature = "hybrid")]
    {
        log::info!("Starting rocket-std (hybrid build)");
        let vehicle = Vehicle::new(
            StdSensors::new(sim.clone()),
            StdOutputs::new(flags),
            mission::NoStorage,
            SitlPropulsion::new(sim.clone()),
        )
        .await;
        let links = Links::init(spawner);
        spawner.spawn(main_loop(vehicle, links, sim)).unwrap();
    }
}

#[embassy_executor::task]
async fn main_loop(mut vehicle: Vehicle, mut links: Links, sim: SharedSimulation) -> ! {
    let mut ticker = Ticker::every(Duration::from_micros(1000));

    loop {
        {
            let mut s = sim.lock().unwrap();
            s.set_flight_mode(vehicle.mode());
            s.tick();
        }

        vehicle.tick().await;

        if let Some(cmd) = links.try_recv_command() {
            match cmd {
                UplinkCommand::SetFlightMode(fm) => {
                    vehicle.set_mode(fm);
                }
                #[cfg(feature = "hybrid")]
                UplinkCommand::CommandValve(valve, valve_cmd) => {
                    if let Err(e) = vehicle.try_command_valve(valve, valve_cmd) {
                        log::warn!("CommandValve {:?} {:?} rejected: {:?}", valve, valve_cmd, e);
                    }
                }
                _ => {}
            }
        }

        links.send_telemetry_messages(&vehicle);
        ticker.next().await;
    }
}
