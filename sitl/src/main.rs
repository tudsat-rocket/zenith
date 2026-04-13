mod networking;
mod sensors;
mod simulation;

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};

use links::UplinkCommand;
use mission::TelemetryLink;

use networking::Links;
use sensors::{StdOutputs, StdSensors};

pub type Vehicle = mission::Vehicle<StdSensors, StdOutputs, mission::NoStorage>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp_millis()
        .init();

    log::info!("Starting rocket-std");

    let vehicle = Vehicle::new(
        StdSensors::default(),
        StdOutputs::default(),
        mission::NoStorage,
    )
    .await;
    let links = Links::init(spawner);

    spawner.spawn(main_loop(vehicle, links)).unwrap();
}

#[embassy_executor::task]
async fn main_loop(mut vehicle: Vehicle, mut links: Links) -> ! {
    let mut ticker = Ticker::every(Duration::from_micros(1000));
    let mut last_mode = vehicle.mode();

    loop {
        vehicle.tick().await;

        // Notify the simulation of mode changes so it knows when to launch
        let mode = vehicle.mode();
        if mode != last_mode {
            vehicle.sensors.set_flight_mode(mode);
            last_mode = mode;
        }

        if let Some(cmd) = links.try_recv_command() {
            #[allow(clippy::collapsible_match)]
            match cmd {
                UplinkCommand::SetFlightMode(fm) => {
                    vehicle.set_mode(fm);
                }
                _ => {}
            }
        }

        links.send_telemetry_messages(&vehicle);
        ticker.next().await;
    }
}
