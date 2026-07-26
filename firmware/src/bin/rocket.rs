#![no_std]
#![no_main]
#![allow(
    clippy::unwrap_used,
    reason = "boot-time peripheral/task init; panic-on-failure is the embedded model"
)]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::*;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker};

use firmware::bus::BusHandler;
use firmware::can::{CanRxSubscriber, CanTxPublisher};
use firmware::links::{Links, UplinkCommand};
use firmware::{Vehicle, buzzer};

use {defmt_rtt as _, panic_probe as _};

use firmware as fw;

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MEDIUM: InterruptExecutor = InterruptExecutor::new();

#[embassy_executor::main]
async fn main(low_priority_spawner: Spawner) {
    let mut board = fw::board::init().await;

    // Start high priority executor
    interrupt::I2C3_EV.set_priority(Priority::P6);
    let high_priority_spawner = EXECUTOR_HIGH.start(interrupt::I2C3_EV);

    // Start medium priority executor
    interrupt::I2C3_ER.set_priority(Priority::P7);
    let medium_priority_spawner = EXECUTOR_MEDIUM.start(interrupt::I2C3_ER);

    let storage = fw::storage::spawn(board.flash, board.params, &low_priority_spawner);

    fw::sensors::power::spawn(board.adc, low_priority_spawner);
    #[cfg(not(feature = "gcs"))]
    fw::sensors::gps::spawn(board.gps, low_priority_spawner);

    // Spawn bus handling tasks
    let can1_rx = fw::can::CAN1_RX_CH.init(PubSubChannel::new());
    let can1_tx = fw::can::CAN1_TX_CH.init(PubSubChannel::new());

    fw::can::spawn_can1(
        board.can1,
        medium_priority_spawner,
        can1_rx.publisher().unwrap(),
        can1_tx.subscriber().unwrap(),
    )
    .await;

    fw::buzzer::spawn(board.buzzer, low_priority_spawner);

    let can_tx_pub: CanTxPublisher = can1_tx.publisher().unwrap();
    let can_rx_sub: CanRxSubscriber = can1_rx.subscriber().unwrap();
    let bus = BusHandler::new(can_tx_pub, can_rx_sub);

    // Initialize main Vehicle & Linkss structs
    let vehicle = Vehicle::new(board.sensors, board.outputs, storage, bus).await;
    let links = Links::init(
        board.ethernet,
        board.seed,
        board.usb,
        board.lora1,
        board.lora2,
        (can1_tx.publisher().unwrap(), can1_rx.subscriber().unwrap()),
        medium_priority_spawner,
        low_priority_spawner,
    )
    .await;

    // Unleash the watchdog and spawn the main loop.
    board.iwdg.unleash();
    high_priority_spawner
        .spawn(main_loop(vehicle, links, board.iwdg))
        .unwrap();
    buzzer::request_song(buzzer::Song::StartupTech);
}

#[embassy_executor::task]
pub async fn main_loop(
    mut vehicle: Vehicle,
    mut links: Links,
    mut iwdg: IndependentWatchdog<'static, IWDG1>,
) -> ! {
    let mut ticker = Ticker::every(Duration::from_micros(1000));
    loop {
        vehicle.tick().await;

        // TODO: this belongs somewhere else
        if let Some(cmd) = links.try_recv_command() {
            match cmd {
                UplinkCommand::SetFlightMode(fm) => {
                    vehicle.set_mode(fm);
                }
                UplinkCommand::CommandValve(valve_id, valve_cmd) => {
                    if vehicle.try_command_valve(valve_id, valve_cmd).is_err() {
                        defmt::warn!(
                            "CommandValve {} {} rejected",
                            defmt::Debug2Format(&valve_id),
                            defmt::Debug2Format(&valve_cmd)
                        );
                    }
                }
                UplinkCommand::SetParam { id, raw } => {
                    vehicle.set_param(id, raw).await;
                }
                _ => {}
            }
        }

        links.send_telemetry_messages(&vehicle);

        iwdg.pet();
        ticker.next().await;
    }
}

#[interrupt]
unsafe fn I2C3_EV() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

#[interrupt]
unsafe fn I2C3_ER() {
    unsafe { EXECUTOR_MEDIUM.on_interrupt() }
}
