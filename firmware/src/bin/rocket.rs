#![no_std]
#![no_main]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::*;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker};

use firmware::links::{Links, UplinkCommand};
use firmware::vehicle::Vehicle;
use rapid_dialect::rapid::messages::Heartbeat;

use {defmt_rtt as _, panic_probe as _};

use firmware as fw;

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MEDIUM: InterruptExecutor = InterruptExecutor::new();

#[embassy_executor::main]
async fn main(low_priority_spawner: Spawner) {
    let mut board = fw::init_board().await;

    // Start high priority executor
    interrupt::I2C3_EV.set_priority(Priority::P6);
    let high_priority_spawner = EXECUTOR_HIGH.start(interrupt::I2C3_EV);

    // Start medium priority executor
    interrupt::I2C3_ER.set_priority(Priority::P7);
    let medium_priority_spawner = EXECUTOR_MEDIUM.start(interrupt::I2C3_ER);

    //let (lora_downlink, downlink_settings) = fw::lora::start_rocket_downlink(
    //    board.lora1,
    //    settings.lora.clone(),
    //    medium_priority_spawner,
    //);
    //let (lora_uplink, rssi_glob) =
    //    fw::lora::start_rocket_uplink(board.lora2, settings.lora.clone(), medium_priority_spawner);

    let can1_rx = fw::can::CAN1_RX_CH.init(PubSubChannel::new());
    let can1_tx = fw::can::CAN1_TX_CH.init(PubSubChannel::new());
    fw::can::spawn_can1(
        board.can1,
        medium_priority_spawner,
        can1_rx.publisher().unwrap(),
        can1_tx.subscriber().unwrap(),
    )
    .await;

    let vehicle = Vehicle::init(
        board.sensors,
        board.outputs,
        board.adc,
        low_priority_spawner,
    )
    .await;

    let links = Links::init(
        board.ethernet,
        board.seed,
        board.usb,
        (can1_tx.publisher().unwrap(), can1_rx.subscriber().unwrap()),
        low_priority_spawner,
    )
    .await;

    board.iwdg.unleash();

    high_priority_spawner
        .spawn(main_loop(vehicle, links, board.iwdg))
        .unwrap();
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
                    links.send_telemetry_message::<Heartbeat>(&vehicle);
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
