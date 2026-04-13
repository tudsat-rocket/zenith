#![no_std]
#![no_main]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::*;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker};

use firmware::Vehicle;
use firmware::links::{Links, UplinkCommand};

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

    let can1_rx = fw::can::CAN1_RX_CH.init(PubSubChannel::new());
    let can1_tx = fw::can::CAN1_TX_CH.init(PubSubChannel::new());
    fw::can::spawn_can1(
        board.can1,
        medium_priority_spawner,
        can1_rx.publisher().unwrap(),
        can1_tx.subscriber().unwrap(),
    )
    .await;

    fw::sensors::power::init(board.adc, low_priority_spawner);
    let vehicle = Vehicle::new(board.sensors, board.outputs, mission::NoStorage).await;

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
            #[allow(clippy::single_match)] // TODO: this will expand
            #[allow(clippy::collapsible_match)] // TODO: this will expand
            match cmd {
                UplinkCommand::SetFlightMode(fm) => {
                    vehicle.set_mode(fm);
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
