#![no_std]
#![no_main]

use rapid_dialect::Rapid;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::select::{Either, select};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::{gpio::Output, interrupt};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Receiver, Sender},
    watch::Watch,
};
use embassy_sync::{channel::Channel, pubsub::PubSubChannel};
use embassy_time::{Duration, Instant, Timer, with_timeout};

use telemetry::config::{DEFAULT_DOWNLINK_CONFIG, DEFAULT_UPLINK_CONFIG};
use telemetry::messages::{DownlinkMessage, SetFlightModeMessage, UplinkMessage};
use telemetry::trx::receiver::HoppingReceiver;
use telemetry::trx::transmitter::HoppingTransmitter;

use firmware::links::UplinkCommand;
use firmware::links::interfaces::InterfaceTxPublisher;
use firmware::links::interfaces::usb::UsbHandle;
use firmware::links::interfaces::{InterfaceCommandSubscriber, ethernet::EthernetHandle};
use firmware::{self as fw, LoraTransceiver};

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MEDIUM: InterruptExecutor = InterruptExecutor::new();

static CONNECTION: Watch<CriticalSectionRawMutex, Option<(Instant, u16)>, 3> = Watch::new();

static DOWNLINK: StaticCell<Channel<CriticalSectionRawMutex, Rapid, 5>> = StaticCell::new();
static UPLINK: StaticCell<Channel<CriticalSectionRawMutex, (u16, UplinkMessage), 5>> =
    StaticCell::new();

#[embassy_executor::main]
async fn main(low_priority_spawner: Spawner) {
    let board = fw::board::init().await;

    // Start high priority executor
    interrupt::I2C3_EV.set_priority(Priority::P6);
    let high_priority_spawner = EXECUTOR_HIGH.start(interrupt::I2C3_EV);

    // Start medium priority executor
    interrupt::I2C3_ER.set_priority(Priority::P7);
    let medium_priority_spawner = EXECUTOR_MEDIUM.start(interrupt::I2C3_ER);

    let can1_rx = fw::can::CAN1_RX_CH.init(PubSubChannel::new());
    let can1_tx = fw::can::CAN1_TX_CH.init(PubSubChannel::new());

    let ethernet = EthernetHandle::init(
        board.ethernet,
        board.seed,
        (can1_tx.publisher().unwrap(), can1_rx.subscriber().unwrap()),
        low_priority_spawner,
    );

    let usb = UsbHandle::init(board.usb, low_priority_spawner);

    let (eth_tx, eth_rx) = ethernet.split();
    let (usb_tx, usb_rx) = usb.split();
    let tx = DOWNLINK.init(Channel::new());
    let rx = UPLINK.init(Channel::new());

    let (led_red, led_yellow, led_green) = board.outputs.leds;

    medium_priority_spawner
        .spawn(split_downlink(tx.receiver(), eth_tx, usb_tx, led_green))
        .unwrap();
    medium_priority_spawner
        .spawn(join_uplink(
            eth_rx,
            usb_rx,
            rx.sender(),
            led_yellow,
            led_red,
        ))
        .unwrap();

    // TODO
    //board.iwdg.unleash();

    let downlink = HoppingReceiver::new(board.lora2, DEFAULT_DOWNLINK_CONFIG, tx.sender());
    high_priority_spawner
        .spawn(run_downlink(downlink, CONNECTION.sender()))
        .unwrap();

    let uplink = HoppingTransmitter::new(board.lora1, DEFAULT_UPLINK_CONFIG, rx.receiver());
    high_priority_spawner
        .spawn(run_uplink(uplink, CONNECTION.receiver().unwrap()))
        .unwrap();
}

#[embassy_executor::task]
async fn run_downlink(
    receiver: HoppingReceiver<
        LoraTransceiver,
        DownlinkMessage,
        Sender<'static, CriticalSectionRawMutex, Rapid, 5>,
    >,
    connection_sender: embassy_sync::watch::Sender<
        'static,
        CriticalSectionRawMutex,
        Option<(Instant, u16)>,
        3,
    >,
) {
    receiver.run_downlink(connection_sender).await;
}

#[embassy_executor::task]
async fn run_uplink(
    transmitter: HoppingTransmitter<
        LoraTransceiver,
        UplinkMessage,
        Receiver<'static, CriticalSectionRawMutex, (u16, UplinkMessage), 5>,
    >,
    connection_receiver: embassy_sync::watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        Option<(Instant, u16)>,
        3,
    >,
) {
    transmitter.run_uplink(connection_receiver).await;
}

#[embassy_executor::task]
async fn split_downlink(
    rx: Receiver<'static, CriticalSectionRawMutex, Rapid, 5>,
    eth_tx: InterfaceTxPublisher,
    usb_tx: InterfaceTxPublisher,
    mut led_activity: Output<'static>,
) -> ! {
    led_activity.set_high();

    loop {
        let msg = rx.receive().await;
        eth_tx.publish_immediate(msg.clone());
        usb_tx.publish_immediate(msg);

        led_activity.set_low();
        Timer::after(Duration::from_millis(2)).await;
        led_activity.set_high();
    }
}

#[embassy_executor::task]
async fn join_uplink(
    mut eth_rx: InterfaceCommandSubscriber,
    mut usb_rx: InterfaceCommandSubscriber,
    tx: Sender<'static, CriticalSectionRawMutex, (u16, UplinkMessage), 5>,
    mut led_activity: Output<'static>,
    mut led_error: Output<'static>,
) -> ! {
    led_activity.set_high();
    led_error.set_low();

    let mut seq: u16 = 0;

    loop {
        let received_command = match with_timeout(
            Duration::from_millis(500),
            select(eth_rx.next_message_pure(), usb_rx.next_message_pure()),
        )
        .await
        {
            Ok(Either::First(cmd) | Either::Second(cmd)) => Some(cmd),
            Err(_timeout) => None,
        };

        let connection = CONNECTION.try_get().flatten();
        led_error.set_level(connection.is_some().into());

        // If we received a command we can send, we do so. If not, we send a heartbeat message,
        // but only if we actually have an active connection.
        let message = match (received_command, connection) {
            (Some(command), _) => match command {
                UplinkCommand::SetFlightMode(fm) => {
                    UplinkMessage::SetFlightMode(SetFlightModeMessage { mode: fm as u8 })
                }
                unsupported => {
                    defmt::warn!(
                        "Unsupported GCS command: {:?}",
                        defmt::Debug2Format(&unsupported)
                    );
                    continue;
                }
            },
            //(None, Some(_)) => UplinkMessage::Heartbeat(()),
            (None, Some(_)) => continue,
            (None, None) => {
                continue;
            }
        };

        seq = seq.wrapping_add(1);

        defmt::info!("Sending {} with seq={}", defmt::Debug2Format(&message), seq);
        tx.send((seq, message)).await;

        led_activity.set_low();
        Timer::after(Duration::from_millis(10)).await;
        led_activity.set_high();
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
