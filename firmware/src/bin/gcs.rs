#![no_std]
#![no_main]
#![allow(
    clippy::unwrap_used,
    reason = "boot-time peripheral/task init; panic-on-failure is the embedded model"
)]

use rapid_dialect::rapid::enums::MavResult;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::select::{Either3, select3};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::{
    gpio::{Input, Output},
    interrupt,
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Receiver, Sender},
    watch::Watch,
};
use embassy_sync::{channel::Channel, pubsub::PubSubChannel};
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};

use telemetry::config::{DEFAULT_DOWNLINK_CONFIG, DEFAULT_UPLINK_CONFIG};
use telemetry::messages::{DownlinkMessage, SetValveMessage, UplinkMessage};
use telemetry::trx::receiver::HoppingReceiver;
use telemetry::trx::transmitter::HoppingTransmitter;

use rapid_dialect::FlightMode;

use links::Downlink;

use firmware::links::UplinkCommand;
use firmware::links::interfaces::InterfaceTxPublisher;
use firmware::links::interfaces::usb::UsbHandle;
use firmware::links::interfaces::{InterfaceCommandSubscriber, ethernet::EthernetHandle};
use firmware::{self as fw, LoraTransceiver};

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MEDIUM: InterruptExecutor = InterruptExecutor::new();

static CONNECTION: Watch<CriticalSectionRawMutex, Option<(Instant, u16)>, 3> = Watch::new();

/// Debounced state of the GSE ignition button: `true` while it is held down. `join_uplink`
/// subscribes with `changed()`, so a press arrives as an event alongside the Ethernet/USB
/// command streams rather than being polled when a command happens to come in.
static IGNITION_BUTTON: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();

static DOWNLINK: StaticCell<Channel<CriticalSectionRawMutex, Downlink, 5>> = StaticCell::new();
static UPLINK: StaticCell<Channel<CriticalSectionRawMutex, (u16, UplinkMessage), 5>> =
    StaticCell::new();

/// One wakeup of the GCS relay loop. Folds the two command streams and the physical ignition
/// button into a single `select`, so the button is handled exactly like a command event.
enum UplinkEvent {
    Eth(UplinkCommand),
    Usb(UplinkCommand),
    IgnitionButton(bool),
    Heartbeat,
}

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

    low_priority_spawner
        .spawn(ignition_button_task(
            board.ignition_button,
            IGNITION_BUTTON.sender(),
        ))
        .unwrap();

    medium_priority_spawner
        .spawn(split_downlink(tx.receiver(), eth_tx, usb_tx, led_green))
        .unwrap();
    medium_priority_spawner
        .spawn(join_uplink(
            eth_rx,
            usb_rx,
            IGNITION_BUTTON.receiver().unwrap(),
            rx.sender(),
            tx.sender(),
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
        Sender<'static, CriticalSectionRawMutex, Downlink, 5>,
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
    rx: Receiver<'static, CriticalSectionRawMutex, Downlink, 5>,
    eth_tx: InterfaceTxPublisher,
    usb_tx: InterfaceTxPublisher,
    mut led_activity: Output<'static>,
) -> ! {
    led_activity.set_high();

    loop {
        let msg = rx.receive().await;
        eth_tx.publish_immediate(msg.clone());
        usb_tx.publish_immediate(msg);

        // One blink per burst, not per message: the receiver blocks on this channel while
        // unpacking, so a slow drain costs it the next packet slot.
        if rx.is_empty() {
            led_activity.set_low();
            Timer::after(Duration::from_millis(2)).await;
            led_activity.set_high();
        }
    }
}

/// Debounce the physical ignition button and publish its state on [`IGNITION_BUTTON`].
///
/// The pin is sampled on a fixed ticker and a new level is only accepted once it has been stable
/// for `DEBOUNCE_SAMPLES` consecutive samples. Writing to the `Watch` then wakes the `changed()`
/// future in [`join_uplink`], giving us an edge-triggered event without a dedicated interrupt.
#[embassy_executor::task]
async fn ignition_button_task(
    button: Input<'static>,
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, bool, 2>,
) -> ! {
    // Stable for 5 samples at 10 ms each = 50 ms of debounce.
    const DEBOUNCE_SAMPLES: u8 = 5;

    let mut ticker = Ticker::every(Duration::from_millis(10));
    let mut stable = button.is_high();
    let mut last = stable;
    let mut count = 0u8;

    sender.send(stable);
    loop {
        ticker.next().await;
        let now = button.is_high();
        if now == last {
            count = count.saturating_add(1);
        } else {
            last = now;
            count = 0;
        }
        if count >= DEBOUNCE_SAMPLES && now != stable {
            stable = now;
            sender.send(stable);
        }
    }
}

#[allow(
    clippy::too_many_lines,
    reason = "single relay state machine; keeping the gating logic in one place is clearer"
)]
#[embassy_executor::task]
async fn join_uplink(
    mut eth_rx: InterfaceCommandSubscriber,
    mut usb_rx: InterfaceCommandSubscriber,
    mut ignition_button_rx: embassy_sync::watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        bool,
        2,
    >,
    tx: Sender<'static, CriticalSectionRawMutex, (u16, UplinkMessage), 5>,
    downlink_tx: Sender<'static, CriticalSectionRawMutex, Downlink, 5>,
    mut led_activity: Output<'static>,
    mut led_error: Output<'static>,
) -> ! {
    led_activity.set_high();
    led_error.set_low();

    let mut seq: u16 = 0;

    // Set once the vehicle has been commanded into `Pressurize`. This is the ignition gate: the
    // physical button may only release an `Ignite` command while it is true. It is cleared again
    // on `Idle`, so an abort cannot be followed by an accidental ignition.
    let mut pressurized = false;

    loop {
        // Wait for any local input: an Ethernet command, a USB command, or an ignition-button
        // edge. The button is a first-class event source here, not a level polled on command.
        let input = match with_timeout(
            Duration::from_millis(500),
            select3(
                eth_rx.next_message_pure(),
                usb_rx.next_message_pure(),
                ignition_button_rx.changed(),
            ),
        )
        .await
        {
            Ok(Either3::First(cmd)) => UplinkEvent::Eth(cmd),
            Ok(Either3::Second(cmd)) => UplinkEvent::Usb(cmd),
            Ok(Either3::Third(pressed)) => UplinkEvent::IgnitionButton(pressed),
            Err(_timeout) => UplinkEvent::Heartbeat,
        };

        let connection = CONNECTION.try_get().flatten();
        led_error.set_level(connection.is_some().into());

        // Only an Eth/Usb command carries a MAVLink command id that needs a terminal ack; the
        // ignition button and heartbeat ticks don't.
        let mut mav_cmd = None;

        // Translate an input into a message for the vehicle. Inputs that must not reach the rocket
        // (a rejected ignition request, a button release, an idle tick) `continue` without sending.
        let message: UplinkMessage = match (input, connection) {
            (UplinkEvent::Eth(command) | UplinkEvent::Usb(command), _) => {
                mav_cmd = command.mav_cmd();
                match command {
                    // Remember that the vehicle is being pressurized, then forward the mode change.
                    // This is what arms the ignition gate below.
                    UplinkCommand::SetFlightMode(fm @ FlightMode::Pressurize) => {
                        pressurized = true;
                        fm.into()
                    }
                    // Ignition may never be requested from the ground software. Swallow it here; the
                    // only path to `Ignite` is the physical button.
                    UplinkCommand::SetFlightMode(FlightMode::Ignite) => {
                        defmt::warn!(
                            "Refusing ground-commanded ignition; use the physical ignition button."
                        );
                        continue;
                    }
                    // Switching to FlightMode::Hold does not change the ignition arming state.
                    UplinkCommand::SetFlightMode(fm @ FlightMode::Hold) => fm.into(),
                    // Any other mode change disarms ignition.
                    UplinkCommand::SetFlightMode(fm) => {
                        pressurized = false;
                        fm.into()
                    }
                    UplinkCommand::CommandValve(valve, cmd) => {
                        UplinkMessage::SetValve(SetValveMessage::new(valve, cmd))
                    }
                    unsupported => {
                        defmt::warn!(
                            "Unsupported GCS command: {:?}",
                            defmt::Debug2Format(&unsupported)
                        );
                        continue;
                    }
                }
            }
            // Physical ignition button: the only way to send `Ignite`, and only while pressurized.
            (UplinkEvent::IgnitionButton(true), _) => {
                if pressurized {
                    defmt::info!("Ignition button pressed while pressurized: sending Ignite.");
                    FlightMode::Ignite.into()
                } else {
                    defmt::warn!("Ignition button pressed but not pressurized; ignoring.");
                    continue;
                }
            }
            (UplinkEvent::IgnitionButton(false), _) | (UplinkEvent::Heartbeat, None) => continue,
            (UplinkEvent::Heartbeat, Some(_)) => UplinkMessage::Heartbeat(()),
        };

        seq = seq.wrapping_add(1);

        defmt::info!("Sending {} with seq={}", defmt::Debug2Format(&message), seq);
        tx.send((seq, message)).await;

        if let Some(command) = mav_cmd {
            let _ = downlink_tx.try_send(Downlink::command_ack(command, MavResult::Accepted));
        }

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
