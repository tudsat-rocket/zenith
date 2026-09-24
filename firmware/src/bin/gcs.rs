#![no_std]
#![no_main]
#![allow(
    clippy::unwrap_used,
    reason = "boot-time peripheral/task init; panic-on-failure is the embedded model"
)]

use rapid_dialect::rapid::enums::{MavCmd, MavResult};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::select::{Either4, select4};
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
use embassy_time::{Duration, Instant, Ticker, Timer, with_deadline};

use telemetry::config::{DEFAULT_DOWNLINK_CONFIG, DEFAULT_UPLINK_CONFIG};
use telemetry::messages::{CommandAck, DownlinkMessage, SetValveMessage, UplinkMessage};
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

static COMMAND_ACK: Watch<CriticalSectionRawMutex, CommandAck, 2> = Watch::new();

const COMMAND_TIMEOUT: Duration = Duration::from_millis(2000);

const UPLINK_HEARTBEAT_INTERVAL: Duration = Duration::from_millis(500);

static DOWNLINK: StaticCell<Channel<CriticalSectionRawMutex, Downlink, 5>> = StaticCell::new();
static UPLINK: StaticCell<Channel<CriticalSectionRawMutex, (u16, UplinkMessage), 5>> =
    StaticCell::new();

/// One wakeup of the GCS relay loop. Folds the two command streams, the physical ignition button
/// and the vehicle's answers into a single `select`, so the button is handled exactly like a
/// command event.
enum UplinkEvent {
    Eth(UplinkCommand),
    Usb(UplinkCommand),
    IgnitionButton(bool),
    Ack(CommandAck),
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
            COMMAND_ACK.receiver().unwrap(),
            led_yellow,
            led_red,
        ))
        .unwrap();

    // TODO
    //board.iwdg.unleash();

    let downlink = HoppingReceiver::new(board.lora2, DEFAULT_DOWNLINK_CONFIG, tx.sender());
    high_priority_spawner
        .spawn(run_downlink(
            downlink,
            CONNECTION.sender(),
            COMMAND_ACK.sender(),
        ))
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
    ack_sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, CommandAck, 2>,
) {
    receiver.run_downlink(connection_sender, ack_sender).await;
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

fn ack_command(
    downlink_tx: &Sender<'static, CriticalSectionRawMutex, Downlink, 5>,
    command: MavCmd,
    result: MavResult,
) {
    defmt::info!(
        "Acking {} with {}",
        defmt::Debug2Format(&command),
        defmt::Debug2Format(&result)
    );

    let _ = downlink_tx.try_send(Downlink::command_ack(command, result));
}

#[allow(
    clippy::too_many_lines,
    reason = "single relay state machine; keeping the gating logic in one place is clearer"
)]
#[embassy_executor::task]
#[allow(
    clippy::arithmetic_side_effects,
    reason = "deadline arithmetic on the monotonic clock"
)]
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
    mut ack_rx: embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, CommandAck, 2>,
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

    let mut outstanding: heapless::Vec<(u16, MavCmd, Instant), 4> = heapless::Vec::new();
    let mut last_ack: Option<CommandAck> = None;
    let mut next_heartbeat = Instant::now();

    loop {
        // Wait for any local input: an Ethernet command, a USB command, an ignition-button edge or
        // the vehicle's answer to an outstanding command.
        let input = match with_deadline(
            next_heartbeat,
            select4(
                eth_rx.next_message_pure(),
                usb_rx.next_message_pure(),
                ignition_button_rx.changed(),
                ack_rx.changed(),
            ),
        )
        .await
        {
            Ok(Either4::First(cmd)) => UplinkEvent::Eth(cmd),
            Ok(Either4::Second(cmd)) => UplinkEvent::Usb(cmd),
            Ok(Either4::Third(pressed)) => UplinkEvent::IgnitionButton(pressed),
            Ok(Either4::Fourth(ack)) => UplinkEvent::Ack(ack),
            Err(_deadline) => UplinkEvent::Heartbeat,
        };

        let connection = CONNECTION.try_get().flatten();
        led_error.set_level(connection.is_some().into());

        // Commands the vehicle never answered are failed locally, so the ground software always
        // gets a terminal result rather than waiting forever.
        let mut expired: heapless::Vec<MavCmd, 4> = heapless::Vec::new();
        outstanding.retain(|(_, command, sent_at)| {
            let timed_out = sent_at.elapsed() > COMMAND_TIMEOUT;
            if timed_out {
                let _ = expired.push(*command);
            }
            !timed_out
        });
        for command in expired {
            ack_command(&downlink_tx, command, MavResult::Failed);
        }

        // Only inputs that carry a MAVLink command id need a terminal ack from the vehicle; a
        // button release and heartbeat tick does not.
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
                    // only path to `Ignite` is the physical button. The rejection is acked so the
                    // operator sees a verdict instead of a command that vanishes.
                    UplinkCommand::SetFlightMode(FlightMode::Ignite) => {
                        defmt::warn!(
                            "Refusing ground-commanded ignition; use the physical ignition button."
                        );
                        if let Some(mav_cmd) = mav_cmd {
                            ack_command(&downlink_tx, mav_cmd, MavResult::Denied);
                        }
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
                    // None of these carry a command id today, so there is nothing to ack; the
                    // rejection is still reported if one ever gains one.
                    unsupported => {
                        defmt::warn!(
                            "Unsupported GCS command: {:?}",
                            defmt::Debug2Format(&unsupported)
                        );
                        if let Some(mav_cmd) = mav_cmd {
                            ack_command(&downlink_tx, mav_cmd, MavResult::Unsupported);
                        }
                        continue;
                    }
                }
            }
            // Physical ignition button: the only way to send `Ignite`, and only while pressurized.
            (UplinkEvent::IgnitionButton(true), _) => {
                if pressurized {
                    defmt::info!("Ignition button pressed while pressurized: sending Ignite.");
                    // Tracked like a ground command, so the vehicle's verdict on the most
                    // important command of the sequence still reaches the ground software.
                    mav_cmd = Some(MavCmd::DoSetMode);
                    FlightMode::Ignite.into()
                } else {
                    defmt::warn!("Ignition button pressed but not pressurized; ignoring.");
                    continue;
                }
            }
            (UplinkEvent::Ack(ack), _) => {
                match last_ack.replace(ack) {
                    // The vehicle's standing answer may predate our boot, so no new command may
                    // alias it.
                    None => seq = ack.seq,
                    Some(last) if last != ack => {
                        if let Some(result) = ack.result
                            && let Some(i) = outstanding
                                .iter()
                                .position(|(seq, _, _)| seq & CommandAck::SEQ_MASK == ack.seq)
                        {
                            let (_, command, _) = outstanding.swap_remove(i);
                            ack_command(&downlink_tx, command, result);
                        }
                    }
                    Some(_) => {}
                }

                continue;
            }
            (UplinkEvent::IgnitionButton(false), _) => continue,
            (UplinkEvent::Heartbeat, Some(_)) => UplinkMessage::Heartbeat(()),
            // Nothing to send without a connection, but the deadline still has to move or
            // `with_deadline` would return immediately from here on.
            (UplinkEvent::Heartbeat, None) => {
                next_heartbeat = Instant::now() + UPLINK_HEARTBEAT_INTERVAL;
                continue;
            }
        };

        seq = seq.wrapping_add(1);

        if let Some(mav_cmd) = mav_cmd {
            if outstanding.is_full() {
                let (_, command, _) = outstanding.swap_remove(0);
                ack_command(&downlink_tx, command, MavResult::Failed);
            }

            let _ = outstanding.push((seq, mav_cmd, Instant::now()));
        }

        defmt::info!("Sending {} with seq={}", defmt::Debug2Format(&message), seq);
        tx.send((seq, message)).await;
        next_heartbeat = Instant::now() + UPLINK_HEARTBEAT_INTERVAL;

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
