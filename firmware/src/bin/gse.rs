//! Ground support equipment bridge: a CAN probe that forwards CAN1 to MAVLink and back.
//!
//! There is no [`firmware::Vehicle`] here, hence no valve controller and no periodic output
//! refresh. The probe transmits when an operator commands it and at no other time, so a flight
//! computer sharing the bus keeps its setpoints.

#![no_std]
#![no_main]
#![allow(
    clippy::unwrap_used,
    reason = "boot-time peripheral/task init; panic-on-failure is the embedded model"
)]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::select::{Either, select};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Instant, Timer};

use mission::bus::ValveState;
use mission::inventory::{InventoryId, ValveId, ValveMap};
use mission::valves::ValveCommand;

use firmware::can::CanTxPublisher;
use firmware::links::UplinkCommand;
use firmware::links::interfaces::InterfaceCommandSubscriber;
use firmware::links::interfaces::ethernet::{
    CanForwarding, EthernetConfig, EthernetHandle, GSE_SYSTEM_ID,
};

use firmware as fw;

use {defmt_rtt as _, panic_probe as _};

static EXECUTOR_MEDIUM: InterruptExecutor = InterruptExecutor::new();

#[embassy_executor::main]
async fn main(low_priority_spawner: Spawner) {
    let board = fw::board::init().await;

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

    let ethernet = EthernetHandle::init(
        board.ethernet,
        board.seed,
        EthernetConfig {
            system_id: GSE_SYSTEM_ID,
            can: Some(CanForwarding {
                tx: can1_tx.publisher().unwrap(),
                rx: can1_rx.subscriber().unwrap(),
                enabled_at_boot: true,
            }),
        },
        low_priority_spawner,
    );

    let (_eth_tx, eth_commands) = ethernet.split();

    low_priority_spawner
        .spawn(manual_valves(eth_commands, can1_tx.publisher().unwrap()))
        .unwrap();
}

#[embassy_executor::task]
async fn manual_valves(mut commands: InterfaceCommandSubscriber, can_tx: CanTxPublisher) -> ! {
    // A command is written once and never refreshed, so nothing needs remembering except when to
    // close a pulse again.
    let mut pulse_deadlines: ValveMap<Option<Instant>> = ValveMap::splat(None);

    loop {
        let next_deadline = ValveId::ALL
            .into_iter()
            .filter_map(|valve| pulse_deadlines[valve])
            .min();

        if let Either::First(UplinkCommand::CommandValve(valve, cmd)) =
            select(commands.next_message_pure(), wait_until(next_deadline)).await
        {
            pulse_deadlines[valve] = match cmd {
                ValveCommand::PulseOpen(duration) => {
                    let millis = u64::try_from(duration.as_millis()).unwrap_or(u64::MAX);
                    Instant::now().checked_add(Duration::from_millis(millis))
                }
                ValveCommand::Open | ValveCommand::Partial(_) | ValveCommand::Close => None,
            };

            write_valve(&can_tx, valve, ValveState::from(cmd));
        }

        let now = Instant::now();
        for valve in ValveId::ALL {
            if pulse_deadlines[valve].is_some_and(|deadline| deadline <= now) {
                pulse_deadlines[valve] = None;
                write_valve(&can_tx, valve, ValveState::fully_closed());
            }
        }
    }
}

async fn wait_until(deadline: Option<Instant>) {
    match deadline {
        Some(deadline) => Timer::at(deadline).await,
        None => core::future::pending::<()>().await,
    }
}

fn write_valve(can_tx: &CanTxPublisher, valve: ValveId, state: ValveState) {
    defmt::info!(
        "Writing valve {} at {} promille",
        defmt::Debug2Format(&valve),
        state.promille()
    );

    let frame = fw::bus::valve_sdo_frame(valve, state);
    if can_tx.try_publish(frame).is_err() {
        can_tx.publish_immediate(frame);
    }
}

#[interrupt]
unsafe fn I2C3_ER() {
    unsafe { EXECUTOR_MEDIUM.on_interrupt() }
}
