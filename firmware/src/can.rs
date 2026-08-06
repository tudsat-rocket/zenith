#![allow(
    clippy::unwrap_used,
    reason = "boot-time CAN init; panic-on-failure is the embedded model"
)]

use defmt::*;
use embassy_executor::SendSpawner;
use embassy_stm32::can::{Can, CanRx, CanTx, Frame};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};

use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

pub const CAN_RX_QUEUE_SIZE: usize = 40;
pub const CAN_TX_QUEUE_SIZE: usize = 40;
pub const NUM_CAN_RX_SUBS: usize = 2;
pub const NUM_CAN_TX_PUBS: usize = 2;

pub type CanRxChannel =
    PubSubChannel<CriticalSectionRawMutex, Frame, CAN_RX_QUEUE_SIZE, NUM_CAN_RX_SUBS, 1>;
pub type CanRxSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, Frame, CAN_RX_QUEUE_SIZE, NUM_CAN_RX_SUBS, 1>;
pub type CanRxPublisher =
    Publisher<'static, CriticalSectionRawMutex, Frame, CAN_RX_QUEUE_SIZE, NUM_CAN_RX_SUBS, 1>;

pub type CanTxChannel =
    PubSubChannel<CriticalSectionRawMutex, Frame, CAN_TX_QUEUE_SIZE, 1, NUM_CAN_TX_PUBS>;
pub type CanTxPublisher =
    Publisher<'static, CriticalSectionRawMutex, Frame, CAN_TX_QUEUE_SIZE, 1, NUM_CAN_TX_PUBS>;
pub type CanTxSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, Frame, CAN_TX_QUEUE_SIZE, 1, NUM_CAN_TX_PUBS>;

// --- can1
pub static CAN1_RX_CH: StaticCell<CanRxChannel> = StaticCell::new();
pub static CAN1_TX_CH: StaticCell<CanTxChannel> = StaticCell::new();

static CAN1_TX: StaticCell<CanTx<'static>> = StaticCell::new();
static CAN1_RX: StaticCell<CanRx<'static>> = StaticCell::new();

// --- can2
pub static CAN2_RX_CH: StaticCell<CanRxChannel> = StaticCell::new();
pub static CAN2_TX_CH: StaticCell<CanTxChannel> = StaticCell::new();

static CAN2_TX: StaticCell<CanTx<'static>> = StaticCell::new();
static CAN2_RX: StaticCell<CanRx<'static>> = StaticCell::new();

async fn run_can_rx(can_rx: &'static mut CanRx<'static>, publisher: CanRxPublisher) -> ! {
    loop {
        match can_rx.read().await {
            Ok(envelope) => {
                let frame = envelope.frame;

                if publisher.try_publish(frame).is_err() {
                    // CAN RX queue full, overwriting oldest frame
                    publisher.publish_immediate(frame);
                }
            }
            Err(_e) => Timer::after(Duration::from_millis(1)).await,
        }
    }
}

async fn run_can_tx(can_tx: &'static mut CanTx<'static>, mut subscriber: CanTxSubscriber) -> ! {
    loop {
        let message = subscriber.next_message_pure().await;
        can_tx.write(&message).await;
    }
}

// --- CAN1
pub async fn spawn_can1(
    can: Can<'static>,
    spawner: SendSpawner,
    rx_publisher: CanRxPublisher,
    tx_subscriber: CanTxSubscriber,
) {
    let (can_tx, can_rx, _properties) = can.split();
    let can_tx = CAN1_TX.init(can_tx);
    let can_rx = CAN1_RX.init(can_rx);

    spawner.spawn(run_can1_tx(can_tx, tx_subscriber)).unwrap();
    spawner.spawn(run_can1_rx(can_rx, rx_publisher)).unwrap();
}

#[embassy_executor::task]
async fn run_can1_tx(can_tx: &'static mut CanTx<'static>, subscriber: CanTxSubscriber) -> ! {
    run_can_tx(can_tx, subscriber).await
}

#[embassy_executor::task]
async fn run_can1_rx(can_rx: &'static mut CanRx<'static>, publisher: CanRxPublisher) -> ! {
    run_can_rx(can_rx, publisher).await
}

// --- CAN2
pub async fn spawn_can2(
    can: Can<'static>,
    spawner: SendSpawner,
    rx_publisher: CanRxPublisher,
    tx_subscriber: CanTxSubscriber,
) {
    let (can_tx, can_rx, _properties) = can.split();
    let can_tx = CAN2_TX.init(can_tx);
    let can_rx = CAN2_RX.init(can_rx);

    spawner.spawn(run_can2_tx(can_tx, tx_subscriber)).unwrap();
    spawner.spawn(run_can2_rx(can_rx, rx_publisher)).unwrap();
}

#[embassy_executor::task]
async fn run_can2_tx(can_tx: &'static mut CanTx<'static>, subscriber: CanTxSubscriber) -> ! {
    run_can_tx(can_tx, subscriber).await
}

#[embassy_executor::task]
async fn run_can2_rx(can_rx: &'static mut CanRx<'static>, publisher: CanRxPublisher) -> ! {
    run_can_rx(can_rx, publisher).await
}
