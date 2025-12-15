use defmt::*;
use embassy_executor::SendSpawner;
use embassy_stm32::can::{Can, CanRx, CanTx, Frame};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};

use static_cell::StaticCell;

pub const CAN_RX_QUEUE_SIZE: usize = 20;
pub const CAN_TX_QUEUE_SIZE: usize = 20;
pub const NUM_CAN_SUBSCRIBERS: usize = 1;
pub const NUM_CAN_PUBLISHERS: usize = 1;

pub type CanRxChannel =
    PubSubChannel<CriticalSectionRawMutex, Frame, CAN_RX_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>;
pub type CanRxSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, Frame, CAN_RX_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>;
pub type CanTxChannel =
    PubSubChannel<CriticalSectionRawMutex, Frame, CAN_TX_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>;
pub type CanTxPublisher =
    Publisher<'static, CriticalSectionRawMutex, Frame, CAN_TX_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>;

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

// --- dedicated tasks for receiving and sending CAN messages for each hardware Bus
async fn run_can_rx(can_rx: &'static mut CanRx<'static>, publisher: CanTxPublisher) -> ! {
    loop {
        match can_rx.read().await {
            Ok(envelope) => {
                debug!("can_rx: received can envelope");
                let frame = envelope.frame;
                publisher.publish_immediate(frame);
            }
            Err(e) => {
                error!(
                    "Can Bus Error: Failed to read can envelope: {:?}",
                    Debug2Format(&e)
                );
            }
        }
    }
}

async fn run_can_tx(can_tx: &'static mut CanTx<'static>, mut subscriber: CanRxSubscriber) -> ! {
    loop {
        let message = subscriber.next_message_pure().await;
        debug!("publishing can message: {}", defmt::Debug2Format(&message));
        can_tx.write(&message).await;
    }
}

// --- CAN1
pub async fn spawn_can1(
    can: Can<'static>,
    spawner: SendSpawner,
    publisher: Publisher<
        'static,
        CriticalSectionRawMutex,
        Frame,
        CAN_RX_QUEUE_SIZE,
        NUM_CAN_SUBSCRIBERS,
        1,
    >,
    subscriber: Subscriber<
        'static,
        CriticalSectionRawMutex,
        Frame,
        CAN_RX_QUEUE_SIZE,
        1,
        NUM_CAN_PUBLISHERS,
    >,
) {
    let (can_tx, can_rx, _properties) = can.split();
    let can_tx = CAN1_TX.init(can_tx);
    let can_rx = CAN1_RX.init(can_rx);

    spawner.spawn(run_can1_tx(can_tx, subscriber)).unwrap();
    spawner.spawn(run_can1_rx(can_rx, publisher)).unwrap();
}

#[embassy_executor::task]
async fn run_can1_tx(can_tx: &'static mut CanTx<'static>, subscriber: CanRxSubscriber) -> ! {
    run_can_tx(can_tx, subscriber).await
}

#[embassy_executor::task]
async fn run_can1_rx(can_rx: &'static mut CanRx<'static>, publisher: CanTxPublisher) -> ! {
    run_can_rx(can_rx, publisher).await
}

// --- CAN2
pub async fn spawn_can2(
    can: Can<'static>,
    spawner: SendSpawner,
    publisher: Publisher<
        'static,
        CriticalSectionRawMutex,
        Frame,
        CAN_RX_QUEUE_SIZE,
        NUM_CAN_SUBSCRIBERS,
        1,
    >,
    subscriber: Subscriber<
        'static,
        CriticalSectionRawMutex,
        Frame,
        CAN_RX_QUEUE_SIZE,
        1,
        NUM_CAN_PUBLISHERS,
    >,
) {
    let (can_tx, can_rx, _properties) = can.split();
    let can_tx = CAN2_TX.init(can_tx);
    let can_rx = CAN2_RX.init(can_rx);

    spawner.spawn(run_can2_tx(can_tx, subscriber)).unwrap();
    spawner.spawn(run_can2_rx(can_rx, publisher)).unwrap();
}

#[embassy_executor::task]
async fn run_can2_tx(can_tx: &'static mut CanTx<'static>, subscriber: CanRxSubscriber) -> ! {
    run_can_tx(can_tx, subscriber).await
}

#[embassy_executor::task]
async fn run_can2_rx(can_rx: &'static mut CanRx<'static>, publisher: CanTxPublisher) -> ! {
    run_can_rx(can_rx, publisher).await
}
