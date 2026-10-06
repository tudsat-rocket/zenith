#![allow(
    clippy::unwrap_used,
    reason = "boot-time CAN init; panic-on-failure is the embedded model"
)]

use core::sync::atomic::{AtomicU8, AtomicU16, AtomicU32, Ordering};

use defmt::*;
use embassy_executor::SendSpawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::can::enums::{BusError, BusErrorMode};
use embassy_stm32::can::{Can, CanRx, CanTx, Frame, Properties};
use embassy_stm32::pac::can::Fdcan;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber, WaitResult};

use embassy_time::{Duration, Timer};
use mission::bus::{CanBusState, CanHealth};
use static_cell::StaticCell;

use crate::board;

pub const CAN_RX_QUEUE_SIZE: usize = 40;
pub const CAN_TX_QUEUE_SIZE: usize = 40;
pub const NUM_CAN_RX_SUBS: usize = 2;
pub const NUM_CAN_TX_PUBS: usize = 2;

/// While the controller reports an error state, `CanRx::read` returns at once instead of waiting
/// for a frame, so we poll. The RX FIFO holds 3 frames, and three back-to-back minimum-length
/// frames take ~300 us at 500 kbit/s, so this must stay below that.
const ERROR_POLL_INTERVAL: Duration = Duration::from_micros(250);

/// The driver never wakes a pending read on an error interrupt, so a bus that goes quiet (or goes
/// bus-off while we are only transmitting) would otherwise never get its state refreshed.
const STATE_SAMPLE_INTERVAL: Duration = Duration::from_millis(100);

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

/// Written by the RX/TX tasks and `BusHandler`, read once per tick by the main loop.
pub struct CanHealthMonitor {
    state: AtomicU8,
    errors: AtomicU16,
    rx_missed: AtomicU32,
    tx_missed: AtomicU32,
}

impl CanHealthMonitor {
    const fn new() -> Self {
        Self {
            state: AtomicU8::new(CanBusState::Unknown as u8),
            errors: AtomicU16::new(0),
            rx_missed: AtomicU32::new(0),
            tx_missed: AtomicU32::new(0),
        }
    }

    /// Received frames that never reached `BusHandler`, wrapping. A lower bound: the hardware
    /// only flags that its RX FIFO overflowed, not by how much, so each overflow counts as one.
    fn rx_missed(&self) -> u32 {
        self.rx_missed.load(Ordering::Relaxed)
    }

    /// Frames handed to the TX queue that never reached the controller, wrapping.
    fn tx_missed(&self) -> u32 {
        self.tx_missed.load(Ordering::Relaxed)
    }

    pub fn count_rx_missed(&self, frames: u64) {
        self.rx_missed
            .fetch_add(frames.try_into().unwrap_or(u32::MAX), Ordering::Relaxed);
    }

    fn count_tx_missed(&self, frames: u64) {
        self.tx_missed
            .fetch_add(frames.try_into().unwrap_or(u32::MAX), Ordering::Relaxed);
    }

    /// Embassy neither reads nor clears the RX FIFO message-lost flags, so we do both here.
    fn sample_rx_overflow(&self, regs: Fdcan) {
        let ir = regs.ir().read();

        for fifo in 0..2 {
            if ir.rfl(fifo) {
                // Write-one-to-clear, so this leaves the bits the interrupt handler owns alone.
                regs.ir().write(|w| w.set_rfl(fifo, true));
                self.count_rx_missed(1);
            }
        }
    }

    #[allow(
        clippy::cast_possible_truncation,
        reason = "the counters wrap, so their low bits are all the ground needs"
    )]
    pub fn snapshot(&self) -> CanHealth {
        let missed = self.rx_missed().wrapping_add(self.tx_missed()) as u16;

        CanHealth {
            state: CanBusState::from_bits(self.state.load(Ordering::Relaxed)),
            errors: self.errors.load(Ordering::Relaxed).wrapping_add(missed),
        }
    }

    fn sample_state(&self, properties: &Properties) {
        let state = match properties.bus_error_mode() {
            BusErrorMode::ErrorActive => CanBusState::ErrorActive,
            BusErrorMode::ErrorPassive => CanBusState::ErrorPassive,
            BusErrorMode::BusOff => CanBusState::BusOff,
        };

        let previous = CanBusState::from_bits(self.state.swap(state as u8, Ordering::Relaxed));
        if (state as u8) > (previous as u8) && previous != CanBusState::Unknown {
            warn!("CAN bus degraded: {} -> {}", previous as u8, state as u8);
            self.count_error();
        }
    }

    fn count_error(&self) {
        self.errors.fetch_add(1, Ordering::Relaxed);
    }
}

pub static CAN1_HEALTH: CanHealthMonitor = CanHealthMonitor::new();
pub static CAN2_HEALTH: CanHealthMonitor = CanHealthMonitor::new();

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

async fn run_can_rx(
    can_rx: &'static mut CanRx<'static>,
    properties: Properties,
    regs: Fdcan,
    publisher: CanRxPublisher,
    health: &'static CanHealthMonitor,
) -> ! {
    health.sample_state(&properties);

    loop {
        let event = select(can_rx.read(), Timer::after(STATE_SAMPLE_INTERVAL)).await;
        health.sample_rx_overflow(regs);

        match event {
            Either::First(Ok(envelope)) => {
                // Drops the oldest frame for any subscriber that has fallen behind.
                publisher.publish_immediate(envelope.frame);
            }
            Either::First(Err(e)) => {
                // The state variants repeat on every read for as long as the state lasts; the
                // rest are one-off protocol errors.
                if !matches!(
                    e,
                    BusError::BusWarning | BusError::BusPassive | BusError::BusOff
                ) {
                    health.count_error();
                }
                health.sample_state(&properties);
                // wait in order to not starve the executor
                Timer::after(ERROR_POLL_INTERVAL).await;
            }
            Either::Second(()) => health.sample_state(&properties),
        }
    }
}

async fn run_can_tx(
    can_tx: &'static mut CanTx<'static>,
    mut subscriber: CanTxSubscriber,
    health: &'static CanHealthMonitor,
) -> ! {
    loop {
        match subscriber.next_message().await {
            // Publishers overwrote frames we had not taken yet.
            WaitResult::Lagged(n) => health.count_tx_missed(n),
            WaitResult::Message(message) => {
                // Only priority mode evicts, and `board.rs` uses FIFO mode.
                if can_tx.write(&message).await.is_some() {
                    health.count_tx_missed(1);
                }
            }
        }
    }
}

// --- CAN1
pub async fn spawn_can1(
    can: Can<'static>,
    spawner: SendSpawner,
    rx_publisher: CanRxPublisher,
    tx_subscriber: CanTxSubscriber,
) {
    let (can_tx, can_rx, properties) = can.split();
    let can_tx = CAN1_TX.init(can_tx);
    let can_rx = CAN1_RX.init(can_rx);

    spawner.spawn(run_can1_tx(can_tx, tx_subscriber)).unwrap();
    spawner
        .spawn(run_can1_rx(can_rx, properties, rx_publisher))
        .unwrap();
}

#[embassy_executor::task]
async fn run_can1_tx(can_tx: &'static mut CanTx<'static>, subscriber: CanTxSubscriber) -> ! {
    run_can_tx(can_tx, subscriber, &CAN1_HEALTH).await
}

#[embassy_executor::task]
async fn run_can1_rx(
    can_rx: &'static mut CanRx<'static>,
    properties: Properties,
    publisher: CanRxPublisher,
) -> ! {
    run_can_rx(
        can_rx,
        properties,
        board::CAN1_REGS,
        publisher,
        &CAN1_HEALTH,
    )
    .await
}

// --- CAN2
pub async fn spawn_can2(
    can: Can<'static>,
    spawner: SendSpawner,
    rx_publisher: CanRxPublisher,
    tx_subscriber: CanTxSubscriber,
) {
    let (can_tx, can_rx, properties) = can.split();
    let can_tx = CAN2_TX.init(can_tx);
    let can_rx = CAN2_RX.init(can_rx);

    spawner.spawn(run_can2_tx(can_tx, tx_subscriber)).unwrap();
    spawner
        .spawn(run_can2_rx(can_rx, properties, rx_publisher))
        .unwrap();
}

#[embassy_executor::task]
async fn run_can2_tx(can_tx: &'static mut CanTx<'static>, subscriber: CanTxSubscriber) -> ! {
    run_can_tx(can_tx, subscriber, &CAN2_HEALTH).await
}

#[embassy_executor::task]
async fn run_can2_rx(
    can_rx: &'static mut CanRx<'static>,
    properties: Properties,
    publisher: CanRxPublisher,
) -> ! {
    run_can_rx(
        can_rx,
        properties,
        board::CAN2_REGS,
        publisher,
        &CAN2_HEALTH,
    )
    .await
}
