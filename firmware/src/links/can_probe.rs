use embassy_stm32::can::Frame;
use embassy_sync::signal::Signal;
use embedded_can::{Id, StandardId};
use mavio::default_dialect::messages::CanFrame;
use static_cell::StaticCell;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};

use mavio::dialects::Common;

use crate::can::{CanRxSubscriber, CanTxPublisher};
use crate::links::ethernet::{EthRxSubscriber, EthTxPublisher};

pub type DownlinkSender = Sender<'static, CriticalSectionRawMutex, Frame, 5>;
pub type DownlinkReceiver = Receiver<'static, CriticalSectionRawMutex, Frame, 5>;
pub type UplinkSender = Sender<'static, CriticalSectionRawMutex, Frame, 5>;
pub type UplinkReceiver = Receiver<'static, CriticalSectionRawMutex, Frame, 5>;

pub static CAN_PROBE_ENABLED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

// bus-to-mavlink
static DOWNLINK: StaticCell<Channel<CriticalSectionRawMutex, Frame, 5>> = StaticCell::new();

// mavlink-to-bus
static UPLINK: StaticCell<Channel<CriticalSectionRawMutex, Frame, 5>> = StaticCell::new();

pub fn start(
    can_tx: CanTxPublisher,
    can_rx: CanRxSubscriber,
    spawner: Spawner,
    eth_tx: EthTxPublisher,
    eth_rx: EthRxSubscriber,
) {
    spawner.spawn(run_downlink(can_rx, eth_tx)).unwrap();

    spawner.spawn(run_uplink(eth_rx, can_tx)).unwrap();
}

#[embassy_executor::task]
async fn run_downlink(mut can_rx: CanRxSubscriber, eth_tx: EthTxPublisher) -> ! {
    while !CAN_PROBE_ENABLED.wait().await {}

    defmt::debug!("can probe enabled");

    loop {
        match select(CAN_PROBE_ENABLED.wait(), can_rx.next_message_pure()).await {
            Either::First(true) => {}
            Either::First(false) => {
                continue;
            }
            Either::Second(frame) => {
                let id = match frame.id() {
                    Id::Standard(sid) => sid.as_raw() as u32,
                    Id::Extended(eid) => eid.as_raw(),
                };
                let mut buffer = [0x00; 8];
                buffer.copy_from_slice(frame.data());
                let _ = eth_tx
                    .publish(Common::CanFrame(CanFrame {
                        target_system: 0xff,    // TODO
                        target_component: 0xff, // TODO
                        bus: 1,
                        id,
                        len: frame.data().len() as u8,
                        data: buffer,
                    }))
                    .await;
            }
        }
    }
}

// Receives Mavlink frames and publishes all valid CanFrames to the TODO: correct can bus.
#[embassy_executor::task]
async fn run_uplink(mut eth_rx: EthRxSubscriber, can_tx: CanTxPublisher) -> ! {
    loop {
        let frame = eth_rx.next_message_pure().await;
        let Ok(msg) = frame.decode::<Common>() else {
            continue;
        };

        // TODO: CAN-FD?
        let Common::CanFrame(can_frame) = msg else {
            continue;
        };
        defmt::info!("can frame: {}", defmt::Debug2Format(&can_frame));
        if can_frame.id > StandardId::MAX.as_raw() as u32 {
            defmt::warn!("refusing to publish non standard frame");
            continue;
        }
        if can_frame.len > 8 {
            defmt::warn!("refusing to publish malformed frame, longer than 8 bytes");
            continue;
        }
        match embassy_stm32::can::Frame::new_standard(can_frame.id as u16, &can_frame.data) {
            Err(e) => defmt::warn!(
                "refusing to publish malformed frame: {}",
                defmt::Debug2Format(&e)
            ),
            Ok(frame) => can_tx.publish(frame).await,
        }
    }
}
