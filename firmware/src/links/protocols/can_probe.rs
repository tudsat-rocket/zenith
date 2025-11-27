//! This task implements CAN bus forwarding using CAN_FRAME MavLink messages.
//!
//! When requested, all CAN frames received by the FC are forwarded using CAN_FRAME.
//! Frames received by the FC using CAN_FRAME MAVLink messages are transmitted on the CAN bus.
//!
//! At this time, there are the following limitations:
//! - CAN1 only
//! - No CAN-FD
//!
//! Executed only for Ethernet at this time.

use embedded_can::{Id, StandardId};
use static_cell::StaticCell;

use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_stm32::can::Frame;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::signal::Signal;

use mavio::dialects::Common;

use rapid_dialect::Rapid;
use rapid_dialect::rapid::messages::CanFrame;

use crate::can::{CanRxSubscriber, CanTxPublisher};
use crate::links::UplinkCommand;
use crate::links::interfaces::{
    InterfaceCommandSubscriber, InterfaceRxSubscriber, InterfaceTxPublisher,
};

#[embassy_executor::task]
pub async fn run(
    can_tx: CanTxPublisher,
    mut can_rx: CanRxSubscriber,
    mut cmd_rx: InterfaceCommandSubscriber,
    eth_tx: InterfaceTxPublisher,
    mut eth_rx: InterfaceRxSubscriber,
) {
    // TODO: handle this separately for our two buses
    let mut can_forwarding_enabled = false;

    loop {
        match select3(
            eth_rx.next_message_pure(),
            cmd_rx.next_message_pure(),
            can_rx.next_message_pure(),
        )
        .await
        {
            // Received a MAVLink message, if it's a CAN_FRAME, publish it on the CAN bus
            Either3::First(frame) => {
                let Ok(msg) = frame.decode::<Rapid>() else {
                    continue;
                };

                let Rapid::CanFrame(can_frame) = msg else {
                    continue;
                };

                defmt::info!("can frame: {}", defmt::Debug2Format(&can_frame));

                if can_frame.id > StandardId::MAX.as_raw() as u32 {
                    defmt::warn!("refusing to publish non-standard frame");
                    continue;
                }

                if can_frame.len > 8 {
                    defmt::warn!("refusing to publish malformed frame, longer than 8 bytes");
                    continue;
                }

                match embassy_stm32::can::Frame::new_standard(can_frame.id as u16, &can_frame.data)
                {
                    Err(e) => defmt::warn!(
                        "refusing to publish malformed frame: {}",
                        defmt::Debug2Format(&e)
                    ),
                    Ok(frame) => can_tx.publish(frame).await,
                }
            }
            // Received a request to enable CAN forwarding
            Either3::Second(cmd) if cmd == UplinkCommand::RequestCanForwarding => {
                defmt::info!("Enabling CAN forwarding.");
                can_forwarding_enabled = true;
            }
            // Received a CAN frame while forwarding is enabled. Publish it as MAVLink.
            Either3::Third(frame) if can_forwarding_enabled => {
                let id = match frame.id() {
                    Id::Standard(sid) => sid.as_raw() as u32,
                    Id::Extended(eid) => eid.as_raw(),
                };
                let mut buffer = [0x00; 8];
                // copy data from frame data 'vec' into buffer
                for (i, byte) in frame.data().iter().enumerate().take(frame.data().len()) {
                    buffer[i] = *byte
                }

                let _ = eth_tx
                    .publish(Rapid::CanFrame(CanFrame {
                        target_system: 0xff,    // TODO
                        target_component: 0xff, // TODO
                        bus: 1,
                        id,
                        len: frame.data().len() as u8,
                        data: buffer,
                    }))
                    .await;
            }
            _ => {}
        }
    }
}
