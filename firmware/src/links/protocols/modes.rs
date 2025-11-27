//! This serves AVAILABLE_MODES MavLink messages when requested using the REQUEST_MESSAGE command.
//!
//! Executed for both Ethernet and USB links.

use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver};

use rapid_dialect::rapid::{
    enums::{MavModeProperty, MavStandardMode},
    messages::AvailableModes,
};
use rapid_dialect::{FlightMode, Rapid};

use crate::links::UplinkCommand;
use crate::links::interfaces::{
    InterfaceCommandSubscriber, InterfaceRxSubscriber, InterfaceTxPublisher,
};

#[embassy_executor::task(pool_size = 2)]
pub async fn run(tx: InterfaceTxPublisher, mut rx: InterfaceCommandSubscriber) {
    loop {
        match rx.next_message_pure().await {
            UplinkCommand::RequestAvailableModes(index) => {
                if index == 0 {
                    defmt::info!("Enumerating AVAILABLE_MODES...");
                    for (i, mode) in FlightMode::ALL.into_iter().enumerate() {
                        let msg = Rapid::AvailableModes(AvailableModes {
                            number_modes: FlightMode::ALL.len() as u8,
                            mode_index: (i + 1) as u8,
                            standard_mode: MavStandardMode::NonStandard,
                            custom_mode: mode as u32,
                            properties: MavModeProperty::default(),
                            mode_name: mode.mavlink_name(),
                        });
                        tx.publish(msg).await;
                        Timer::after(Duration::from_millis(50)).await;
                    }
                } else if (index - 1) < FlightMode::ALL.len() {
                    let mode = FlightMode::ALL[index - 1];
                    let msg = Rapid::AvailableModes(AvailableModes {
                        number_modes: FlightMode::ALL.len() as u8,
                        mode_index: index as u8,
                        standard_mode: MavStandardMode::NonStandard,
                        custom_mode: mode as u32,
                        properties: MavModeProperty::default(),
                        mode_name: mode.mavlink_name(),
                    });
                    tx.publish(msg).await;
                }
            }
            _ => {}
        };
    }
}
