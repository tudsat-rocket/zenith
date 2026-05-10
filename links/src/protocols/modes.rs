//! This serves AVAILABLE_MODES MavLink messages when requested using the REQUEST_MESSAGE command.
//!
//! Executed for both Ethernet and USB links.

use embassy_time::{Duration, Timer};

use rapid_dialect::rapid::{
    enums::{MavModeProperty, MavStandardMode},
    messages::AvailableModes,
};
use rapid_dialect::{FlightMode, Rapid};

use crate::{InterfaceCommandSubscriber, InterfaceTxPublisher, UplinkCommand};

fn mode_properties(mode: FlightMode) -> MavModeProperty {
    let mut props = MavModeProperty::default();

    let hidden = match mode {
        #[cfg(feature = "hybrid")]
        FlightMode::Armed => true,
        #[cfg(not(feature = "hybrid"))]
        FlightMode::Filling
        | FlightMode::Venting
        | FlightMode::Pressurizing
        | FlightMode::Hold
        | FlightMode::Ignition => true,
        _ => false,
    };

    if hidden {
        props |= MavModeProperty::NOT_USER_SELECTABLE;
    }

    if !matches!(
        mode,
        FlightMode::Idle | FlightMode::HardwareArmed | FlightMode::Landed
    ) {
        props |= MavModeProperty::ADVANCED;
    }

    if matches!(
        mode,
        FlightMode::Burn
            | FlightMode::Coast
            | FlightMode::RecoveryDrogue
            | FlightMode::RecoveryMain
    ) {
        props |= MavModeProperty::AUTO_MODE;
    }

    props
}

pub async fn run(tx: InterfaceTxPublisher, mut rx: InterfaceCommandSubscriber) {
    log::info!("modes: task started");
    loop {
        let cmd = rx.next_message_pure().await;
        log::debug!("modes: received command: {cmd:?}");
        match cmd {
            UplinkCommand::RequestAvailableModes(index) => {
                if index == 0 {
                    log::info!("Enumerating AVAILABLE_MODES...");
                    for (i, mode) in FlightMode::ALL.into_iter().enumerate() {
                        let msg = Rapid::AvailableModes(AvailableModes {
                            number_modes: FlightMode::ALL.len() as u8,
                            mode_index: (i + 1) as u8,
                            standard_mode: MavStandardMode::NonStandard,
                            custom_mode: mode as u32,
                            properties: mode_properties(mode),
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
                        properties: mode_properties(mode),
                        mode_name: mode.mavlink_name(),
                    });
                    tx.publish(msg).await;
                }
            }
            _ => {}
        }
    }
}
