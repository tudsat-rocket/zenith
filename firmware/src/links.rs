use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::eth::{Ethernet, GenericPhy};
use embassy_stm32::peripherals::*;
use embassy_stm32::usb::Driver;
use embassy_time::Delay;

use links::Downlink;
use lora_phy::LoRa;
use mission::TelemetryLink;
use rapid_dialect::FlightMode;
use rapid_dialect::rapid::enums::{MavCmd, MavResult};

use crate::LoraTransceiver;
use crate::Vehicle;
use crate::can::{CanRxSubscriber, CanTxPublisher};
use crate::links::interfaces::ethernet::EthernetHandle;
use crate::links::interfaces::lora::LoraHandle;
use crate::links::interfaces::usb::UsbHandle;

pub mod interfaces;
mod protocols;

pub use links::UplinkCommand;

#[derive(Copy, Clone, Debug)]
pub enum CommandToken {
    Lora(u16),
    Ethernet(MavCmd),
    Usb(MavCmd),
    /// Acked by the protocol that received it, e.g. a PARAM_SET by its PARAM_VALUE.
    Unanswered,
}

pub struct Links {
    lora: LoraHandle,
    ethernet: EthernetHandle,
    usb: UsbHandle,
}

impl Links {
    pub async fn init(
        ethernet: Ethernet<'static, ETH, GenericPhy>,
        seed: u64,
        usb: Driver<'static, USB_OTG_FS>,
        lora1: LoRa<LoraTransceiver, Delay>,
        lora2: LoRa<LoraTransceiver, Delay>,
        can: (CanTxPublisher, CanRxSubscriber),
        medium_priority_spawner: SendSpawner,
        low_priority_spawner: Spawner,
    ) -> Self {
        let lora = LoraHandle::init(lora1, lora2, medium_priority_spawner);
        let ethernet = EthernetHandle::init(ethernet, seed, can, low_priority_spawner);
        let usb = UsbHandle::init(usb, low_priority_spawner);

        Self {
            lora,
            ethernet,
            usb,
        }
    }

    pub fn send_telemetry_messages(&mut self, vehicle: &Vehicle) {
        self.lora.send_telemetry_messages(vehicle);

        let snapshot = vehicle.snapshot();
        snapshot.send_telemetry(&mut self.ethernet);
        snapshot.send_telemetry(&mut self.usb);
    }

    pub fn try_recv_command(&mut self) -> Option<(CommandToken, UplinkCommand)> {
        if let Some((seq, cmd)) = self.lora.try_recv_command() {
            return Some((CommandToken::Lora(seq), cmd));
        }

        if let Some(cmd) = self.ethernet.try_recv_command() {
            return Self::from_wired(&mut self.ethernet, cmd, CommandToken::Ethernet);
        }

        if let Some(cmd) = self.usb.try_recv_command() {
            return Self::from_wired(&mut self.usb, cmd, CommandToken::Usb);
        }

        None
    }

    pub fn note_command_result(&mut self, token: CommandToken, result: MavResult) {
        match token {
            CommandToken::Lora(seq) => self.lora.note_command_result(seq, result),
            CommandToken::Ethernet(command) => {
                self.ethernet
                    .send_message(Downlink::command_ack(command, result));
            }
            CommandToken::Usb(command) => {
                self.usb
                    .send_message(Downlink::command_ack(command, result));
            }
            CommandToken::Unanswered => {}
        }
    }

    /// Filters a command received on a wired link.
    ///
    /// Ignition is reachable via LoRa only.
    fn from_wired(
        link: &mut impl TelemetryLink,
        cmd: UplinkCommand,
        token: fn(MavCmd) -> CommandToken,
    ) -> Option<(CommandToken, UplinkCommand)> {
        let token = cmd.mav_cmd().map_or(CommandToken::Unanswered, token);

        if matches!(cmd, UplinkCommand::SetFlightMode(FlightMode::Ignite)) {
            defmt::warn!("Denying Ignite commanded on a wired link.");
            link.send_message(Downlink::command_ack(MavCmd::DoSetMode, MavResult::Denied));
            return None;
        }

        Some((token, cmd))
    }
}
