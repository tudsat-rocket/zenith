use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::eth::{Ethernet, GenericPhy};
use embassy_stm32::peripherals::*;
use embassy_stm32::usb::Driver;
use embassy_time::Delay;

use lora_phy::LoRa;
use mission::TelemetryLink;

use crate::LoraTransceiver;
use crate::Vehicle;
use crate::can::{CanRxSubscriber, CanTxPublisher};
use crate::links::interfaces::ethernet::EthernetHandle;
use crate::links::interfaces::lora::LoraHandle;
use crate::links::interfaces::usb::UsbHandle;

pub mod interfaces;
mod protocols;

pub use links::UplinkCommand;

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
        vehicle.send_telemetry(&mut self.ethernet);
        vehicle.send_telemetry(&mut self.usb);
    }

    pub fn try_recv_command(&mut self) -> Option<UplinkCommand> {
        if let Some(cmd) = self.lora.try_recv_command() {
            return Some(cmd);
        }

        if let Some(cmd) = self.ethernet.try_recv_command() {
            return Some(cmd);
        }

        if let Some(cmd) = self.usb.try_recv_command() {
            return Some(cmd);
        }

        None
    }
}
