use core::f32::consts::PI;
use core::u16;

use embassy_executor::Spawner;
use embassy_stm32::eth::{Ethernet, GenericPhy};
use embassy_stm32::peripherals::*;
use embassy_stm32::usb::Driver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_sync::pubsub::PubSubChannel;

use mavio::prelude::V2;
use mavio::{Frame, Message};
use nalgebra::{Quaternion, Unit};

use rapid_dialect::rapid::enums::{
    MavAutopilot, MavBatteryChargeState, MavBatteryFault, MavBatteryFunction, MavBatteryMode,
    MavBatteryType, MavCmd, MavModeFlag, MavResult, MavType,
};
use rapid_dialect::rapid::messages::{
    Attitude, AvailableModes, BatteryStatus, CommandAck, Heartbeat, LocalPositionNed, ScaledImu,
    ScaledImu2, ScaledImu3, ScaledPressure, ScaledPressure2, ScaledPressure3,
};
use rapid_dialect::{FlightMode, Rapid};

use crate::can::{CanRxSubscriber, CanTxPublisher};
use crate::links::interfaces::ethernet::EthernetHandle;
use crate::links::interfaces::usb::UsbHandle;
use crate::vehicle::Vehicle;

mod interfaces;
mod protocols;
mod vehicle;

#[derive(Clone, PartialEq, Eq)]
pub enum UplinkCommand {
    SetFlightMode(FlightMode),
    RequestAvailableModes(usize),
    RequestCanForwarding,
}

trait TelemetryLink {
    const HEARTBEAT_INTERVAL_MS: u32 = 500;
    const SENSOR_INTERVAL_MS: u32 = 100;

    // TODO: error type
    fn send_message(&mut self, message: Rapid);

    fn send_telemetry_message<M: Message + Into<Rapid>>(&mut self, vehicle: &Vehicle)
    where
        for<'a> &'a Vehicle: Into<M>,
    {
        let m: M = vehicle.into();
        self.send_message(m.into());
    }

    fn send_telemetry_messages(&mut self, vehicle: &Vehicle) {
        if vehicle.time.0 % Self::HEARTBEAT_INTERVAL_MS == 0 {
            self.send_telemetry_message::<Heartbeat>(vehicle);
        }

        if vehicle.time.0 % Self::SENSOR_INTERVAL_MS == 0 {
            self.send_telemetry_message::<Attitude>(vehicle);
            self.send_telemetry_message::<LocalPositionNed>(vehicle);
            self.send_telemetry_message::<ScaledImu>(vehicle);
            self.send_telemetry_message::<ScaledImu2>(vehicle);
            self.send_telemetry_message::<ScaledImu3>(vehicle);
        }

        if vehicle.time.0 % Self::SENSOR_INTERVAL_MS == Self::SENSOR_INTERVAL_MS / 2 {
            self.send_telemetry_message::<ScaledPressure>(vehicle);
            self.send_telemetry_message::<ScaledPressure2>(vehicle);
            self.send_telemetry_message::<ScaledPressure3>(vehicle);
        }

        if vehicle.time.0 % 200 == 0 {
            self.send_telemetry_message::<BatteryStatus>(vehicle);
        }
    }

    fn try_recv_command(&mut self) -> Option<UplinkCommand>;
}

pub struct Links {
    ethernet: EthernetHandle,
    usb: UsbHandle,
}

impl Links {
    pub async fn init(
        ethernet: Ethernet<'static, ETH, GenericPhy>,
        seed: u64,
        usb: Driver<'static, USB_OTG_FS>,
        can: (CanTxPublisher, CanRxSubscriber),
        spawner: Spawner,
    ) -> Self {
        let ethernet = EthernetHandle::init(ethernet, seed, can, spawner);
        let usb = UsbHandle::init(usb, spawner);

        Self { ethernet, usb }
    }

    pub fn send_telemetry_message<M: Message + Into<Rapid>>(&mut self, vehicle: &Vehicle)
    where
        for<'a> &'a Vehicle: Into<M>,
    {
        self.ethernet.send_telemetry_message(vehicle);
        self.usb.send_telemetry_message(vehicle);
    }

    pub fn send_telemetry_messages(&mut self, vehicle: &Vehicle) {
        self.ethernet.send_telemetry_messages(vehicle);
        self.usb.send_telemetry_messages(vehicle);
    }

    pub fn try_recv_command(&mut self) -> Option<UplinkCommand> {
        if let Some(cmd) = self.ethernet.try_recv_command() {
            return Some(cmd);
        }

        if let Some(cmd) = self.usb.try_recv_command() {
            return Some(cmd);
        }

        None
    }
}
