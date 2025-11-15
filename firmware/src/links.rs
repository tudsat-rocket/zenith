use embassy_executor::Spawner;
use embassy_stm32::eth::{Ethernet, GenericPhy};
use embassy_stm32::{can, peripherals::*};

use embassy_sync::pubsub::PubSubChannel;
use mavio::default_dialect::enums::{MavAutopilot, MavModeFlag, MavType};
use mavio::default_dialect::enums::{MavCmd, MavResult};
use mavio::default_dialect::messages::Heartbeat;
use mavio::default_dialect::messages::{CommandAck, CommandLong};
use mavio::default_dialect::messages::{ScaledImu, ScaledImu2, ScaledImu3};
use mavio::dialects::Common;
use mavio::prelude::V2;
use mavio::{Frame, Message};

use crate::can::{CanRxSubscriber, CanTxPublisher};
use crate::vehicle::Vehicle;

mod can_probe;
mod ethernet;

pub struct Links {
    eth_tx: ethernet::EthTxPublisher,
    eth_rx: ethernet::EthRxSubscriber,
}

impl Links {
    pub async fn init(
        ethernet: Ethernet<'static, ETH, GenericPhy>,
        seed: u64,
        can: (CanTxPublisher, CanRxSubscriber),
        spawner: Spawner,
    ) -> Self {
        let eth_tx = ethernet::DOWNLINK.init(PubSubChannel::new());
        let eth_rx = ethernet::UPLINK.init(PubSubChannel::new());

        ethernet::start(
            ethernet,
            spawner,
            seed,
            eth_tx.subscriber().unwrap(),
            eth_rx.publisher().unwrap(),
        );

        can_probe::start(
            can.0,
            can.1,
            spawner,
            eth_tx.publisher().unwrap(),
            eth_rx.subscriber().unwrap(),
        );

        Self {
            eth_tx: eth_tx.publisher().unwrap(),
            eth_rx: eth_rx.subscriber().unwrap(),
        }
    }

    pub fn handle_frame(&mut self, vehicle: &mut Vehicle, frame: Frame<V2>) {
        let Ok(message) = frame.decode::<Common>() else {
            return;
        };

        defmt::info!("Handling frame {:?}", defmt::Debug2Format(&frame));

        match message {
            Common::CommandLong(cmd) => match cmd.command {
                MavCmd::CanForward => {
                    // 0 disables all forwarding, non-zero values give the bus
                    let bus_enable = cmd.param1 as u8;
                    // For now we only support bus1
                    can_probe::CAN_PROBE_ENABLED.signal(bus_enable > 0);
                    self.ack(
                        cmd.command,
                        MavResult::Accepted,
                        frame.system_id(),
                        frame.component_id(),
                    )
                }
                _ => self.ack(
                    cmd.command,
                    MavResult::Unsupported,
                    frame.system_id(),
                    frame.component_id(),
                ),
            },
            Common::CommandInt(cmd) => self.ack(
                cmd.command,
                MavResult::CommandLongOnly,
                frame.system_id(),
                frame.component_id(),
            ),
            _ => {}
        }
    }

    pub fn ack(&mut self, cmd: MavCmd, result: MavResult, target_system: u8, target_component: u8) {
        let nack = CommandAck {
            command: cmd,
            result,
            target_system,
            target_component,
            ..Default::default()
        };
        let _ = self.eth_tx.publish_immediate(Common::CommandAck(nack));
    }

    pub fn receive_uplink(&mut self, vehicle: &mut Vehicle) {
        while let Some(frame) = self.eth_rx.try_next_message_pure() {
            defmt::info!("got frame");
            self.handle_frame(vehicle, frame);
        }
    }

    pub fn send_common_message<M: Message + Into<Common>>(&mut self, vehicle: &Vehicle)
    where
        for<'a> &'a Vehicle: Into<M>,
    {
        let m: M = vehicle.into();
        let _ = self.eth_tx.publish_immediate(m.into());
    }

    pub fn transmit_downlink(&mut self, vehicle: &Vehicle) {
        if vehicle.time.0 % 500 == 0 {
            let heartbeat = Heartbeat {
                type_: MavType::Rocket,
                autopilot: MavAutopilot::Generic,
                base_mode: MavModeFlag::empty(), // TODO
                custom_mode: 0,                  // TODO
                system_status: mavio::dialects::minimal::enums::MavState::Standby, // TODO
                mavlink_version: 2,              // TODO: check
            };
            let _ = self.eth_tx.publish_immediate(Common::Heartbeat(heartbeat));
        }

        if vehicle.time.0 % 100 == 0 {
            self.send_common_message::<ScaledImu>(vehicle);
            self.send_common_message::<ScaledImu2>(vehicle);
            self.send_common_message::<ScaledImu3>(vehicle);
        }
    }
}

// TODO: where do we put high-g accelerometer?
impl Into<ScaledImu> for &Vehicle {
    fn into(self) -> ScaledImu {
        let acc1 = self.sensors.imu1.accelerometer();
        let gyro1 = self.sensors.imu1.gyroscope();
        let mag1 = self.sensors.mag.magnetometer();

        ScaledImu {
            time_boot_ms: self.time.0,
            xacc: (acc1.map(|v| v.x).unwrap_or_default() * 101.972) as i16,
            yacc: (acc1.map(|v| v.y).unwrap_or_default() * 101.972) as i16,
            zacc: (acc1.map(|v| v.z).unwrap_or_default() * 101.972) as i16,
            xgyro: (gyro1.map(|v| v.x).unwrap_or_default() * 17.45329) as i16,
            ygyro: (gyro1.map(|v| v.y).unwrap_or_default() * 17.45329) as i16,
            zgyro: (gyro1.map(|v| v.z).unwrap_or_default() * 17.45329) as i16,
            xmag: (mag1.map(|v| v.x).unwrap_or_default() * 10.0) as i16,
            ymag: (mag1.map(|v| v.y).unwrap_or_default() * 10.0) as i16,
            zmag: (mag1.map(|v| v.z).unwrap_or_default() * 10.0) as i16,
            temperature: 0, // TODO
        }
    }
}

impl Into<ScaledImu2> for &Vehicle {
    fn into(self) -> ScaledImu2 {
        let acc2 = self.sensors.imu2.accelerometer();
        let gyro2 = self.sensors.imu2.gyroscope();

        ScaledImu2 {
            time_boot_ms: self.time.0,
            xacc: (acc2.map(|v| v.x).unwrap_or_default() * 101.972) as i16,
            yacc: (acc2.map(|v| v.y).unwrap_or_default() * 101.972) as i16,
            zacc: (acc2.map(|v| v.z).unwrap_or_default() * 101.972) as i16,
            xgyro: (gyro2.map(|v| v.x).unwrap_or_default() * 17.45329) as i16,
            ygyro: (gyro2.map(|v| v.y).unwrap_or_default() * 17.45329) as i16,
            zgyro: (gyro2.map(|v| v.z).unwrap_or_default() * 17.45329) as i16,
            ..Default::default()
        }
    }
}

impl Into<ScaledImu3> for &Vehicle {
    fn into(self) -> ScaledImu3 {
        let acc3 = self.sensors.imu3.accelerometer();
        let gyro3 = self.sensors.imu3.gyroscope();

        ScaledImu3 {
            time_boot_ms: self.time.0,
            xacc: (acc3.map(|v| v.x).unwrap_or_default() * 101.972) as i16,
            yacc: (acc3.map(|v| v.y).unwrap_or_default() * 101.972) as i16,
            zacc: (acc3.map(|v| v.z).unwrap_or_default() * 101.972) as i16,
            xgyro: (gyro3.map(|v| v.x).unwrap_or_default() * 17.45329) as i16,
            ygyro: (gyro3.map(|v| v.y).unwrap_or_default() * 17.45329) as i16,
            zgyro: (gyro3.map(|v| v.z).unwrap_or_default() * 17.45329) as i16,
            ..Default::default()
        }
    }
}
