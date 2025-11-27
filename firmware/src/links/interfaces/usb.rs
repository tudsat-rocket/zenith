use heapless::Vec;
use rapid_dialect::Rapid;
use static_cell::StaticCell;

use embassy_executor::{SendSpawner, Spawner};
use embassy_futures::select::{Either, select};
use embassy_stm32::Peri;
use embassy_stm32::bind_interrupts;
use embassy_stm32::peripherals::{PA11, PA12, USB_OTG_FS};
use embassy_stm32::usb;
use embassy_stm32::usb::Driver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, TimeoutError, Timer, with_timeout};
use embassy_usb::class::cdc_acm;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender};
use embassy_usb::{Builder, UsbDevice, driver::EndpointError};

use mavio::Frame;
use mavio::error::FrameError;
use mavio::prelude::V2;

use crate::links::interfaces::{
    InterfaceCommandSubscriber, InterfaceCommands, InterfaceRx, InterfaceRxPublisher,
    InterfaceRxSubscriber, InterfaceTx, InterfaceTxPublisher, InterfaceTxSubscriber,
};
use crate::links::{TelemetryLink, UplinkCommand, protocols};

pub const USB_SYSTEM_ID: u8 = 0x05;

pub static DOWNLINK: StaticCell<InterfaceTx> = StaticCell::new();
pub static UPLINK: StaticCell<InterfaceRx> = StaticCell::new();
pub static COMMANDS: StaticCell<InterfaceCommands> = StaticCell::new();

static EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static CONFIG_DESCRIPTOR_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESCRIPTOR_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESCRIPTOR_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUFFER: StaticCell<[u8; 128]> = StaticCell::new();

static CDC_ACM_STATE: StaticCell<cdc_acm::State> = StaticCell::new();

pub struct UsbHandle {
    tx: InterfaceTxPublisher,
    cmd_rx: InterfaceCommandSubscriber,
}

impl UsbHandle {
    pub fn init(driver: Driver<'static, USB_OTG_FS>, spawner: Spawner) -> Self {
        let tx = DOWNLINK.init(PubSubChannel::new());
        let rx = UPLINK.init(PubSubChannel::new());
        let commands = COMMANDS.init(PubSubChannel::new());

        let mut config = embassy_usb::Config::new(0x0483, 0x5740);
        config.manufacturer = Some("TUDSaT");
        // The trailing USB UART is important, since USB ports matching the regex "USB UART$" are
        // connected to by QGroundControl on Android.
        config.product = Some("Stigma USB UART");
        config.serial_number = Some("12345678"); // TODO

        // Required for windows compatibility.
        // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
        config.device_class = 0xEF;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;

        let mut builder = Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR_BUFFER.init([0; 256]),
            BOS_DESCRIPTOR_BUFFER.init([0; 256]),
            MSOS_DESCRIPTOR_BUFFER.init([0; 256]),
            CONTROL_BUFFER.init([0; 128]),
        );

        let class = CdcAcmClass::new(&mut builder, CDC_ACM_STATE.init(cdc_acm::State::new()), 64);

        let usb = builder.build();
        spawner.spawn(run_usb(usb)).unwrap();

        let (usb_sender, usb_receiver) = class.split();
        spawner
            .spawn(run_downlink(usb_sender, tx.subscriber().unwrap()))
            .unwrap();
        spawner
            .spawn(run_uplink(usb_receiver, rx.publisher().unwrap()))
            .unwrap();

        spawner
            .spawn(protocols::commands::run(
                USB_SYSTEM_ID,
                0x01,
                tx.publisher().unwrap(),
                rx.subscriber().unwrap(),
                commands.publisher().unwrap(),
            ))
            .unwrap();

        spawner
            .spawn(protocols::modes::run(
                tx.publisher().unwrap(),
                commands.subscriber().unwrap(),
            ))
            .unwrap();

        UsbHandle {
            tx: tx.publisher().unwrap(),
            cmd_rx: commands.subscriber().unwrap(),
        }
    }
}

impl TelemetryLink for UsbHandle {
    const MAVLINK_SYSTEM_ID: u8 = USB_SYSTEM_ID;

    const SENSOR_INTERVAL_MS: u32 = 200;

    fn send_message(&mut self, message: Rapid) {
        let _ = self.tx.publish_immediate(message.into());
    }

    fn try_recv_command(&mut self) -> Option<UplinkCommand> {
        while let Some(cmd) = self.cmd_rx.try_next_message_pure() {
            match cmd {
                UplinkCommand::SetFlightMode(..) => return Some(cmd),
                _ => {}
            }
        }

        None
    }
}

#[embassy_executor::task]
async fn run_usb(mut usb: UsbDevice<'static, usb::Driver<'static, USB_OTG_FS>>) -> ! {
    defmt::info!("Started running usb driver.");
    usb.run().await
}

async fn write_message(
    sender: &mut Sender<'static, Driver<'static, USB_OTG_FS>>,
    serialized: &[u8],
) -> Result<(), EndpointError> {
    for chunk in serialized.chunks(64) {
        sender.write_packet(chunk).await?;
    }

    if serialized.len() % 64 == 0 {
        sender.write_packet(&[]).await?;
    }

    Ok(())
}

#[embassy_executor::task]
async fn run_downlink(
    mut sender: Sender<'static, Driver<'static, USB_OTG_FS>>,
    mut subscriber: InterfaceTxSubscriber,
) -> ! {
    let endpoint = mavio::Endpoint::v2(mavio::MavLinkId::new(USB_SYSTEM_ID, 0x01));

    loop {
        defmt::info!("Waiting for usb connection.");
        sender.wait_connection().await;
        defmt::info!("Usb connection established");

        loop {
            let message = subscriber.next_message_pure().await;

            // TODO: this is awful
            let frame = match message {
                Rapid::Heartbeat(m) => endpoint.next_frame(&m).unwrap(),
                Rapid::CommandAck(m) => endpoint.next_frame(&m).unwrap(),
                Rapid::AvailableModes(m) => endpoint.next_frame(&m).unwrap(),
                Rapid::CanFrame(m) => endpoint.next_frame(&m).unwrap(),
                Rapid::Attitude(m) => endpoint.next_frame(&m).unwrap(),
                Rapid::LocalPositionNed(m) => endpoint.next_frame(&m).unwrap(),
                Rapid::ScaledImu(m) => endpoint.next_frame(&m).unwrap(),
                Rapid::ScaledImu2(m) => endpoint.next_frame(&m).unwrap(),
                Rapid::ScaledImu3(m) => endpoint.next_frame(&m).unwrap(),
                Rapid::ScaledPressure(m) => endpoint.next_frame(&m).unwrap(),
                Rapid::ScaledPressure2(m) => endpoint.next_frame(&m).unwrap(),
                Rapid::ScaledPressure3(m) => endpoint.next_frame(&m).unwrap(),
                Rapid::BatteryStatus(m) => endpoint.next_frame(&m).unwrap(),
                _ => continue,
            };

            let mut transmit_buffer = [0; 1024];
            let n = frame.serialize(&mut transmit_buffer).unwrap();
            let serialized = &transmit_buffer[..n];

            match with_timeout(
                Duration::from_millis(10),
                write_message(&mut sender, &serialized),
            )
            .await
            {
                Ok(Ok(())) => {}
                Ok(Err(EndpointError::BufferOverflow)) => {
                    defmt::error!("buffer overflow");
                }
                Ok(Err(EndpointError::Disabled)) => {
                    defmt::error!("disabled");
                    break;
                }
                Err(_e) => {
                    continue;
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn run_uplink(
    mut receiver: Receiver<'static, Driver<'static, USB_OTG_FS>>,
    publisher: InterfaceRxPublisher,
) -> ! {
    const UPLINK_BUFFER_SIZE: usize = 512;

    // NOTE: change to [] at some point
    let uplink_buffer: Vec<u8, UPLINK_BUFFER_SIZE> = Vec::new();
    // NOTE: enforce packet size
    let mut packet_buffer: [u8; 64] = [0; 64];

    let endpoint = mavio::Endpoint::v2(mavio::MavLinkId::new(USB_SYSTEM_ID, 0x01));
    let mut mavlink_buffer = heapless::Vec::<u8, 1024>::new();

    loop {
        defmt::debug!("uplink waiting for usb connection.");
        receiver.wait_connection().await;
        defmt::debug!("uplink usb connection established");

        loop {
            let len = match receiver.read_packet(&mut packet_buffer).await {
                Ok(len) => len,
                Err(EndpointError::BufferOverflow) => {
                    defmt::error!("buffer overflow");
                    continue;
                }
                Err(EndpointError::Disabled) => {
                    defmt::error!("disabled");
                    break;
                }
            };

            if let Err(..) = mavlink_buffer.extend_from_slice(&packet_buffer[..len]) {
                mavlink_buffer.truncate(0);
                continue;
            }

            let frame_result = unsafe { mavio::Frame::deserialize(&mavlink_buffer) };
            match frame_result {
                Ok(frame) => {
                    publisher.publish(frame).await;
                    mavlink_buffer.truncate(0);
                }
                Err(FrameError::FrameBufferIsTooSmall { .. }) => {}
                Err(..) => {
                    mavlink_buffer.truncate(0);
                }
            }
        }
    }
}
