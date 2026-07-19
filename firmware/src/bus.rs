use core::{cmp::*, num::Wrapping};

use embassy_stm32::can::Frame;
use embassy_stm32::time;
use embassy_time::{Duration, Instant};
use embedded_can::Id;

use heapless::Vec;
use num_traits::float::Float;
use zencan_common::{CanId, CanMessage, sdo::SdoRequest};

use rapid_dialect::rapid::enums::{ValveId, valve_id};

use mission::bus::{
    Bus, BusDataError, BusInputImage, BusOutputImage, DataWithTime, IoAddr, ValveState,
};
use mission::inventory::{BinaryOutputId, BinaryOutputMap, InventoryId, ValveMap};

use crate::bus::mapping::{BINARY_OUTPUT_ID_MAP, VALVE_ID_MAP};
use crate::bus::pdo_mapping::{
    PdMessageKind, ProcessDataCanId, binary_output_msg_to_bo, valve_msg_to_valve,
};
use crate::can::{CanRxSubscriber, CanTxPublisher};

mod mapping;
mod pdo_mapping;

pub const VERY_FRESH_DURATION: Duration = Duration::from_millis(50);
pub const BINARY_OUTPUT_MESSAGE_INTERVAL: Duration = Duration::from_millis(50);
pub const VALVE_MESSAGE_INTERVAL: Duration = Duration::from_millis(50);

pub struct BusHandler {
    pub input: BusInputImage,
    pub outputs_last: BusOutputImage,
    pub can: (CanTxPublisher, CanRxSubscriber),
    last_binary_output_messages: BinaryOutputMap<Option<Instant>>,
    last_valve_messages: ValveMap<Option<Instant>>,
}

impl BusHandler {
    pub fn new(can_tx_pub: CanTxPublisher, can_rx_sub: CanRxSubscriber) -> Self {
        Self {
            input: BusInputImage::default(),
            outputs_last: BusOutputImage::default(),
            can: (can_tx_pub, can_rx_sub),
            last_binary_output_messages: BinaryOutputMap::splat(None),
            last_valve_messages: ValveMap::splat(None),
        }
    }
}

impl Bus for BusHandler {
    fn get_input_image(&mut self) -> BusInputImage {
        while let Some(msg) = self.can.1.try_next_message_pure() {
            try_injest_can_msg(
                &mut self.input,
                msg,
                Wrapping(Instant::now().as_millis() as u32),
            );
        }

        self.input.clone()
    }

    fn set_output_image(&mut self, outputs: BusOutputImage) {
        // Each output gets a message every time that state changes or every x_MESSAGE_INTERVAL.

        for i in BinaryOutputId::ALL {
            let last_message = self.last_binary_output_messages[i];
            let is_due = last_message
                .map(|i| i.elapsed() > BINARY_OUTPUT_MESSAGE_INTERVAL)
                .unwrap_or(true);
            let has_changed = outputs.binary_output[i] != self.outputs_last.binary_output[i];

            if !has_changed && !is_due {
                continue;
            }

            let target_state = outputs.binary_output[i];
            let io_addr = &BINARY_OUTPUT_ID_MAP[i];
            let msg = set_boolean_msg(target_state, io_addr);
            let frame = can_msg_to_frame(&msg);

            if self.can.0.try_publish(frame).is_err() {
                // Can't log here, too noisy.
                self.can.0.publish_immediate(frame);
            }

            self.last_binary_output_messages[i] = Some(Instant::now());
        }

        for i in ValveId::ALL {
            let last_message = self.last_valve_messages[i];
            let is_due = last_message
                .map(|i| i.elapsed() > VALVE_MESSAGE_INTERVAL)
                .unwrap_or(true);
            let has_changed = outputs.valve[i] != self.outputs_last.valve[i];

            if !has_changed && !is_due {
                continue;
            }

            let target_state = outputs.valve[i];
            let io_addr = &VALVE_ID_MAP[i];

            let msg = sdo_write_msg(
                &heapless::Vec::from_slice(&(target_state.promille().to_le_bytes())).unwrap(),
                io_addr,
            );
            let frame = can_msg_to_frame(&msg);

            if self.can.0.try_publish(frame).is_err() {
                // Can't log here, too noisy.
                self.can.0.publish_immediate(frame);
            }

            self.last_valve_messages[i] = Some(Instant::now());
        }

        self.outputs_last = outputs;
    }
}

fn try_injest_can_msg(image: &mut BusInputImage, frame: Frame, time: Wrapping<u32>) {
    let Id::Standard(cob_id) = frame.header().id() else {
        return;
    };
    if frame.header().len() != 8 {
        return;
    }
    let data = frame.data();

    let Ok(pd_id) = ProcessDataCanId::try_from(cob_id.as_raw()) else {
        return;
    };

    let node_id = pd_id.node_id;

    match pd_id.kind {
        PdMessageKind::Valves => {
            for (id, state) in valve_msg_to_valve(node_id as u16, data) {
                image.valve_state[id] = Some(DataWithTime::new(state, time));
            }
        }
        // TODO: add
        PdMessageKind::PwmUs
        | PdMessageKind::RawBus0a
        | PdMessageKind::RawBus1a
        | PdMessageKind::RawBus0b
        | PdMessageKind::RawBus1b => (),

        PdMessageKind::BinaryOutpus => {
            for (id, bool_state) in binary_output_msg_to_bo(node_id as u16, data) {
                image.binary_outputs[id] = Some(DataWithTime::new(bool_state, time));
            }
        }
        // FIXME: add
        PdMessageKind::Sensor0 | PdMessageKind::Sensor1 => (),
    }
}

// technically const
/// Convert between different CanMessage types
pub fn can_msg_to_frame(msg: &CanMessage) -> embassy_stm32::can::Frame {
    use embassy_stm32::can::frame::Header;
    use embedded_can::{ExtendedId, Id, StandardId};

    let id: embedded_can::Id = match msg.id() {
        CanId::Std(id) => Id::Standard(StandardId::new(id).unwrap()),
        CanId::Extended(id) => Id::Extended(ExtendedId::new(id).unwrap()),
    };

    let len = msg.dlc;
    let rtr = msg.rtr;
    let header = Header::new(id, len, rtr);

    embassy_stm32::can::Frame::new(header, msg.data()).unwrap()
}

/// Generate a message for writing to a data to an IO Board.
/// Following CanOpen sdo request initiate download.
pub fn sdo_write_msg(data: &heapless::Vec<u8, 4>, addr: &IoAddr) -> CanMessage {
    let n = data.len() as u8; // data size
    let e = true; // expedited transfer, single packet transfer only
    let s = true; // data size specified in n
    let sdo = SdoRequest::InitiateDownload {
        n,
        e,
        s,
        index: addr.index,
        sub: addr.subindex,
        data: [
            *data.first().unwrap_or(&0),
            *data.get(1).unwrap_or(&0),
            *data.get(2).unwrap_or(&0),
            *data.get(3).unwrap_or(&0),
        ],
    };
    let sdo_request_cob_id: u16 = 0x600;
    sdo.to_can_message(CanId::std(sdo_request_cob_id + u16::from(addr.node_id)))
}

// for now unused helper functions

pub fn set_servo_pwm_msg(micros: u16, servo: &IoAddr) -> CanMessage {
    let data: heapless::Vec<u8, 4> = heapless::Vec::from_slice(&micros.to_le_bytes()).unwrap();
    sdo_write_msg(&data, servo)
}

pub fn set_boolean_msg(state: bool, device: &IoAddr) -> CanMessage {
    let data: heapless::Vec<u8, 4> = heapless::Vec::from_slice(&[state as u8]).unwrap();
    sdo_write_msg(&data, device)
}
