use core::{cmp::*, num::Wrapping};
use embassy_stm32::can::Frame;
use embassy_stm32::time;
use embassy_time::{Duration, Instant};
use embedded_can::Id;
use heapless::Vec;
use mission::foreign_io::{
    DataWithTime, ForeignDataError, ForeignInputImage, ForeignIo, ForeignOutputImage, IoAddr,
    ValveState,
};
use mission::propulsion::{
    ALL_BINARY_OUTPUTS, ALL_PRESS_SENS, ALL_TEMP_SENS, ALL_VALVES, BinaryOutputId, PressSensId,
    TempSensId,
};
use num_traits::float::Float;
use rapid_dialect::rapid::enums::{ValveId, valve_id};
use zencan_common::{CanId, CanMessage, sdo::SdoRequest};

use crate::can::{CanRxSubscriber, CanTxPublisher};
use crate::foreign::mapping::{BINARY_OUTPUT_ID_MAP, VALVE_ID_MAP};
use crate::foreign::pdo_mapping::{
    PdMessageKind, ProcessDataCanId, binary_output_msg_to_bo, valve_msg_to_valve,
};

mod mapping;
mod pdo_mapping;

pub struct FIoHandler {
    pub input: ForeignInputImage,
    pub output_current: ForeignOutputImage,
    pub output_next: ForeignOutputImage,
    pub can: (CanTxPublisher, CanRxSubscriber),
}
impl FIoHandler {
    pub fn new(can_tx_pub: CanTxPublisher, can_rx_sub: CanRxSubscriber) -> Self {
        Self {
            input: ForeignInputImage::default(),
            output_current: ForeignOutputImage::default(),
            output_next: ForeignOutputImage::default(),
            can: (can_tx_pub, can_rx_sub),
        }
    }
}

pub const VERY_FRESH_DURATION: Duration = Duration::from_millis(50);

fn very_fresh<T: Copy>(value: Option<DataWithTime<T>>) -> Option<T> {
    let value = value?;
    let stale = Instant::from_millis(value.time.0.into()).elapsed() < VERY_FRESH_DURATION;
    if stale {
        return None;
    }
    Some(value.data)
}

fn try_injest_can_msg(image: &mut ForeignInputImage, frame: Frame, time: Wrapping<u32>) {
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
                image.valve_state[id as usize] = Some(DataWithTime::new(state, time));
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
                image.binary_outputs[id as usize] = Some(DataWithTime::new(bool_state, time));
            }
        }
        // FIXME: add
        PdMessageKind::Sensor0 | PdMessageKind::Sensor1 => (),
    }
}

impl ForeignIo for FIoHandler {
    fn tick(&mut self) {
        while let Some(msg) = self.can.1.try_next_message_pure() {
            try_injest_can_msg(
                &mut self.input,
                msg,
                Wrapping(Instant::now().as_millis() as u32),
            );
        }
        // NOTE: we would definetey would need a timeout for repeatedly sending these messages

        // for valve in &ALL_VALVES {
        //     let target_state = self.output_next.get_valve(*valve);
        //
        //     let fresh_state = very_fresh(*self.input.valve_state(*valve));
        //     // TODO: fix already correct
        //     let alread_correct = fresh_state.is_some_and(|inp_s| inp_s == target_state);
        //     if !alread_correct {
        //         let io_addr = VALVE_ID_MAP.get_io_addr(*valve);
        //         // let pwm = valve_state_to_servo_us(target_state, *valve);
        //         let msg = set_foreign_data_msg(
        //             &heapless::Vec::from_slice(&(target_state.promille().to_le_bytes())).unwrap(),
        //             io_addr,
        //         );
        //         let frame = can_msg_to_frame(&msg);
        //         defmt::warn!("try publish");
        //         //let _ = self.can.0.try_publish(frame).unwrap();
        //     }
        // }
        for output in &ALL_BINARY_OUTPUTS {
            let has_changed = self.output_next.binary_outpus()[*output as usize]
                != self.output_current.binary_outpus()[*output as usize];
            if has_changed {
                let target_state = self.output_next.binary_outpus()[*output as usize];
                let io_addr = &BINARY_OUTPUT_ID_MAP.0[*output as usize];
                let msg = set_boolean_msg(target_state, io_addr);
                let frame = can_msg_to_frame(&msg);
                self.can.0.try_publish(frame).expect("can_bus_queue_full");
            }
        }
        for valve_id in &ALL_VALVES {
            let has_changed =
                self.output_next.get_valve(*valve_id) != self.output_current.get_valve(*valve_id);
            if has_changed {
                let target_state = self.output_next.get_valve(*valve_id);
                let io_addr = VALVE_ID_MAP.get_io_addr(*valve_id);

                let msg = set_foreign_data_msg(
                    &heapless::Vec::from_slice(&(target_state.promille().to_le_bytes())).unwrap(),
                    io_addr,
                );
                let frame = can_msg_to_frame(&msg);
                defmt::warn!("try publish");
                self.can.0.try_publish(frame).expect("can_bus_queue_full");
            }
        }

        self.output_current = self.output_next;
    }
    fn get_input_image(&mut self) -> ForeignInputImage {
        self.input.clone()
    }
    fn set_output_image(&mut self, outputs: ForeignOutputImage) {
        self.output_next = outputs;
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

    let frame = embassy_stm32::can::Frame::new(header, msg.data()).unwrap();
    frame
}
/// Generate a message for writing to a data to an IO Board.
/// Following CanOpen sdo request initiate download.
pub fn set_foreign_data_msg(data: &heapless::Vec<u8, 4>, foreign: &IoAddr) -> CanMessage {
    let addr = foreign;

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
    set_foreign_data_msg(&data, servo)
}
pub fn set_boolean_msg(state: bool, device: &IoAddr) -> CanMessage {
    let data: heapless::Vec<u8, 4> = heapless::Vec::from_slice(&[state as u8]).unwrap();
    set_foreign_data_msg(&data, device)
}
