use core::num::Wrapping;

use serde::{Deserialize, Serialize};

use mission::bus::{NodeSet, ValveState};
use mission::inventory::{InventoryId, ServoId, ServoMap, ValveId, valve_is_heated};
use mission::mavlink::{VehicleSnapshot, centi_celsius, servo_output_raw, valve_heater_flags};
use mission::valves::ValveController;
use rapid_dialect::rapid::enums::ValveFlag;
use rapid_dialect::rapid::messages::{ServoOutputRaw, Valve};

use super::{ConnectionContext, DownlinkTelemetryMessage, pack_temperature, unpack_temperature};

// `ComponentsMessage` only has room for one valve temperature and one heater bit.
#[allow(
    clippy::indexing_slicing,
    clippy::arithmetic_side_effects,
    reason = "const-evaluated walk over a fixed-length array"
)]
const _: () = {
    let mut heated = 0;
    let mut i = 0;
    while i < ValveId::ALL.len() {
        if valve_is_heated(ValveId::ALL[i]) {
            heated += 1;
        }
        i += 1;
    }
    assert!(heated <= 1);
};

const _: () = assert!(
    ValveCode::HEATER_SHIFT < u32::BITS as usize,
    "valve codes, servo codes and the heater bit no longer fit one word"
);

/// Where each servo's code goes in `commanded`: after the valves, one [`ValveCode::BITS`] slot
/// each.
#[allow(
    clippy::arithmetic_side_effects,
    reason = "idx() is 0..4 by the InventoryId contract"
)]
fn servo_slot(servo: ServoId) -> usize {
    ValveId::ALL.len() + servo.idx()
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
enum ValveCode {
    Closed = 0b00,
    Open = 0b01,
    Partial = 0b10,
    /// No reading on the bus for the measured state, a released servo for the commanded one.
    Unknown = 0b11,
}

impl ValveCode {
    const BITS: usize = 2;
    const MASK: u32 = 0b11;

    /// Bits above the valve codes.
    #[allow(clippy::arithmetic_side_effects, reason = "constant 2 * 9")]
    const SPARE_SHIFT: usize = Self::BITS * ValveId::ALL.len();

    /// Bit above the servo codes.
    #[allow(clippy::arithmetic_side_effects, reason = "constant 2 * (9 + 4)")]
    const HEATER_SHIFT: usize = Self::BITS * (ValveId::ALL.len() + ServoId::ALL.len());

    fn from_state(state: Option<ValveState>) -> Self {
        match state.map(|s| s.promille()) {
            None => Self::Unknown,
            Some(0) => Self::Closed,
            Some(1000) => Self::Open,
            Some(_) => Self::Partial,
        }
    }

    /// Each code at `BITS * slot`.
    #[allow(
        clippy::arithmetic_side_effects,
        reason = "slots are valve idx() or servo_slot(), which the const assert keeps below 32"
    )]
    fn pack(codes: impl IntoIterator<Item = (usize, Self)>) -> u32 {
        codes.into_iter().fold(0, |bits, (slot, code)| {
            bits | (code as u32) << (Self::BITS * slot)
        })
    }

    #[allow(clippy::arithmetic_side_effects, reason = "same bound as `pack`")]
    fn unpack_one(bits: u32, slot: usize) -> Self {
        match (bits >> (Self::BITS * slot)) & Self::MASK {
            0b00 => Self::Closed,
            0b01 => Self::Open,
            0b10 => Self::Partial,
            _ => Self::Unknown,
        }
    }

    fn position(self) -> f32 {
        match self {
            Self::Closed => 0.0,
            Self::Open => 1.0,
            Self::Partial => 0.5,
            Self::Unknown => f32::NAN,
        }
    }

    /// `None` is a released servo.
    fn commanded_state(self) -> Option<ValveState> {
        match self {
            Self::Closed => Some(ValveState::fully_closed()),
            Self::Open => Some(ValveState::fully_open()),
            Self::Partial => Some(ValveState::from_promille_clamped(500)),
            Self::Unknown => None,
        }
    }
}

/// Valve and node state, with some room for other actuators
#[derive(Debug, Serialize, Deserialize)]
pub struct ComponentsMessage {
    /// One [`ValveCode`] per valve, as the bus reports it, then the temperature code of the
    /// heated valve.
    #[serde(with = "postcard::fixint::le")]
    reported: u32,
    /// One [`ValveCode`] per valve and servo, as the vehicle last resolved it, then the heater
    /// bit of the heated valve.
    #[serde(with = "postcard::fixint::le")]
    commanded: u32,
    /// Bit n is node id n.
    #[serde(with = "postcard::fixint::le")]
    node_presence: u16,
    /// The armed subset of those.
    #[serde(with = "postcard::fixint::le")]
    node_armed: u16,
}

impl DownlinkTelemetryMessage for ComponentsMessage {
    const ID: u8 = 0x04;
    type Input<'a> = &'a VehicleSnapshot<'a>;
    /// One per [`ValveId`], the commanded servos, then the nodes on the bus and the armed subset
    /// of them.
    type Output = ([Valve; 9], ServoOutputRaw, NodeSet, NodeSet);

    fn pack(snapshot: Self::Input<'_>) -> Self {
        let mut heater = false;
        let mut temperature = None;
        for valve in ValveId::ALL {
            if let Some((heater_on, celsius)) = snapshot.valve_heater(valve) {
                heater = heater_on;
                temperature = celsius;
            }
        }

        let reported = ValveCode::pack(ValveId::ALL.map(|valve| {
            let state = snapshot.input_image.valve_state[valve].map(|s| s.data);
            (valve.idx(), ValveCode::from_state(state))
        }));
        let commanded_valves = ValveCode::pack(ValveId::ALL.map(|valve| {
            let state = Some(snapshot.output_image.valve[valve]);
            (valve.idx(), ValveCode::from_state(state))
        }));
        let commanded_servos = ValveCode::pack(ServoId::ALL.map(|servo| {
            let state = snapshot.output_image.servo[servo];
            (servo_slot(servo), ValveCode::from_state(state))
        }));

        Self {
            reported: reported | u32::from(pack_temperature(temperature)) << ValveCode::SPARE_SHIFT,
            commanded: commanded_valves
                | commanded_servos
                | u32::from(heater) << ValveCode::HEATER_SHIFT,
            node_presence: snapshot.input_image.nodes.bits(),
            node_armed: snapshot.input_image.nodes_armed.bits(),
        }
    }

    fn unpack(self, context: &mut ConnectionContext) -> Self::Output {
        #[allow(
            clippy::cast_possible_truncation,
            reason = "the code is the byte above the valves"
        )]
        let temperature = unpack_temperature((self.reported >> ValveCode::SPARE_SHIFT) as u8);
        let heater_on = self.commanded >> ValveCode::HEATER_SHIFT & 1 != 0;

        let valves = ValveId::ALL.map(|valve| {
            let (mut flags, temperature) = if valve_is_heated(valve) {
                (valve_heater_flags(heater_on), centi_celsius(temperature))
            } else {
                (ValveFlag::empty(), i16::MAX)
            };
            flags.set(
                ValveFlag::COMMANDABLE,
                context
                    .mode
                    .is_some_and(|mode| ValveController::manual_valve_allowed(mode, valve)),
            );
            Valve {
                id: valve,
                state: ValveCode::unpack_one(self.reported, valve.idx()).position(),
                commanded: ValveCode::unpack_one(self.commanded, valve.idx()).position(),
                flags,
                temperature,
                drive_current: context.valve_currents[valve].unwrap_or(u16::MAX),
            }
        });

        let servos = ServoMap::from_fn(|servo| {
            ValveCode::unpack_one(self.commanded, servo_slot(servo)).commanded_state()
        });

        (
            valves,
            servo_output_raw(Wrapping(context.time), &servos),
            NodeSet::from_bits(self.node_presence),
            NodeSet::from_bits(self.node_armed),
        )
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::super::tests::{SnapshotParts, through_packet};
    use super::*;

    use mission::bus::{BusOutputImage, DataWithTime};
    use rapid_dialect::FlightMode;

    use crate::messages::DownlinkMessage;

    /// Every valve commanded open, for the payload-length check.
    pub(crate) fn saturated_outputs() -> BusOutputImage {
        let mut outputs = BusOutputImage::default();
        for valve in ValveId::ALL {
            outputs.valve[valve] = ValveState::fully_open();
        }
        for servo in ServoId::ALL {
            outputs.servo[servo] = Some(ValveState::fully_open());
        }
        outputs
    }

    #[test]
    fn valve_states_survive_the_packet() {
        let mut parts = SnapshotParts::default();

        parts.inputs.valve_state[ValveId::OxidizerVent] =
            Some(DataWithTime::new(ValveState::fully_open(), Wrapping(0)));
        parts.inputs.valve_state[ValveId::OxidizerFill] =
            Some(DataWithTime::new(ValveState::fully_closed(), Wrapping(0)));
        parts.inputs.valve_state[ValveId::Main] = Some(DataWithTime::new(
            ValveState::from_percent_open(40),
            Wrapping(0),
        ));
        // PressurantVent deliberately left unreported.

        parts.outputs.valve[ValveId::OxidizerVent] = ValveState::fully_open();
        parts.outputs.valve[ValveId::Main] = ValveState::fully_closed();

        let msg = DownlinkMessage::Components(ComponentsMessage::pack(&parts.snapshot()));
        let DownlinkMessage::Components(decoded) = through_packet(msg) else {
            panic!("decoded as the wrong message")
        };
        let (valves, ..) = decoded.unpack(&mut ConnectionContext::init(0));

        let by_id = |id: ValveId| &valves[id.idx()];

        assert_eq!(by_id(ValveId::OxidizerVent).state, 1.0);
        assert_eq!(by_id(ValveId::OxidizerVent).commanded, 1.0);
        assert_eq!(by_id(ValveId::OxidizerFill).state, 0.0);
        assert_eq!(by_id(ValveId::Main).state, 0.5);
        assert_eq!(by_id(ValveId::Main).commanded, 0.0);
        assert!(by_id(ValveId::PressurantVent).state.is_nan());

        // Every valve gets its own slot, and the ids come back in order.
        for valve in ValveId::ALL {
            assert_eq!(by_id(valve).id, valve);
        }
    }

    /// The heater bit and temperature share words with the valve and servo codes, so none may
    /// disturb the others.
    #[test]
    fn heater_survives_the_packet() {
        for mode in [FlightMode::Idle, FlightMode::FillOxidizer] {
            let mut parts = SnapshotParts {
                mode,
                ..SnapshotParts::default()
            };
            parts.outputs = saturated_outputs();

            let msg = DownlinkMessage::Components(ComponentsMessage::pack(&parts.snapshot()));
            let DownlinkMessage::Components(decoded) = through_packet(msg) else {
                panic!("decoded as the wrong message")
            };
            let (valves, servos, ..) = decoded.unpack(&mut ConnectionContext::init(0));

            assert_eq!(
                [
                    servos.servo1_raw,
                    servos.servo2_raw,
                    servos.servo3_raw,
                    servos.servo4_raw
                ],
                [2000; 4]
            );
            for valve in &valves {
                assert_eq!(valve.commanded, 1.0, "{:?}", valve.id);
                assert!(valve.state.is_nan(), "{:?}", valve.id);
                if let Some((heater_on, celsius)) = parts.snapshot().valve_heater(valve.id) {
                    assert_eq!(valve.flags, valve_heater_flags(heater_on));
                    let expected = centi_celsius(celsius);
                    assert!(
                        (valve.temperature - expected).abs() <= 50,
                        "{}",
                        valve.temperature
                    );
                } else {
                    assert_eq!(valve.flags, ValveFlag::empty(), "{:?}", valve.id);
                    assert_eq!(valve.temperature, i16::MAX, "{:?}", valve.id);
                }
            }
        }
    }

    /// Commandability is not in the packet; the receiver derives it from the last heartbeat's mode.
    #[test]
    fn commandable_follows_the_received_mode() {
        let parts = SnapshotParts::default();
        let msg = ComponentsMessage::pack(&parts.snapshot());

        let mut context = ConnectionContext::init(0);
        let (valves, ..) = msg.unpack(&mut context);
        assert!(
            valves
                .iter()
                .all(|v| !v.flags.contains(ValveFlag::COMMANDABLE))
        );

        for mode in (0u8..).map_while(|m| FlightMode::try_from(m).ok()) {
            context.mode = Some(mode);
            let msg = ComponentsMessage::pack(&parts.snapshot());
            let (valves, ..) = msg.unpack(&mut context);
            for valve in &valves {
                assert_eq!(
                    valve.flags.contains(ValveFlag::COMMANDABLE),
                    ValveController::manual_valve_allowed(mode, valve.id),
                    "{mode:?} {:?}",
                    valve.id
                );
            }
        }
    }

    /// The commanded side is always determined, so it must never come back unknown.
    #[test]
    fn commanded_positions_are_never_unknown() {
        let parts = SnapshotParts::default();
        let (valves, ..) =
            ComponentsMessage::pack(&parts.snapshot()).unpack(&mut ConnectionContext::init(0));

        for valve in &valves {
            assert!(!valve.commanded.is_nan(), "{:?}", valve.id);
            assert!(valve.state.is_nan(), "{:?} was never reported", valve.id);
        }
    }

    /// Separate words, and node 8 is what a byte-wide field would have dropped.
    #[test]
    fn node_presence_and_arming_survive_the_packet() {
        const PRESENT: [u8; 3] = [2, 8, 15];
        const ARMED: [u8; 2] = [8, 15];

        let mut parts = SnapshotParts::default();

        for node_id in PRESENT {
            parts.inputs.nodes.set(node_id, true);
        }
        for node_id in ARMED {
            parts.inputs.nodes_armed.set(node_id, true);
        }

        let msg = DownlinkMessage::Components(ComponentsMessage::pack(&parts.snapshot()));
        let DownlinkMessage::Components(decoded) = through_packet(msg) else {
            panic!("decoded as the wrong message")
        };
        let (_, _, nodes, armed) = decoded.unpack(&mut ConnectionContext::init(0));

        for node_id in 0..16 {
            assert_eq!(
                nodes.contains(node_id),
                PRESENT.contains(&node_id),
                "node {node_id} presence"
            );
            assert_eq!(
                armed.contains(node_id),
                ARMED.contains(&node_id),
                "node {node_id} arming"
            );
        }
    }

    #[test]
    fn commanded_servos_survive_the_packet() {
        let mut parts = SnapshotParts::default();
        parts.outputs.servo[ServoId::OxidizerDisconnect] = Some(ValveState::fully_open());
        parts.outputs.servo[ServoId::OxidizerRetract] = Some(ValveState::from_percent_open(20));

        let msg = DownlinkMessage::Components(ComponentsMessage::pack(&parts.snapshot()));
        let DownlinkMessage::Components(decoded) = through_packet(msg) else {
            panic!("decoded as the wrong message")
        };
        let (_, servos, ..) = decoded.unpack(&mut ConnectionContext::init(0));

        // The servos left alone are released, which SERVO_OUTPUT_RAW reports as 0.
        assert_eq!(
            [
                servos.servo1_raw,
                servos.servo2_raw,
                servos.servo3_raw,
                servos.servo4_raw
            ],
            [0, 2000, 0, 1500]
        );
    }

    /// Codes must not bleed into their neighbours: nine valves and four servos share a word.
    #[test]
    fn codes_do_not_overlap() {
        let slots = || {
            ValveId::ALL
                .into_iter()
                .map(|valve| valve.idx())
                .chain(ServoId::ALL.into_iter().map(servo_slot))
        };

        for subject in slots() {
            for code in [
                ValveCode::Closed,
                ValveCode::Open,
                ValveCode::Partial,
                ValveCode::Unknown,
            ] {
                let at = |slot| {
                    if slot == subject {
                        code
                    } else {
                        ValveCode::Closed
                    }
                };
                let bits = ValveCode::pack(slots().map(|slot| (slot, at(slot))));

                for other in slots() {
                    assert_eq!(
                        ValveCode::unpack_one(bits, other),
                        at(other),
                        "{code:?} in slot {subject} leaked into slot {other}"
                    );
                }
            }
        }
    }
}
