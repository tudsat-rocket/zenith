use serde::{Deserialize, Serialize};

use mission::bus::{NodeSet, ValveState};
use mission::inventory::{InventoryId, ValveId};
use mission::mavlink::VehicleSnapshot;
use rapid_dialect::rapid::messages::Valve;

use super::{ConnectionContext, DownlinkTelemetryMessage};

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
enum ValveCode {
    Closed = 0b00,
    Open = 0b01,
    Partial = 0b10,
    /// No reading on the bus. Only reachable for the measured state.
    Unknown = 0b11,
}

impl ValveCode {
    const BITS: usize = 2;
    const MASK: u32 = 0b11;

    fn from_state(state: Option<ValveState>) -> Self {
        match state.map(|s| s.promille()) {
            None => Self::Unknown,
            Some(0) => Self::Closed,
            Some(1000) => Self::Open,
            Some(_) => Self::Partial,
        }
    }

    /// One code per valve, at `BITS * idx()`, so eighteen of the thirty-two bits are used.
    #[allow(
        clippy::arithmetic_side_effects,
        reason = "idx() is 0..9 by the InventoryId contract, so the shift stays below 32"
    )]
    fn pack_all(mut f: impl FnMut(ValveId) -> Self) -> u32 {
        let mut bits = 0;
        for valve in ValveId::ALL {
            bits |= (f(valve) as u32) << (Self::BITS * valve.idx());
        }
        bits
    }

    #[allow(clippy::arithmetic_side_effects, reason = "same bound as `pack_all`")]
    fn unpack_one(bits: u32, valve: ValveId) -> Self {
        match (bits >> (Self::BITS * valve.idx())) & Self::MASK {
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
}

/// Valve and node state, with some room for other actuators
#[derive(Debug, Serialize, Deserialize)]
pub struct ComponentsMessage {
    /// One [`ValveCode`] per valve, as the bus reports it.
    #[serde(with = "postcard::fixint::le")]
    reported: u32,
    /// One [`ValveCode`] per valve, as the vehicle last resolved it.
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
    /// One per [`ValveId`], then the nodes on the bus and the armed subset of them.
    type Output = ([Valve; 9], NodeSet, NodeSet);

    fn pack(snapshot: Self::Input<'_>) -> Self {
        Self {
            reported: ValveCode::pack_all(|valve| {
                ValveCode::from_state(snapshot.input_image.valve_state[valve].map(|s| s.data))
            }),
            commanded: ValveCode::pack_all(|valve| {
                ValveCode::from_state(Some(snapshot.output_image.valve[valve]))
            }),
            node_presence: snapshot.input_image.nodes.bits(),
            node_armed: snapshot.input_image.nodes_armed.bits(),
        }
    }

    fn unpack(self, _context: &mut ConnectionContext) -> Self::Output {
        let valves = ValveId::ALL.map(|valve| Valve {
            id: valve,
            state: ValveCode::unpack_one(self.reported, valve).position(),
            commanded: ValveCode::unpack_one(self.commanded, valve).position(),
        });

        (
            valves,
            NodeSet::from_bits(self.node_presence),
            NodeSet::from_bits(self.node_armed),
        )
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::super::tests::{SnapshotParts, through_packet};
    use super::*;

    use core::num::Wrapping;

    use mission::bus::{BusOutputImage, DataWithTime};

    use crate::messages::DownlinkMessage;

    /// Every valve commanded open, for the payload-length check.
    pub(crate) fn saturated_outputs() -> BusOutputImage {
        let mut outputs = BusOutputImage::default();
        for valve in ValveId::ALL {
            outputs.valve[valve] = ValveState::fully_open();
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
        let (_, nodes, armed) = decoded.unpack(&mut ConnectionContext::init(0));

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

    /// Codes must not bleed into their neighbours: nine of them share one word.
    #[test]
    fn valve_codes_do_not_overlap() {
        for subject in ValveId::ALL {
            for code in [
                ValveCode::Closed,
                ValveCode::Open,
                ValveCode::Partial,
                ValveCode::Unknown,
            ] {
                let bits = ValveCode::pack_all(|valve| {
                    if valve == subject {
                        code
                    } else {
                        ValveCode::Closed
                    }
                });

                for other in ValveId::ALL {
                    let expected = if other == subject {
                        code
                    } else {
                        ValveCode::Closed
                    };
                    assert_eq!(
                        ValveCode::unpack_one(bits, other),
                        expected,
                        "{code:?} on {subject:?} leaked into {other:?}"
                    );
                }
            }
        }
    }
}
