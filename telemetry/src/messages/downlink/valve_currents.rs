use serde::{Deserialize, Serialize};

use mission::inventory::{InventoryId, ValveId, ValveMap};
use mission::mavlink::VehicleSnapshot;

use super::{ConnectionContext, DownlinkTelemetryMessage, FULL_SCALE, UNKNOWN};

/// 0 - 5.08 A, saturating.
const CURRENT_MA_PER_CODE: u16 = 20;

/// Valve drive currents. There is no MAVLink message of their own to turn these into, so unpacking
/// only stores them in the [`ConnectionContext`], and they go out with the VALVE messages rebuilt
/// from the next [`ComponentsMessage`](super::ComponentsMessage).
#[derive(Debug, Serialize, Deserialize)]
pub struct ValveCurrentsMessage {
    /// One code per [`ValveId`], or [`UNKNOWN`].
    currents: [u8; ValveId::ALL.len()],
}

impl DownlinkTelemetryMessage for ValveCurrentsMessage {
    const ID: u8 = 0x0A;
    type Input<'a> = &'a VehicleSnapshot<'a>;
    type Output = ();

    fn pack(snapshot: Self::Input<'_>) -> Self {
        Self {
            currents: ValveId::ALL.map(|valve| {
                snapshot.input_image.valve_current[valve].map_or(UNKNOWN, |d| {
                    #[allow(clippy::arithmetic_side_effects, reason = "nonzero constant divisor")]
                    let code = d.data / CURRENT_MA_PER_CODE;
                    code.min(FULL_SCALE as u16) as u8
                })
            }),
        }
    }

    fn unpack(self, context: &mut ConnectionContext) -> Self::Output {
        context.valve_currents = ValveMap::from_fn(|valve| {
            #[allow(
                clippy::indexing_slicing,
                reason = "the array is sized by ValveId::ALL, which idx() indexes"
            )]
            let code = self.currents[valve.idx()];
            #[allow(
                clippy::arithmetic_side_effects,
                reason = "at most 254 * 20, well within u16"
            )]
            (code != UNKNOWN).then(|| u16::from(code) * CURRENT_MA_PER_CODE)
        });
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::super::tests::{SnapshotParts, through_packet};
    use super::*;

    use core::num::Wrapping;

    use mission::bus::{BusInputImage, DataWithTime};

    use crate::messages::DownlinkMessage;

    /// Every valve reporting the largest current there is, for the payload-length check.
    pub(crate) fn saturate_valve_currents(inputs: &mut BusInputImage) {
        inputs.valve_current = ValveMap::splat(Some(DataWithTime {
            data: u16::MAX,
            time: Wrapping(0),
        }));
    }

    fn round_trip(parts: &SnapshotParts) -> ValveMap<Option<u16>> {
        let msg = DownlinkMessage::ValveCurrents(ValveCurrentsMessage::pack(&parts.snapshot()));
        let DownlinkMessage::ValveCurrents(decoded) = through_packet(msg) else {
            panic!("decoded as the wrong message")
        };
        let mut context = ConnectionContext::init(0);
        decoded.unpack(&mut context);
        context.valve_currents
    }

    #[test]
    fn currents_survive_the_packet() {
        let reading = |ma| {
            Some(DataWithTime {
                data: ma,
                time: Wrapping(0),
            })
        };
        let mut parts = SnapshotParts::default();
        parts.inputs.valve_current[ValveId::Main] = reading(1_234);
        parts.inputs.valve_current[ValveId::OxidizerVent] = reading(0);
        parts.inputs.valve_current[ValveId::PressurantVent] = reading(9_000);

        let currents = round_trip(&parts);

        let main = currents[ValveId::Main].expect("main reported");
        assert!(main.abs_diff(1_234) < CURRENT_MA_PER_CODE, "{main}");
        assert_eq!(currents[ValveId::OxidizerVent], Some(0));
        // Saturates instead of wrapping or turning into unknown.
        assert_eq!(currents[ValveId::PressurantVent], Some(5_080));
        assert_eq!(currents[ValveId::OxidizerFill], None);
    }
}
