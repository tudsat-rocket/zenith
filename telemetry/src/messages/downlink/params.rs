use serde::{Deserialize, Serialize};

use mission::params::Params;
use rapid_dialect::rapid::messages::ParamValue;

use super::{ConnectionContext, DownlinkTelemetryMessage};

/// One parameter's stable id and raw value bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ParamEntry {
    #[serde(with = "postcard::fixint::le")]
    pub id: u16,
    #[serde(with = "postcard::fixint::le")]
    pub raw: u32,
}

/// Values of up to two parameters
///
/// The receiver recovers one PARAM_VALUE per distinct entry, filling in name, type, index and
/// count from its own copy of the parameter definitions. A single value is sent in both entries.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ParamValuesMessage {
    entries: [ParamEntry; 2],
}

impl DownlinkTelemetryMessage for ParamValuesMessage {
    const ID: u8 = 0x08;
    type Input<'a> = [ParamEntry; 2];
    type Output = heapless::Vec<ParamValue, 2>;

    fn pack(entries: Self::Input<'_>) -> Self {
        Self { entries }
    }

    fn unpack(self, _context: &mut ConnectionContext) -> Self::Output {
        let [first, second] = self.entries;
        let distinct = if first == second {
            &[first][..]
        } else {
            &[first, second][..]
        };

        distinct
            .iter()
            .filter_map(|entry| {
                let (index, descriptor) = Params::by_id(entry.id)?;
                let value = descriptor.ty.decode_raw(entry.raw);
                Some(ParamValue::from(&Params::info(index, &descriptor, value)))
            })
            .collect()
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    use crate::messages::DownlinkMessage;
    use crate::messages::downlink::tests::through_packet;

    fn unpacked(entries: [ParamEntry; 2]) -> heapless::Vec<ParamValue, 2> {
        let DownlinkMessage::ParamValues(msg) = through_packet(DownlinkMessage::ParamValues(
            ParamValuesMessage::pack(entries),
        )) else {
            panic!("decoded as the wrong message")
        };
        msg.unpack(&mut ConnectionContext::init(0))
    }

    #[test]
    fn values_come_back_with_their_metadata() {
        let (index, descriptor) = Params::by_id(0x0201).expect("SM_MIN_T_APOGEE exists");
        let entry = ParamEntry {
            id: 0x0201,
            raw: 0xdead_beef,
        };

        let values = unpacked([entry, entry]);

        assert_eq!(values.len(), 1, "a repeated entry is one value");
        let value = &values[0];
        assert_eq!(value.param_id, descriptor.mavlink_name());
        assert_eq!(value.param_value.to_bits(), 0xdead_beef);
        assert_eq!(value.param_type, descriptor.ty.into());
        assert_eq!(value.param_index, index);
        assert_eq!(value.param_count, Params::count());
    }

    #[test]
    fn unknown_ids_are_dropped() {
        let known = ParamEntry {
            id: 0x0200,
            raw: 1.0f32.to_bits(),
        };
        let unknown = ParamEntry { id: 0xffff, raw: 0 };

        let values = unpacked([unknown, known]);

        assert_eq!(values.len(), 1);
        assert_eq!(values[0].param_value, 1.0);
    }
}
