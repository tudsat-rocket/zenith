use serde::{Deserialize, Serialize};

use mission::bus::PowerBoardReading;
use mission::inventory::{InventoryId, PowerBoardId};
use mission::mavlink::{VehicleSnapshot, battery_id, battery_status};
use rapid_dialect::rapid::enums::MavBatteryChargeState;
use rapid_dialect::rapid::messages::BatteryStatus;

use super::{ConnectionContext, DownlinkTelemetryMessage, FULL_SCALE, UNKNOWN};

/// 5.00 - 15.16 V at 40 mV, which covers a 3S pack from flat to full with room for a charger's
/// overshoot.
const VOLTAGE_OFFSET_MV: u16 = 5_000;
const VOLTAGE_MV_PER_CODE: u16 = 40;

/// Matches BATTERY_STATUS.current_battery, so nothing is lost on the way to MAVLink.
const CURRENT_MA_PER_CODE: i32 = 10;
/// -1 is a real reading at this scale, so unknown needs a code of its own.
const CURRENT_UNKNOWN: i16 = i16::MIN;

/// No reading from this board at all, as opposed to one whose charger state is undefined.
const BOARD_ABSENT: u8 = u8::MAX;

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
struct PackedBoard {
    voltage: u8,
    /// Positive while discharging.
    #[serde(with = "postcard::fixint::le")]
    current: i16,
    /// [`MavBatteryChargeState`], or [`BOARD_ABSENT`].
    charge_state: u8,
}

impl PackedBoard {
    #[allow(
        clippy::arithmetic_side_effects,
        reason = "saturating subtraction, nonzero constant divisors"
    )]
    fn pack(reading: Option<PowerBoardReading>) -> Self {
        let Some(reading) = reading else {
            return Self {
                voltage: UNKNOWN,
                current: CURRENT_UNKNOWN,
                charge_state: BOARD_ABSENT,
            };
        };

        let voltage = match reading.voltage_mv {
            Some(mv) => (mv.saturating_sub(VOLTAGE_OFFSET_MV) / VOLTAGE_MV_PER_CODE)
                .min(FULL_SCALE as u16) as u8,
            None => UNKNOWN,
        };

        // One below i16::MIN would be unknown, so the clamp starts above it.
        let current = match reading.current_ma {
            Some(ma) => (ma / CURRENT_MA_PER_CODE)
                .clamp(i32::from(CURRENT_UNKNOWN) + 1, i32::from(i16::MAX))
                as i16,
            None => CURRENT_UNKNOWN,
        };

        Self {
            voltage,
            current,
            charge_state: reading.charge_state as u8,
        }
    }

    #[allow(
        clippy::arithmetic_side_effects,
        reason = "bounded by the u8/i16 codes"
    )]
    fn unpack(self) -> Option<PowerBoardReading> {
        if self.charge_state == BOARD_ABSENT {
            return None;
        }

        Some(PowerBoardReading {
            voltage_mv: (self.voltage != UNKNOWN)
                .then(|| VOLTAGE_OFFSET_MV + u16::from(self.voltage) * VOLTAGE_MV_PER_CODE),
            current_ma: (self.current != CURRENT_UNKNOWN)
                .then(|| i32::from(self.current) * CURRENT_MA_PER_CODE),
            charge_state: MavBatteryChargeState::try_from(self.charge_state).unwrap_or_default(),
        })
    }
}

/// Every power board's pack, so the receiver can rebuild the same BATTERY_STATUS per board that
/// the wired links send.
#[derive(Debug, Serialize, Deserialize)]
pub struct BatteryMessage {
    /// Indexed by [`PowerBoardId::idx`].
    boards: [PackedBoard; 3],
}

impl DownlinkTelemetryMessage for BatteryMessage {
    const ID: u8 = 0x09;
    type Input<'a> = &'a VehicleSnapshot<'a>;
    /// One per [`PowerBoardId`], `None` for a board that is not reporting.
    type Output = [Option<BatteryStatus>; 3];

    fn pack(snapshot: Self::Input<'_>) -> Self {
        Self {
            boards: PowerBoardId::ALL
                .map(|board| PackedBoard::pack(snapshot.input_image.power_boards[board])),
        }
    }

    fn unpack(self, _context: &mut ConnectionContext) -> Self::Output {
        PowerBoardId::ALL.map(|board| {
            #[allow(
                clippy::indexing_slicing,
                reason = "idx() is 0..3 by the InventoryId contract"
            )]
            let reading = self.boards[board.idx()].unpack()?;
            Some(battery_status(
                battery_id(board),
                reading.voltage_mv,
                reading.current_ma,
                reading.charge_state,
            ))
        })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::super::tests::{SnapshotParts, through_packet};
    use super::*;

    use mission::bus::BusInputImage;

    use crate::messages::DownlinkMessage;

    /// Every board reporting at the top of its range, for the payload-length check.
    pub(crate) fn saturate_power_boards(inputs: &mut BusInputImage) {
        for board in PowerBoardId::ALL {
            inputs.power_boards[board] = Some(PowerBoardReading {
                voltage_mv: Some(u16::MAX),
                current_ma: Some(i32::MAX),
                charge_state: MavBatteryChargeState::Charging,
            });
        }
    }

    fn round_trip(parts: &SnapshotParts) -> [Option<BatteryStatus>; 3] {
        let msg = DownlinkMessage::Battery(BatteryMessage::pack(&parts.snapshot()));
        let DownlinkMessage::Battery(decoded) = through_packet(msg) else {
            panic!("decoded as the wrong message")
        };
        decoded.unpack(&mut ConnectionContext::init(0))
    }

    #[test]
    fn each_board_survives_the_packet() {
        let mut parts = SnapshotParts::default();
        parts.inputs.power_boards[PowerBoardId::Board1] = Some(PowerBoardReading {
            voltage_mv: Some(12_150),
            current_ma: Some(2_400),
            charge_state: MavBatteryChargeState::Ok,
        });
        parts.inputs.power_boards[PowerBoardId::Board3] = Some(PowerBoardReading {
            voltage_mv: Some(11_020),
            current_ma: Some(-180),
            charge_state: MavBatteryChargeState::Charging,
        });

        let [one, two, three] = round_trip(&parts);
        let one = one.expect("board 1 reported");
        let three = three.expect("board 3 reported");

        assert!(two.is_none(), "board 2 never reported");

        assert_eq!(one.id, 1);
        assert!(one.voltages[0].abs_diff(12_150) < VOLTAGE_MV_PER_CODE);
        assert_eq!(one.current_battery, 240);
        assert_eq!(one.charge_state, MavBatteryChargeState::Ok);

        assert_eq!(three.id, 3);
        assert!(three.voltages[0].abs_diff(11_020) < VOLTAGE_MV_PER_CODE);
        assert_eq!(three.current_battery, -18);
        assert_eq!(three.charge_state, MavBatteryChargeState::Charging);
    }

    /// A board that is on the bus but has not sent every frame yet must say so field by field,
    /// rather than come back as a 0 A, 8 V pack.
    #[test]
    fn missing_fields_stay_missing() {
        let mut parts = SnapshotParts::default();
        parts.inputs.power_boards[PowerBoardId::Board2] = Some(PowerBoardReading::default());

        let [_, status, _] = round_trip(&parts);
        let status = status.expect("board 2 is present");

        assert_eq!(status.voltages[0], u16::MAX);
        assert_eq!(status.current_battery, -1);
        assert_eq!(status.charge_state, MavBatteryChargeState::Undefined);
    }

    /// Out-of-range readings saturate instead of wrapping into a plausible value, and in
    /// particular never land on the unknown codes.
    #[test]
    fn extremes_saturate() {
        for (mv, ma) in [(0, i32::MIN), (u16::MAX, i32::MAX)] {
            let packed = PackedBoard::pack(Some(PowerBoardReading {
                voltage_mv: Some(mv),
                current_ma: Some(ma),
                charge_state: MavBatteryChargeState::Ok,
            }));
            assert_ne!(packed.voltage, UNKNOWN, "{mv} mV");
            assert_ne!(packed.current, CURRENT_UNKNOWN, "{ma} mA");
        }
    }
}
