use serde::{Deserialize, Serialize};

use mission::inventory::{InventoryId, PressSensId, TankId, TempSensId};
use mission::mavlink::VehicleSnapshot;
use rapid_dialect::rapid::messages::PressureVessel;

use super::{
    ConnectionContext, DownlinkTelemetryMessage, pack_full_scale, pack_temperature,
    unpack_full_scale, unpack_temperature,
};

/// Tank pressures, temperatures, and levels
///
/// Each pressure is scaled against its own sensor's full scale ([`PressSensId::full_scale_bar`]).
///
/// Unpacks into the six PRESSURE_VESSEL messages.
#[derive(Debug, Serialize, Deserialize)]
pub struct PressuresMessage {
    /// In [`PressSensId`] order, each as a fraction of that sensor's full scale.
    pressures: [u8; 9],
    /// In [`TempSensId`] order, -60.0 - +67.0 C in 0.5 C.
    temperatures: [u8; 2],
    /// Oxidizer tank fill level, 0 - 1 of the tank.
    ox_tank_level: u8,
}

impl DownlinkTelemetryMessage for PressuresMessage {
    const ID: u8 = 0x03;
    type Input<'a> = &'a VehicleSnapshot<'a>;
    /// One per [`TankId`].
    type Output = [PressureVessel; 6];

    fn pack(snapshot: Self::Input<'_>) -> Self {
        let pressures = PressSensId::ALL.map(|id| {
            let bar = snapshot.input_image.press_sens[id].map(|d| d.data);
            pack_full_scale(bar, id.full_scale_bar())
        });

        let temperatures = TempSensId::ALL
            .map(|id| pack_temperature(snapshot.input_image.temp_sens[id].map(|d| d.data)));

        Self {
            pressures,
            temperatures,
            ox_tank_level: pack_full_scale(snapshot.input_image.ox_tank_level.map(|d| d.data), 1.0),
        }
    }

    #[allow(
        clippy::indexing_slicing,
        reason = "the arrays are one per inventory id and idx() is a dense 0..N index into them, \
                  by the InventoryId<N> contract"
    )]
    fn unpack(self, _context: &mut ConnectionContext) -> Self::Output {
        let pressure = |id: Option<PressSensId>| {
            id.and_then(|id| unpack_full_scale(self.pressures[id.idx()], id.full_scale_bar()))
                .map(|bar| (bar * 100.0).clamp(0.0, f32::from(u16::MAX)) as u16)
                .unwrap_or(u16::MAX)
        };

        let temperature = |id: Option<TempSensId>| {
            id.and_then(|id| unpack_temperature(self.temperatures[id.idx()]))
                .map(|c| (c * 100.0).clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16)
                .unwrap_or(i16::MAX)
        };

        TankId::ALL.map(|tank| {
            let p = tank.pressure_sensors();
            let t = tank.temperature_sensors();

            let level = (tank == TankId::Oxidizer)
                .then(|| unpack_full_scale(self.ox_tank_level, 1.0))
                .flatten()
                .map(|l| (l * 10000.0).clamp(0.0, f32::from(u16::MAX)) as u16)
                .unwrap_or(u16::MAX);

            PressureVessel {
                id: tank as u8,
                flags: tank.flags(),
                fluid: tank.fluid(),
                pressure1: pressure(p[0]),
                pressure2: pressure(p[1]),
                rated_pressure: (tank.pressure_rating_bar() * 100.0) as u16,
                temperature1: temperature(t[0]),
                temperature2: temperature(t[1]),
                volume: (tank.volume_l() * 1000.0) as u16,
                level,
            }
        })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::super::tests::{SnapshotParts, through_packet};
    use super::*;

    use core::num::Wrapping;

    use mission::bus::{BusInputImage, DataWithTime};

    use crate::messages::DownlinkMessage;

    /// Every sensor reporting near its full scale, which is where a varint-encoded field would
    /// have grown past the payload.
    pub(crate) fn saturated_inputs() -> BusInputImage {
        let mut inputs = BusInputImage::default();

        for id in PressSensId::ALL {
            inputs.press_sens[id] =
                Some(DataWithTime::new(id.full_scale_bar() * 0.99, Wrapping(0)));
        }
        for id in TempSensId::ALL {
            inputs.temp_sens[id] = Some(DataWithTime::new(35.0, Wrapping(0)));
        }
        inputs.ox_tank_level = Some(DataWithTime::new(0.87, Wrapping(0)));

        inputs
    }

    #[test]
    fn tank_readings_survive_the_packet() {
        let mut parts = SnapshotParts::default();
        parts.inputs.press_sens[PressSensId::OxTankUpper] =
            Some(DataWithTime::new(42.0, Wrapping(0)));
        parts.inputs.press_sens[PressSensId::OxTankLower] =
            Some(DataWithTime::new(41.5, Wrapping(0)));
        parts.inputs.press_sens[PressSensId::PressurantTank] =
            Some(DataWithTime::new(210.0, Wrapping(0)));
        parts.inputs.temp_sens[TempSensId::OxTankUpper] =
            Some(DataWithTime::new(18.0, Wrapping(0)));
        parts.inputs.ox_tank_level = Some(DataWithTime::new(0.75, Wrapping(0)));

        let msg = DownlinkMessage::Pressures(PressuresMessage::pack(&parts.snapshot()));
        let DownlinkMessage::Pressures(decoded) = through_packet(msg) else {
            panic!("decoded as the wrong message")
        };
        let vessels = decoded.unpack(&mut ConnectionContext::init(0));

        let ox = &vessels[TankId::Oxidizer.idx()];
        // Resolution is the sensor's full scale over 254 codes: 55 bar / 254 ~ 0.22 bar, in
        // centibar here.
        assert!(ox.pressure1.abs_diff(4200) <= 22, "{}", ox.pressure1);
        assert!(ox.pressure2.abs_diff(4150) <= 22, "{}", ox.pressure2);
        // 0.5 C resolution, in centidegrees.
        assert!((ox.temperature1 - 1800).abs() <= 50, "{}", ox.temperature1);
        // Level is centi-percent, 1/254 resolution.
        assert!(ox.level.abs_diff(7500) <= 40, "{}", ox.level);

        let pressurant = &vessels[TankId::Pressurant.idx()];
        // 300 bar / 254 ~ 1.2 bar.
        assert!(
            pressurant.pressure1.abs_diff(21000) <= 120,
            "{}",
            pressurant.pressure1
        );

        // Static fields come back from the inventory tables, not the wire.
        assert_eq!(ox.rated_pressure, 5500);
        assert_eq!(ox.volume, 8000);
        assert_eq!(ox.fluid, TankId::Oxidizer.fluid());
        assert_eq!(ox.id, TankId::Oxidizer as u8);
    }

    /// A sensor that has never reported has to stay unreported, rather than reading as zero bar.
    #[test]
    fn missing_readings_stay_missing() {
        let parts = SnapshotParts::default();
        let vessels =
            PressuresMessage::pack(&parts.snapshot()).unpack(&mut ConnectionContext::init(0));

        for vessel in &vessels {
            assert_eq!(vessel.pressure1, u16::MAX);
            assert_eq!(vessel.pressure2, u16::MAX);
            assert_eq!(vessel.temperature1, i16::MAX);
            assert_eq!(vessel.temperature2, i16::MAX);
            assert_eq!(vessel.level, u16::MAX);
        }
    }

    /// Only the oxidizer tank has a level sensor; the rest must not inherit its reading.
    #[test]
    fn only_the_oxidizer_tank_reports_a_level() {
        let mut parts = SnapshotParts::default();
        parts.inputs.ox_tank_level = Some(DataWithTime::new(0.5, Wrapping(0)));

        let vessels =
            PressuresMessage::pack(&parts.snapshot()).unpack(&mut ConnectionContext::init(0));

        for tank in TankId::ALL {
            let expected_known = tank == TankId::Oxidizer;
            assert_eq!(
                vessels[tank.idx()].level != u16::MAX,
                expected_known,
                "{tank:?}"
            );
        }
    }
}
