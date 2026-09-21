use serde::{Deserialize, Serialize};

use mission::inventory::{InventoryId, PressSensId, TankId, TempSensId};
use mission::mavlink::VehicleSnapshot;
use rapid_dialect::rapid::messages::PressureVessel;

use super::{
    ConnectionContext, DOWNLINK_PAYLOAD_SIZE, DownlinkTelemetryMessage, pack_full_scale,
    pack_temperature, unpack_full_scale, unpack_temperature,
};

/// Builds the [`PressureVessel`] a tank reports, given a reading for each of its sensors.
///
/// Both messages below carry one half of the inventory, and everything but the readings
/// themselves comes back out of the inventory tables the receiver already has.
fn vessel(
    tank: TankId,
    pressure: impl Fn(Option<PressSensId>) -> u16,
    temperature: impl Fn(Option<TempSensId>) -> i16,
    level: u16,
) -> PressureVessel {
    let p = tank.pressure_sensors();
    let t = tank.temperature_sensors();

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
}

/// Centibar, as PRESSURE_VESSEL carries it, or its unknown sentinel.
fn centibar(bar: Option<f32>) -> u16 {
    bar.map(|bar| (bar * 100.0).clamp(0.0, f32::from(u16::MAX)) as u16)
        .unwrap_or(u16::MAX)
}

/// On-vehicle tank pressures, temperatures, and levels
///
/// Each pressure is scaled against its own sensor's full scale ([`PressSensId::full_scale_bar`]).
///
/// Unpacks into the four on-vehicle PRESSURE_VESSEL messages; the ground side travels in
/// [`ExternalPressuresMessage`].
#[derive(Debug, Serialize, Deserialize)]
pub struct PressuresMessage {
    /// In [`PressSensId::INTERNAL`] order, each as a fraction of that sensor's full scale.
    pressures: [u8; PressSensId::INTERNAL.len()],
    /// In [`TempSensId`] order, -60.0 - +67.0 C in 0.5 C.
    temperatures: [u8; 2],
    /// Oxidizer tank fill level, 0 - 1 of the tank.
    ox_tank_level: u8,
}

/// A code per internal sensor, a code per temperature sensor, and the oxidizer level.
const PRESSURES_PAYLOAD_LEN: usize = PressSensId::INTERNAL.len() + TempSensId::ALL.len() + 1;

// A tenth on-vehicle sensor is what overflowed this payload once already, and the transmitter
// unwraps the encode, so catch it here rather than in flight.
const _: () = assert!(
    PRESSURES_PAYLOAD_LEN <= DOWNLINK_PAYLOAD_SIZE,
    "the on-vehicle pressures no longer fit their packet"
);

impl DownlinkTelemetryMessage for PressuresMessage {
    const ID: u8 = 0x03;
    type Input<'a> = &'a VehicleSnapshot<'a>;
    /// One per [`TankId::INTERNAL`].
    type Output = [PressureVessel; TankId::INTERNAL.len()];

    fn pack(snapshot: Self::Input<'_>) -> Self {
        let pressures = PressSensId::INTERNAL.map(|id| {
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
                  by the InventoryId<N> contract; INTERNAL is a prefix of ALL, asserted in \
                  mission::inventory"
    )]
    fn unpack(self, _context: &mut ConnectionContext) -> Self::Output {
        let pressure = |id: Option<PressSensId>| {
            centibar(
                id.and_then(|id| unpack_full_scale(self.pressures[id.idx()], id.full_scale_bar())),
            )
        };

        let temperature = |id: Option<TempSensId>| {
            id.and_then(|id| unpack_temperature(self.temperatures[id.idx()]))
                .map(|c| (c * 100.0).clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16)
                .unwrap_or(i16::MAX)
        };

        TankId::INTERNAL.map(|tank| {
            let level = (tank == TankId::Oxidizer)
                .then(|| unpack_full_scale(self.ox_tank_level, 1.0))
                .flatten()
                .map(|l| (l * 10000.0).clamp(0.0, f32::from(u16::MAX)) as u16)
                .unwrap_or(u16::MAX);

            vessel(tank, pressure, temperature, level)
        })
    }
}

/// Ground-side tank pressures, read over the umbilical
///
/// Each pressure is scaled against its own sensor's full scale ([`PressSensId::full_scale_bar`]).
///
/// Unpacks into the two external PRESSURE_VESSEL messages. These tanks carry no temperature or
/// level instrumentation, so those fields come back as their unknown sentinels.
#[derive(Debug, Serialize, Deserialize)]
pub struct ExternalPressuresMessage {
    /// In [`PressSensId::EXTERNAL`] order, each as a fraction of that sensor's full scale.
    pressures: [u8; PressSensId::EXTERNAL.len()],
}

impl DownlinkTelemetryMessage for ExternalPressuresMessage {
    const ID: u8 = 0x07;
    type Input<'a> = &'a VehicleSnapshot<'a>;
    /// One per [`TankId::EXTERNAL`].
    type Output = [PressureVessel; TankId::EXTERNAL.len()];

    fn pack(snapshot: Self::Input<'_>) -> Self {
        Self {
            pressures: PressSensId::EXTERNAL.map(|id| {
                let bar = snapshot.input_image.press_sens[id].map(|d| d.data);
                pack_full_scale(bar, id.full_scale_bar())
            }),
        }
    }

    #[allow(
        clippy::indexing_slicing,
        clippy::arithmetic_side_effects,
        reason = "EXTERNAL is the suffix of ALL that starts at INTERNAL.len(), asserted in \
                  mission::inventory, so the shifted index stays in 0..EXTERNAL.len()"
    )]
    fn unpack(self, _context: &mut ConnectionContext) -> Self::Output {
        let pressure = |id: Option<PressSensId>| {
            centibar(id.and_then(|id| {
                let code = self.pressures[id.idx() - PressSensId::INTERNAL.len()];
                unpack_full_scale(code, id.full_scale_bar())
            }))
        };

        TankId::EXTERNAL.map(|tank| vessel(tank, pressure, |_| i16::MAX, u16::MAX))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::super::tests::{SnapshotParts, through_packet};
    use super::*;

    use core::num::Wrapping;

    use mission::bus::{BusInputImage, DataWithTime};
    use rapid_dialect::rapid::enums::PressureVesselFlag;

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

    /// Position of a tank in the message that carries it.
    fn at(tanks: &[TankId], tank: TankId) -> usize {
        tanks
            .iter()
            .position(|t| *t == tank)
            .expect("tank is not in that half of the inventory")
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

        let ox = &vessels[at(&TankId::INTERNAL, TankId::Oxidizer)];
        // Resolution is the sensor's full scale over 254 codes: 55 bar / 254 ~ 0.22 bar, in
        // centibar here.
        assert!(ox.pressure1.abs_diff(4200) <= 22, "{}", ox.pressure1);
        assert!(ox.pressure2.abs_diff(4150) <= 22, "{}", ox.pressure2);
        // 0.5 C resolution, in centidegrees.
        assert!((ox.temperature1 - 1800).abs() <= 50, "{}", ox.temperature1);
        // Level is centi-percent, 1/254 resolution.
        assert!(ox.level.abs_diff(7500) <= 40, "{}", ox.level);

        let pressurant = &vessels[at(&TankId::INTERNAL, TankId::Pressurant)];
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

    #[test]
    fn external_readings_survive_the_packet() {
        let mut parts = SnapshotParts::default();
        parts.inputs.press_sens[PressSensId::ExternalPressurant] =
            Some(DataWithTime::new(180.0, Wrapping(0)));
        parts.inputs.press_sens[PressSensId::ExternalOxidizer] =
            Some(DataWithTime::new(45.0, Wrapping(0)));

        let msg =
            DownlinkMessage::ExternalPressures(ExternalPressuresMessage::pack(&parts.snapshot()));
        let DownlinkMessage::ExternalPressures(decoded) = through_packet(msg) else {
            panic!("decoded as the wrong message")
        };
        let vessels = decoded.unpack(&mut ConnectionContext::init(0));

        let pressurant = &vessels[at(&TankId::EXTERNAL, TankId::ExternalPressurant)];
        // 300 bar / 254 ~ 1.2 bar.
        assert!(
            pressurant.pressure1.abs_diff(18000) <= 120,
            "{}",
            pressurant.pressure1
        );

        let oxidizer = &vessels[at(&TankId::EXTERNAL, TankId::ExternalOxidizer)];
        // 60 bar / 254 ~ 0.24 bar.
        assert!(
            oxidizer.pressure1.abs_diff(4500) <= 24,
            "{}",
            oxidizer.pressure1
        );

        for v in &vessels {
            assert!(v.flags.contains(PressureVesselFlag::EXTERNAL), "{v:?}");
            // These tanks carry no second transducer, no thermocouple and no level sensor.
            assert_eq!(v.pressure2, u16::MAX);
            assert_eq!(v.temperature1, i16::MAX);
            assert_eq!(v.temperature2, i16::MAX);
            assert_eq!(v.level, u16::MAX);
        }
    }

    /// A sensor that has never reported has to stay unreported, rather than reading as zero bar.
    #[test]
    fn missing_readings_stay_missing() {
        let parts = SnapshotParts::default();
        let internal =
            PressuresMessage::pack(&parts.snapshot()).unpack(&mut ConnectionContext::init(0));
        let external = ExternalPressuresMessage::pack(&parts.snapshot())
            .unpack(&mut ConnectionContext::init(0));

        // Between them the two messages still account for every tank.
        assert_eq!(internal.len() + external.len(), TankId::ALL.len());

        for vessel in internal.iter().chain(external.iter()) {
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

        for tank in TankId::INTERNAL {
            let expected_known = tank == TankId::Oxidizer;
            assert_eq!(
                vessels[at(&TankId::INTERNAL, tank)].level != u16::MAX,
                expected_known,
                "{tank:?}"
            );
        }
    }
}
