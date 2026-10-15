//! The tunable parameters and the bridge to the MAVLink parameter protocol.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use nalgebra::Vector3;

use rapid_dialect::rapid::enums::MavParamType;
use rapid_dialect::rapid::messages::ParamMetadata;

use state_estimator::StateEstimatorParams;

use links::UplinkCommand;
use links::protocols::params::{ParamInfo, ParamRequest, ParamSetError, ParamStore};

use crate::traits::SensorReadings;

pub use params::{ParamDescriptor, ParamId, ParamType, ParamValue, ParameterField, ParameterGroup};

/// Top-level params, aggregating the per-subsystem parameter groups.
///
/// Each field is expected to implement [`ParameterGroup`].
#[derive(Debug, Default, Clone, macros::ParameterGroups)]
pub struct Params {
    pub state_estimator: StateEstimatorParams,
    pub state_machine: StateMachineParams,
    pub propulsion: PropulsionParams,
    pub qd: QdParams,
    pub failsafe: FailsafeParams,
    pub tank_level: TankLevelParams,
    pub misc: MiscParams,
    pub calibration: CalibrationParams,
    pub lora: LoraParams,
}

/// State machine parameters, exposed over MAVLink as `SM_*`.
#[derive(Debug, Clone, macros::ParameterGroup)]
#[param_group(prefix = "SM")]
pub struct StateMachineParams {
    /// Altitude AGL at which to deploy the main parachute
    #[param(id = 0x0200, name = "MAIN_ALT", default = 400.0, unit = "m")]
    pub main_deploy_altitude: f32,
    /// Minimum time after launch before allowing drogue deployment
    #[param(id = 0x0201, name = "MIN_T_APOGEE", default = 1000, unit = "ms")]
    pub min_time_to_drogue: u32,
    /// Minimum delay after drogue before allowing main deployment
    #[param(id = 0x0202, name = "DLY_MAIN", default = 3000, unit = "ms")]
    pub min_time_to_main: u32,
    /// Time after liftoff after which drogue is automatically deployed. Must be larger than
    /// MAX_T_BURN.
    #[param(id = 0x0203, name = "MAX_T_APOGEE", default = 30_000, unit = "ms")]
    pub max_time_to_drogue: u32,
    /// Time limit for burn, after which automatically transition to Coast. Must be smaller than
    /// MAX_T_APOGEE.
    #[param(id = 0x0204, name = "MAX_T_BURN", default = 15_000, unit = "ms")]
    pub max_time_in_burn: u32,
    /// Acceleration to be exceeded for LODEC_T in order to detect liftoff
    #[param(id = 0x0205, name = "LODEC_ACC", default = 3.0, unit = "G")]
    pub liftoff_detection_acceleration: f32,
    /// Time to exceed LODEC_ACC in order to detect liftoff
    #[param(id = 0x0206, name = "LODEC_T", default = 50, unit = "ms")]
    pub liftoff_detection_time: u32,
    /// Time the main output is high within each firing pulse
    #[param(id = 0x0207, name = "MAIN_ON_T", default = 250, unit = "ms")]
    pub main_on_time: u32,
    /// Time the main output is low between firing pulses
    #[param(id = 0x0208, name = "MAIN_GAP_T", default = 100, unit = "ms")]
    pub main_pulse_gap: u32,
    /// Number of main firing pulses. The whole train must fit inside the time the
    /// vehicle spends in DeployMain, which can be as short as 3 seconds.
    #[param(id = 0x0209, name = "MAIN_PULSES", default = 1)]
    pub main_pulses: u32,
}

/// Ignition sequence parameters, exposed over MAVLink as `PROP_*`.
#[derive(Debug, Clone, macros::ParameterGroup)]
#[param_group(prefix = "PROP")]
pub struct PropulsionParams {
    /// Time the igniter outputs are high after entering ignition mode
    #[param(id = 0x0300, name = "IGNTR_TIME", default = 1500, unit = "ms")]
    pub igniter_on_time: u32,
    /// Delay after ignition mode is entered after which main valve is opened
    #[param(id = 0x0301, name = "MAIN_DELAY", default = 700, unit = "ms")]
    pub main_valve_delay: u32,
}

/// Quick disconnect release sequence parameters, exposed over MAVLink as `QD_*`.
///
/// See [`crate::servos`] for how these are used.
#[derive(Debug, Clone, macros::ParameterGroup)]
#[param_group(prefix = "QD")]
pub struct QdParams {
    // oxidizer quick disconnect
    /// Time the disconnect servo is actuated after a gripper release before the retract servo moves
    /// to RETR_OX_PERC
    #[param(id = 0x0600, name = "HOLD_T_OX", default = 1000, unit = "ms")]
    pub disconnect_hold_time_oxidizer: u32,
    /// Time after the retract servo moved after which the disconnect servo is closed again
    #[param(id = 0x0603, name = "RETR_T_OX", default = 700, unit = "ms")]
    pub disconnect_retract_time_oxidizer: u32,
    /// Position the oxidizer retract servo moves to after a gripper release
    #[param(
        id = 0x0602,
        name = "RETR_OX_PERC",
        default = 0,
        unit = "%",
        min = 0,
        max = 100
    )]
    pub disconnect_retract_percent_oxidizer: u32,

    // pressurant (n2) quick disconnect
    /// Time the disconnect servo is actuated after a gripper release before the retract servo moves
    /// to RETR_PR_PERC
    #[param(id = 0x0604, name = "HOLD_T_PR", default = 1000, unit = "ms")]
    pub disconnect_hold_time_pressurant: u32,
    /// Time after the retract servo moved after which the disconnect servo is closed again
    #[param(id = 0x0605, name = "RETR_T_PR", default = 4000, unit = "ms")]
    pub disconnect_retract_time_pressurant: u32,
    /// Position the pressurant retract servo moves to after a gripper release
    #[param(
        id = 0x0601,
        name = "RETR_PR_PERC",
        default = 10,
        unit = "%",
        min = 0,
        max = 100
    )]
    pub disconnect_retract_percent_pressurant: u32,
}

/// Oxidizer tank level parameters, exposed over MAVLink as `OXL_*`.
///
/// See [`crate::tank_level`] for how these are used.
#[derive(Debug, Clone, macros::ParameterGroup)]
#[param_group(prefix = "OXL")]
pub struct TankLevelParams {
    /// Share of its largest possible value at which the running sum up the probe row marks the
    /// level. Higher reads higher.
    #[param(id = 0xF000, name = "THRESHOLD", default = 0.02)]
    pub threshold: f32,
    /// Time constant of the low-pass on the reported level.
    #[param(id = 0xF001, name = "TAU", default = 5000, unit = "ms")]
    pub tau: u32,
    /// Smallest spread the probe readings are scaled by, so that a column whose spread shrinks as
    /// the tank fills towards the top probe is not stretched to full scale.
    #[param(id = 0xF002, name = "MIN_SCALE", default = 10.0, unit = "C")]
    pub min_scale: f32,
    /// Temperature above the coldest probe up to which a probe counts as no warmer than it.
    #[param(id = 0xF003, name = "DEADBAND", default = 1.0, unit = "C")]
    pub deadband: f32,
}

/// Uplink-loss failsafe parameters, exposed over MAVLink as `FS_*`.
///
/// The timeouts are measured from boot, so the failsafe survives reboots.
#[derive(Debug, Clone, macros::ParameterGroup)]
#[param_group(prefix = "FS")]
pub struct FailsafeParams {
    /// Time without ground station contact after which the vehicle returns to Idle. 0 disables.
    #[param(id = 0x0400, name = "UPLINK_IDLE", default = 10_000, unit = "ms")]
    pub uplink_idle_timeout: u32,
    /// Time without ground station contact after which the vehicle vents. 0 disables.
    #[param(id = 0x0401, name = "UPLINK_VENT", default = 1_200_000, unit = "ms")]
    pub uplink_vent_timeout: u32,
}

/// Miscellaneous parameters, exposed over MAVLink as `MISC_*`.
#[derive(Debug, Clone, macros::ParameterGroup)]
#[param_group(prefix = "MISC")]
pub struct MiscParams {
    /// Duty cycle of the buzzer PWM signal. A buzzer is loudest at 50%, other values trade volume
    /// for a lower current draw. 0 disables the buzzer.
    #[param(
        id = 0x0500,
        name = "BUZZ_VOL",
        default = 50,
        unit = "%",
        min = 0,
        max = 100
    )]
    pub buzzer_volume: u32,
    /// Enables valve heating while on the pad, up to and including `Ignite`. 0 disables, 1 enables.
    #[param(id = 0x0501, name = "HEAT_PAD", default = 1, min = 0, max = 1)]
    pub heat_en_pad: u32,
    /// Enables valve heating after launch, from `Burn` onward. 0 disables, 1 enables.
    #[param(id = 0x0502, name = "HEAT_POST", default = 0, min = 0, max = 1)]
    pub heat_en_post: u32,
}

/// LoRa settings, including secrets. Exposed over MAVLink as `LORA_*`. Apart from the transmit
/// powers, read once at boot, so a change takes effect on the next one. Ground station modems carry
/// the same link values as `GCS_*`.
#[derive(Debug, Clone, macros::ParameterGroup)]
#[param_group(prefix = "LORA")]
pub struct LoraParams {
    /// Seed of the frequency hopping sequence, low word first. Ground software derives it from a
    /// binding phrase.
    #[param(id = 0x0800..=0x0801, name = "BIND", default = [0, 0], reboot_required)]
    pub seed: [u32; 2],
    /// Key authenticating every packet, as little-endian words. Never sent over LoRa, which reads
    /// it back as zeros.
    #[param(id = 0x0802..=0x0805, name = "KEY", default = [0, 0, 0, 0], reboot_required)]
    pub key: [u32; 4],
    /// Downlink transmit power until the vehicle arms. A write applies immediately, also switching
    /// back from TX_PWR_HI until the vehicle next enters an armed mode.
    #[param(id = 0x0806, name = "TX_PWR", default = 10.0, unit = "dBm", min = -9.0, max = 22.0)]
    pub tx_power: f32,
    /// Downlink transmit power switched to on every entry into an armed mode, including transitions
    /// between them. Dropping out of the armed modes does not switch back.
    #[param(id = 0x0807, name = "TX_PWR_HI", default = 22.0, unit = "dBm", min = -9.0, max = 22.0)]
    pub tx_power_armed: f32,
    /// Downlink hopping channels, bit i = 863.25 + 0.5 i MHz. Has to match GCS_DN_CHAN. Both 0
    /// disables LoRa.
    #[param(
        id = 0x0808,
        name = "DN_CHAN",
        default = 0b00_0001_0110_1100,
        min = 0,
        max = 0x3FFF,
        reboot_required
    )]
    pub downlink_channels: u32,
    /// Uplink hopping channels, bit i = 863.25 + 0.5 i MHz. Has to match GCS_UP_CHAN.
    #[param(
        id = 0x0809,
        name = "UP_CHAN",
        default = 0b01_0000_1001_0010,
        min = 0,
        max = 0x3FFF,
        reboot_required
    )]
    pub uplink_channels: u32,
}

/// Id of [`LoraParams::tx_power`], which the vehicle recognizes writes to. The attribute takes a
/// literal, so this cannot be derived from the declaration.
pub const LORA_TX_POWER: ParamId = ParamId::new(0x0806);

/// Sensor offsets, exposed over MAVLink as `CAL_*`. Subtracted from the raw readings in the body
/// frame (after each driver's axis remap) and in the reading's own unit, before anything else
/// sees them.
#[derive(Debug, Clone, macros::ParameterGroup)]
#[param_group(prefix = "CAL")]
pub struct CalibrationParams {
    /// IMU1 gyro bias
    #[param(id = 0x0700..=0x0702, name = "IMU1_GYR", default = [0.0, 0.0, 0.0], unit = "deg/s")]
    pub imu1_gyro: Vector3<f32>,
    /// IMU1 accelerometer bias
    #[param(id = 0x0703..=0x0705, name = "IMU1_ACC", default = [0.0, 0.0, 0.0], unit = "m/s^2")]
    pub imu1_accel: Vector3<f32>,
    /// IMU2 gyro bias
    #[param(id = 0x0706..=0x0708, name = "IMU2_GYR", default = [0.0, 0.0, 0.0], unit = "deg/s")]
    pub imu2_gyro: Vector3<f32>,
    /// IMU2 accelerometer bias
    #[param(id = 0x0709..=0x070B, name = "IMU2_ACC", default = [0.0, 0.0, 0.0], unit = "m/s^2")]
    pub imu2_accel: Vector3<f32>,
    /// IMU3 gyro bias
    #[param(id = 0x070C..=0x070E, name = "IMU3_GYR", default = [0.0, 0.0, 0.0], unit = "deg/s")]
    pub imu3_gyro: Vector3<f32>,
    /// IMU3 accelerometer bias
    #[param(id = 0x070F..=0x0711, name = "IMU3_ACC", default = [0.0, 0.0, 0.0], unit = "m/s^2")]
    pub imu3_accel: Vector3<f32>,
    /// Magnetometer hard-iron offset
    #[param(id = 0x0712..=0x0714, name = "MAG", default = [0.0, 0.0, 0.0], unit = "uT")]
    pub mag: Vector3<f32>,
    /// Barometer 1 pressure offset
    #[param(id = 0x0715, name = "BARO1", default = 0.0, unit = "hPa")]
    pub baro1: f32,
    /// Barometer 2 pressure offset
    #[param(id = 0x0716, name = "BARO2", default = 0.0, unit = "hPa")]
    pub baro2: f32,
    /// Barometer 3 pressure offset
    #[param(id = 0x0717, name = "BARO3", default = 0.0, unit = "hPa")]
    pub baro3: f32,
}

impl LoraParams {
    pub fn hop_seed(&self) -> u64 {
        hop_seed(self.seed)
    }

    pub fn hmac_key(&self) -> [u8; 16] {
        hmac_key(self.key)
    }

    /// Whether a param must not leave the vehicle over the air.
    pub fn is_secret(id: ParamId) -> bool {
        (0x0802..=0x0805).contains(&id.get())
    }
}

/// The hop seed from its two param words, low word first.
pub fn hop_seed([low, high]: [u32; 2]) -> u64 {
    u64::from(high) << 32 | u64::from(low)
}

/// The HMAC key from its four param words.
#[allow(
    clippy::indexing_slicing,
    reason = "four words of four bytes fill the sixteen exactly"
)]
pub fn hmac_key(words: [u32; 4]) -> [u8; 16] {
    let mut key = [0; 16];
    for (chunk, word) in key.chunks_exact_mut(4).zip(words) {
        chunk.copy_from_slice(&word.to_le_bytes());
    }
    key
}

impl CalibrationParams {
    pub fn apply(&self, r: &mut SensorReadings) {
        let vectors = [
            (&mut r.imu1_gyro, self.imu1_gyro),
            (&mut r.imu1_accel, self.imu1_accel),
            (&mut r.imu2_gyro, self.imu2_gyro),
            (&mut r.imu2_accel, self.imu2_accel),
            (&mut r.imu3_gyro, self.imu3_gyro),
            (&mut r.imu3_accel, self.imu3_accel),
            (&mut r.mag, self.mag),
        ];
        for (reading, offset) in vectors {
            if let Some(v) = reading {
                #[allow(
                    clippy::arithmetic_side_effects,
                    reason = "f32 vector subtraction cannot overflow or panic"
                )]
                {
                    *v -= offset;
                }
            }
        }

        let baros = [
            (&mut r.baro1, self.baro1),
            (&mut r.baro2, self.baro2),
            (&mut r.baro3, self.baro3),
        ];
        for (baro, offset) in baros {
            if let Some(p) = &mut baro.pressure {
                *p -= offset;
            }
        }
    }
}

/// Live mirror of the current [`Params`] for the MAVLink param protocol tasks. Starts empty (the
/// protocol won't answer until [`SharedParams::init`] runs at boot) and is updated on every
/// accepted PARAM_SET. The vehicle applies the same value via
/// [`UplinkCommand::SetParam`](links::UplinkCommand) one main-loop tick later; the param path is
/// the only writer to both, so the two copies cannot diverge.
pub struct SharedParams<G = Params> {
    inner: Mutex<CriticalSectionRawMutex, RefCell<Option<G>>>,
}

/// MAVLink lookups over the parameters of a group.
pub trait ParamTable: ParameterGroup + Sized {
    /// Total number of parameters exposed over MAVLink.
    fn count() -> u16 {
        Self::PARAM_COUNT as u16
    }

    /// Descriptor at a MAVLink `param_index`.
    fn descriptor_by_index(index: u16) -> Option<ParamDescriptor> {
        Self::descriptor(usize::from(index))
    }

    /// (`param_index`, descriptor) for a stable parameter id (linear scan; the set is small).
    fn by_id(id: u16) -> Option<(u16, ParamDescriptor)> {
        (0..Self::count()).find_map(|i| {
            Self::descriptor_by_index(i).and_then(|d| (d.id.get() == id).then_some((i, d)))
        })
    }

    /// (`param_index`, descriptor) for a MAVLink `param_id` name.
    fn by_name(name: &[u8; 16]) -> Option<(u16, ParamDescriptor)> {
        (0..Self::count()).find_map(|i| {
            Self::descriptor_by_index(i).and_then(|d| (d.mavlink_name() == *name).then_some((i, d)))
        })
    }

    /// The value a PARAM_SET of `value` as `ty` gives the param, if it is a valid one.
    fn from_mavlink(
        descriptor: &ParamDescriptor,
        value: f32,
        ty: MavParamType,
    ) -> Option<ParamValue> {
        (ty == MavParamType::from(descriptor.ty))
            .then(|| descriptor.ty.from_mavlink(value))
            .flatten()
            .filter(|value| descriptor.accepts(*value))
    }

    /// The PARAM_METADATA of the param at `index`.
    fn metadata(index: u16) -> Option<ParamMetadata>
    where
        Self: Default,
    {
        let descriptor = Self::descriptor_by_index(index)?;
        let default = Self::default().get(descriptor.id)?;
        Some(descriptor.metadata(default))
    }

    /// The PARAM_VALUE for `value` of the param at `index`.
    fn info(index: u16, descriptor: &ParamDescriptor, value: ParamValue) -> ParamInfo {
        ParamInfo {
            name: descriptor.mavlink_name(),
            value: value.to_mavlink(),
            ty: descriptor.ty.into(),
            index,
            count: Self::count(),
        }
    }

    /// Apply a raw stored value (as read from flash). Unknown ids and values out of bounds, e.g.
    /// after a firmware update narrowed them, are ignored.
    fn apply_raw(&mut self, id: u16, raw: u32) {
        if let Some((_, descriptor)) = Self::by_id(id) {
            let value = descriptor.ty.decode_raw(raw);
            if descriptor.accepts(value) {
                self.set(descriptor.id, value);
            }
        }
    }
}

impl<G: ParameterGroup> ParamTable for G {}

impl Params {
    /// The commands that have the vehicle carry out `request` and answer it with PARAM_VALUEs.
    /// Nothing for requests about unknown params, which get no answer.
    pub fn commands(
        request: &ParamRequest,
    ) -> heapless::Vec<UplinkCommand, { Params::PARAM_COUNT.div_ceil(u32::BITS as usize) }> {
        let read = |index| UplinkCommand::RequestParams {
            first: index,
            mask: 1,
        };

        match *request {
            ParamRequest::List => (0..Self::count())
                .step_by(u32::BITS as usize)
                .map(|first| UplinkCommand::RequestParams {
                    first,
                    mask: u32::MAX,
                })
                .collect(),
            ParamRequest::ReadIndex(index) => Self::descriptor_by_index(index)
                .map(|_| read(index))
                .into_iter()
                .collect(),
            ParamRequest::ReadName(name) => Self::by_name(&name)
                .map(|(index, _)| read(index))
                .into_iter()
                .collect(),
            ParamRequest::Set { name, value, ty } => Self::by_name(&name)
                .map(
                    |(index, descriptor)| match Self::from_mavlink(&descriptor, value, ty) {
                        Some(value) => UplinkCommand::SetParam {
                            id: descriptor.id.get(),
                            raw: value.encode_raw(),
                        },
                        // A rejected write is answered with the current value.
                        None => read(index),
                    },
                )
                .into_iter()
                .collect(),
        }
    }
}

impl<G: ParameterGroup> SharedParams<G> {
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(None)),
        }
    }

    pub fn init(&self, params: G) {
        self.inner.lock(|inner| {
            *inner.borrow_mut() = Some(params);
        });
    }

    fn info(&self, index: u16, descriptor: &ParamDescriptor) -> Option<ParamInfo> {
        self.inner.lock(|inner| {
            inner.borrow().as_ref().and_then(|params| {
                params
                    .get(descriptor.id)
                    .map(|value| G::info(index, descriptor, value))
            })
        })
    }
}

impl<G: ParameterGroup> Default for SharedParams<G> {
    fn default() -> Self {
        Self::new()
    }
}

impl<G: ParameterGroup + Default + Send> ParamStore for SharedParams<G> {
    fn count(&self) -> u16 {
        G::count()
    }

    fn metadata(&self, index: u16) -> Option<ParamMetadata> {
        G::metadata(index)
    }

    fn by_index(&self, index: u16) -> Option<ParamInfo> {
        G::descriptor_by_index(index).and_then(|d| self.info(index, &d))
    }

    fn by_name(&self, name: &[u8; 16]) -> Option<ParamInfo> {
        G::by_name(name).and_then(|(index, d)| self.info(index, &d))
    }

    fn set(
        &self,
        name: &[u8; 16],
        value: f32,
        ty: MavParamType,
    ) -> Result<(u16, u32, ParamInfo), ParamSetError> {
        let (index, descriptor) = G::by_name(name).ok_or(ParamSetError::UnknownParam)?;

        let value = G::from_mavlink(&descriptor, value, ty).ok_or(ParamSetError::InvalidValue)?;

        self.inner.lock(|inner| {
            let mut borrowed = inner.borrow_mut();

            // An uninitialized mirror means boot hasn't finished; don't accept writes we couldn't
            // echo truthfully.
            let params = borrowed.as_mut().ok_or(ParamSetError::UnknownParam)?;

            params.set(descriptor.id, value);
            let stored = params
                .get(descriptor.id)
                .ok_or(ParamSetError::UnknownParam)?;

            Ok((
                descriptor.id.get(),
                value.encode_raw(),
                G::info(index, &descriptor, stored),
            ))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::traits::BaroReading;

    use rapid_dialect::rapid::enums::ParamMetadataFlag;

    #[test]
    fn lora_tx_power_id_matches() {
        assert_eq!(
            Params::by_id(LORA_TX_POWER.get()).unwrap().1.name,
            "LORA_TX_PWR"
        );
    }

    #[test]
    fn registry_invariants() {
        // Every declared parameter has a unique id and a unique, <=16 char name.
        for i in 0..Params::count() {
            let a = Params::descriptor_by_index(i).unwrap();
            assert!(a.name.len() <= 16, "name too long: {}", a.name);
            assert!(a.name.is_ascii(), "name not ascii: {}", a.name);
            for j in (i + 1)..Params::count() {
                let b = Params::descriptor_by_index(j).unwrap();
                assert_ne!(a.id, b.id, "duplicate id {:#x}", a.id.get());
                assert_ne!(a.name, b.name, "duplicate name {}", a.name);
            }
        }
    }

    #[test]
    fn defaults_match_declared() {
        // The derive builds Default from the #[param(default = ..)] args.
        let s = Params::default();
        assert_eq!(s.state_estimator.mahony_kp, 0.1);
        assert_eq!(s.state_estimator.std_dev_barometer_transsonic, 70.0);
        assert_eq!(s.state_machine.main_deploy_altitude, 400.0);
        assert_eq!(s.state_machine.min_time_to_drogue, 1000);
        assert_eq!(s.misc.buzzer_volume, 50);
        assert_eq!(s.state_machine.main_on_time, 250);
        assert_eq!(s.state_machine.main_pulses, 1);
        assert_eq!(s.qd.disconnect_retract_percent_pressurant, 10);
    }

    #[test]
    fn metadata_describes_every_param() {
        let defaults = Params::default();
        for i in 0..Params::count() {
            let descriptor = Params::descriptor_by_index(i).unwrap();
            let metadata = Params::metadata(i).unwrap();
            assert_eq!(metadata.param_id, descriptor.mavlink_name());
            assert_eq!(
                metadata.default_value.to_bits(),
                defaults.get(descriptor.id).unwrap().to_mavlink().to_bits(),
                "{}",
                descriptor.name
            );
        }
        assert!(Params::metadata(Params::count()).is_none());

        let (index, _) = Params::by_id(0x0500).unwrap(); // MISC_BUZZ_VOL
        let metadata = Params::metadata(index).unwrap();
        assert_eq!(&metadata.units[..2], b"%\0");
        assert_eq!(metadata.min_value.to_bits(), 0);
        assert_eq!(metadata.max_value.to_bits(), 100);
        assert!(metadata.flags.is_empty());
        assert!(
            metadata
                .description
                .starts_with(b"Duty cycle of the buzzer PWM signal.")
        );

        // Without declared bounds, the type's own are reported.
        let (index, _) = Params::by_id(0x0200).unwrap(); // SM_MAIN_ALT
        let metadata = Params::metadata(index).unwrap();
        assert_eq!(metadata.min_value, f32::NEG_INFINITY);
        assert_eq!(metadata.max_value, f32::INFINITY);

        let (index, _) = Params::by_id(0x0803).unwrap(); // LORA_KEY_1
        assert_eq!(
            Params::metadata(index).unwrap().flags,
            ParamMetadataFlag::REBOOT_REQUIRED
        );
    }

    #[test]
    fn values_out_of_bounds_are_rejected() {
        let store = SharedParams::<Params>::new();
        store.init(Params::default());

        let name = Params::by_id(0x0500).unwrap().1.mavlink_name(); // MISC_BUZZ_VOL, 0-100
        assert_eq!(
            store.set(&name, f32::from_bits(101), MavParamType::Uint32),
            Err(ParamSetError::InvalidValue)
        );
        assert!(
            store
                .set(&name, f32::from_bits(100), MavParamType::Uint32)
                .is_ok()
        );

        let mut s = Params::default();
        s.apply_raw(0x0500, 101);
        assert_eq!(s.misc.buzzer_volume, 50);
    }

    #[test]
    fn get_set_by_id_routes_across_groups() {
        let mut s = Params::default();
        // A state-estimator param and a recovery param, by their ids.
        assert!(s.set(ParamId::new(0x0100), ParamValue::F32(0.25)));
        assert!(s.set(ParamId::new(0x0200), ParamValue::F32(250.0)));
        assert!(s.set(ParamId::new(0x0201), ParamValue::U32(1500)));
        assert_eq!(s.state_estimator.mahony_kp, 0.25);
        assert_eq!(s.state_machine.main_deploy_altitude, 250.0);
        assert_eq!(s.state_machine.min_time_to_drogue, 1500);
        assert_eq!(s.get(ParamId::new(0x0100)), Some(ParamValue::F32(0.25)));
        assert_eq!(s.get(ParamId::new(0x0200)), Some(ParamValue::F32(250.0)));
        // Unknown id and wrong-type set are rejected.
        assert!(!s.set(ParamId::new(0xffff), ParamValue::F32(1.0)));
        assert!(!s.set(ParamId::new(0x0200), ParamValue::U32(1)));
        assert_eq!(s.get(ParamId::new(0xffff)), None);
    }

    #[test]
    fn apply_raw_decodes_by_type() {
        let mut s = Params::default();
        s.apply_raw(0x0200, 275.0f32.to_bits());
        s.apply_raw(0x0202, 7000);
        assert_eq!(s.state_machine.main_deploy_altitude, 275.0);
        assert_eq!(s.state_machine.min_time_to_main, 7000);
        s.apply_raw(0xffff, 123); // ignored
    }

    #[test]
    fn shared_params_set_applies_and_echoes() {
        let store = SharedParams::<Params>::new();
        store.init(Params::default());

        let name = Params::by_id(0x0200).unwrap().1.mavlink_name(); // RC_MAIN_ALT
        let (id, raw, info) = store.set(&name, 275.0, MavParamType::Real32).unwrap();
        assert_eq!(id, 0x0200);
        assert_eq!(raw, 275.0f32.to_bits());
        assert_eq!(info.value, 275.0);
        assert_eq!(store.by_name(&name).unwrap().value, 275.0);

        // u32 params travel bytewise, so the wire float carries the integer's bit pattern rather
        // than its numeric value.
        let name = Params::by_id(0x0201).unwrap().1.mavlink_name(); // RC_MIN_T_DROGUE (u32)
        let (_, raw, info) = store
            .set(&name, f32::from_bits(1500), MavParamType::Uint32)
            .unwrap();
        assert_eq!(raw, 1500);
        assert_eq!(info.value.to_bits(), 1500);
    }

    #[test]
    fn shared_params_bytewise_survives_large_u32() {
        let store = SharedParams::<Params>::new();
        store.init(Params::default());

        // The whole point of bytewise: a by-value cast through f32 would round this to the nearest
        // representable float and lose the exact value.
        const LARGE: u32 = 0x0100_0001;
        let name = Params::by_id(0x0201).unwrap().1.mavlink_name();
        let (_, raw, _) = store
            .set(&name, f32::from_bits(LARGE), MavParamType::Uint32)
            .unwrap();
        assert_eq!(raw, LARGE);
        assert_ne!(LARGE as f32 as u32, LARGE, "cast would have been lossy");
    }

    #[test]
    fn shared_params_rejects_bad_input() {
        let store = SharedParams::<Params>::new();
        store.init(Params::default());

        let f32_name = Params::by_id(0x0200).unwrap().1.mavlink_name();
        let u32_name = Params::by_id(0x0201).unwrap().1.mavlink_name();
        assert_eq!(
            store.set(&f32_name, 1.0, MavParamType::Uint32),
            Err(ParamSetError::InvalidValue)
        );
        assert_eq!(
            store.set(&f32_name, f32::NAN, MavParamType::Real32),
            Err(ParamSetError::InvalidValue)
        );
        // Under bytewise encoding every bit pattern is a valid u32, so a u32 param has nothing to
        // reject: the float that reads as -5.0 is just one such pattern.
        assert!(store.set(&u32_name, -5.0, MavParamType::Uint32).is_ok());
        assert_eq!(
            store.set(&[0xab; 16], 1.0, MavParamType::Real32),
            Err(ParamSetError::UnknownParam)
        );
    }

    #[test]
    fn shared_params_uninitialized_answers_nothing() {
        let store = SharedParams::<Params>::new();
        assert!(store.by_index(0).is_none());
        assert_eq!(
            store.set(
                &Params::by_id(0x0200).unwrap().1.mavlink_name(),
                1.0,
                MavParamType::Real32
            ),
            Err(ParamSetError::UnknownParam)
        );
    }

    // A synthetic group exercising a Vector3 composite param (3 slots -> _X/_Y/_Z).
    #[derive(Debug, macros::ParameterGroup)]
    #[param_group(prefix = "TST")]
    struct VectorGroup {
        /// A vector
        #[param(id = 0x0300..=0x0302, name = "BIAS", default = [1.0, 2.0, 3.0])]
        bias: Vector3<f32>,
        /// A scalar
        #[param(id = 0x0303, name = "GAIN", default = 4.0)]
        gain: f32,
    }

    #[test]
    fn composite_vector_field() {
        assert_eq!(VectorGroup::PARAM_COUNT, 4);
        // Names get _X/_Y/_Z suffixes; the scalar has none.
        let names = [
            VectorGroup::descriptor(0).unwrap().name,
            VectorGroup::descriptor(1).unwrap().name,
            VectorGroup::descriptor(2).unwrap().name,
            VectorGroup::descriptor(3).unwrap().name,
        ];
        assert_eq!(
            names,
            ["TST_BIAS_X", "TST_BIAS_Y", "TST_BIAS_Z", "TST_GAIN"]
        );

        let mut g = VectorGroup::default();
        assert_eq!(g.bias, Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(g.gain, 4.0);

        // Each vector slot is independently addressable by id.
        assert_eq!(g.get(ParamId::new(0x0301)), Some(ParamValue::F32(2.0)));
        assert!(g.set(ParamId::new(0x0302), ParamValue::F32(9.0)));
        assert_eq!(g.bias, Vector3::new(1.0, 2.0, 9.0));
    }

    #[test]
    fn calibration_subtracts_offsets() {
        let cal = CalibrationParams {
            imu1_gyro: Vector3::new(1.0, 2.0, 3.0),
            mag: Vector3::new(10.0, 0.0, -10.0),
            baro2: 1.5,
            ..Default::default()
        };
        let mut r = SensorReadings {
            imu1_gyro: Some(Vector3::new(1.0, 1.0, 1.0)),
            mag: Some(Vector3::new(20.0, 20.0, 20.0)),
            baro2: BaroReading {
                pressure: Some(1000.0),
                temperature: Some(20.0),
            },
            ..Default::default()
        };
        let uncorrected_alt = r.baro2.altitude().unwrap();
        cal.apply(&mut r);

        assert_eq!(r.imu1_gyro, Some(Vector3::new(0.0, -1.0, -2.0)));
        assert_eq!(r.mag, Some(Vector3::new(10.0, 20.0, 30.0)));
        assert_eq!(r.imu2_gyro, None);
        assert_eq!(r.baro2.pressure, Some(998.5));
        assert_eq!(r.baro2.temperature, Some(20.0));
        assert_eq!(r.baro1.pressure, None);
        // ~8.3 m/hPa near sea level; lower pressure reads higher.
        let rise = r.baro2.altitude().unwrap() - uncorrected_alt;
        assert!((12.0..13.5).contains(&rise), "rise {rise}");
    }
}
