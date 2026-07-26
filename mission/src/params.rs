//! The tunable parameters and the bridge to the MAVLink parameter protocol.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use rapid_dialect::rapid::enums::MavParamType;

use state_estimator::StateEstimatorParams;

use links::protocols::params::{ParamInfo, ParamSetError, ParamStore};

pub use params::{ParamDescriptor, ParamId, ParamType, ParamValue, ParameterField, ParameterGroup};

/// Top-level params, aggregating the per-subsystem parameter groups.
///
/// Each field is expected to implement [`ParameterGroup`].
#[derive(Debug, Default, Clone, macros::ParameterGroups)]
pub struct Params {
    pub state_estimator: StateEstimatorParams,
    pub recovery: RecoveryParams,
}

/// Recovery / parachute deployment parameters, exposed over MAVLink as `REC_*`.
#[derive(Debug, Clone, macros::ParameterGroup)]
#[param_group(prefix = "REC")]
pub struct RecoveryParams {
    /// Altitude AGL (meters) at which to deploy the main parachute
    #[param(id = 0x0200, name = "MAIN_ALT", default = 400.0)]
    pub main_deploy_altitude: f32,
    /// Minimum time (ms) after launch before allowing drogue deployment
    #[param(id = 0x0201, name = "MIN_T_DROGUE", default = 1000)]
    pub min_time_to_drogue: u32,
    /// Minimum time (ms) after drogue before allowing main deployment
    #[param(id = 0x0202, name = "MIN_T_MAIN", default = 3000)]
    pub min_time_to_main: u32,
}

/// Live mirror of the current [`Params`] for the MAVLink param protocol tasks. Starts empty (the
/// protocol won't answer until [`SharedParams::init`] runs at boot) and is updated on every
/// accepted PARAM_SET. The vehicle applies the same value via
/// [`UplinkCommand::SetParam`](links::UplinkCommand) one main-loop tick later; the param path is
/// the only writer to both, so the two copies cannot diverge.
pub struct SharedParams {
    inner: Mutex<CriticalSectionRawMutex, RefCell<Option<Params>>>,
}

impl Params {
    /// Total number of parameters exposed over MAVLink.
    pub fn count() -> u16 {
        Self::PARAM_COUNT as u16
    }

    /// Descriptor at a MAVLink `param_index`.
    pub fn descriptor_by_index(index: u16) -> Option<ParamDescriptor> {
        Self::descriptor(usize::from(index))
    }

    /// Descriptor for a stable parameter id (linear scan; the set is small).
    pub fn by_id(id: u16) -> Option<ParamDescriptor> {
        (0..Self::count()).find_map(|i| Self::descriptor_by_index(i).filter(|d| d.id.get() == id))
    }

    /// (`param_index`, descriptor) for a MAVLink `param_id` name.
    pub fn by_name(name: &[u8; 16]) -> Option<(u16, ParamDescriptor)> {
        (0..Self::count()).find_map(|i| {
            Self::descriptor_by_index(i).and_then(|d| (d.mavlink_name() == *name).then_some((i, d)))
        })
    }

    /// Apply a raw stored value (as read from flash) to this struct. Unknown ids are ignored.
    pub fn apply_raw(&mut self, id: u16, raw: u32) {
        if let Some(descriptor) = Self::by_id(id) {
            self.set(descriptor.id, descriptor.ty.decode_raw(raw));
        }
    }
}

impl SharedParams {
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(None)),
        }
    }

    pub fn init(&self, params: Params) {
        self.inner.lock(|inner| {
            *inner.borrow_mut() = Some(params);
        });
    }

    fn info(&self, index: u16, descriptor: &ParamDescriptor) -> Option<ParamInfo> {
        self.inner.lock(|inner| {
            inner.borrow().as_ref().and_then(|params| {
                params.get(descriptor.id).map(|value| ParamInfo {
                    name: descriptor.mavlink_name(),
                    value: value.to_mavlink(),
                    ty: descriptor.ty.into(),
                    index,
                    count: Params::count(),
                })
            })
        })
    }
}

impl Default for SharedParams {
    fn default() -> Self {
        Self::new()
    }
}

impl ParamStore for SharedParams {
    fn count(&self) -> u16 {
        Params::count()
    }

    fn by_index(&self, index: u16) -> Option<ParamInfo> {
        Params::descriptor_by_index(index).and_then(|d| self.info(index, &d))
    }

    fn by_name(&self, name: &[u8; 16]) -> Option<ParamInfo> {
        Params::by_name(name).and_then(|(index, d)| self.info(index, &d))
    }

    fn set(
        &self,
        name: &[u8; 16],
        value: f32,
        ty: MavParamType,
    ) -> Result<(u16, u32, ParamInfo), ParamSetError> {
        let (index, descriptor) = Params::by_name(name).ok_or(ParamSetError::UnknownParam)?;

        if ty != MavParamType::from(descriptor.ty) {
            return Err(ParamSetError::InvalidValue);
        }

        let value = descriptor
            .ty
            .from_mavlink(value)
            .ok_or(ParamSetError::InvalidValue)?;

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
                ParamInfo {
                    name: descriptor.mavlink_name(),
                    value: stored.to_mavlink(),
                    ty: descriptor.ty.into(),
                    index,
                    count: Params::count(),
                },
            ))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

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
        assert_eq!(s.state_estimator.std_dev_barometer_transsonic, 5000.0);
        assert_eq!(s.recovery.main_deploy_altitude, 400.0);
        assert_eq!(s.recovery.min_time_to_drogue, 1000);
    }

    #[test]
    fn get_set_by_id_routes_across_groups() {
        let mut s = Params::default();
        // A state-estimator param and a recovery param, by their ids.
        assert!(s.set(ParamId::new(0x0100), ParamValue::F32(0.25)));
        assert!(s.set(ParamId::new(0x0200), ParamValue::F32(250.0)));
        assert!(s.set(ParamId::new(0x0201), ParamValue::U32(1500)));
        assert_eq!(s.state_estimator.mahony_kp, 0.25);
        assert_eq!(s.recovery.main_deploy_altitude, 250.0);
        assert_eq!(s.recovery.min_time_to_drogue, 1500);
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
        assert_eq!(s.recovery.main_deploy_altitude, 275.0);
        assert_eq!(s.recovery.min_time_to_main, 7000);
        s.apply_raw(0xffff, 123); // ignored
    }

    #[test]
    fn shared_params_set_applies_and_echoes() {
        let store = SharedParams::new();
        store.init(Params::default());

        let name = Params::by_id(0x0200).unwrap().mavlink_name(); // RC_MAIN_ALT
        let (id, raw, info) = store.set(&name, 275.0, MavParamType::Real32).unwrap();
        assert_eq!(id, 0x0200);
        assert_eq!(raw, 275.0f32.to_bits());
        assert_eq!(info.value, 275.0);
        assert_eq!(store.by_name(&name).unwrap().value, 275.0);

        // u32 params travel bytewise, so the wire float carries the integer's bit pattern rather
        // than its numeric value.
        let name = Params::by_id(0x0201).unwrap().mavlink_name(); // RC_MIN_T_DROGUE (u32)
        let (_, raw, info) = store
            .set(&name, f32::from_bits(1500), MavParamType::Uint32)
            .unwrap();
        assert_eq!(raw, 1500);
        assert_eq!(info.value.to_bits(), 1500);
    }

    #[test]
    fn shared_params_bytewise_survives_large_u32() {
        let store = SharedParams::new();
        store.init(Params::default());

        // The whole point of bytewise: a by-value cast through f32 would round this to the nearest
        // representable float and lose the exact value.
        const LARGE: u32 = 0x0100_0001;
        let name = Params::by_id(0x0201).unwrap().mavlink_name();
        let (_, raw, _) = store
            .set(&name, f32::from_bits(LARGE), MavParamType::Uint32)
            .unwrap();
        assert_eq!(raw, LARGE);
        assert_ne!(LARGE as f32 as u32, LARGE, "cast would have been lossy");
    }

    #[test]
    fn shared_params_rejects_bad_input() {
        let store = SharedParams::new();
        store.init(Params::default());

        let f32_name = Params::by_id(0x0200).unwrap().mavlink_name();
        let u32_name = Params::by_id(0x0201).unwrap().mavlink_name();
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
        let store = SharedParams::new();
        assert!(store.by_index(0).is_none());
        assert_eq!(
            store.set(
                &Params::by_id(0x0200).unwrap().mavlink_name(),
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
        #[param(id = 0x0300..=0x0302, name = "BIAS", default = [1.0, 2.0, 3.0])]
        bias: Vector3<f32>,
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
}
