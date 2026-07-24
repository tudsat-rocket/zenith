//! The inventory of off-PCB vehicle components addressed by the flight computer.
//!
//! ID enums for every valve, sensor and output, and [`InventoryMap`] for storing one value per
//! component.

use core::marker::PhantomData;
use core::ops::{Index, IndexMut};

pub use rapid_dialect::rapid::enums::ValveId;
use rapid_dialect::rapid::enums::{FluidType, PressureVesselFlag};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum TankId {
    Pressurant,
    Oxidizer,
    CombustionChamber,
    RegulatedPressurant,
    ExternalPressurant,
    ExternalOxidizer,
}

/// Every temperature sensor on the IO boards.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum TempSensId {
    OxTankUpper,
    OxTankLower,
}

/// Every pressure sensor on the IO boards.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum PressSensId {
    Nosecone,
    PressurantTank,
    PReg1,
    PReg2,
    OxTankUpper,
    OxTankLower,
    CombustionChamber,
    ExternalPressurant,
    ExternalOxidizer,
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum BinaryOutputId {
    Igniter1,
    Igniter2,
    Camera1,
    Camera2,
    Camera3,
}

/// A fixed-size array indexed by an id enum instead of a raw usize.
pub struct InventoryMap<I, T, const N: usize> {
    values: [T; N],
    _ids: PhantomData<I>,
}

pub type ValveMap<T> = InventoryMap<ValveId, T, 9>;
pub type TemperatureSensorMap<T> = InventoryMap<TempSensId, T, 2>;
pub type PressureSensorMap<T> = InventoryMap<PressSensId, T, 9>;
pub type BinaryOutputMap<T> = InventoryMap<BinaryOutputId, T, 5>;
pub type TankMap<T> = InventoryMap<TankId, T, 6>;

/// An id enum that can key an [`InventoryMap`]: N variants, each mapping to a unique dense index
/// in 0..N.
pub trait InventoryId<const N: usize>: Copy {
    /// All variants, in index order (`ALL[i].idx() == i`).
    const ALL: [Self; N];

    /// Dense storage index. Not necessarily the wire discriminant: MAVLink ids may be 1-indexed.
    fn idx(self) -> usize;
}

impl TankId {
    pub fn flags(&self) -> PressureVesselFlag {
        match self {
            TankId::ExternalPressurant | TankId::ExternalOxidizer => PressureVesselFlag::EXTERNAL,
            TankId::Pressurant
            | TankId::Oxidizer
            | TankId::CombustionChamber
            | TankId::RegulatedPressurant => PressureVesselFlag::empty(),
        }
    }

    pub fn volume_l(&self) -> f32 {
        // TODO
        match self {
            TankId::Pressurant => 2.0,
            TankId::Oxidizer => 8.0,
            TankId::CombustionChamber | TankId::RegulatedPressurant => 0.0,
            TankId::ExternalPressurant => 50.0,
            TankId::ExternalOxidizer => 40.0,
        }
    }

    pub fn fluid(&self) -> FluidType {
        match self {
            TankId::Pressurant | TankId::RegulatedPressurant | TankId::ExternalPressurant => {
                FluidType::Nitrogen
            }
            TankId::Oxidizer | TankId::ExternalOxidizer => FluidType::NitrousOxide,
            TankId::CombustionChamber => FluidType::Combustion,
        }
    }

    pub fn pressure_rating_bar(&self) -> f32 {
        // TODO
        match self {
            TankId::Pressurant | TankId::ExternalPressurant => 300.0,
            TankId::RegulatedPressurant => 60.0,
            TankId::Oxidizer | TankId::CombustionChamber | TankId::ExternalOxidizer => 55.0,
        }
    }

    pub fn pressure_sensors(&self) -> [Option<PressSensId>; 2] {
        match self {
            TankId::Pressurant => [Some(PressSensId::PressurantTank), None],
            TankId::Oxidizer => [
                Some(PressSensId::OxTankUpper),
                Some(PressSensId::OxTankLower),
            ],
            TankId::CombustionChamber => [Some(PressSensId::CombustionChamber), None],
            TankId::RegulatedPressurant => [Some(PressSensId::PReg1), Some(PressSensId::PReg2)],
            TankId::ExternalPressurant => [Some(PressSensId::ExternalPressurant), None],
            TankId::ExternalOxidizer => [Some(PressSensId::ExternalOxidizer), None],
        }
    }

    pub fn temperature_sensors(&self) -> [Option<TempSensId>; 2] {
        match self {
            TankId::Oxidizer => [Some(TempSensId::OxTankUpper), Some(TempSensId::OxTankLower)],
            TankId::Pressurant
            | TankId::CombustionChamber
            | TankId::RegulatedPressurant
            | TankId::ExternalPressurant
            | TankId::ExternalOxidizer => [None, None],
        }
    }
}

impl InventoryId<9> for ValveId {
    // The MavLink type has enum values Extra{1..10} with higher values, we simply ignore those
    const ALL: [Self; 9] = [
        Self::PressurantVent,
        Self::Pressurization,
        Self::OxidizerVent,
        Self::OxidizerFill,
        Self::Main,
        Self::ExternalPressurantFill,
        Self::ExternalOxidizerFill,
        Self::ExternalPressurantVent,
        Self::ExternalOxidizerVent,
    ];

    fn idx(self) -> usize {
        self as usize
    }
}

impl InventoryId<2> for TempSensId {
    const ALL: [Self; 2] = [Self::OxTankUpper, Self::OxTankLower];

    fn idx(self) -> usize {
        self as usize
    }
}

impl InventoryId<9> for PressSensId {
    const ALL: [Self; 9] = [
        Self::Nosecone,
        Self::PressurantTank,
        Self::PReg1,
        Self::PReg2,
        Self::OxTankUpper,
        Self::OxTankLower,
        Self::CombustionChamber,
        Self::ExternalPressurant,
        Self::ExternalOxidizer,
    ];

    fn idx(self) -> usize {
        self as usize
    }
}

impl InventoryId<5> for BinaryOutputId {
    const ALL: [Self; 5] = [
        Self::Igniter1,
        Self::Igniter2,
        Self::Camera1,
        Self::Camera2,
        Self::Camera3,
    ];

    fn idx(self) -> usize {
        self as usize
    }
}

impl InventoryId<6> for TankId {
    const ALL: [Self; 6] = [
        Self::Pressurant,
        Self::Oxidizer,
        Self::CombustionChamber,
        Self::RegulatedPressurant,
        Self::ExternalPressurant,
        Self::ExternalOxidizer,
    ];

    fn idx(self) -> usize {
        self as usize
    }
}

impl<I, T: Clone, const N: usize> Clone for InventoryMap<I, T, N> {
    fn clone(&self) -> Self {
        Self {
            values: self.values.clone(),
            _ids: PhantomData,
        }
    }
}

impl<I, T: Copy, const N: usize> Copy for InventoryMap<I, T, N> {}

impl<I, T: PartialEq, const N: usize> PartialEq for InventoryMap<I, T, N> {
    fn eq(&self, other: &Self) -> bool {
        self.values == other.values
    }
}

impl<I, T: Eq, const N: usize> Eq for InventoryMap<I, T, N> {}

impl<I, T: core::fmt::Debug, const N: usize> core::fmt::Debug for InventoryMap<I, T, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.values.fmt(f)
    }
}

impl<I: InventoryId<N>, T, const N: usize> InventoryMap<I, T, N> {
    pub const fn new(values: [T; N]) -> Self {
        Self {
            values,
            _ids: PhantomData,
        }
    }

    pub fn from_fn(f: impl FnMut(I) -> T) -> Self {
        Self::new(I::ALL.map(f))
    }

    /// Visit every entry together with its id. An exhaustive `match` on the
    /// id in `f` turns adding a variant into a compile error at every call
    /// site, forcing the new id to be handled.
    pub fn update(&mut self, mut f: impl FnMut(I, &mut T)) {
        for (id, value) in I::ALL.into_iter().zip(self.values.iter_mut()) {
            f(id, value);
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = (I, &T)> {
        I::ALL.into_iter().zip(self.values.iter())
    }

    pub const fn values(&self) -> &[T; N] {
        &self.values
    }
}

impl<I: InventoryId<N>, T: Copy, const N: usize> InventoryMap<I, T, N> {
    pub const fn splat(value: T) -> Self {
        Self::new([value; N])
    }
}

impl<I: InventoryId<N>, T, const N: usize> Index<I> for InventoryMap<I, T, N> {
    type Output = T;

    #[allow(
        clippy::indexing_slicing,
        reason = "idx() is a dense 0..N index by the InventoryId<N> contract"
    )]
    fn index(&self, id: I) -> &T {
        &self.values[id.idx()]
    }
}

impl<I: InventoryId<N>, T, const N: usize> IndexMut<I> for InventoryMap<I, T, N> {
    #[allow(
        clippy::indexing_slicing,
        reason = "idx() is a dense 0..N index by the InventoryId<N> contract"
    )]
    fn index_mut(&mut self, id: I) -> &mut T {
        &mut self.values[id.idx()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // InventoryMap storage relies on the ALL lists being in index order
    #[test]
    fn id_lists_are_in_index_order() {
        fn check<I: InventoryId<N>, const N: usize>() {
            for (i, id) in I::ALL.into_iter().enumerate() {
                assert_eq!(id.idx(), i);
            }
        }
        check::<ValveId, 9>();
        check::<TempSensId, 2>();
        check::<PressSensId, 9>();
        check::<BinaryOutputId, 4>();
        check::<TankId, 6>();
    }
}
