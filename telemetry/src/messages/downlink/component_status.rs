use serde::{Deserialize, Serialize};

use mission::bus::{IO_NODE_IDS, NodeStatus};
use mission::mavlink::{NodeHealth, VehicleSnapshot, io_node_sys_status};
use rapid_dialect::rapid::messages::SysStatus;

use super::{ConnectionContext, DownlinkTelemetryMessage};

const ENTRIES: usize = 3;

/// Node ids are four bits wide, the bits above them carry the SYS_STATUS sensor bits.
const NODE_ID_MASK: u32 = 0b1111;
const ENABLED: u32 = 1 << 4;
const HEALTHY: u32 = 1 << 5;

/// The error counters follow the id and flags, five bits each.
const COUNT_SHIFT: u32 = 6;
const COUNT_BITS: u32 = 5;
const COUNT_MASK: u32 = (1 << COUNT_BITS) - 1;

/// An empty entry: node 0 is MAVLink's "all components", never a board.
const NO_NODE: u8 = 0;

const _: () = assert!(mission::bus::NODE_ID_COUNT == NODE_ID_MASK as usize + 1);
const _: () = assert!(COUNT_SHIFT + COUNT_BITS * ERROR_COUNTS as u32 <= u32::BITS);

/// SYS_STATUS's error counters in field order: `errors_comm`, then `errors_count1` to `4`.
pub type ErrorCounts = [u16; ERROR_COUNTS];
const ERROR_COUNTS: usize = 5;

pub(crate) fn error_counts(status: &SysStatus) -> ErrorCounts {
    [
        status.errors_comm,
        status.errors_count1,
        status.errors_count2,
        status.errors_count3,
        status.errors_count4,
    ]
}

pub(crate) fn set_error_counts(status: &mut SysStatus, counts: ErrorCounts) {
    [
        status.errors_comm,
        status.errors_count1,
        status.errors_count2,
        status.errors_count3,
        status.errors_count4,
    ] = counts;
}

/// One component's SYS_STATUS error counters, each cut down to its low five bits. The ground
/// station alerts on a counter going up, so an occasional wrap is a missed alert at worst, where
/// saturating would hide every error after the first few dozen.
#[derive(Copy, Clone, Debug, Default, Serialize, Deserialize)]
struct PackedComponent(#[serde(with = "postcard::fixint::le")] u32);

/// The error counters of the flight computer and every board on the bus, a few per packet.
#[derive(Debug, Serialize, Deserialize)]
pub struct ComponentStatusMessage {
    components: [PackedComponent; ENTRIES],
}

impl PackedComponent {
    /// `id_flags` is the node id, plus [`ENABLED`] and [`HEALTHY`] for an IO board.
    #[allow(
        clippy::arithmetic_side_effects,
        reason = "shifts by constants that the const assert keeps within the u32"
    )]
    fn new(id_flags: u32, status: &SysStatus) -> Self {
        let counts = error_counts(status)
            .into_iter()
            .enumerate()
            .fold(0, |acc, (i, count)| {
                acc | ((u32::from(count) & COUNT_MASK) << (COUNT_SHIFT + COUNT_BITS * i as u32))
            });
        Self(id_flags | counts)
    }

    fn node_id(self) -> u8 {
        (self.0 & NODE_ID_MASK) as u8
    }

    #[allow(
        clippy::arithmetic_side_effects,
        reason = "shifts by constants that the const assert keeps within the u32"
    )]
    fn counts(self) -> ErrorCounts {
        core::array::from_fn(|i| {
            ((self.0 >> (COUNT_SHIFT + COUNT_BITS * i as u32)) & COUNT_MASK) as u16
        })
    }
}

impl DownlinkTelemetryMessage for ComponentStatusMessage {
    const ID: u8 = 0x0a;
    /// The snapshot, and how many of these messages have been sent before, which picks the page.
    type Input<'a> = (&'a VehicleSnapshot<'a>, u32);
    /// The IO boards' SYS_STATUS, by component id.
    type Output = [Option<(u8, SysStatus)>; ENTRIES];

    fn pack((snapshot, sequence): Self::Input<'_>) -> Self {
        let present = || {
            core::iter::once(links::SELF_COMPONENT_ID).chain(
                IO_NODE_IDS
                    .into_iter()
                    .filter(|&id| snapshot.input_image.nodes.contains(id)),
            )
        };

        // The flight computer is always present, so there is always at least one page.
        let pages = present().count().div_ceil(ENTRIES).max(1);
        #[allow(
            clippy::arithmetic_side_effects,
            reason = "pages >= 1, and page * ENTRIES is bounded by the node count"
        )]
        let skip = (sequence as usize % pages) * ENTRIES;

        let mut components = [PackedComponent::default(); ENTRIES];
        for (slot, node_id) in components.iter_mut().zip(present().skip(skip)) {
            *slot = if node_id == links::SELF_COMPONENT_ID {
                PackedComponent::new(u32::from(node_id), &snapshot.into())
            } else if let Some(health) = snapshot.node_health(node_id) {
                let mut flags = u32::from(node_id);
                if health.enabled {
                    flags |= ENABLED;
                }
                if health.healthy {
                    flags |= HEALTHY;
                }
                PackedComponent::new(flags, &io_node_sys_status(node_id, &health))
            } else {
                PackedComponent::default()
            };
        }

        Self { components }
    }

    fn unpack(self, context: &mut ConnectionContext) -> Self::Output {
        self.components.map(|c| {
            let node_id = c.node_id();
            let [comm_errors, errors @ ..] = c.counts();

            if node_id == links::SELF_COMPONENT_ID {
                context.self_errors = Some(c.counts());
                return None;
            }
            if node_id == NO_NODE {
                return None;
            }

            // Voltage and current only travel in the battery message, for the power boards.
            let health = NodeHealth {
                enabled: c.0 & ENABLED != 0,
                healthy: c.0 & HEALTHY != 0,
                status: NodeStatus {
                    comm_errors,
                    errors,
                    ..NodeStatus::UNKNOWN
                },
            };
            Some((node_id, io_node_sys_status(node_id, &health)))
        })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::super::tests::{SnapshotParts, through_packet};
    use super::*;

    use mission::bus::{BusInputImage, NodeSet};

    use crate::messages::DownlinkMessage;

    /// Every board present, armed and at the top of every counter, for the payload-length check.
    pub(crate) fn saturate_nodes(inputs: &mut BusInputImage) {
        inputs.nodes = NodeSet::from_bits(u16::MAX);
        inputs.nodes_armed = NodeSet::from_bits(u16::MAX);
        for status in &mut inputs.node_status {
            status.comm_errors = u16::MAX;
            status.errors = [u16::MAX; 4];
        }
    }

    fn round_trip(
        parts: &SnapshotParts,
        sequence: u32,
        context: &mut ConnectionContext,
    ) -> [Option<(u8, SysStatus)>; ENTRIES] {
        let msg = DownlinkMessage::ComponentStatus(ComponentStatusMessage::pack((
            &parts.snapshot(),
            sequence,
        )));
        let DownlinkMessage::ComponentStatus(decoded) = through_packet(msg) else {
            panic!("decoded as the wrong message")
        };
        decoded.unpack(context)
    }

    /// Paging through the present components reaches each of them, the flight computer included,
    /// within one round of pages.
    #[test]
    fn every_present_component_is_reached() {
        let mut parts = SnapshotParts::default();
        let boards = [2, 5, 11, 12, 15];
        for id in boards {
            parts.inputs.nodes.set(id, true);
        }

        let mut context = ConnectionContext::init(0);
        let mut seen = NodeSet::NONE;
        // Six components, three per page.
        for sequence in 0..2 {
            for (node_id, _) in round_trip(&parts, sequence, &mut context)
                .into_iter()
                .flatten()
            {
                seen.set(node_id, true);
            }
        }

        for id in boards {
            assert!(seen.contains(id), "node {id} was never sent");
        }
        assert!(
            context.self_errors.is_some(),
            "the flight computer was never sent"
        );
    }

    #[test]
    fn counters_and_flags_survive_the_packet() {
        let mut parts = SnapshotParts::default();
        parts.inputs.nodes.set(4, true);
        parts.inputs.nodes_armed.set(4, true);
        parts.inputs.node_status[4].comm_errors = 7;
        parts.inputs.node_status[4].errors = [31, 32, 33, 0];

        let mut context = ConnectionContext::init(0);
        let [_, board, _] = round_trip(&parts, 0, &mut context);
        let (node_id, status) = board.expect("node 4 is present");

        let health = parts.snapshot().node_health(4).expect("node 4 is present");
        let wired = io_node_sys_status(4, &health);

        assert_eq!(node_id, 4);
        assert_eq!(
            status.onboard_control_sensors_enabled,
            wired.onboard_control_sensors_enabled
        );
        assert_eq!(
            status.onboard_control_sensors_health,
            wired.onboard_control_sensors_health
        );
        // Wrapped rather than saturated, so a counter keeps moving.
        assert_eq!(error_counts(&status), [7, 31, 0, 1, 0]);
    }
}
