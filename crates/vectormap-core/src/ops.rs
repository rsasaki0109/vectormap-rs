//! Serializable high-level commands.
//!
//! A [`Command`] is the data form of an editing operation. It is what a tool
//! layer (CLI script, MCP server, language model) produces; the library
//! executes it deterministically with [`Map::apply`] or, atomically for a
//! batch, [`Map::apply_all`].
//!
//! ```json
//! {"op": "split_lane", "lane": 12, "at": {"fraction": 0.5}}
//! {"op": "connect_lanes", "from": 3, "to": 4}
//! {"op": "set_speed_limit", "lanes": [3, 4], "kmh": 40}
//! ```

use serde::{Deserialize, Serialize};

use crate::edit::{
    ChangeSet, EditError, NewConnector, NewCrosswalk, NewLane, NewRoad, NewStopLine,
    NewTrafficSignal, SplitAt, SplitOptions,
};
use crate::entities::{BoundaryKind, LaneKind, Side, SpeedLimit, TurnDirection};
use crate::geometry::{Polygon3, Polyline3};
use crate::id::{BoundaryId, CrosswalkId, EntityRef, LaneId, RegulatoryElementId, StopLineId};
use crate::map::Map;
use crate::topology::Neighbor;

fn default_true() -> bool {
    true
}

/// A high-level editing command.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum Command {
    /// [`Map::add_lane`]
    AddLane(NewLane),
    /// [`Map::build_road`]
    BuildRoad(NewRoad),
    /// [`Map::add_connector`]
    AddConnector(NewConnector),
    /// [`Map::remove_lane`]
    RemoveLane {
        /// Lane to remove.
        lane: LaneId,
    },
    /// [`Map::remove_entity`]
    RemoveEntity {
        /// Entity to remove.
        entity: EntityRef,
    },
    /// [`Map::connect`]
    ConnectLanes {
        /// Predecessor.
        from: LaneId,
        /// Successor.
        to: LaneId,
    },
    /// [`Map::disconnect`]
    DisconnectLanes {
        /// Predecessor.
        from: LaneId,
        /// Successor.
        to: LaneId,
    },
    /// [`Map::set_neighbor`]
    SetNeighbor {
        /// Lane.
        lane: LaneId,
        /// Side.
        side: Side,
        /// New neighbor, or `null` to clear.
        #[serde(default)]
        neighbor: Option<Neighbor>,
    },
    /// [`Map::split_lane`]
    SplitLane {
        /// Lane to split.
        lane: LaneId,
        /// Position.
        at: SplitAt,
        /// Split the whole lateral group (default `true`).
        #[serde(default = "default_true")]
        include_neighbors: bool,
    },
    /// [`Map::merge_lanes`]
    MergeLanes {
        /// Lane that is kept.
        first: LaneId,
        /// Its successor, merged into `first`.
        second: LaneId,
    },
    /// [`Map::add_stop_line`]
    AddStopLine(NewStopLine),
    /// [`Map::add_traffic_signal`]
    AddTrafficSignal(NewTrafficSignal),
    /// [`Map::add_crosswalk`]
    AddCrosswalk(NewCrosswalk),
    /// Change explicit control/crossing targets and stop-line links of an existing rule.
    SetRegulatoryLinks {
        /// Existing equipment rule; its physical feature references are preserved.
        regulatory_element: RegulatoryElementId,
        /// Controlled or crossing vehicle lanes.
        #[serde(default)]
        lanes: Vec<LaneId>,
        /// Controlled pedestrian crosswalks, separate from vehicle lanes.
        #[serde(default)]
        controlled_crosswalks: Vec<CrosswalkId>,
        /// Existing stop lines; at most one for a traffic light.
        #[serde(default)]
        stop_lines: Vec<StopLineId>,
    },
    /// [`Map::set_speed_limit`]
    SetSpeedLimit {
        /// Lanes.
        lanes: Vec<LaneId>,
        /// Limit in km/h, or `null` to clear.
        #[serde(default)]
        kmh: Option<f64>,
    },
    /// [`Map::set_turn_direction`]
    SetTurnDirection {
        /// Lane.
        lane: LaneId,
        /// Direction, or `null` to clear.
        #[serde(default)]
        turn_direction: Option<TurnDirection>,
    },
    /// [`Map::set_lane_kind`]
    SetLaneKind {
        /// Lane.
        lane: LaneId,
        /// New kind.
        kind: LaneKind,
    },
    /// [`Map::set_boundary_kind`]
    SetBoundaryKind {
        /// Boundary.
        boundary: BoundaryId,
        /// New kind.
        kind: BoundaryKind,
    },
    /// [`Map::set_boundary_geometry`]
    SetBoundaryGeometry {
        /// Boundary.
        boundary: BoundaryId,
        /// New geometry, in the boundary's own direction.
        geometry: Polyline3,
    },
    /// [`Map::set_attribute`]
    SetAttribute {
        /// Entity.
        entity: EntityRef,
        /// Key.
        key: String,
        /// Value, or `null` to remove.
        #[serde(default)]
        value: Option<String>,
    },
    /// [`Map::add_road`]
    AddRoad {
        /// Name.
        #[serde(default)]
        name: Option<String>,
        /// Member lanes.
        lanes: Vec<LaneId>,
    },
    /// [`Map::add_junction`]
    AddJunction {
        /// Name.
        #[serde(default)]
        name: Option<String>,
        /// Connecting lanes.
        lanes: Vec<LaneId>,
        /// Outline.
        #[serde(default)]
        outline: Option<Polygon3>,
    },
}

impl Command {
    /// Snake-case name of the operation (the `op` field).
    pub fn name(&self) -> &'static str {
        match self {
            Command::AddLane(_) => "add_lane",
            Command::BuildRoad(_) => "build_road",
            Command::AddConnector(_) => "add_connector",
            Command::RemoveLane { .. } => "remove_lane",
            Command::RemoveEntity { .. } => "remove_entity",
            Command::ConnectLanes { .. } => "connect_lanes",
            Command::DisconnectLanes { .. } => "disconnect_lanes",
            Command::SetNeighbor { .. } => "set_neighbor",
            Command::SplitLane { .. } => "split_lane",
            Command::MergeLanes { .. } => "merge_lanes",
            Command::AddStopLine(_) => "add_stop_line",
            Command::AddTrafficSignal(_) => "add_traffic_signal",
            Command::AddCrosswalk(_) => "add_crosswalk",
            Command::SetRegulatoryLinks { .. } => "set_regulatory_links",
            Command::SetSpeedLimit { .. } => "set_speed_limit",
            Command::SetTurnDirection { .. } => "set_turn_direction",
            Command::SetLaneKind { .. } => "set_lane_kind",
            Command::SetBoundaryKind { .. } => "set_boundary_kind",
            Command::SetBoundaryGeometry { .. } => "set_boundary_geometry",
            Command::SetAttribute { .. } => "set_attribute",
            Command::AddRoad { .. } => "add_road",
            Command::AddJunction { .. } => "add_junction",
        }
    }
}

/// A failed command inside [`Map::apply_all`].
#[derive(Debug, Clone, PartialEq, thiserror::Error, Serialize, Deserialize)]
#[error("command #{index} ({op}) failed: {error}")]
pub struct BatchError {
    /// Zero-based index of the failing command.
    pub index: usize,
    /// Its operation name.
    pub op: String,
    /// The error.
    pub error: EditError,
}

impl Map {
    /// Executes a single command.
    pub fn apply(&mut self, command: &Command) -> Result<ChangeSet, EditError> {
        match command.clone() {
            Command::AddLane(spec) => self.add_lane(spec).map(|(_, cs)| cs),
            Command::BuildRoad(spec) => self.build_road(spec).map(|(_, cs)| cs),
            Command::AddConnector(spec) => self.add_connector(spec).map(|(_, cs)| cs),
            Command::RemoveLane { lane } => self.remove_lane(lane),
            Command::RemoveEntity { entity } => self.remove_entity(entity),
            Command::ConnectLanes { from, to } => self.connect(from, to),
            Command::DisconnectLanes { from, to } => self.disconnect(from, to),
            Command::SetNeighbor {
                lane,
                side,
                neighbor,
            } => self.set_neighbor(lane, side, neighbor),
            Command::SplitLane {
                lane,
                at,
                include_neighbors,
            } => self
                .split_lane(lane, at, SplitOptions { include_neighbors })
                .map(|(_, cs)| cs),
            Command::MergeLanes { first, second } => self.merge_lanes(first, second),
            Command::AddStopLine(spec) => self.add_stop_line(spec).map(|(_, cs)| cs),
            Command::AddTrafficSignal(spec) => self.add_traffic_signal(spec).map(|(_, cs)| cs),
            Command::AddCrosswalk(spec) => self.add_crosswalk(spec).map(|(_, cs)| cs),
            Command::SetRegulatoryLinks {
                regulatory_element,
                lanes,
                controlled_crosswalks,
                stop_lines,
            } => self.set_regulatory_links(
                regulatory_element,
                &lanes,
                &controlled_crosswalks,
                &stop_lines,
            ),
            Command::SetSpeedLimit { lanes, kmh } => {
                self.set_speed_limit(&lanes, kmh.map(SpeedLimit::from_kmh))
            }
            Command::SetTurnDirection {
                lane,
                turn_direction,
            } => self.set_turn_direction(lane, turn_direction),
            Command::SetLaneKind { lane, kind } => self.set_lane_kind(lane, kind),
            Command::SetBoundaryKind { boundary, kind } => self.set_boundary_kind(boundary, kind),
            Command::SetBoundaryGeometry { boundary, geometry } => {
                self.set_boundary_geometry(boundary, geometry)
            }
            Command::SetAttribute { entity, key, value } => {
                self.set_attribute(entity, &key, value.as_deref())
            }
            Command::AddRoad { name, lanes } => self.add_road(name, &lanes).map(|(_, cs)| cs),
            Command::AddJunction {
                name,
                lanes,
                outline,
            } => self.add_junction(name, &lanes, outline).map(|(_, cs)| cs),
        }
    }

    /// Executes a batch of commands atomically: either all succeed and their
    /// change sets are returned, or the map is left unchanged and the first
    /// failure is reported.
    pub fn apply_all(&mut self, commands: &[Command]) -> Result<Vec<ChangeSet>, BatchError> {
        let mut scratch = self.clone();
        let mut results = Vec::with_capacity(commands.len());
        for (index, cmd) in commands.iter().enumerate() {
            match scratch.apply(cmd) {
                Ok(cs) => results.push(cs),
                Err(error) => {
                    return Err(BatchError {
                        index,
                        op: cmd.name().to_string(),
                        error,
                    });
                }
            }
        }
        *self = scratch;
        Ok(results)
    }
}
