//! High-level editing operations.
//!
//! Every operation is a method on [`Map`] that
//!
//! 1. validates all of its inputs **before** touching the map, so a failed
//!    operation leaves the map unchanged, and
//! 2. returns a [`ChangeSet`] describing exactly what was created, modified
//!    and deleted, plus structured warnings.
//!
//! The same operations are available as serializable
//! [`Command`](crate::Command)s for tools such as MCP servers.

mod features;
mod lane;
mod merge;
mod regulatory;
mod road;
mod split;

use serde::{Deserialize, Serialize};

use crate::diagnostics::{Issue, codes};
use crate::entities::{BoundaryKind, LaneKind, Side, SpeedLimit, TurnDirection};
use crate::geometry::{Polygon3, Polyline3};
use crate::id::{BoundaryId, EntityKind, EntityRef, JunctionId, LaneId, RoadId};
use crate::map::Map;
use crate::topology::{Neighbor, reciprocal_side};
use crate::{Junction, Road};

pub use features::{
    CrosswalkGeometry, DEFAULT_SIGNAL_ELEVATION, DEFAULT_SIGNAL_HEIGHT, DEFAULT_SIGNAL_WIDTH,
    NewCrosswalk, NewStopLine, NewTrafficSignal, StopLineChoice, StopLinePlacement, StopRule,
};
pub use lane::{BoundarySpec, LaneGeometry, NewLane};
pub use road::{BuiltRoad, LaneDirection, NewConnector, NewRoad, RoadLane, TURN_THRESHOLD_DEG};
pub use split::{MIN_PIECE_LENGTH, SplitAt, SplitOptions};

/// Summary of the effects of an editing operation.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct ChangeSet {
    /// Newly created entities.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub created: Vec<EntityRef>,
    /// Existing entities that were changed.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub modified: Vec<EntityRef>,
    /// Entities that were removed.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub deleted: Vec<EntityRef>,
    /// Things the caller should know about.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub warnings: Vec<Issue>,
}

impl ChangeSet {
    /// An empty change set.
    pub fn new() -> Self {
        Self::default()
    }

    /// `true` if nothing changed (warnings are ignored).
    pub fn is_empty(&self) -> bool {
        self.created.is_empty() && self.modified.is_empty() && self.deleted.is_empty()
    }

    pub(crate) fn created(&mut self, e: impl Into<EntityRef>) {
        self.created.push(e.into());
    }

    pub(crate) fn modified(&mut self, e: impl Into<EntityRef>) {
        self.modified.push(e.into());
    }

    pub(crate) fn deleted(&mut self, e: impl Into<EntityRef>) {
        self.deleted.push(e.into());
    }

    pub(crate) fn warn(&mut self, issue: Issue) {
        self.warnings.push(issue);
    }

    /// Appends another change set.
    pub fn extend(&mut self, other: ChangeSet) {
        self.created.extend(other.created);
        self.modified.extend(other.modified);
        self.deleted.extend(other.deleted);
        self.warnings.extend(other.warnings);
        self.normalize();
    }

    /// Sorts and deduplicates the lists. An entity that was created in this
    /// change set is never also listed as modified; a deleted entity is
    /// listed only as deleted (or not at all if it was also created).
    pub(crate) fn normalize(&mut self) {
        self.created.sort();
        self.created.dedup();
        self.deleted.sort();
        self.deleted.dedup();
        let created_and_deleted: Vec<EntityRef> = self
            .created
            .iter()
            .filter(|e| self.deleted.binary_search(e).is_ok())
            .copied()
            .collect();
        self.created.retain(|e| !created_and_deleted.contains(e));
        self.deleted.retain(|e| !created_and_deleted.contains(e));
        self.modified.sort();
        self.modified.dedup();
        self.modified.retain(|e| {
            self.created.binary_search(e).is_err()
                && self.deleted.binary_search(e).is_err()
                && !created_and_deleted.contains(e)
        });
    }

    pub(crate) fn finish(mut self) -> Self {
        self.normalize();
        self
    }

    /// Raw IDs of created entities of the given kind.
    pub fn created_ids(&self, kind: EntityKind) -> Vec<u64> {
        self.created
            .iter()
            .filter(|e| e.kind() == kind)
            .map(|e| e.raw())
            .collect()
    }
}

/// Why an editing operation was rejected. The map is unchanged in all cases.
#[derive(Debug, Clone, PartialEq, thiserror::Error, Serialize, Deserialize)]
#[serde(tag = "code", rename_all = "snake_case")]
pub enum EditError {
    /// A referenced entity does not exist.
    #[error("{entity} does not exist")]
    NotFound {
        /// The missing entity.
        entity: EntityRef,
    },
    /// An ID requested for a new entity is already taken.
    #[error("{entity} already exists")]
    AlreadyExists {
        /// The existing entity.
        entity: EntityRef,
    },
    /// An argument is out of range or inconsistent.
    #[error("invalid argument `{name}`: {reason}")]
    InvalidArgument {
        /// Argument name.
        name: String,
        /// Explanation.
        reason: String,
    },
    /// Geometry is unusable for the operation.
    #[error("invalid geometry: {reason}")]
    InvalidGeometry {
        /// Explanation.
        reason: String,
    },
    /// The entity is still referenced and cannot be removed.
    #[error("{entity} is still referenced by {referenced_by:?}")]
    InUse {
        /// The entity to remove.
        entity: EntityRef,
        /// Entities referencing it.
        referenced_by: Vec<EntityRef>,
    },
    /// Two lanes cannot be merged.
    #[error("cannot merge {first} and {second}: {reason}")]
    NotMergeable {
        /// First lane.
        first: LaneId,
        /// Second lane.
        second: LaneId,
        /// Explanation.
        reason: String,
    },
}

impl EditError {
    pub(crate) fn not_found(entity: impl Into<EntityRef>) -> Self {
        EditError::NotFound {
            entity: entity.into(),
        }
    }

    pub(crate) fn invalid(name: &str, reason: impl Into<String>) -> Self {
        EditError::InvalidArgument {
            name: name.to_string(),
            reason: reason.into(),
        }
    }

    pub(crate) fn geometry(reason: impl Into<String>) -> Self {
        EditError::InvalidGeometry {
            reason: reason.into(),
        }
    }
}

/// Result type of editing operations.
pub type EditResult<T> = Result<T, EditError>;

/// Distance above which connected lanes are reported as geometrically
/// discontinuous (metres).
pub const GAP_TOLERANCE: f64 = 0.1;

pub(crate) fn check_polyline(name: &str, p: &Polyline3) -> EditResult<()> {
    if p.points.iter().any(|q| !q.is_finite()) {
        return Err(EditError::geometry(format!(
            "`{name}` has non-finite coordinates"
        )));
    }
    if !p.is_valid() {
        return Err(EditError::geometry(format!(
            "`{name}` needs at least two distinct points"
        )));
    }
    Ok(())
}

impl Map {
    pub(crate) fn require_lane(&self, lane: LaneId) -> EditResult<()> {
        if self.lanes.contains_key(&lane) {
            Ok(())
        } else {
            Err(EditError::not_found(lane))
        }
    }

    /// Connects `from → to` (`to` becomes a successor of `from`).
    ///
    /// Idempotent; also repairs a half-present link. Warns if the lanes do
    /// not meet geometrically.
    pub fn connect(&mut self, from: LaneId, to: LaneId) -> EditResult<ChangeSet> {
        self.require_lane(from)?;
        self.require_lane(to)?;
        if from == to {
            return Err(EditError::invalid(
                "to",
                "a lane cannot be its own successor",
            ));
        }
        let mut cs = ChangeSet::new();
        if let Some(gap) = self.connection_gap(from, to)
            && gap > GAP_TOLERANCE
        {
            cs.warn(
                Issue::warning(
                    codes::GEOMETRIC_GAP,
                    format!(
                        "{to} starts {gap:.3} m away from the end of {from}; formats with \
                         implicit topology (Lanelet2) will not preserve this link"
                    ),
                )
                .with_entity(from)
                .with_related(to),
            );
        }
        if self.topology.add_link(from, to) {
            cs.modified(from);
            cs.modified(to);
        } else {
            cs.warn(
                Issue::info(codes::NO_OP, format!("{from} is already connected to {to}"))
                    .with_entity(from),
            );
        }
        Ok(cs.finish())
    }

    /// Removes the link `from → to`.
    pub fn disconnect(&mut self, from: LaneId, to: LaneId) -> EditResult<ChangeSet> {
        self.require_lane(from)?;
        self.require_lane(to)?;
        let mut cs = ChangeSet::new();
        if self.topology.remove_link(from, to) {
            cs.modified(from);
            cs.modified(to);
        } else {
            cs.warn(
                Issue::info(codes::NO_OP, format!("{from} is not connected to {to}"))
                    .with_entity(from),
            );
        }
        Ok(cs.finish())
    }

    /// Largest distance between the end of `from` and the start of `to`
    /// (compared boundary by boundary).
    pub fn connection_gap(&self, from: LaneId, to: LaneId) -> Option<f64> {
        let fl = self.oriented_boundary(from, Side::Left)?.last()?;
        let fr = self.oriented_boundary(from, Side::Right)?.last()?;
        let tl = self.oriented_boundary(to, Side::Left)?.first()?;
        let tr = self.oriented_boundary(to, Side::Right)?.first()?;
        Some(fl.distance(tl).max(fr.distance(tr)))
    }

    /// Sets or clears the neighbor of `lane` on `side`. The reciprocal
    /// relation on the neighbor is maintained automatically.
    pub fn set_neighbor(
        &mut self,
        lane: LaneId,
        side: Side,
        neighbor: Option<Neighbor>,
    ) -> EditResult<ChangeSet> {
        self.require_lane(lane)?;
        if let Some(n) = neighbor {
            self.require_lane(n.lane)?;
            if n.lane == lane {
                return Err(EditError::invalid(
                    "neighbor",
                    "a lane cannot neighbor itself",
                ));
            }
        }
        let mut cs = ChangeSet::new();
        let touched = self.topology.set_neighbor(lane, side, neighbor);
        if touched.is_empty() {
            cs.warn(Issue::info(codes::NO_OP, "neighbor relation unchanged").with_entity(lane));
        }
        if let Some(n) = neighbor {
            let rside = reciprocal_side(side, n.direction);
            let shared = self.lanes[&lane].boundary(side).boundary
                == self.lanes[&n.lane].boundary(rside).boundary;
            if !shared {
                cs.warn(
                    Issue::warning(
                        codes::BOUNDARY_UNSHARED,
                        format!(
                            "{lane} and {} do not share a boundary; formats with implicit \
                             topology (Lanelet2) will not preserve this neighbor relation",
                            n.lane
                        ),
                    )
                    .with_entity(lane)
                    .with_related(n.lane),
                );
            }
        }
        for id in touched {
            cs.modified(id);
        }
        Ok(cs.finish())
    }

    /// Sets (or clears) the speed limit of one or more lanes.
    pub fn set_speed_limit(
        &mut self,
        lanes: &[LaneId],
        limit: Option<SpeedLimit>,
    ) -> EditResult<ChangeSet> {
        if lanes.is_empty() {
            return Err(EditError::invalid("lanes", "at least one lane is required"));
        }
        for &l in lanes {
            self.require_lane(l)?;
        }
        if let Some(s) = limit
            && !s.is_valid()
        {
            return Err(EditError::invalid(
                "speed_limit",
                "must be finite and positive",
            ));
        }
        let mut cs = ChangeSet::new();
        for &l in lanes {
            let lane = self.lanes.get_mut(&l).expect("checked");
            if lane.speed_limit != limit {
                lane.speed_limit = limit;
                cs.modified(l);
            }
        }
        Ok(cs.finish())
    }

    /// Sets (or clears) the turn direction of a junction lane.
    pub fn set_turn_direction(
        &mut self,
        lane: LaneId,
        turn: Option<TurnDirection>,
    ) -> EditResult<ChangeSet> {
        let l = self
            .lanes
            .get_mut(&lane)
            .ok_or(EditError::not_found(lane))?;
        let mut cs = ChangeSet::new();
        if l.turn_direction != turn {
            l.turn_direction = turn;
            cs.modified(lane);
        }
        Ok(cs.finish())
    }

    /// Changes the functional kind of a lane.
    pub fn set_lane_kind(&mut self, lane: LaneId, kind: LaneKind) -> EditResult<ChangeSet> {
        let l = self
            .lanes
            .get_mut(&lane)
            .ok_or(EditError::not_found(lane))?;
        let mut cs = ChangeSet::new();
        if l.kind != kind {
            l.kind = kind;
            cs.modified(lane);
        }
        Ok(cs.finish())
    }

    /// Changes the kind of a boundary (e.g. solid → dashed).
    pub fn set_boundary_kind(
        &mut self,
        boundary: BoundaryId,
        kind: BoundaryKind,
    ) -> EditResult<ChangeSet> {
        let b = self
            .boundaries
            .get_mut(&boundary)
            .ok_or(EditError::not_found(boundary))?;
        let mut cs = ChangeSet::new();
        if b.kind != kind {
            b.kind = kind;
            cs.modified(boundary);
        }
        Ok(cs.finish())
    }

    /// Sets (`Some`) or removes (`None`) an attribute on any entity.
    pub fn set_attribute(
        &mut self,
        entity: EntityRef,
        key: &str,
        value: Option<&str>,
    ) -> EditResult<ChangeSet> {
        if key.is_empty() {
            return Err(EditError::invalid("key", "must not be empty"));
        }
        let attrs = self
            .attributes_mut(entity)
            .ok_or(EditError::not_found(entity))?;
        let changed = match value {
            Some(v) => attrs.insert(key, v).as_deref() != Some(v),
            None => attrs.remove(key).is_some(),
        };
        let mut cs = ChangeSet::new();
        if changed {
            cs.modified(entity);
        }
        Ok(cs.finish())
    }

    fn check_group_lanes(&self, lanes: &[LaneId]) -> EditResult<()> {
        for &l in lanes {
            self.require_lane(l)?;
        }
        let mut sorted = lanes.to_vec();
        sorted.sort();
        if sorted.windows(2).any(|w| w[0] == w[1]) {
            return Err(EditError::invalid("lanes", "contains duplicates"));
        }
        Ok(())
    }

    /// Creates a road grouping the given lanes. A lane can belong to at most
    /// one road.
    pub fn add_road(
        &mut self,
        name: Option<String>,
        lanes: &[LaneId],
    ) -> EditResult<(RoadId, ChangeSet)> {
        self.check_group_lanes(lanes)?;
        if let Some((l, r)) = lanes.iter().find_map(|&l| self.road_of(l).map(|r| (l, r))) {
            return Err(EditError::invalid(
                "lanes",
                format!("{l} already belongs to {r}"),
            ));
        }
        let id = RoadId(self.alloc_raw());
        self.roads.insert(
            id,
            Road {
                id,
                name,
                lanes: lanes.to_vec(),
                attributes: Default::default(),
            },
        );
        let mut cs = ChangeSet::new();
        cs.created(id);
        Ok((id, cs.finish()))
    }

    /// Creates a junction containing the given connecting lanes. A lane can
    /// belong to at most one junction.
    pub fn add_junction(
        &mut self,
        name: Option<String>,
        lanes: &[LaneId],
        outline: Option<Polygon3>,
    ) -> EditResult<(JunctionId, ChangeSet)> {
        self.check_group_lanes(lanes)?;
        if let Some(l) = lanes.iter().find(|l| self.junction_of(**l).is_some()) {
            return Err(EditError::invalid(
                "lanes",
                format!("{l} already belongs to a junction"),
            ));
        }
        if let Some(o) = &outline
            && (o.points.len() < 3 || o.area_2d() <= 0.0)
        {
            return Err(EditError::geometry(
                "junction outline needs a non-zero area",
            ));
        }
        let id = JunctionId(self.alloc_raw());
        self.junctions.insert(
            id,
            Junction {
                id,
                name,
                lanes: lanes.to_vec(),
                outline,
                attributes: Default::default(),
            },
        );
        let mut cs = ChangeSet::new();
        cs.created(id);
        Ok((id, cs.finish()))
    }
}
