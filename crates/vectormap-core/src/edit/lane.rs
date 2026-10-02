//! Creating and removing lanes (and removing other entities).

use serde::{Deserialize, Serialize};

use super::{ChangeSet, EditError, EditResult, check_polyline};
use crate::attributes::Attributes;
use crate::diagnostics::{Issue, codes};
use crate::entities::{
    Boundary, BoundaryKind, BoundaryRef, Lane, LaneKind, Rule, Side, SpeedLimit, TurnDirection,
};
use crate::geometry::Polyline3;
use crate::id::{BoundaryId, EntityRef, JunctionId, LaneId, RoadId};
use crate::map::Map;
use crate::topology::Neighbor;

fn default_true() -> bool {
    true
}

/// One side of a new lane.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoundarySpec {
    /// Reuse an existing boundary (e.g. shared with a neighbor).
    Existing(BoundaryRef),
    /// Create a new boundary.
    New {
        /// Geometry, oriented along the new lane.
        geometry: Polyline3,
        /// Kind of the new boundary.
        #[serde(default)]
        kind: BoundaryKind,
    },
}

/// How the geometry of a new lane is specified.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LaneGeometry {
    /// Explicit left and right boundaries.
    Boundaries {
        /// Left side.
        left: BoundarySpec,
        /// Right side.
        right: BoundarySpec,
    },
    /// A centreline and a constant width; two new boundaries are created.
    Centerline {
        /// Centreline in the direction of travel.
        centerline: Polyline3,
        /// Lane width (metres).
        width: f64,
        /// Kind of the new left boundary.
        #[serde(default)]
        left_kind: BoundaryKind,
        /// Kind of the new right boundary.
        #[serde(default)]
        right_kind: BoundaryKind,
    },
    /// A lane next to an existing lane, sharing its boundary and running in
    /// the same direction. The neighbor relation is set automatically.
    BesideLane {
        /// The existing lane.
        lane: LaneId,
        /// On which side of it the new lane is created.
        side: Side,
        /// Width of the new lane (metres).
        width: f64,
        /// Kind of the new outer boundary.
        #[serde(default)]
        outer_kind: BoundaryKind,
        /// New kind for the shared boundary (e.g. dashed, to allow lane
        /// changes). Unchanged when absent.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        shared_kind: Option<BoundaryKind>,
    },
}

/// Parameters of [`Map::add_lane`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NewLane {
    /// Explicit ID; allocated automatically when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<LaneId>,
    /// Functional kind.
    #[serde(default)]
    pub kind: LaneKind,
    /// Geometry.
    pub geometry: LaneGeometry,
    /// Speed limit.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_limit: Option<SpeedLimit>,
    /// One-way lane.
    #[serde(default = "default_true")]
    pub one_way: bool,
    /// Turn direction inside a junction.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub turn_direction: Option<TurnDirection>,
    /// Lanes to connect as predecessors.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub predecessors: Vec<LaneId>,
    /// Lanes to connect as successors.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub successors: Vec<LaneId>,
    /// Road to add the lane to.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub road: Option<RoadId>,
    /// Junction to add the lane to.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub junction: Option<JunctionId>,
    /// Extension attributes.
    #[serde(default, skip_serializing_if = "Attributes::is_empty")]
    pub attributes: Attributes,
}

impl NewLane {
    /// A driving lane with the given geometry and default settings.
    pub fn new(geometry: LaneGeometry) -> Self {
        Self {
            id: None,
            kind: LaneKind::Driving,
            geometry,
            speed_limit: None,
            one_way: true,
            turn_direction: None,
            predecessors: Vec::new(),
            successors: Vec::new(),
            road: None,
            junction: None,
            attributes: Attributes::new(),
        }
    }

    /// A lane between two new boundaries with default kinds.
    pub fn from_boundaries(left: Polyline3, right: Polyline3) -> Self {
        Self::new(LaneGeometry::Boundaries {
            left: BoundarySpec::New {
                geometry: left,
                kind: BoundaryKind::default(),
            },
            right: BoundarySpec::New {
                geometry: right,
                kind: BoundaryKind::default(),
            },
        })
    }

    /// A lane from a centreline and a width.
    pub fn from_centerline(centerline: Polyline3, width: f64) -> Self {
        Self::new(LaneGeometry::Centerline {
            centerline,
            width,
            left_kind: BoundaryKind::default(),
            right_kind: BoundaryKind::default(),
        })
    }

    /// Sets the speed limit (builder style).
    pub fn with_speed_limit(mut self, limit: SpeedLimit) -> Self {
        self.speed_limit = Some(limit);
        self
    }

    /// Sets the predecessors (builder style).
    pub fn with_predecessors(mut self, lanes: Vec<LaneId>) -> Self {
        self.predecessors = lanes;
        self
    }
}

/// A boundary side resolved during validation, before mutation.
enum ResolvedSide {
    Existing(BoundaryRef),
    New(Polyline3, BoundaryKind),
}

impl Map {
    fn resolve_spec(&self, name: &str, spec: &BoundarySpec) -> EditResult<ResolvedSide> {
        match spec {
            BoundarySpec::Existing(r) => {
                if !self.boundaries.contains_key(&r.boundary) {
                    return Err(EditError::not_found(r.boundary));
                }
                Ok(ResolvedSide::Existing(*r))
            }
            BoundarySpec::New { geometry, kind } => {
                check_polyline(name, geometry)?;
                Ok(ResolvedSide::New(geometry.clone(), *kind))
            }
        }
    }

    /// Creates a lane.
    ///
    /// Returns the new lane's ID. New boundaries, topology links and
    /// road/junction membership are reported in the change set.
    pub fn add_lane(&mut self, spec: NewLane) -> EditResult<(LaneId, ChangeSet)> {
        // ---- validation (no mutation) ----
        if let Some(id) = spec.id
            && self.lanes.contains_key(&id)
        {
            return Err(EditError::AlreadyExists { entity: id.into() });
        }
        if let Some(s) = spec.speed_limit
            && !s.is_valid()
        {
            return Err(EditError::invalid(
                "speed_limit",
                "must be finite and positive",
            ));
        }
        for &l in spec.predecessors.iter().chain(&spec.successors) {
            self.require_lane(l)?;
        }
        if let Some(r) = spec.road
            && !self.roads.contains_key(&r)
        {
            return Err(EditError::not_found(r));
        }
        if let Some(j) = spec.junction
            && !self.junctions.contains_key(&j)
        {
            return Err(EditError::not_found(j));
        }
        let mut beside: Option<(LaneId, Side, Option<BoundaryKind>)> = None;
        let (left, right) = match &spec.geometry {
            LaneGeometry::Boundaries { left, right } => (
                self.resolve_spec("left", left)?,
                self.resolve_spec("right", right)?,
            ),
            LaneGeometry::Centerline {
                centerline,
                width,
                left_kind,
                right_kind,
            } => {
                check_polyline("centerline", centerline)?;
                if !(width.is_finite() && *width > 0.0) {
                    return Err(EditError::invalid("width", "must be positive"));
                }
                (
                    ResolvedSide::New(centerline.offset(width / 2.0), *left_kind),
                    ResolvedSide::New(centerline.offset(-width / 2.0), *right_kind),
                )
            }
            LaneGeometry::BesideLane {
                lane,
                side,
                width,
                outer_kind,
                shared_kind,
            } => {
                self.require_lane(*lane)?;
                if !(width.is_finite() && *width > 0.0) {
                    return Err(EditError::invalid("width", "must be positive"));
                }
                if self.neighbor(*lane, *side).is_some() {
                    return Err(EditError::invalid(
                        "side",
                        format!("{lane} already has a {side:?} neighbor"),
                    ));
                }
                let shared = self.lanes[lane].boundary(*side);
                let geom = self
                    .oriented_boundary(*lane, *side)
                    .ok_or(EditError::not_found(shared.boundary))?;
                check_polyline("shared boundary", &geom)?;
                beside = Some((*lane, *side, *shared_kind));
                match side {
                    Side::Left => (
                        ResolvedSide::New(geom.offset(*width), *outer_kind),
                        ResolvedSide::Existing(shared),
                    ),
                    Side::Right => (
                        ResolvedSide::Existing(shared),
                        ResolvedSide::New(geom.offset(-*width), *outer_kind),
                    ),
                }
            }
        };
        if let (ResolvedSide::Existing(a), ResolvedSide::Existing(b)) = (&left, &right)
            && a.boundary == b.boundary
        {
            return Err(EditError::invalid(
                "geometry",
                "left and right boundary must differ",
            ));
        }

        // ---- mutation ----
        let mut cs = ChangeSet::new();
        let mut materialize = |map: &mut Map, side: ResolvedSide| -> BoundaryRef {
            match side {
                ResolvedSide::Existing(r) => r,
                ResolvedSide::New(geometry, kind) => {
                    let id = map.allocate_boundary_id();
                    map.boundaries.insert(id, Boundary::new(id, kind, geometry));
                    cs.created(id);
                    BoundaryRef::forward(id)
                }
            }
        };
        let left = materialize(self, left);
        let right = materialize(self, right);
        let id = match spec.id {
            Some(id) => {
                self.bump_next_id(id.0);
                id
            }
            None => self.allocate_lane_id(),
        };
        let lane = Lane {
            id,
            kind: spec.kind,
            left,
            right,
            centerline: None,
            speed_limit: spec.speed_limit,
            one_way: spec.one_way,
            turn_direction: spec.turn_direction,
            attributes: spec.attributes,
        };
        self.lanes.insert(id, lane);
        cs.created(id);

        for p in &spec.predecessors {
            cs.extend(self.connect(*p, id)?);
        }
        for s in &spec.successors {
            cs.extend(self.connect(id, *s)?);
        }
        if let Some(r) = spec.road {
            self.roads.get_mut(&r).expect("checked").lanes.push(id);
            cs.modified(r);
        }
        if let Some(j) = spec.junction {
            self.junctions.get_mut(&j).expect("checked").lanes.push(id);
            cs.modified(j);
        }
        if let Some((other, side, shared_kind)) = beside {
            cs.extend(self.set_neighbor(other, side, Some(Neighbor::same(id)))?);
            if let Some(kind) = shared_kind {
                let b = self.lanes[&other].boundary(side).boundary;
                cs.extend(self.set_boundary_kind(b, kind)?);
            }
        }
        Ok((id, cs.finish()))
    }

    /// Removes a lane together with its topology links and its memberships
    /// in roads, junctions and rules. Boundaries that are no longer used by
    /// any lane are deleted too.
    pub fn remove_lane(&mut self, lane: LaneId) -> EditResult<ChangeSet> {
        let removed = self.lanes.remove(&lane).ok_or(EditError::not_found(lane))?;
        let mut cs = ChangeSet::new();
        cs.deleted(lane);
        for other in self.topology.remove_lane(lane) {
            cs.modified(other);
        }
        for road in self.roads.values_mut() {
            if road.lanes.contains(&lane) {
                road.lanes.retain(|l| *l != lane);
                cs.modified(road.id);
            }
        }
        for j in self.junctions.values_mut() {
            if j.lanes.contains(&lane) {
                j.lanes.retain(|l| *l != lane);
                cs.modified(j.id);
            }
        }
        for re in self.regulatory_elements.values_mut() {
            let mut changed = false;
            if re.lanes.contains(&lane) {
                re.lanes.retain(|l| *l != lane);
                changed = true;
            }
            if let Rule::RightOfWay {
                priority, yielding, ..
            } = &mut re.rule
                && (priority.contains(&lane) || yielding.contains(&lane))
            {
                priority.retain(|l| *l != lane);
                yielding.retain(|l| *l != lane);
                changed = true;
            }
            if changed {
                cs.modified(re.id);
                if re.lanes.is_empty() {
                    cs.warn(
                        Issue::warning(
                            codes::ORPHAN_RULE,
                            format!("{} no longer applies to any lane", re.id),
                        )
                        .with_entity(re.id)
                        .with_fix(crate::Command::RemoveEntity {
                            entity: re.id.into(),
                        }),
                    );
                }
            }
        }
        for b in [removed.left.boundary, removed.right.boundary] {
            if self.boundaries.contains_key(&b) && self.lanes_using_boundary(b).is_empty() {
                self.boundaries.remove(&b);
                cs.deleted(b);
            }
        }
        Ok(cs.finish())
    }

    /// Removes any entity, cascading to references where that is
    /// unambiguous:
    ///
    /// - lanes: see [`Map::remove_lane`];
    /// - boundaries: rejected while a lane uses them;
    /// - stop lines: detached from rules; `stop_line` rules are deleted;
    /// - signals: removed from their rules; empty traffic-light rules are deleted;
    /// - crosswalks: yield rules are deleted and control targets detached;
    /// - roads, junctions, rules: removed without touching lanes.
    pub fn remove_entity(&mut self, entity: EntityRef) -> EditResult<ChangeSet> {
        if !self.contains(entity) {
            return Err(EditError::not_found(entity));
        }
        let mut cs = ChangeSet::new();
        match entity {
            EntityRef::Lane(id) => return self.remove_lane(id),
            EntityRef::Boundary(id) => {
                let users = self.lanes_using_boundary(id);
                if !users.is_empty() {
                    return Err(EditError::InUse {
                        entity,
                        referenced_by: users.into_iter().map(EntityRef::Lane).collect(),
                    });
                }
                self.boundaries.remove(&id);
            }
            EntityRef::Road(id) => {
                self.roads.remove(&id);
            }
            EntityRef::Junction(id) => {
                self.junctions.remove(&id);
            }
            EntityRef::RegulatoryElement(id) => {
                self.regulatory_elements.remove(&id);
            }
            EntityRef::StopLine(id) => {
                self.stop_lines.remove(&id);
                let mut to_delete = Vec::new();
                for re in self.regulatory_elements.values_mut() {
                    let changed = match &mut re.rule {
                        Rule::TrafficLight { stop_line, .. }
                        | Rule::TrafficSign { stop_line, .. } => {
                            if *stop_line == Some(id) {
                                *stop_line = None;
                                true
                            } else {
                                false
                            }
                        }
                        Rule::StopLine { stop_line } => {
                            if *stop_line == id {
                                to_delete.push(re.id);
                            }
                            false
                        }
                        Rule::RightOfWay { stop_lines, .. }
                        | Rule::Crosswalk { stop_lines, .. } => {
                            let before = stop_lines.len();
                            stop_lines.retain(|s| *s != id);
                            before != stop_lines.len()
                        }
                        Rule::Other { .. } => false,
                    };
                    if changed {
                        cs.modified(re.id);
                    }
                }
                for re in to_delete {
                    self.regulatory_elements.remove(&re);
                    cs.deleted(re);
                }
            }
            EntityRef::TrafficSignal(id) => {
                self.traffic_signals.remove(&id);
                let mut to_delete = Vec::new();
                for re in self.regulatory_elements.values_mut() {
                    if let Rule::TrafficLight { signals, .. } = &mut re.rule
                        && signals.contains(&id)
                    {
                        signals.retain(|s| *s != id);
                        if signals.is_empty() {
                            to_delete.push(re.id);
                        } else {
                            cs.modified(re.id);
                        }
                    }
                }
                for re in to_delete {
                    self.regulatory_elements.remove(&re);
                    cs.deleted(re);
                }
            }
            EntityRef::Crosswalk(id) => {
                self.crosswalks.remove(&id);
                for re in self.regulatory_elements.values_mut() {
                    if re.controlled_crosswalks.contains(&id) {
                        re.controlled_crosswalks.retain(|c| *c != id);
                        cs.modified(re.id);
                    }
                }
                let to_delete: Vec<_> = self
                    .regulatory_elements
                    .values()
                    .filter(|re| re.rule.crosswalk() == Some(id))
                    .map(|re| re.id)
                    .collect();
                for re in to_delete {
                    self.regulatory_elements.remove(&re);
                    cs.deleted(re);
                }
            }
        }
        cs.deleted(entity);
        Ok(cs.finish())
    }

    /// Adds a boundary with a freshly allocated ID.
    pub fn add_boundary(
        &mut self,
        kind: BoundaryKind,
        geometry: Polyline3,
    ) -> EditResult<(BoundaryId, ChangeSet)> {
        check_polyline("geometry", &geometry)?;
        let id = self.allocate_boundary_id();
        self.boundaries
            .insert(id, Boundary::new(id, kind, geometry));
        let mut cs = ChangeSet::new();
        cs.created(id);
        Ok((id, cs.finish()))
    }
}
