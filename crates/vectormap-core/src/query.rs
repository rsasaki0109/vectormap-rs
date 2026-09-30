//! Read-only queries with serializable results (map summary, lane details,
//! nearest lane). These back tools such as `vectormap info` and future MCP
//! tools like `get_map_summary`, `get_lane` and `find_nearest_lane`.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::entities::{Lane, Side};
use crate::geometry::{BoundingBox, Point2, Point3};
use crate::id::{JunctionId, LaneId, RegulatoryElementId, RoadId, SignalId, StopLineId};
use crate::map::{GeoReference, Map};
use crate::topology::Neighbor;

/// Number of entities per kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct EntityCounts {
    /// Lanes.
    pub lanes: usize,
    /// Boundaries.
    pub boundaries: usize,
    /// Roads.
    pub roads: usize,
    /// Junctions.
    pub junctions: usize,
    /// Stop lines.
    pub stop_lines: usize,
    /// Traffic signals.
    pub traffic_signals: usize,
    /// Crosswalks.
    pub crosswalks: usize,
    /// Regulatory elements.
    pub regulatory_elements: usize,
}

/// Topology statistics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct TopologySummary {
    /// Number of predecessor → successor links.
    pub links: usize,
    /// Number of (directed) neighbor relations.
    pub neighbor_relations: usize,
    /// Lanes without predecessors.
    pub entry_lanes: usize,
    /// Lanes without successors.
    pub exit_lanes: usize,
    /// Lanes without any predecessor or successor.
    pub isolated_lanes: usize,
}

/// Overview of a map.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MapSummary {
    /// Map name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Georeference.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub georeference: Option<GeoReference>,
    /// Entity counts.
    pub counts: EntityCounts,
    /// Lanes per kind.
    pub lane_kinds: BTreeMap<String, usize>,
    /// Rules per type.
    pub rule_types: BTreeMap<String, usize>,
    /// Sum of all lane centreline lengths (metres).
    pub total_lane_length: f64,
    /// Bounding box of all geometry.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bounding_box: Option<BoundingBox>,
    /// Topology statistics.
    pub topology: TopologySummary,
}

/// Everything known about one lane.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LaneInfo {
    /// The lane itself.
    pub lane: Lane,
    /// Centreline length (metres).
    pub length: f64,
    /// Predecessors.
    pub predecessors: Vec<LaneId>,
    /// Successors.
    pub successors: Vec<LaneId>,
    /// Left neighbor.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub left_neighbor: Option<Neighbor>,
    /// Right neighbor.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub right_neighbor: Option<Neighbor>,
    /// Road containing the lane.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub road: Option<RoadId>,
    /// Junction containing the lane.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub junction: Option<JunctionId>,
    /// Rules applying to the lane.
    pub rules: Vec<RegulatoryElementId>,
    /// Stop lines of those rules.
    pub stop_lines: Vec<StopLineId>,
    /// Signals of those rules.
    pub traffic_signals: Vec<SignalId>,
}

/// Result of [`Map::find_nearest_lane`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct NearestLane {
    /// The lane.
    pub lane: LaneId,
    /// Distance from the query point to the centreline (metres).
    pub distance: f64,
    /// Station of the closest centreline point (metres).
    pub station: f64,
    /// Signed lateral offset from the centreline (positive = left).
    pub lateral: f64,
    /// Closest centreline point.
    pub point: Point3,
    /// `true` if the query point lies inside the lane polygon.
    pub inside: bool,
}

impl Map {
    /// Summary statistics of the map.
    pub fn summary(&self) -> MapSummary {
        let counts = EntityCounts {
            lanes: self.lanes.len(),
            boundaries: self.boundaries.len(),
            roads: self.roads.len(),
            junctions: self.junctions.len(),
            stop_lines: self.stop_lines.len(),
            traffic_signals: self.traffic_signals.len(),
            crosswalks: self.crosswalks.len(),
            regulatory_elements: self.regulatory_elements.len(),
        };
        let mut lane_kinds = BTreeMap::new();
        let mut total = 0.0;
        let mut topo = TopologySummary::default();
        for lane in self.lanes.values() {
            let kind = lane.kind.as_str().to_string();
            *lane_kinds.entry(kind).or_insert(0) += 1;
            total += self.lane_length(lane.id).unwrap_or(0.0);
            let (p, s) = (self.predecessors(lane.id), self.successors(lane.id));
            topo.links += s.len();
            topo.entry_lanes += usize::from(p.is_empty());
            topo.exit_lanes += usize::from(s.is_empty());
            topo.isolated_lanes += usize::from(p.is_empty() && s.is_empty());
            topo.neighbor_relations += [Side::Left, Side::Right]
                .iter()
                .filter(|side| self.neighbor(lane.id, **side).is_some())
                .count();
        }
        let mut rule_types = BTreeMap::new();
        for re in self.regulatory_elements.values() {
            *rule_types
                .entry(re.rule.type_name().to_string())
                .or_insert(0) += 1;
        }
        MapSummary {
            name: self.metadata.name.clone(),
            georeference: self.metadata.georeference,
            counts,
            lane_kinds,
            rule_types,
            total_lane_length: total,
            bounding_box: self.bounding_box(),
            topology: topo,
        }
    }

    /// Details of one lane, or `None` if it does not exist.
    pub fn lane_info(&self, lane: LaneId) -> Option<LaneInfo> {
        let l = self.lanes.get(&lane)?;
        let rules = self.rules_for_lane(lane);
        let mut stop_lines: Vec<StopLineId> =
            rules.iter().flat_map(|r| r.rule.stop_lines()).collect();
        stop_lines.sort();
        stop_lines.dedup();
        let mut traffic_signals: Vec<SignalId> = rules
            .iter()
            .flat_map(|r| r.rule.signals().iter().copied())
            .collect();
        traffic_signals.sort();
        traffic_signals.dedup();
        Some(LaneInfo {
            lane: l.clone(),
            length: self.lane_length(lane).unwrap_or(0.0),
            predecessors: self.predecessors(lane).to_vec(),
            successors: self.successors(lane).to_vec(),
            left_neighbor: self.neighbor(lane, Side::Left),
            right_neighbor: self.neighbor(lane, Side::Right),
            road: self.road_of(lane),
            junction: self.junction_of(lane),
            rules: rules.iter().map(|r| r.id).collect(),
            stop_lines,
            traffic_signals,
        })
    }

    /// The lane closest to `point`.
    ///
    /// Lanes whose polygon contains the point are preferred; among equal
    /// candidates the one with the smallest centreline distance, then the
    /// smallest ID, wins.
    pub fn find_nearest_lane(&self, point: Point2) -> Option<NearestLane> {
        let mut best: Option<NearestLane> = None;
        for lane in self.lanes.values() {
            let Some(center) = self.centerline(lane.id) else {
                continue;
            };
            let Some(n) = center.nearest_point(point) else {
                continue;
            };
            let inside = self
                .lane_polygon(lane.id)
                .is_some_and(|p| p.to_2d().contains(point));
            let candidate = NearestLane {
                lane: lane.id,
                distance: n.distance,
                station: n.station,
                lateral: n.lateral,
                point: n.point,
                inside,
            };
            let better = match best {
                None => true,
                Some(b) => {
                    (candidate.inside && !b.inside)
                        || (candidate.inside == b.inside && candidate.distance < b.distance - 1e-9)
                }
            };
            if better {
                best = Some(candidate);
            }
        }
        best
    }

    /// Lanes whose polygon contains `point`, in ID order.
    pub fn lanes_at(&self, point: Point2) -> Vec<LaneId> {
        self.lanes
            .keys()
            .copied()
            .filter(|&id| {
                self.lane_polygon(id)
                    .is_some_and(|p| p.to_2d().contains(point))
            })
            .collect()
    }
}
