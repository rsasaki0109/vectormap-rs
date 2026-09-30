//! Building whole roads, connecting them, and reshaping boundaries.
//!
//! [`Map::build_road`] turns a reference line (a road centre, a driven path,
//! a surveyed edge) and a cross-section (lane widths and directions, or
//! observed boundary lines) into lanes that share their boundaries, know
//! their neighbors and, optionally, are cut into connected pieces.
//! [`Map::add_connector`] joins the end of one lane to the start of another
//! with a smooth lane, as inside junctions. They are the operations map
//! builders start from; everything else is ordinary editing.

use serde::{Deserialize, Serialize};

use super::{
    ChangeSet, EditError, EditResult, GAP_TOLERANCE, SplitAt, SplitOptions, check_polyline,
};
use crate::attributes::Attributes;
use crate::diagnostics::{Issue, codes};
use crate::entities::{
    Boundary, BoundaryKind, BoundaryRef, Lane, LaneKind, Side, SpeedLimit, TurnDirection,
};
use crate::geometry::{Point2, Point3, Polyline3};
use crate::id::{BoundaryId, JunctionId, LaneId, RoadId};
use crate::map::Map;
use crate::topology::Neighbor;

/// Direction of travel of a lane relative to the road's reference line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LaneDirection {
    /// Along the reference line.
    #[default]
    Forward,
    /// Against the reference line.
    Backward,
}

/// One lane of a road cross-section.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RoadLane {
    /// Width in metres. Required unless the road gives explicit
    /// [`NewRoad::boundaries`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub width: Option<f64>,
    /// Direction of travel.
    #[serde(default)]
    pub direction: LaneDirection,
    /// Functional kind.
    #[serde(default)]
    pub kind: LaneKind,
}

impl RoadLane {
    /// A driving lane of the given width and direction.
    pub fn new(width: f64, direction: LaneDirection) -> Self {
        Self {
            width: Some(width),
            direction,
            kind: LaneKind::Driving,
        }
    }
}

/// Parameters of [`Map::build_road`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NewRoad {
    /// Reference line. "Forward" lanes run along it; "left" and "right" are
    /// as seen looking along it.
    pub reference: Polyline3,
    /// The lanes from left to right, looking along the reference line.
    pub lanes: Vec<RoadLane>,
    /// Lateral position of the road's left edge relative to the reference
    /// line (metres, positive = left). Default: the road is centred on it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub left_edge: Option<f64>,
    /// Explicit boundary lines from left to right (one more than lanes),
    /// e.g. lane markings and road edges found in a point cloud. Replaces
    /// the widths and `left_edge`. Lines running against the reference line
    /// are reversed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub boundaries: Option<Vec<Polyline3>>,
    /// Resample the reference line (or the explicit boundaries) at this
    /// spacing in metres before building, to thin out dense input such as a
    /// trajectory.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub resample: Option<f64>,
    /// Cut the road into connected pieces of about this length (metres).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub segment_length: Option<f64>,
    /// Speed limit of every lane.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_limit: Option<SpeedLimit>,
    /// Kind of the two outer boundaries (default: solid line).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub edge_kind: Option<BoundaryKind>,
    /// Kind of boundaries between lanes of the same direction (default:
    /// dashed line).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lane_line_kind: Option<BoundaryKind>,
    /// Kind of the boundary between opposite directions (default: solid
    /// line).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub center_line_kind: Option<BoundaryKind>,
    /// Also create a road entity grouping the lanes, with this name.
    /// (Lanelet2 has no road entity; it is not exported there.)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Extension attributes set on every lane.
    #[serde(default, skip_serializing_if = "Attributes::is_empty")]
    pub attributes: Attributes,
}

impl NewRoad {
    /// A road along `reference` with the given lanes (left to right).
    pub fn new(reference: Polyline3, lanes: Vec<RoadLane>) -> Self {
        Self {
            reference,
            lanes,
            left_edge: None,
            boundaries: None,
            resample: None,
            segment_length: None,
            speed_limit: None,
            edge_kind: None,
            lane_line_kind: None,
            center_line_kind: None,
            name: None,
            attributes: Attributes::new(),
        }
    }
}

/// What [`Map::build_road`] created.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct BuiltRoad {
    /// For each lane of the cross-section (left to right), its pieces in
    /// its direction of travel.
    pub lanes: Vec<Vec<LaneId>>,
    /// The road entity, if one was requested.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub road: Option<RoadId>,
}

/// Heading change (degrees) above which a connector is a left or right turn.
pub const TURN_THRESHOLD_DEG: f64 = 30.0;

/// Parameters of [`Map::add_connector`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NewConnector {
    /// The lane the connector starts from (at its end).
    pub from: LaneId,
    /// The lane the connector leads to (at its start).
    pub to: LaneId,
    /// Turn direction; derived from the change of heading when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub turn_direction: Option<TurnDirection>,
    /// Junction to add the connector to.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub junction: Option<JunctionId>,
    /// Speed limit; default: the lower one of `from` and `to`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_limit: Option<SpeedLimit>,
    /// Kind of both boundaries (default: virtual, as inside junctions).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub boundary_kind: Option<BoundaryKind>,
}

impl NewConnector {
    /// A connector from the end of `from` to the start of `to`.
    pub fn new(from: LaneId, to: LaneId) -> Self {
        Self {
            from,
            to,
            turn_direction: None,
            junction: None,
            speed_limit: None,
            boundary_kind: None,
        }
    }
}

fn positive(name: &str, v: Option<f64>) -> EditResult<()> {
    match v {
        Some(x) if !(x.is_finite() && x > 0.0) => {
            Err(EditError::invalid(name, "must be finite and positive"))
        }
        _ => Ok(()),
    }
}

/// `line`, reversed if it runs against `reference`.
fn along(line: Polyline3, reference: &Polyline3) -> Polyline3 {
    let (Some(a), Some(b)) = (line.first(), line.last()) else {
        return line;
    };
    let sa = reference.nearest_point(a.xy()).map_or(0.0, |n| n.station);
    let sb = reference.nearest_point(b.xy()).map_or(0.0, |n| n.station);
    if sb < sa { line.reversed() } else { line }
}

/// Unit heading of a polyline at its start (`end = false`) or end.
fn heading(line: &Polyline3, end: bool) -> Option<Point2> {
    let station = if end { line.length() } else { 0.0 };
    line.direction_at(station)
}

/// Cubic Hermite curve from `a` (heading `ta`) to `b` (heading `tb`), with
/// tangents scaled to the chord, sampled at about one point per metre.
fn hermite(a: Point3, ta: Point2, b: Point3, tb: Point2) -> Polyline3 {
    let chord = a.distance_2d(b);
    let (ta, tb) = (ta * chord, tb * chord);
    let n = (chord.ceil() as usize).clamp(4, 64);
    let points = (0..=n)
        .map(|i| {
            let t = i as f64 / n as f64;
            let (t2, t3) = (t * t, t * t * t);
            let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
            let h10 = t3 - 2.0 * t2 + t;
            let h01 = -2.0 * t3 + 3.0 * t2;
            let h11 = t3 - t2;
            let (pa, pb) = (a.xy(), b.xy());
            let p = pa * h00 + ta * h10 + pb * h01 + tb * h11;
            Point3::new(p.x, p.y, a.z + (b.z - a.z) * t)
        })
        .collect();
    Polyline3::new(points)
}

impl Map {
    /// Builds a road: lanes side by side along a reference line, sharing
    /// their boundaries, with neighbor relations (same or opposite
    /// direction) and boundary kinds set, optionally cut into connected
    /// pieces. Validates everything before changing the map.
    pub fn build_road(&mut self, spec: NewRoad) -> EditResult<(BuiltRoad, ChangeSet)> {
        // ---- validation ----
        check_polyline("reference", &spec.reference)?;
        if spec.lanes.is_empty() {
            return Err(EditError::invalid("lanes", "needs at least one lane"));
        }
        positive("resample", spec.resample)?;
        positive("segment_length", spec.segment_length)?;
        if let Some(s) = spec.speed_limit
            && !s.is_valid()
        {
            return Err(EditError::invalid(
                "speed_limit",
                "must be finite and positive",
            ));
        }
        let resample = |p: &Polyline3| match spec.resample {
            Some(step) => p.resample(step),
            None => p.clone(),
        };
        let reference = resample(&spec.reference);
        let lines: Vec<Polyline3> = match &spec.boundaries {
            Some(lines) => {
                if lines.len() != spec.lanes.len() + 1 {
                    return Err(EditError::invalid(
                        "boundaries",
                        format!(
                            "{} lanes need {} boundary lines, got {}",
                            spec.lanes.len(),
                            spec.lanes.len() + 1,
                            lines.len()
                        ),
                    ));
                }
                for (i, l) in lines.iter().enumerate() {
                    check_polyline(&format!("boundaries[{i}]"), l)?;
                }
                lines
                    .iter()
                    .map(|l| along(resample(l), &spec.reference))
                    .collect()
            }
            None => {
                let mut widths = Vec::with_capacity(spec.lanes.len());
                for (i, l) in spec.lanes.iter().enumerate() {
                    match l.width {
                        Some(w) if w.is_finite() && w > 0.0 => widths.push(w),
                        Some(_) => {
                            return Err(EditError::invalid(
                                &format!("lanes[{i}].width"),
                                "must be finite and positive",
                            ));
                        }
                        None => {
                            return Err(EditError::invalid(
                                &format!("lanes[{i}].width"),
                                "is required without explicit boundaries",
                            ));
                        }
                    }
                }
                let total: f64 = widths.iter().sum();
                let left = spec.left_edge.unwrap_or(total / 2.0);
                if !left.is_finite() {
                    return Err(EditError::invalid("left_edge", "must be finite"));
                }
                let mut offset = left;
                let mut lines = vec![reference.offset(offset)];
                for w in widths {
                    offset -= w;
                    lines.push(reference.offset(offset));
                }
                lines
            }
        };
        for (i, l) in lines.iter().enumerate() {
            check_polyline(&format!("boundary line {i}"), l)?;
        }
        if let Some(name) = &spec.name
            && self.roads.values().any(|r| r.name.as_deref() == Some(name))
        {
            return Err(EditError::invalid(
                "name",
                format!("a road named {name:?} already exists"),
            ));
        }

        // ---- mutation (on a scratch copy, so a late failure changes nothing) ----
        let mut map = self.clone();
        let mut cs = ChangeSet::new();
        let n = spec.lanes.len();
        let kind_of = |i: usize| -> BoundaryKind {
            if i == 0 || i == n {
                spec.edge_kind.unwrap_or(BoundaryKind::SOLID)
            } else if spec.lanes[i - 1].direction == spec.lanes[i].direction {
                spec.lane_line_kind.unwrap_or(BoundaryKind::DASHED)
            } else {
                spec.center_line_kind.unwrap_or(BoundaryKind::SOLID)
            }
        };
        let boundary_ids: Vec<BoundaryId> = lines
            .into_iter()
            .enumerate()
            .map(|(i, geometry)| {
                let id = map.allocate_boundary_id();
                map.boundaries
                    .insert(id, Boundary::new(id, kind_of(i), geometry));
                cs.created(id);
                id
            })
            .collect();
        let mut lane_ids = Vec::with_capacity(n);
        for (i, l) in spec.lanes.iter().enumerate() {
            let (left, right) = match l.direction {
                LaneDirection::Forward => (
                    BoundaryRef::forward(boundary_ids[i]),
                    BoundaryRef::forward(boundary_ids[i + 1]),
                ),
                LaneDirection::Backward => (
                    BoundaryRef::backward(boundary_ids[i + 1]),
                    BoundaryRef::backward(boundary_ids[i]),
                ),
            };
            let id = map.allocate_lane_id();
            let mut lane = Lane::new(id, left, right);
            lane.kind = l.kind;
            lane.speed_limit = spec.speed_limit;
            lane.attributes = spec.attributes.clone();
            map.lanes.insert(id, lane);
            cs.created(id);
            lane_ids.push(id);
        }
        // Neighbors, pair by pair from left to right.
        for i in 0..n.saturating_sub(1) {
            use LaneDirection::{Backward, Forward};
            let b = lane_ids[i + 1];
            let (side, neighbor) = match (spec.lanes[i].direction, spec.lanes[i + 1].direction) {
                (Forward, Forward) => (Side::Right, Neighbor::same(b)),
                (Backward, Backward) => (Side::Left, Neighbor::same(b)),
                (Forward, Backward) => (Side::Right, Neighbor::opposite(b)),
                (Backward, Forward) => (Side::Left, Neighbor::opposite(b)),
            };
            cs.extend(map.set_neighbor(lane_ids[i], side, Some(neighbor))?);
        }
        // Pieces: cut the lateral group at cross-sections along the reference.
        let mut chains: Vec<Vec<LaneId>> = lane_ids.iter().map(|&l| vec![l]).collect();
        if let Some(len) = spec.segment_length {
            let total = reference.length();
            let pieces = (total / len).round().max(1.0) as usize;
            let cuts: Vec<Point2> = (1..pieces)
                .filter_map(|k| reference.point_at(total * k as f64 / pieces as f64))
                .map(|p| p.xy())
                .collect();
            // Cut along the first forward lane (or the first lane), in its
            // direction of travel, always splitting its last piece.
            let pivot = spec
                .lanes
                .iter()
                .position(|l| l.direction == LaneDirection::Forward)
                .unwrap_or(0);
            let ordered: Vec<Point2> = match spec.lanes[pivot].direction {
                LaneDirection::Forward => cuts,
                LaneDirection::Backward => cuts.into_iter().rev().collect(),
            };
            let mut last = lane_ids[pivot];
            for cut in ordered {
                let center = map
                    .centerline(last)
                    .ok_or_else(|| EditError::geometry(format!("{last} has no geometry")))?;
                let station = center.nearest_point(cut).expect("non-empty").station;
                let (second, split) =
                    map.split_lane(last, SplitAt::Station(station), SplitOptions::default())?;
                cs.extend(split);
                last = second;
            }
            for chain in &mut chains {
                let mut at = chain[0];
                while let [next] = map.successors(at) {
                    at = *next;
                    chain.push(at);
                }
            }
        }
        let mut built = BuiltRoad {
            lanes: chains,
            road: None,
        };
        if spec.name.is_some() {
            let all: Vec<LaneId> = built.lanes.iter().flatten().copied().collect();
            let (road, road_cs) = map.add_road(spec.name.clone(), &all)?;
            cs.extend(road_cs);
            built.road = Some(road);
        }
        *self = map;
        Ok((built, cs.finish()))
    }

    /// Joins the end of `from` to the start of `to` with a new lane whose
    /// boundaries are smooth curves leaving and arriving along the lanes'
    /// headings, so the three meet exactly (formats with implicit topology
    /// keep the links). The turn direction is derived from the change of
    /// heading unless given.
    pub fn add_connector(&mut self, spec: NewConnector) -> EditResult<(LaneId, ChangeSet)> {
        // ---- validation ----
        self.require_lane(spec.from)?;
        self.require_lane(spec.to)?;
        if spec.from == spec.to {
            return Err(EditError::invalid("to", "must differ from `from`"));
        }
        if let Some(j) = spec.junction
            && !self.junctions.contains_key(&j)
        {
            return Err(EditError::not_found(j));
        }
        if let Some(s) = spec.speed_limit
            && !s.is_valid()
        {
            return Err(EditError::invalid(
                "speed_limit",
                "must be finite and positive",
            ));
        }
        let geometry = |lane: LaneId| -> EditResult<(Polyline3, Polyline3, Polyline3)> {
            let missing = || EditError::geometry(format!("{lane} has no usable geometry"));
            Ok((
                self.oriented_boundary(lane, Side::Left)
                    .ok_or_else(missing)?,
                self.oriented_boundary(lane, Side::Right)
                    .ok_or_else(missing)?,
                self.centerline(lane).ok_or_else(missing)?,
            ))
        };
        let (from_left, from_right, from_center) = geometry(spec.from)?;
        let (to_left, to_right, to_center) = geometry(spec.to)?;
        let (Some(h_from), Some(h_to)) = (heading(&from_center, true), heading(&to_center, false))
        else {
            return Err(EditError::geometry("cannot determine the lane headings"));
        };
        let ends = [
            from_left.last(),
            from_right.last(),
            to_left.first(),
            to_right.first(),
        ];
        let [Some(fl), Some(fr), Some(tl), Some(tr)] = ends else {
            return Err(EditError::geometry("a lane boundary is empty"));
        };
        if fl.distance_2d(tl) < GAP_TOLERANCE && fr.distance_2d(tr) < GAP_TOLERANCE {
            return Err(EditError::invalid(
                "to",
                format!(
                    "{} already starts where {} ends; use connect_lanes",
                    spec.to, spec.from
                ),
            ));
        }
        let left = hermite(fl, h_from, tl, h_to);
        let right = hermite(fr, h_from, tr, h_to);
        let turn = spec.turn_direction.unwrap_or_else(|| {
            let angle = h_from.cross(h_to).atan2(h_from.dot(h_to)).to_degrees();
            if angle > TURN_THRESHOLD_DEG {
                TurnDirection::Left
            } else if angle < -TURN_THRESHOLD_DEG {
                TurnDirection::Right
            } else {
                TurnDirection::Straight
            }
        });
        let speed_limit = spec.speed_limit.or_else(|| {
            let limits = [
                self.lanes[&spec.from].speed_limit,
                self.lanes[&spec.to].speed_limit,
            ];
            limits
                .into_iter()
                .flatten()
                .min_by(|a, b| a.kmh.total_cmp(&b.kmh))
        });

        // ---- mutation ----
        let mut cs = ChangeSet::new();
        let kind = spec.boundary_kind.unwrap_or(BoundaryKind::Virtual);
        let mut boundary = |map: &mut Map, geometry: Polyline3| {
            let id = map.allocate_boundary_id();
            map.boundaries.insert(id, Boundary::new(id, kind, geometry));
            cs.created(id);
            BoundaryRef::forward(id)
        };
        let (left, right) = (boundary(self, left), boundary(self, right));
        let id = self.allocate_lane_id();
        let mut lane = Lane::new(id, left, right);
        lane.kind = self.lanes[&spec.from].kind;
        lane.turn_direction = Some(turn);
        lane.speed_limit = speed_limit;
        self.lanes.insert(id, lane);
        cs.created(id);
        cs.extend(self.connect(spec.from, id)?);
        cs.extend(self.connect(id, spec.to)?);
        if let Some(j) = spec.junction {
            self.junctions.get_mut(&j).expect("checked").lanes.push(id);
            cs.modified(j);
        }
        Ok((id, cs.finish()))
    }

    /// Replaces the geometry of a boundary (e.g. to snap it to an observed
    /// lane marking). Give the line in the boundary's own direction. Warns
    /// when a lane using it no longer meets its predecessors or successors.
    pub fn set_boundary_geometry(
        &mut self,
        boundary: BoundaryId,
        geometry: Polyline3,
    ) -> EditResult<ChangeSet> {
        check_polyline("geometry", &geometry)?;
        let b = self
            .boundaries
            .get_mut(&boundary)
            .ok_or(EditError::not_found(boundary))?;
        let mut cs = ChangeSet::new();
        if b.geometry == geometry {
            return Ok(cs);
        }
        b.geometry = geometry;
        cs.modified(boundary);
        for lane in self.lanes_using_boundary(boundary) {
            cs.modified(lane);
            let links: Vec<(LaneId, LaneId)> = self
                .predecessors(lane)
                .iter()
                .map(|&p| (p, lane))
                .chain(self.successors(lane).iter().map(|&s| (lane, s)))
                .collect();
            for (from, to) in links {
                if let Some(gap) = self.connection_gap(from, to)
                    && gap > GAP_TOLERANCE
                {
                    cs.warn(
                        Issue::warning(
                            codes::GEOMETRIC_GAP,
                            format!(
                                "{to} now starts {gap:.3} m away from the end of {from}; \
                                 move the matching boundary too"
                            ),
                        )
                        .with_entity(from)
                        .with_related(to),
                    );
                }
            }
        }
        Ok(cs.finish())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topology::NeighborDirection;

    fn straight(len: f64) -> Polyline3 {
        Polyline3::from_xy(&[[0.0, 0.0], [len, 0.0]])
    }

    fn unsized_lane() -> RoadLane {
        RoadLane {
            width: None,
            direction: LaneDirection::Forward,
            kind: LaneKind::Driving,
        }
    }

    #[test]
    fn two_way_road_shares_boundaries_and_links_opposite_neighbors() {
        let mut map = Map::new();
        // Forward on the left, as in left-hand traffic.
        let spec = NewRoad::new(
            straight(100.0),
            vec![
                RoadLane::new(3.5, LaneDirection::Forward),
                RoadLane::new(3.0, LaneDirection::Backward),
            ],
        );
        let (built, cs) = map.build_road(spec).unwrap();
        assert_eq!(built.lanes.len(), 2);
        let (f, b) = (built.lanes[0][0], built.lanes[1][0]);
        assert_eq!(map.lane_count(), 2);
        assert_eq!(map.boundary_count(), 3);
        assert_eq!(cs.created.len(), 5);
        // The forward lane runs +x between y = 3.25 and y = -0.25.
        let fl = map.oriented_boundary(f, Side::Left).unwrap();
        assert!((fl.first().unwrap().y - 3.25).abs() < 1e-9);
        assert!(fl.first().unwrap().x < fl.last().unwrap().x);
        // The backward lane runs -x; the centre line is on its right too.
        let br = map.oriented_boundary(b, Side::Right).unwrap();
        assert!((br.first().unwrap().y + 0.25).abs() < 1e-9);
        assert!(br.first().unwrap().x > br.last().unwrap().x);
        assert_eq!(map.lanes[&f].right.boundary, map.lanes[&b].right.boundary);
        assert_eq!(map.neighbor(f, Side::Right), Some(Neighbor::opposite(b)));
        assert_eq!(map.neighbor(b, Side::Right), Some(Neighbor::opposite(f)));
        let centre = map.lanes[&f].right.boundary;
        assert_eq!(map.boundaries[&centre].kind, BoundaryKind::SOLID);
    }

    #[test]
    fn segments_are_connected_in_each_direction() {
        let mut map = Map::new();
        let mut spec = NewRoad::new(
            straight(90.0),
            vec![
                RoadLane::new(3.5, LaneDirection::Forward),
                RoadLane::new(3.5, LaneDirection::Forward),
                RoadLane::new(3.5, LaneDirection::Backward),
            ],
        );
        spec.segment_length = Some(30.0);
        spec.speed_limit = Some(SpeedLimit::from_kmh(40.0));
        spec.name = Some("main street".into());
        let (built, _) = map.build_road(spec).unwrap();
        assert!(built.lanes.iter().all(|c| c.len() == 3), "{built:?}");
        assert_eq!(map.lane_count(), 9);
        for chain in &built.lanes {
            for w in chain.windows(2) {
                assert_eq!(map.successors(w[0]), &[w[1]]);
                assert!(map.connection_gap(w[0], w[1]).unwrap() < 1e-9);
            }
        }
        // Forward pieces run +x, the backward chain -x.
        let x0 = |l: LaneId| map.centerline(l).unwrap().first().unwrap().x;
        assert!(x0(built.lanes[0][0]) < x0(built.lanes[0][1]));
        assert!(x0(built.lanes[2][0]) > x0(built.lanes[2][1]));
        // Pieces side by side are neighbors: the forward lanes share a dashed line.
        let (a, b) = (built.lanes[0][1], built.lanes[1][1]);
        assert_eq!(map.neighbor(a, Side::Right), Some(Neighbor::same(b)));
        let shared = map.lanes[&a].right.boundary;
        assert_eq!(map.boundaries[&shared].kind, BoundaryKind::DASHED);
        let opposite = map.neighbor(b, Side::Right).unwrap();
        assert_eq!(opposite.direction, NeighborDirection::Opposite);
        assert!(
            (x0(opposite.lane) - 60.0).abs() < 1e-6,
            "the middle backward piece starts at x = 60"
        );
        let road = built.road.unwrap();
        assert_eq!(map.roads[&road].lanes.len(), 9);
        assert!(
            map.lanes()
                .all(|l| l.speed_limit == Some(SpeedLimit::from_kmh(40.0)))
        );
    }

    #[test]
    fn explicit_boundaries_are_used_and_oriented() {
        let mut map = Map::new();
        let mut spec = NewRoad::new(straight(50.0), vec![unsized_lane()]);
        spec.boundaries = Some(vec![
            Polyline3::from_xy(&[[0.0, 2.0], [25.0, 2.2], [50.0, 1.9]]),
            // Given backwards: it is reversed.
            Polyline3::from_xy(&[[50.0, -1.6], [0.0, -1.5]]),
        ]);
        let (built, _) = map.build_road(spec).unwrap();
        let lane = built.lanes[0][0];
        let right = map.oriented_boundary(lane, Side::Right).unwrap();
        assert_eq!(right.first().unwrap().x, 0.0);
        let left = map.oriented_boundary(lane, Side::Left).unwrap();
        assert_eq!(left.points.len(), 3);
    }

    #[test]
    fn invalid_input_leaves_the_map_unchanged() {
        let mut map = Map::new();
        let before = map.clone();
        assert!(
            map.build_road(NewRoad::new(straight(10.0), vec![]))
                .is_err()
        );
        assert!(matches!(
            map.build_road(NewRoad::new(straight(10.0), vec![unsized_lane()])),
            Err(EditError::InvalidArgument { .. })
        ));
        assert_eq!(map, before);
    }

    #[test]
    fn connector_turns_left_and_meets_both_lanes() {
        let mut map = Map::new();
        // An eastbound road ending at x = 0 and a northbound one starting
        // at (10, 10): a left turn.
        let east = Polyline3::from_xy(&[[-50.0, 0.0], [0.0, 0.0]]);
        let north = Polyline3::from_xy(&[[10.0, 10.0], [10.0, 60.0]]);
        let one = |r: Polyline3, kmh: f64| {
            let mut s = NewRoad::new(r, vec![RoadLane::new(3.5, LaneDirection::Forward)]);
            s.speed_limit = Some(SpeedLimit::from_kmh(kmh));
            s
        };
        let a = map.build_road(one(east, 50.0)).unwrap().0.lanes[0][0];
        let b = map.build_road(one(north, 30.0)).unwrap().0.lanes[0][0];
        let (c, cs) = map.add_connector(NewConnector::new(a, b)).unwrap();
        assert!(cs.warnings.is_empty(), "{:?}", cs.warnings);
        assert_eq!(map.successors(a), &[c]);
        assert_eq!(map.successors(c), &[b]);
        assert!(map.connection_gap(a, c).unwrap() < 1e-9);
        assert!(map.connection_gap(c, b).unwrap() < 1e-9);
        let lane = &map.lanes[&c];
        assert_eq!(lane.turn_direction, Some(TurnDirection::Left));
        assert_eq!(lane.speed_limit, Some(SpeedLimit::from_kmh(30.0)));
        let left = map.boundaries[&lane.left.boundary].clone();
        assert_eq!(left.kind, BoundaryKind::Virtual);
        // The curve leaves eastwards and arrives northwards.
        let h0 = heading(&left.geometry, false).unwrap();
        let h1 = heading(&left.geometry, true).unwrap();
        assert!(h0.x > 0.9 && h1.y > 0.9, "{h0:?} {h1:?}");
        // Already touching lanes are refused.
        assert!(map.add_connector(NewConnector::new(a, c)).is_err());
    }

    #[test]
    fn set_boundary_geometry_warns_about_gaps() {
        let mut map = Map::new();
        let mut spec = NewRoad::new(
            straight(60.0),
            vec![RoadLane::new(3.5, LaneDirection::Forward)],
        );
        spec.segment_length = Some(30.0);
        let (built, _) = map.build_road(spec).unwrap();
        let first = built.lanes[0][0];
        let left = map.lanes[&first].left.boundary;
        let cs = map
            .set_boundary_geometry(left, Polyline3::from_xy(&[[0.0, 1.75], [30.0, 2.5]]))
            .unwrap();
        assert!(cs.modified.contains(&left.into()));
        assert!(cs.modified.contains(&first.into()));
        assert_eq!(cs.warnings.len(), 1);
        assert_eq!(cs.warnings[0].code, codes::GEOMETRIC_GAP);
        assert!(map.set_boundary_geometry(left, straight(0.0)).is_err());
    }
}
