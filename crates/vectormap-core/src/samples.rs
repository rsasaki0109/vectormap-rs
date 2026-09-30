//! Small, deterministic sample maps built with the public editing API.
//!
//! They serve as integration fixtures, documentation and demo data:
//!
//! - [`straight_road`]: `A → B → C`
//! - [`two_lane_road`]: two parallel lanes, each split into two sections
//! - [`intersection`]: a signalized 4-way intersection with a crosswalk
//!
//! All samples use right-hand traffic, 3.5 m lanes and a georeference in
//! Tokyo so that they can be exported to Lanelet2 / Autoware directly.

use crate::edit::{
    BoundarySpec, CrosswalkGeometry, LaneGeometry, NewCrosswalk, NewLane, NewStopLine,
    NewTrafficSignal, StopLineChoice, StopLinePlacement, StopRule,
};
use crate::entities::{BoundaryKind, BoundaryRef, Side, SpeedLimit, TurnDirection};
use crate::geometry::{Point2, Point3, Polygon3, Polyline3};
use crate::id::LaneId;
use crate::map::{GeoPoint, GeoReference, Map, ProjectionKind};
use crate::topology::Neighbor;

/// Lane width used by all samples (metres).
pub const LANE_WIDTH: f64 = 3.5;

fn georeference() -> GeoReference {
    GeoReference {
        projection: ProjectionKind::Utm,
        origin: GeoPoint::new(35.681236, 139.767125),
    }
}

fn existing(r: BoundaryRef) -> BoundarySpec {
    BoundarySpec::Existing(r)
}

/// Straight road with three consecutive 20 m lanes: `A → B → C`.
///
/// Returns the map and the lane IDs `[A, B, C]`.
pub fn straight_road() -> (Map, [LaneId; 3]) {
    let mut map = Map::new();
    map.metadata_mut().name = Some("straight_road".into());
    map.metadata_mut().georeference = Some(georeference());
    let mut ids = Vec::new();
    for i in 0..3 {
        let x0 = 20.0 * i as f64;
        let mut spec = NewLane::from_centerline(
            Polyline3::from_xy(&[[x0, 0.0], [x0 + 10.0, 0.0], [x0 + 20.0, 0.0]]),
            LANE_WIDTH,
        )
        .with_speed_limit(SpeedLimit::from_kmh(50.0));
        spec.predecessors = ids.last().copied().into_iter().collect();
        let (id, _) = map.add_lane(spec).expect("valid sample");
        ids.push(id);
    }
    map.add_road(Some("Main Street".into()), &ids)
        .expect("valid sample");
    (map, [ids[0], ids[1], ids[2]])
}

/// Two-lane one-way road, split into two sections:
///
/// ```text
/// A ---> B     (left lane)
/// |      |
/// C ---> D     (right lane)
/// ```
///
/// The lanes of a section share a dashed boundary and are neighbors.
/// Returns the map and `[A, B, C, D]`.
pub fn two_lane_road() -> (Map, [LaneId; 4]) {
    let mut map = Map::new();
    map.metadata_mut().name = Some("two_lane_road".into());
    map.metadata_mut().georeference = Some(georeference());
    let line = |x0: f64, y: f64| Polyline3::from_xy(&[[x0, y], [x0 + 15.0, y], [x0 + 30.0, y]]);
    let mut sections: Vec<(LaneId, LaneId)> = Vec::new();
    for section in 0..2 {
        let x0 = 30.0 * section as f64;
        let (top, _) = map
            .add_boundary(BoundaryKind::SOLID, line(x0, LANE_WIDTH))
            .unwrap();
        let (mid, _) = map
            .add_boundary(BoundaryKind::DASHED, line(x0, 0.0))
            .unwrap();
        let (bot, _) = map
            .add_boundary(BoundaryKind::SOLID, line(x0, -LANE_WIDTH))
            .unwrap();
        let prev = sections.last().copied();
        let mut make = |left, right, pred: Option<LaneId>| {
            let mut spec = NewLane::new(LaneGeometry::Boundaries {
                left: existing(BoundaryRef::forward(left)),
                right: existing(BoundaryRef::forward(right)),
            })
            .with_speed_limit(SpeedLimit::from_kmh(60.0));
            spec.predecessors = pred.into_iter().collect();
            map.add_lane(spec).expect("valid sample").0
        };
        let upper = make(top, mid, prev.map(|p| p.0));
        let lower = make(mid, bot, prev.map(|p| p.1));
        map.set_neighbor(upper, Side::Right, Some(Neighbor::same(lower)))
            .unwrap();
        map.add_road(Some(format!("Section {}", section + 1)), &[upper, lower])
            .unwrap();
        sections.push((upper, lower));
    }
    let [(a, c), (b, d)] = [sections[0], sections[1]];
    (map, [a, b, c, d])
}

/// Lane IDs of the [`intersection`] sample.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IntersectionLanes {
    /// Incoming lane of each arm (east, north, west, south).
    pub incoming: [LaneId; 4],
    /// Outgoing lane of each arm (east, north, west, south).
    pub outgoing: [LaneId; 4],
    /// Connecting lanes inside the junction.
    pub connectors: Vec<LaneId>,
}

const ARM_DIRS: [(f64, f64); 4] = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
const ARM_NAMES: [&str; 4] = ["East", "North", "West", "South"];
const JUNCTION_HALF: f64 = 2.0 * LANE_WIDTH;
const ARM_LENGTH: f64 = 30.0;

/// Point at distance `s` from the centre along arm `arm`, lateral offset `t`
/// (positive to the left of the outward direction).
fn arm_point(arm: usize, s: f64, t: f64) -> Point3 {
    let (ux, uy) = ARM_DIRS[arm];
    let (vx, vy) = (-uy, ux);
    Point3::new(s * ux + t * vx, s * uy + t * vy, 0.0)
}

fn arm_line(arm: usize, t: f64) -> Polyline3 {
    let s0 = JUNCTION_HALF;
    let s1 = JUNCTION_HALF + ARM_LENGTH;
    Polyline3::new(vec![
        arm_point(arm, s0, t),
        arm_point(arm, (s0 + s1) / 2.0, t),
        arm_point(arm, s1, t),
    ])
}

/// Quadratic Bézier from `p0` to `p2` whose tangents are `d0` at the start
/// and `d2` at the end (straight line if they are parallel).
fn curve(p0: Point3, d0: Point2, p2: Point3, d2: Point2) -> Polyline3 {
    let denom = d0.cross(d2);
    if denom.abs() < 1e-9 {
        return Polyline3::new(vec![p0, p0.lerp(p2, 0.5), p2]);
    }
    // Intersection of p0 + a*d0 and p2 - b*d2.
    let a = (p2.xy() - p0.xy()).cross(d2) / denom;
    let c = p0.xy() + d0 * a;
    let n = 8;
    let mut pts = Vec::with_capacity(n + 1);
    pts.push(p0);
    for i in 1..n {
        let t = i as f64 / n as f64;
        let q = p0.xy() * ((1.0 - t) * (1.0 - t)) + c * (2.0 * (1.0 - t) * t) + p2.xy() * (t * t);
        pts.push(q.with_z(0.0));
    }
    pts.push(p2);
    Polyline3::new(pts)
}

/// A signalized 4-way intersection.
///
/// - four arms (east, north, west, south), each with one incoming and one
///   outgoing 30 m lane separated by a solid centre line;
/// - twelve connecting lanes (straight, left, right) inside a junction with
///   an outline;
/// - a stop line and a traffic light on every incoming lane;
/// - a crosswalk across the north arm.
pub fn intersection() -> (Map, IntersectionLanes) {
    let mut map = Map::new();
    map.metadata_mut().name = Some("intersection".into());
    map.metadata_mut().georeference = Some(georeference());
    let arm_speed = SpeedLimit::from_kmh(40.0);

    let mut incoming = Vec::new();
    let mut outgoing = Vec::new();
    for (arm, arm_name) in ARM_NAMES.iter().enumerate() {
        let (outer_left, _) = map
            .add_boundary(BoundaryKind::Curb, arm_line(arm, LANE_WIDTH))
            .unwrap();
        let (center, _) = map
            .add_boundary(BoundaryKind::SOLID, arm_line(arm, 0.0))
            .unwrap();
        let (outer_right, _) = map
            .add_boundary(BoundaryKind::Curb, arm_line(arm, -LANE_WIDTH))
            .unwrap();
        // Incoming lane drives towards the centre, i.e. against the
        // boundaries' outward orientation.
        let (inc, _) = map
            .add_lane(
                NewLane::new(LaneGeometry::Boundaries {
                    left: existing(BoundaryRef::backward(center)),
                    right: existing(BoundaryRef::backward(outer_left)),
                })
                .with_speed_limit(arm_speed),
            )
            .unwrap();
        let (out, _) = map
            .add_lane(
                NewLane::new(LaneGeometry::Boundaries {
                    left: existing(BoundaryRef::forward(center)),
                    right: existing(BoundaryRef::forward(outer_right)),
                })
                .with_speed_limit(arm_speed),
            )
            .unwrap();
        map.set_neighbor(inc, Side::Left, Some(Neighbor::opposite(out)))
            .unwrap();
        map.add_road(Some(format!("{arm_name} Arm")), &[inc, out])
            .unwrap();
        incoming.push(inc);
        outgoing.push(out);
    }

    // Connecting lanes: from incoming arm i to outgoing arm j.
    let mut connectors = Vec::new();
    for i in 0..4 {
        for (j, turn) in [
            ((i + 2) % 4, TurnDirection::Straight),
            ((i + 1) % 4, TurnDirection::Right),
            ((i + 3) % 4, TurnDirection::Left),
        ] {
            // Arms are ordered counter-clockwise, so the next arm is on the
            // right of a vehicle coming in from arm `i`.
            let (ui, uj) = (ARM_DIRS[i], ARM_DIRS[j]);
            let d_in = Point2::new(-ui.0, -ui.1);
            let d_out = Point2::new(uj.0, uj.1);
            let left = curve(
                arm_point(i, JUNCTION_HALF, 0.0),
                d_in,
                arm_point(j, JUNCTION_HALF, 0.0),
                d_out,
            );
            let right = curve(
                arm_point(i, JUNCTION_HALF, LANE_WIDTH),
                d_in,
                arm_point(j, JUNCTION_HALF, -LANE_WIDTH),
                d_out,
            );
            let mut spec = NewLane::new(LaneGeometry::Boundaries {
                left: BoundarySpec::New {
                    geometry: left,
                    kind: BoundaryKind::Virtual,
                },
                right: BoundarySpec::New {
                    geometry: right,
                    kind: BoundaryKind::Virtual,
                },
            })
            .with_speed_limit(if turn == TurnDirection::Straight {
                arm_speed
            } else {
                SpeedLimit::from_kmh(20.0)
            });
            spec.turn_direction = Some(turn);
            spec.predecessors = vec![incoming[i]];
            spec.successors = vec![outgoing[j]];
            let (id, _) = map.add_lane(spec).unwrap();
            connectors.push(id);
        }
    }
    let h = JUNCTION_HALF;
    let outline = Polygon3::new(vec![
        Point3::new(-h, -h, 0.0),
        Point3::new(h, -h, 0.0),
        Point3::new(h, h, 0.0),
        Point3::new(-h, h, 0.0),
    ]);
    map.add_junction(Some("Central Junction".into()), &connectors, Some(outline))
        .unwrap();

    // Stop lines and traffic lights on every incoming lane.
    for (arm, &lane) in incoming.iter().enumerate() {
        let (stop_line, _) = map
            .add_stop_line(NewStopLine {
                lanes: vec![lane],
                placement: StopLinePlacement::AtLaneEnd { offset: 6.0 },
                rule: StopRule::None,
            })
            .unwrap();
        // Signal on the far side of the junction, facing the incoming traffic.
        let far = arm_point(arm, -JUNCTION_HALF - 1.0, LANE_WIDTH / 2.0);
        let (ux, uy) = ARM_DIRS[arm];
        // Seen from the traffic (heading -u), "left" is -v = (uy, -ux).
        let left = Point2::new(uy, -ux) * 0.6;
        let geometry = Polyline3::new(vec![
            Point3::new(far.x + left.x, far.y + left.y, 5.0),
            Point3::new(far.x - left.x, far.y - left.y, 5.0),
        ]);
        let mut spec = NewTrafficSignal::for_lanes(vec![lane]);
        spec.stop_line = StopLineChoice::Existing(stop_line);
        spec.geometry = Some(geometry);
        map.add_traffic_signal(spec).unwrap();
    }

    // Crosswalk across the north arm, between the junction and the stop line.
    map.add_crosswalk(NewCrosswalk {
        geometry: CrosswalkGeometry::Across {
            lane: outgoing[1],
            station: 2.5,
            width: 4.0,
            margin: 0.5,
        },
        crossing_lanes: None,
        stop_line_offset: None,
    })
    .unwrap();

    let lanes = IntersectionLanes {
        incoming: incoming.try_into().unwrap(),
        outgoing: outgoing.try_into().unwrap(),
        connectors,
    };
    (map, lanes)
}
