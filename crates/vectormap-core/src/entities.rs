//! Map entities: lanes, boundaries, roads, junctions and traffic features.
//!
//! Entities carry *geometry* and *semantics*. Lane-to-lane connectivity is
//! deliberately not stored here but in [`Topology`](crate::Topology).

use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::attributes::Attributes;
use crate::geometry::{Point3, Polygon3, Polyline3};
use crate::id::{
    BoundaryId, CrosswalkId, JunctionId, LaneId, RegulatoryElementId, RoadId, SignalId, StopLineId,
};

fn is_false(b: &bool) -> bool {
    !*b
}

fn is_true(b: &bool) -> bool {
    *b
}

fn default_true() -> bool {
    true
}

/// Left or right, relative to the direction of travel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Side {
    /// Left side.
    Left,
    /// Right side.
    Right,
}

impl Side {
    /// The other side.
    pub const fn opposite(self) -> Side {
        match self {
            Side::Left => Side::Right,
            Side::Right => Side::Left,
        }
    }
}

// ---------------------------------------------------------------------------
// Boundary
// ---------------------------------------------------------------------------

/// Pattern of a painted line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MarkingPattern {
    /// Single solid line.
    #[default]
    Solid,
    /// Single dashed line.
    Dashed,
    /// Double solid line.
    SolidSolid,
    /// Solid on the left, dashed on the right (relative to the line direction).
    SolidDashed,
    /// Dashed on the left, solid on the right (relative to the line direction).
    DashedSolid,
}

/// Width class of a painted line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MarkingWeight {
    /// Regular lane line.
    #[default]
    Thin,
    /// Thick line (e.g. motorway edge).
    Thick,
}

/// What physically separates a lane from its surroundings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum BoundaryKind {
    /// No physical marking (e.g. inside junctions).
    Virtual,
    /// Painted lane marking.
    LaneMarking {
        /// Line pattern.
        #[serde(default)]
        pattern: MarkingPattern,
        /// Line weight.
        #[serde(default)]
        weight: MarkingWeight,
    },
    /// Curbstone.
    Curb,
    /// Edge of the drivable surface without a curb (road border, barrier).
    RoadEdge,
    /// Anything else; details are kept in the boundary attributes.
    Other,
}

impl Default for BoundaryKind {
    fn default() -> Self {
        BoundaryKind::LaneMarking {
            pattern: MarkingPattern::Solid,
            weight: MarkingWeight::Thin,
        }
    }
}

impl BoundaryKind {
    /// A thin solid line.
    pub const SOLID: BoundaryKind = BoundaryKind::LaneMarking {
        pattern: MarkingPattern::Solid,
        weight: MarkingWeight::Thin,
    };
    /// A thin dashed line.
    pub const DASHED: BoundaryKind = BoundaryKind::LaneMarking {
        pattern: MarkingPattern::Dashed,
        weight: MarkingWeight::Thin,
    };

    /// Whether a vehicle on `side` of the boundary (relative to the boundary's
    /// own direction) may cross it.
    pub fn crossable_from(self, side: Side) -> bool {
        match self {
            BoundaryKind::Virtual => true,
            BoundaryKind::LaneMarking { pattern, .. } => matches!(
                (pattern, side),
                (MarkingPattern::Dashed, _)
                    | (MarkingPattern::SolidDashed, Side::Right)
                    | (MarkingPattern::DashedSolid, Side::Left)
            ),
            BoundaryKind::Curb | BoundaryKind::RoadEdge | BoundaryKind::Other => false,
        }
    }
}

/// A lane boundary line. Adjacent lanes share the boundary between them.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Boundary {
    /// Identifier.
    pub id: BoundaryId,
    /// Kind of boundary.
    #[serde(default)]
    pub kind: BoundaryKind,
    /// Geometry. Its direction is the boundary's own direction; lanes refer
    /// to it with [`BoundaryRef::reversed`] when they traverse it backwards.
    pub geometry: Polyline3,
    /// Extension attributes.
    #[serde(default, skip_serializing_if = "Attributes::is_empty")]
    pub attributes: Attributes,
}

impl Boundary {
    /// Creates a boundary without attributes.
    pub fn new(id: BoundaryId, kind: BoundaryKind, geometry: Polyline3) -> Self {
        Self {
            id,
            kind,
            geometry,
            attributes: Attributes::new(),
        }
    }
}

/// A lane's reference to one of its boundaries.
///
/// Serializes as a plain integer when not reversed and as
/// `{"boundary": 7, "reversed": true}` otherwise.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BoundaryRef {
    /// The referenced boundary.
    pub boundary: BoundaryId,
    /// `true` if the lane runs against the boundary's direction.
    pub reversed: bool,
}

impl BoundaryRef {
    /// Reference following the boundary's direction.
    pub const fn forward(boundary: BoundaryId) -> Self {
        Self {
            boundary,
            reversed: false,
        }
    }

    /// Reference running against the boundary's direction.
    pub const fn backward(boundary: BoundaryId) -> Self {
        Self {
            boundary,
            reversed: true,
        }
    }

    /// Geometry of `boundary` oriented along the lane.
    pub fn oriented(&self, boundary: &Boundary) -> Polyline3 {
        if self.reversed {
            boundary.geometry.reversed()
        } else {
            boundary.geometry.clone()
        }
    }
}

impl From<BoundaryId> for BoundaryRef {
    fn from(id: BoundaryId) -> Self {
        BoundaryRef::forward(id)
    }
}

#[derive(Serialize, Deserialize)]
#[serde(untagged)]
enum BoundaryRefRepr {
    Id(BoundaryId),
    Full {
        boundary: BoundaryId,
        #[serde(default, skip_serializing_if = "is_false")]
        reversed: bool,
    },
}

impl Serialize for BoundaryRef {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        if self.reversed {
            BoundaryRefRepr::Full {
                boundary: self.boundary,
                reversed: true,
            }
            .serialize(s)
        } else {
            self.boundary.serialize(s)
        }
    }
}

impl<'de> Deserialize<'de> for BoundaryRef {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        Ok(match BoundaryRefRepr::deserialize(d)? {
            BoundaryRefRepr::Id(boundary) => BoundaryRef::forward(boundary),
            BoundaryRefRepr::Full { boundary, reversed } => BoundaryRef { boundary, reversed },
        })
    }
}

// ---------------------------------------------------------------------------
// Lane
// ---------------------------------------------------------------------------

/// Functional kind of a lane.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LaneKind {
    /// Regular lane for motor vehicles.
    #[default]
    Driving,
    /// Road shoulder.
    Shoulder,
    /// Bus lane.
    Bus,
    /// Bicycle lane.
    Bicycle,
    /// Sidewalk / walkway for pedestrians.
    Walkway,
    /// Parking lane.
    Parking,
    /// Emergency lane.
    Emergency,
    /// Anything else; details are kept in the lane attributes.
    Other,
}

impl LaneKind {
    /// Snake-case name as used in serialized data.
    pub const fn as_str(self) -> &'static str {
        match self {
            LaneKind::Driving => "driving",
            LaneKind::Shoulder => "shoulder",
            LaneKind::Bus => "bus",
            LaneKind::Bicycle => "bicycle",
            LaneKind::Walkway => "walkway",
            LaneKind::Parking => "parking",
            LaneKind::Emergency => "emergency",
            LaneKind::Other => "other",
        }
    }

    /// `true` for lanes used by motor vehicles in normal traffic.
    pub const fn is_vehicle_lane(self) -> bool {
        matches!(
            self,
            LaneKind::Driving | LaneKind::Bus | LaneKind::Emergency
        )
    }
}

/// Manoeuvre performed by a lane inside a junction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TurnDirection {
    /// Going straight.
    Straight,
    /// Turning left.
    Left,
    /// Turning right.
    Right,
}

/// A speed limit, stored in km/h (the unit used by road signs and by the
/// Lanelet2 / Autoware conventions).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct SpeedLimit {
    /// Limit in kilometres per hour.
    pub kmh: f64,
}

impl SpeedLimit {
    /// Creates a limit from km/h.
    pub const fn from_kmh(kmh: f64) -> Self {
        Self { kmh }
    }

    /// Creates a limit from m/s.
    pub fn from_mps(mps: f64) -> Self {
        Self { kmh: mps * 3.6 }
    }

    /// Limit in km/h.
    pub const fn kmh(self) -> f64 {
        self.kmh
    }

    /// Limit in m/s.
    pub fn mps(self) -> f64 {
        self.kmh / 3.6
    }

    /// `true` if the value is finite and positive.
    pub fn is_valid(self) -> bool {
        self.kmh.is_finite() && self.kmh > 0.0
    }
}

/// A lane: the basic unit of drivable (or walkable) space.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Lane {
    /// Identifier.
    pub id: LaneId,
    /// Functional kind.
    #[serde(default)]
    pub kind: LaneKind,
    /// Left boundary (relative to the direction of travel).
    pub left: BoundaryRef,
    /// Right boundary (relative to the direction of travel).
    pub right: BoundaryRef,
    /// Explicit centerline. When absent it is derived from the boundaries
    /// (see [`Map::centerline`](crate::Map::centerline)).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub centerline: Option<Polyline3>,
    /// Speed limit.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_limit: Option<SpeedLimit>,
    /// `false` if the lane may be driven in both directions.
    #[serde(default = "default_true", skip_serializing_if = "is_true")]
    pub one_way: bool,
    /// Manoeuvre inside a junction.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub turn_direction: Option<TurnDirection>,
    /// Extension attributes.
    #[serde(default, skip_serializing_if = "Attributes::is_empty")]
    pub attributes: Attributes,
}

impl Lane {
    /// Creates a one-way driving lane between two boundaries.
    pub fn new(id: LaneId, left: impl Into<BoundaryRef>, right: impl Into<BoundaryRef>) -> Self {
        Self {
            id,
            kind: LaneKind::Driving,
            left: left.into(),
            right: right.into(),
            centerline: None,
            speed_limit: None,
            one_way: true,
            turn_direction: None,
            attributes: Attributes::new(),
        }
    }

    /// Boundary reference on `side`.
    pub const fn boundary(&self, side: Side) -> BoundaryRef {
        match side {
            Side::Left => self.left,
            Side::Right => self.right,
        }
    }

    /// Mutable boundary reference on `side`.
    pub fn boundary_mut(&mut self, side: Side) -> &mut BoundaryRef {
        match side {
            Side::Left => &mut self.left,
            Side::Right => &mut self.right,
        }
    }
}

// ---------------------------------------------------------------------------
// Road / Junction
// ---------------------------------------------------------------------------

/// A named group of lanes, e.g. one road section.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Road {
    /// Identifier.
    pub id: RoadId,
    /// Human readable name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Member lanes (ordered, typically left to right).
    #[serde(default)]
    pub lanes: Vec<LaneId>,
    /// Extension attributes.
    #[serde(default, skip_serializing_if = "Attributes::is_empty")]
    pub attributes: Attributes,
}

/// An area where roads meet, and the lanes that connect them.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Junction {
    /// Identifier.
    pub id: JunctionId,
    /// Human readable name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Connecting lanes inside the junction.
    #[serde(default)]
    pub lanes: Vec<LaneId>,
    /// Outline of the junction area.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub outline: Option<Polygon3>,
    /// Extension attributes.
    #[serde(default, skip_serializing_if = "Attributes::is_empty")]
    pub attributes: Attributes,
}

// ---------------------------------------------------------------------------
// Traffic features
// ---------------------------------------------------------------------------

/// A painted stop line. The lanes it governs are defined by the rules
/// ([`RegulatoryElement`]) that reference it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StopLine {
    /// Identifier.
    pub id: StopLineId,
    /// Geometry, usually a single segment across the lane(s).
    pub geometry: Polyline3,
    /// Extension attributes.
    #[serde(default, skip_serializing_if = "Attributes::is_empty")]
    pub attributes: Attributes,
}

/// Who a traffic signal is meant for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignalKind {
    /// Vehicle signal (typically red / yellow / green).
    #[default]
    Vehicle,
    /// Pedestrian signal (typically red / green).
    Pedestrian,
}

/// Colour of a signal bulb.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BulbColor {
    /// Red.
    Red,
    /// Yellow / amber.
    Yellow,
    /// Green.
    Green,
    /// White (e.g. tram or pedestrian signals).
    White,
}

/// Arrow shape of a signal bulb.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BulbArrow {
    /// Up (straight).
    Up,
    /// Down.
    Down,
    /// Left.
    Left,
    /// Right.
    Right,
    /// Up-left.
    UpLeft,
    /// Up-right.
    UpRight,
    /// Down-left.
    DownLeft,
    /// Down-right.
    DownRight,
}

/// A single lamp of a traffic signal.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SignalBulb {
    /// Position of the lamp centre.
    pub position: Point3,
    /// Colour.
    pub color: BulbColor,
    /// Arrow shape, if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub arrow: Option<BulbArrow>,
}

/// A physical traffic signal head. Which lanes it controls is defined by
/// [`Rule::TrafficLight`] regulatory elements.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrafficSignal {
    /// Identifier.
    pub id: SignalId,
    /// Vehicle or pedestrian signal.
    #[serde(default)]
    pub kind: SignalKind,
    /// Bottom edge of the signal housing, from left to right as seen by the
    /// controlled traffic, at its real elevation.
    pub geometry: Polyline3,
    /// Height of the housing (metres).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub height: Option<f64>,
    /// Individual lamps.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bulbs: Vec<SignalBulb>,
    /// Extension attributes.
    #[serde(default, skip_serializing_if = "Attributes::is_empty")]
    pub attributes: Attributes,
}

/// A pedestrian crossing.
///
/// `left_edge` and `right_edge` are the sides of the crossing relative to the
/// pedestrian walking direction; both run across the road.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Crosswalk {
    /// Identifier.
    pub id: CrosswalkId,
    /// Left side of the crossing (walking direction).
    pub left_edge: Polyline3,
    /// Right side of the crossing (walking direction).
    pub right_edge: Polyline3,
    /// Explicit outline. When absent it is derived from the edges.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub polygon: Option<Polygon3>,
    /// Extension attributes.
    #[serde(default, skip_serializing_if = "Attributes::is_empty")]
    pub attributes: Attributes,
}

impl Crosswalk {
    /// Outline of the crosswalk: the explicit polygon or the area between
    /// the two edges.
    pub fn outline(&self) -> Polygon3 {
        self.polygon
            .clone()
            .unwrap_or_else(|| Polygon3::from_sides(&self.left_edge, &self.right_edge))
    }
}

// ---------------------------------------------------------------------------
// Regulatory elements
// ---------------------------------------------------------------------------

/// The traffic rule expressed by a [`RegulatoryElement`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Rule {
    /// Traffic lights (a signal group) with an optional stop line.
    TrafficLight {
        /// Signal heads showing the same state.
        signals: Vec<SignalId>,
        /// Where to stop.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        stop_line: Option<StopLineId>,
    },
    /// A traffic sign, e.g. `stop_sign`, optionally with a stop line.
    TrafficSign {
        /// Sign type, e.g. `stop_sign`, `de206`, `usR1-1`.
        sign_type: String,
        /// Geometry of the sign, if known.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        sign: Option<Polyline3>,
        /// Where to stop.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        stop_line: Option<StopLineId>,
    },
    /// A stop line marking without an associated sign or light.
    StopLine {
        /// The stop line.
        stop_line: StopLineId,
    },
    /// Right of way between lanes.
    RightOfWay {
        /// Lanes that have priority.
        #[serde(default)]
        priority: Vec<LaneId>,
        /// Lanes that must yield.
        #[serde(default)]
        yielding: Vec<LaneId>,
        /// Stop / yield lines for the yielding lanes.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        stop_lines: Vec<StopLineId>,
    },
    /// Crossing lanes must give way to pedestrians on a crosswalk.
    Crosswalk {
        /// The crosswalk.
        crosswalk: CrosswalkId,
        /// Stop lines in front of the crosswalk.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        stop_lines: Vec<StopLineId>,
    },
    /// A rule the IR does not model; details are kept in the attributes.
    Other {
        /// Source-specific kind, e.g. `detection_area`.
        kind: String,
    },
}

impl Rule {
    /// Short machine-readable name of the rule type.
    pub fn type_name(&self) -> &str {
        match self {
            Rule::TrafficLight { .. } => "traffic_light",
            Rule::TrafficSign { .. } => "traffic_sign",
            Rule::StopLine { .. } => "stop_line",
            Rule::RightOfWay { .. } => "right_of_way",
            Rule::Crosswalk { .. } => "crosswalk",
            Rule::Other { kind } => kind,
        }
    }

    /// All stop lines referenced by the rule.
    pub fn stop_lines(&self) -> Vec<StopLineId> {
        match self {
            Rule::TrafficLight { stop_line, .. } | Rule::TrafficSign { stop_line, .. } => {
                stop_line.iter().copied().collect()
            }
            Rule::StopLine { stop_line } => vec![*stop_line],
            Rule::RightOfWay { stop_lines, .. } | Rule::Crosswalk { stop_lines, .. } => {
                stop_lines.clone()
            }
            Rule::Other { .. } => Vec::new(),
        }
    }

    /// All traffic signals referenced by the rule.
    pub fn signals(&self) -> &[SignalId] {
        match self {
            Rule::TrafficLight { signals, .. } => signals,
            _ => &[],
        }
    }

    /// Lanes referenced *inside* the rule (not the lanes it applies to).
    pub fn referenced_lanes(&self) -> Vec<LaneId> {
        match self {
            Rule::RightOfWay {
                priority, yielding, ..
            } => priority.iter().chain(yielding).copied().collect(),
            _ => Vec::new(),
        }
    }

    /// The crosswalk referenced by the rule.
    pub fn crosswalk(&self) -> Option<CrosswalkId> {
        match self {
            Rule::Crosswalk { crosswalk, .. } => Some(*crosswalk),
            _ => None,
        }
    }
}

/// A traffic rule applying to a set of lanes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RegulatoryElement {
    /// Identifier.
    pub id: RegulatoryElementId,
    /// The rule.
    pub rule: Rule,
    /// Lanes the rule applies to.
    #[serde(default)]
    pub lanes: Vec<LaneId>,
    /// Extension attributes.
    #[serde(default, skip_serializing_if = "Attributes::is_empty")]
    pub attributes: Attributes,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boundary_ref_serialization() {
        let fwd = BoundaryRef::forward(BoundaryId(4));
        let bwd = BoundaryRef::backward(BoundaryId(5));
        assert_eq!(serde_json::to_string(&fwd).unwrap(), "4");
        assert_eq!(
            serde_json::to_string(&bwd).unwrap(),
            r#"{"boundary":5,"reversed":true}"#
        );
        let parsed: BoundaryRef = serde_json::from_str(r#"{"boundary":6}"#).unwrap();
        assert_eq!(parsed, BoundaryRef::forward(BoundaryId(6)));
        let parsed: BoundaryRef = serde_json::from_str("9").unwrap();
        assert_eq!(parsed, BoundaryRef::forward(BoundaryId(9)));
    }

    #[test]
    fn lane_defaults_are_omitted() {
        let lane = Lane::new(LaneId(1), BoundaryId(2), BoundaryId(3));
        let json = serde_json::to_string(&lane).unwrap();
        assert_eq!(json, r#"{"id":1,"kind":"driving","left":2,"right":3}"#);
        let back: Lane = serde_json::from_str(&json).unwrap();
        assert_eq!(back, lane);
    }

    #[test]
    fn lane_marking_crossability() {
        assert!(BoundaryKind::DASHED.crossable_from(Side::Left));
        assert!(!BoundaryKind::SOLID.crossable_from(Side::Right));
        let sd = BoundaryKind::LaneMarking {
            pattern: MarkingPattern::SolidDashed,
            weight: MarkingWeight::Thin,
        };
        assert!(sd.crossable_from(Side::Right));
        assert!(!sd.crossable_from(Side::Left));
        assert!(!BoundaryKind::Curb.crossable_from(Side::Left));
    }

    #[test]
    fn rule_serialization() {
        let rule = Rule::TrafficLight {
            signals: vec![SignalId(1)],
            stop_line: Some(StopLineId(2)),
        };
        assert_eq!(
            serde_json::to_string(&rule).unwrap(),
            r#"{"type":"traffic_light","signals":[1],"stop_line":2}"#
        );
        assert_eq!(rule.stop_lines(), vec![StopLineId(2)]);
    }

    #[test]
    fn speed_limit_units() {
        let s = SpeedLimit::from_mps(10.0);
        assert!((s.kmh() - 36.0).abs() < 1e-12);
        assert!((SpeedLimit::from_kmh(36.0).mps() - 10.0).abs() < 1e-12);
        assert!(!SpeedLimit::from_kmh(0.0).is_valid());
    }
}
