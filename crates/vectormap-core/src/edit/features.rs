//! Adding traffic features: stop lines, traffic signals and crosswalks.
//!
//! Placement helpers compute geometry deterministically from lane geometry,
//! so callers (including language models) can say "a stop line at the end
//! of lane 12" instead of providing coordinates.

use serde::{Deserialize, Serialize};

use super::{ChangeSet, EditError, EditResult, check_polyline};
use crate::diagnostics::{Issue, codes};
use crate::entities::{
    BulbColor, Crosswalk, RegulatoryElement, Rule, Side, SignalBulb, SignalKind, StopLine,
    TrafficSignal,
};
use crate::geometry::{Point3, Polygon2, Polyline3, segment_intersection};
use crate::id::{CrosswalkId, LaneId, RegulatoryElementId, SignalId, StopLineId};
use crate::map::Map;

/// Default mounting height of synthesized signals above the road (metres).
pub const DEFAULT_SIGNAL_ELEVATION: f64 = 5.0;
/// Default housing height of signals (metres).
pub const DEFAULT_SIGNAL_HEIGHT: f64 = 0.5;
/// Default width of synthesized signal housings (metres).
pub const DEFAULT_SIGNAL_WIDTH: f64 = 1.2;

fn default_crosswalk_width() -> f64 {
    4.0
}

/// Where to put a new stop line.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StopLinePlacement {
    /// Across the lane(s), `offset` metres before the end of the first lane.
    AtLaneEnd {
        /// Distance back from the lane end (metres).
        #[serde(default)]
        offset: f64,
    },
    /// Across the lane(s) at a station of the first lane.
    AtStation {
        /// Distance from the start of the first lane (metres).
        station: f64,
    },
    /// Explicit geometry.
    Geometry {
        /// The stop line.
        geometry: Polyline3,
    },
}

impl Default for StopLinePlacement {
    fn default() -> Self {
        StopLinePlacement::AtLaneEnd { offset: 0.0 }
    }
}

/// Which rule makes vehicles stop at a new stop line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StopRule {
    /// A stop sign (`traffic_sign` rule with `sign_type = "stop_sign"`).
    #[default]
    StopSign,
    /// A stop line marking without a sign (`stop_line` rule).
    Marking,
    /// No rule: the stop line is referenced later (e.g. by a traffic light).
    None,
}

/// Parameters of [`Map::add_stop_line`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NewStopLine {
    /// Lanes the stop line spans and applies to (first lane is the
    /// placement reference).
    pub lanes: Vec<LaneId>,
    /// Placement.
    #[serde(default)]
    pub placement: StopLinePlacement,
    /// Rule to create.
    #[serde(default)]
    pub rule: StopRule,
}

/// Which stop line a new traffic signal uses.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StopLineChoice {
    /// Reuse the stop line of the signal group or of an existing stop rule on
    /// the first lane; otherwise create one at the end of the first lane.
    #[default]
    Auto,
    /// Use an existing stop line.
    Existing(StopLineId),
    /// Create a new stop line.
    New(StopLinePlacement),
    /// No stop line.
    None,
}

/// Parameters of [`Map::add_traffic_signal`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NewTrafficSignal {
    /// Controlled lanes.
    pub lanes: Vec<LaneId>,
    /// Stop line.
    #[serde(default)]
    pub stop_line: StopLineChoice,
    /// Housing bottom edge (left to right as seen by the traffic).
    /// Synthesized above the stop line when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub geometry: Option<Polyline3>,
    /// Housing height; defaults to [`DEFAULT_SIGNAL_HEIGHT`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub height: Option<f64>,
    /// Vehicle or pedestrian signal.
    #[serde(default)]
    pub kind: SignalKind,
    /// Lamps; a standard layout is generated when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bulbs: Option<Vec<SignalBulb>>,
    /// Add the signal to an existing traffic-light rule (signal group)
    /// instead of creating a new rule.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub group: Option<RegulatoryElementId>,
}

impl NewTrafficSignal {
    /// A vehicle signal for `lanes` with all defaults.
    pub fn for_lanes(lanes: Vec<LaneId>) -> Self {
        Self {
            lanes,
            stop_line: StopLineChoice::Auto,
            geometry: None,
            height: None,
            kind: SignalKind::Vehicle,
            bulbs: None,
            group: None,
        }
    }
}

/// Geometry of a new crosswalk.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CrosswalkGeometry {
    /// Perpendicular to `lane` at `station`, spanning the lane and all its
    /// lateral neighbors plus `margin` on each side.
    Across {
        /// Reference lane.
        lane: LaneId,
        /// Station of the crosswalk centre on the reference lane (metres).
        station: f64,
        /// Width of the crosswalk along the lane (metres, default 4).
        #[serde(default = "default_crosswalk_width")]
        width: f64,
        /// Extra length beyond the outermost boundaries (metres).
        #[serde(default)]
        margin: f64,
    },
    /// Explicit edges (in the pedestrian walking direction).
    Edges {
        /// Left edge.
        left_edge: Polyline3,
        /// Right edge.
        right_edge: Polyline3,
    },
}

/// Parameters of [`Map::add_crosswalk`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NewCrosswalk {
    /// Geometry.
    pub geometry: CrosswalkGeometry,
    /// Lanes crossing the crosswalk; detected from geometry when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub crossing_lanes: Option<Vec<LaneId>>,
    /// If set, a stop line is created on each crossing lane this many metres
    /// before the crosswalk.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stop_line_offset: Option<f64>,
}

impl Map {
    fn check_lane_list(&self, lanes: &[LaneId]) -> EditResult<()> {
        if lanes.is_empty() {
            return Err(EditError::invalid("lanes", "at least one lane is required"));
        }
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

    /// A segment across `lanes` at `station` of `reference`, from the
    /// leftmost to the rightmost boundary.
    pub fn cross_section(
        &self,
        reference: LaneId,
        station: f64,
        lanes: &[LaneId],
    ) -> EditResult<Polyline3> {
        let center = self
            .centerline(reference)
            .ok_or_else(|| EditError::geometry(format!("{reference} has no usable geometry")))?;
        let p = center
            .point_at(station)
            .ok_or_else(|| EditError::geometry("empty centerline"))?;
        let dir = center
            .direction_at(station)
            .ok_or_else(|| EditError::geometry("degenerate centerline"))?;
        let normal = dir.perp();
        let mut left: Option<(f64, Point3)> = None;
        let mut right: Option<(f64, Point3)> = None;
        for &l in lanes {
            for side in [Side::Left, Side::Right] {
                let b = self
                    .oriented_boundary(l, side)
                    .ok_or_else(|| EditError::geometry(format!("{l} has a missing boundary")))?;
                let Some(n) = b.nearest_point(p.xy()) else {
                    continue;
                };
                let lat = normal.dot(n.point.xy() - p.xy());
                if left.is_none_or(|(v, _)| lat > v) {
                    left = Some((lat, n.point));
                }
                if right.is_none_or(|(v, _)| lat < v) {
                    right = Some((lat, n.point));
                }
            }
        }
        match (left, right) {
            (Some((a, pa)), Some((b, pb))) if a - b > 1e-3 => Ok(Polyline3::new(vec![pa, pb])),
            _ => Err(EditError::geometry(
                "cannot build a cross-section at this position",
            )),
        }
    }

    fn place_stop_line(
        &self,
        lanes: &[LaneId],
        placement: &StopLinePlacement,
        cs: &mut ChangeSet,
    ) -> EditResult<Polyline3> {
        let reference = lanes[0];
        let len = self
            .lane_length(reference)
            .ok_or_else(|| EditError::geometry(format!("{reference} has no usable geometry")))?;
        let station = match placement {
            StopLinePlacement::Geometry { geometry } => {
                check_polyline("geometry", geometry)?;
                return Ok(geometry.clone());
            }
            StopLinePlacement::AtLaneEnd { offset } => {
                if !offset.is_finite() || *offset < 0.0 {
                    return Err(EditError::invalid(
                        "offset",
                        "must be a non-negative distance",
                    ));
                }
                len - offset
            }
            StopLinePlacement::AtStation { station } => *station,
        };
        if !station.is_finite() {
            return Err(EditError::invalid("station", "must be finite"));
        }
        let clamped = station.clamp(0.0, len);
        if (clamped - station).abs() > 1e-9 {
            cs.warn(
                Issue::warning(
                    codes::PLACEMENT_CLAMPED,
                    format!(
                        "stop line station {station:.3} m is outside {reference} (length \
                         {len:.3} m); clamped to {clamped:.3} m"
                    ),
                )
                .with_entity(reference),
            );
        }
        self.cross_section(reference, clamped, lanes)
    }

    /// Adds a stop line across one or more lanes, together with the rule
    /// that makes vehicles stop there (a stop sign by default).
    pub fn add_stop_line(&mut self, spec: NewStopLine) -> EditResult<(StopLineId, ChangeSet)> {
        self.check_lane_list(&spec.lanes)?;
        let mut cs = ChangeSet::new();
        let geometry = self.place_stop_line(&spec.lanes, &spec.placement, &mut cs)?;

        let id = StopLineId(self.alloc_raw());
        self.stop_lines.insert(
            id,
            StopLine {
                id,
                geometry,
                attributes: Default::default(),
            },
        );
        cs.created(id);
        let rule = match spec.rule {
            StopRule::StopSign => Some(Rule::TrafficSign {
                sign_type: "stop_sign".into(),
                sign: None,
                stop_line: Some(id),
            }),
            StopRule::Marking => Some(Rule::StopLine { stop_line: id }),
            StopRule::None => {
                cs.warn(
                    Issue::info(
                        codes::ORPHAN_RULE,
                        format!("{id} is not referenced by any rule yet"),
                    )
                    .with_entity(id),
                );
                None
            }
        };
        if let Some(rule) = rule {
            let re = self.insert_rule(rule, spec.lanes.clone());
            cs.created(re);
        }
        Ok((id, cs.finish()))
    }

    pub(crate) fn insert_rule(&mut self, rule: Rule, lanes: Vec<LaneId>) -> RegulatoryElementId {
        let id = RegulatoryElementId(self.alloc_raw());
        self.regulatory_elements.insert(
            id,
            RegulatoryElement {
                id,
                rule,
                lanes,
                attributes: Default::default(),
            },
        );
        id
    }

    /// Adds a traffic signal controlling `lanes`.
    ///
    /// Creates (or extends, with `group`) a traffic-light rule, picks or
    /// creates the stop line, and synthesizes geometry and lamps when they
    /// are not given. Unconditional stop rules (stop signs, stop line
    /// markings) on the same stop line are replaced for the controlled lanes.
    pub fn add_traffic_signal(
        &mut self,
        spec: NewTrafficSignal,
    ) -> EditResult<(SignalId, ChangeSet)> {
        // ---- validation ----
        self.check_lane_list(&spec.lanes)?;
        if let Some(h) = spec.height
            && !(h.is_finite() && h > 0.0)
        {
            return Err(EditError::invalid("height", "must be positive"));
        }
        if let Some(g) = &spec.geometry {
            check_polyline("geometry", g)?;
        }
        let group_stop_line = match spec.group {
            Some(g) => match self.regulatory_elements.get(&g).map(|r| &r.rule) {
                Some(Rule::TrafficLight { stop_line, .. }) => Some(*stop_line),
                Some(_) => {
                    return Err(EditError::invalid(
                        "group",
                        format!("{g} is not a traffic light"),
                    ));
                }
                None => return Err(EditError::not_found(g)),
            },
            None => None,
        };
        if let StopLineChoice::Existing(s) = spec.stop_line
            && !self.stop_lines.contains_key(&s)
        {
            return Err(EditError::not_found(s));
        }
        if let (Some(Some(gs)), StopLineChoice::Existing(s)) = (group_stop_line, &spec.stop_line)
            && gs != *s
        {
            return Err(EditError::invalid(
                "stop_line",
                format!("the signal group already uses {gs}"),
            ));
        }
        let mut cs = ChangeSet::new();
        enum Plan {
            Use(Option<StopLineId>),
            Create(Polyline3),
        }
        let plan = match &spec.stop_line {
            StopLineChoice::Existing(s) => Plan::Use(Some(*s)),
            StopLineChoice::None => Plan::Use(None),
            StopLineChoice::New(p) => {
                Plan::Create(self.place_stop_line(&spec.lanes, p, &mut cs)?)
            }
            StopLineChoice::Auto => match group_stop_line.flatten() {
                Some(s) => Plan::Use(Some(s)),
                None => match self.existing_stop_line(spec.lanes[0]) {
                    Some(s) => Plan::Use(Some(s)),
                    None => Plan::Create(self.place_stop_line(
                        &spec.lanes,
                        &StopLinePlacement::default(),
                        &mut cs,
                    )?),
                },
            },
        };
        // Geometry anchor for synthesized signals.
        let anchor = match &plan {
            Plan::Use(Some(s)) => self.stop_lines[s].geometry.clone(),
            Plan::Create(g) => g.clone(),
            Plan::Use(None) => {
                let len = self.lane_length(spec.lanes[0]).unwrap_or(0.0);
                self.cross_section(spec.lanes[0], len, &spec.lanes)?
            }
        };
        let height = spec.height.unwrap_or(DEFAULT_SIGNAL_HEIGHT);
        let geometry = match &spec.geometry {
            Some(g) => g.clone(),
            None => {
                let g = self.synthesize_signal_geometry(spec.lanes[0], &anchor)?;
                cs.warn(Issue::warning(
                    codes::GEOMETRY_SYNTHESIZED,
                    "signal geometry was synthesized above the stop line; replace it with the \
                     surveyed position",
                ));
                g
            }
        };

        // ---- mutation ----
        let stop_line = match plan {
            Plan::Use(s) => s,
            Plan::Create(g) => {
                let id = StopLineId(self.alloc_raw());
                self.stop_lines.insert(
                    id,
                    StopLine {
                        id,
                        geometry: g,
                        attributes: Default::default(),
                    },
                );
                cs.created(id);
                Some(id)
            }
        };
        let id = SignalId(self.alloc_raw());
        let bulbs = spec
            .bulbs
            .clone()
            .unwrap_or_else(|| default_bulbs(spec.kind, &geometry, height));
        self.traffic_signals.insert(
            id,
            TrafficSignal {
                id,
                kind: spec.kind,
                geometry,
                height: Some(height),
                bulbs,
                attributes: Default::default(),
            },
        );
        cs.created(id);

        match spec.group {
            Some(g) => {
                let re = self.regulatory_elements.get_mut(&g).expect("checked");
                if let Rule::TrafficLight {
                    signals,
                    stop_line: sl,
                } = &mut re.rule
                {
                    signals.push(id);
                    if sl.is_none() {
                        *sl = stop_line;
                    }
                }
                for l in &spec.lanes {
                    if !re.lanes.contains(l) {
                        re.lanes.push(*l);
                    }
                }
                cs.modified(g);
            }
            None => {
                let re = self.insert_rule(
                    Rule::TrafficLight {
                        signals: vec![id],
                        stop_line,
                    },
                    spec.lanes.clone(),
                );
                cs.created(re);
            }
        }

        // Replace unconditional stop rules on the same stop line.
        if let Some(s) = stop_line {
            let candidates: Vec<RegulatoryElementId> = self
                .regulatory_elements
                .values()
                .filter(|re| match &re.rule {
                    Rule::StopLine { stop_line } => *stop_line == s,
                    Rule::TrafficSign {
                        sign_type,
                        stop_line,
                        ..
                    } => sign_type == "stop_sign" && *stop_line == Some(s),
                    _ => false,
                })
                .map(|re| re.id)
                .collect();
            for re_id in candidates {
                let re = self.regulatory_elements.get_mut(&re_id).expect("exists");
                let before = re.lanes.len();
                re.lanes.retain(|l| !spec.lanes.contains(l));
                if re.lanes.len() == before {
                    continue;
                }
                cs.warn(
                    Issue::warning(
                        codes::RULE_REPLACED,
                        format!("stop rule {re_id} on {s} was replaced by the traffic light"),
                    )
                    .with_entity(re_id),
                );
                if re.lanes.is_empty() {
                    self.regulatory_elements.remove(&re_id);
                    cs.deleted(re_id);
                } else {
                    cs.modified(re_id);
                }
            }
        }
        Ok((id, cs.finish()))
    }

    /// Stop line of a stop / signal rule applying to `lane` (smallest rule ID).
    fn existing_stop_line(&self, lane: LaneId) -> Option<StopLineId> {
        self.rules_for_lane(lane)
            .into_iter()
            .find_map(|re| match &re.rule {
                Rule::TrafficLight { stop_line, .. } | Rule::TrafficSign { stop_line, .. } => {
                    *stop_line
                }
                Rule::StopLine { stop_line } => Some(*stop_line),
                _ => None,
            })
    }

    fn synthesize_signal_geometry(
        &self,
        lane: LaneId,
        anchor: &Polyline3,
    ) -> EditResult<Polyline3> {
        let mid = anchor
            .point_at_fraction(0.5)
            .ok_or_else(|| EditError::geometry("empty stop line"))?;
        let center = self
            .centerline(lane)
            .ok_or_else(|| EditError::geometry(format!("{lane} has no usable geometry")))?;
        let station = center.nearest_point(mid.xy()).map_or(0.0, |n| n.station);
        let dir = center
            .direction_at(station)
            .ok_or_else(|| EditError::geometry("degenerate centerline"))?;
        let left = dir.perp() * (DEFAULT_SIGNAL_WIDTH / 2.0);
        let z = mid.z + DEFAULT_SIGNAL_ELEVATION;
        Ok(Polyline3::new(vec![
            Point3::new(mid.x + left.x, mid.y + left.y, z),
            Point3::new(mid.x - left.x, mid.y - left.y, z),
        ]))
    }

    /// Adds a crosswalk, the crosswalk rule for the lanes crossing it and,
    /// optionally, stop lines in front of it.
    pub fn add_crosswalk(&mut self, spec: NewCrosswalk) -> EditResult<(CrosswalkId, ChangeSet)> {
        // ---- validation / geometry ----
        let (left_edge, right_edge) = match &spec.geometry {
            CrosswalkGeometry::Edges {
                left_edge,
                right_edge,
            } => {
                check_polyline("left_edge", left_edge)?;
                check_polyline("right_edge", right_edge)?;
                (left_edge.clone(), right_edge.clone())
            }
            CrosswalkGeometry::Across {
                lane,
                station,
                width,
                margin,
            } => {
                self.require_lane(*lane)?;
                if !(width.is_finite() && *width > 0.0) {
                    return Err(EditError::invalid("width", "must be positive"));
                }
                if !(margin.is_finite() && *margin >= 0.0) {
                    return Err(EditError::invalid("margin", "must be non-negative"));
                }
                let len = self.lane_length(*lane).unwrap_or(0.0);
                if !(*station >= 0.0 && *station <= len) {
                    return Err(EditError::invalid(
                        "station",
                        format!("must be within the lane length ({len:.3} m)"),
                    ));
                }
                let group = self.lateral_group(*lane);
                let section = self.cross_section(*lane, *station, &group)?;
                let center = self.centerline(*lane).expect("checked");
                let dir = center.direction_at(*station).expect("checked");
                let (l, r) = (section.points[0], section.points[1]);
                let walk = dir.perp(); // from the right side of the road to the left
                let lo = r.xy() - walk * *margin;
                let hi = l.xy() + walk * *margin;
                let half = dir * (width / 2.0);
                let z = center.point_at(*station).expect("checked").z;
                let edge = |offset: crate::geometry::Point2| {
                    Polyline3::new(vec![(lo + offset).with_z(z), (hi + offset).with_z(z)])
                };
                // The pedestrian's left is the upstream side (-dir).
                (edge(half * -1.0), edge(half))
            }
        };
        let crosswalk = Crosswalk {
            id: CrosswalkId(0),
            left_edge,
            right_edge,
            polygon: None,
            attributes: Default::default(),
        };
        let outline = crosswalk.outline().to_2d();
        if outline.area() <= 1e-6 {
            return Err(EditError::geometry("crosswalk has zero area"));
        }
        let crossing: Vec<LaneId> = match &spec.crossing_lanes {
            Some(lanes) => {
                self.check_lane_list(lanes)?;
                lanes.clone()
            }
            None => self
                .lanes
                .values()
                .filter(|l| l.kind.is_vehicle_lane())
                .filter(|l| {
                    self.centerline(l.id)
                        .is_some_and(|c| outline.intersects_polyline(&c))
                })
                .map(|l| l.id)
                .collect(),
        };
        if let Some(o) = spec.stop_line_offset
            && !(o.is_finite() && o >= 0.0)
        {
            return Err(EditError::invalid(
                "stop_line_offset",
                "must be non-negative",
            ));
        }
        let mut cs = ChangeSet::new();
        let mut stop_line_geometry = Vec::new();
        if let Some(offset) = spec.stop_line_offset {
            for &l in &crossing {
                let center = self.centerline(l).expect("lane exists");
                let Some(entry) = entry_station(&center, &outline) else {
                    continue;
                };
                let station = entry - offset;
                if station < 0.0 {
                    cs.warn(
                        Issue::warning(
                            codes::PLACEMENT_CLAMPED,
                            format!(
                                "stop line for {l} would start before the lane; placed at the \
                                 lane start"
                            ),
                        )
                        .with_entity(l),
                    );
                }
                stop_line_geometry.push(self.cross_section(l, station.max(0.0), &[l])?);
            }
        }

        // ---- mutation ----
        let id = CrosswalkId(self.alloc_raw());
        self.crosswalks.insert(id, Crosswalk { id, ..crosswalk });
        cs.created(id);
        let mut stop_lines = Vec::new();
        for geometry in stop_line_geometry {
            let sl = StopLineId(self.alloc_raw());
            self.stop_lines.insert(
                sl,
                StopLine {
                    id: sl,
                    geometry,
                    attributes: Default::default(),
                },
            );
            cs.created(sl);
            stop_lines.push(sl);
        }
        if crossing.is_empty() {
            cs.warn(
                Issue::warning(
                    codes::ORPHAN_RULE,
                    format!("no lane crosses {id}; no crosswalk rule was created"),
                )
                .with_entity(id),
            );
        } else {
            let re = self.insert_rule(
                Rule::Crosswalk {
                    crosswalk: id,
                    stop_lines,
                },
                crossing,
            );
            cs.created(re);
        }
        Ok((id, cs.finish()))
    }
}

/// Station where `center` first enters `polygon`.
fn entry_station(center: &Polyline3, polygon: &Polygon2) -> Option<f64> {
    let first = center.first()?;
    if polygon.contains(first.xy()) {
        return Some(0.0);
    }
    let stations = center.stations();
    let mut best: Option<f64> = None;
    for (i, w) in center.points.windows(2).enumerate() {
        let seg = w[0].distance(w[1]);
        for (a, b) in polygon.edges() {
            if let Some((t, _)) = segment_intersection(w[0].xy(), w[1].xy(), a, b) {
                let s = stations[i] + t * seg;
                if best.is_none_or(|v| s < v) {
                    best = Some(s);
                }
            }
        }
        if best.is_some() {
            break;
        }
    }
    best
}

fn default_bulbs(kind: SignalKind, geometry: &Polyline3, height: f64) -> Vec<SignalBulb> {
    let colors: &[BulbColor] = match kind {
        SignalKind::Vehicle => &[BulbColor::Green, BulbColor::Yellow, BulbColor::Red],
        SignalKind::Pedestrian => &[BulbColor::Green, BulbColor::Red],
    };
    let n = colors.len() as f64;
    colors
        .iter()
        .enumerate()
        .filter_map(|(i, &color)| {
            let p = geometry.point_at_fraction((i as f64 + 0.5) / n)?;
            Some(SignalBulb {
                position: Point3::new(p.x, p.y, p.z + height / 2.0),
                color,
                arrow: None,
            })
        })
        .collect()
}
