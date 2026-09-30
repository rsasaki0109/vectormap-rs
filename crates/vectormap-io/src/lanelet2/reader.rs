//! Lanelet2 OSM → IR.

use std::collections::{BTreeMap, BTreeSet};

use vectormap_core::{
    Attributes, Boundary, BoundaryId, BoundaryRef, Crosswalk, CrosswalkId, GeoPoint, GeoReference,
    Issue, Junction, JunctionId, Lane, LaneId, Map, Point3, Polygon3, Polyline3, ProjectionKind,
    RegulatoryElement, RegulatoryElementId, Rule, SignalBulb, SignalId, StopLine, StopLineId,
    TrafficSignal, infer_topology,
};

use super::osm::{self, MemberType, OsmData, OsmRelation, OsmWay, Tags};
use super::tags::{
    boundary_kind_from_tags, boundary_tags, lane_kind_from_subtype, lane_subtype, parse_arrow,
    parse_color, parse_speed, parse_turn, signal_kind_from_subtype, signal_subtype,
};
use super::{ATTRIBUTE_PREFIX, LoadOptions, ProjectionChoice, codes};
use crate::projection::{LocalProjector, recover_georeference};
use crate::{IoError, Loaded};

/// Parses Lanelet2 OSM XML into a map.
pub fn read_str(text: &str, options: &LoadOptions) -> Result<Loaded, IoError> {
    let (osm, issues) = osm::parse(text)?;
    let mut reader = Reader::new(&osm, issues);
    reader.project(options)?;
    reader.read_lanelets();
    reader.read_regulatory_elements();
    reader.read_remaining_ways();
    reader.read_junctions();
    reader.report_unsupported_relations();
    let (topology, topo_issues) = infer_topology(&reader.map, options.infer);
    reader.map.set_topology(topology);
    reader.issues.extend(topo_issues);
    Ok(Loaded {
        map: reader.map,
        issues: reader.issues,
    })
}

struct Reader<'a> {
    osm: &'a OsmData,
    points: BTreeMap<i64, Point3>,
    map: Map,
    issues: Vec<Issue>,
    /// OSM IDs (ways and relations) → IR raw IDs.
    ids: BTreeMap<i64, u64>,
    consumed_ways: BTreeSet<i64>,
    consumed_relations: BTreeSet<i64>,
    /// Regulatory element relation → lanes referencing it.
    rule_lanes: BTreeMap<i64, Vec<LaneId>>,
    /// Lanelet relation → crosswalk created from it.
    crosswalks: BTreeMap<i64, CrosswalkId>,
    /// Lanelet relation → lane created from it.
    lanes: BTreeMap<i64, LaneId>,
    /// intersection_area way → lanes tagged with it.
    junction_lanes: BTreeMap<i64, Vec<LaneId>>,
}

fn prefixed(tags: &Tags, skip: &[&str]) -> Attributes {
    tags.iter()
        .filter(|(k, _)| !skip.contains(&k.as_str()))
        .map(|(k, v)| (format!("{ATTRIBUTE_PREFIX}:{k}"), v.clone()))
        .collect()
}

fn set_attr(attrs: &mut Attributes, key: &str, value: &str) {
    attrs.insert(format!("{ATTRIBUTE_PREFIX}:{key}"), value);
}

/// Middle point as used by Lanelet2's `geometry::align`.
fn middle_point(ls: &Polyline3) -> Point3 {
    let n = ls.len();
    if n > 2 {
        ls.points[n / 2]
    } else {
        ls.points[0].lerp(ls.points[n - 1], 0.5)
    }
}

fn signed_distance(ls: &Polyline3, p: Point3) -> f64 {
    ls.nearest_point(p.xy()).map_or(0.0, |n| n.lateral)
}

/// Replicates Lanelet2's `geometry::align(left, right)`: returns whether the
/// left and the right bound must be inverted.
fn align(left: &Polyline3, right: &Polyline3) -> (bool, bool) {
    let right_of_left = signed_distance(left, middle_point(right)) < 0.0;
    let left_rev = !right_of_left && left.len() > 1;
    let left_oriented = if left_rev {
        left.reversed()
    } else {
        left.clone()
    };
    let left_of_right = signed_distance(right, middle_point(&left_oriented)) > 0.0;
    let right_rev = !left_of_right && right.len() > 1;
    (left_rev, right_rev)
}

impl<'a> Reader<'a> {
    fn new(osm: &'a OsmData, issues: Vec<Issue>) -> Self {
        // Positive IDs are kept; negative (JOSM "new") IDs are mapped above
        // the largest positive way / relation ID, in descending order.
        let max_pos = osm
            .ways
            .keys()
            .chain(osm.relations.keys())
            .copied()
            .filter(|id| *id > 0)
            .max()
            .unwrap_or(0) as u64;
        let mut negatives: Vec<i64> = osm
            .ways
            .keys()
            .chain(osm.relations.keys())
            .copied()
            .filter(|id| *id <= 0)
            .collect();
        negatives.sort_by(|a, b| b.cmp(a));
        negatives.dedup();
        let mut ids = BTreeMap::new();
        for (i, id) in negatives.into_iter().enumerate() {
            ids.insert(id, max_pos + 1 + i as u64);
        }
        Self {
            osm,
            points: BTreeMap::new(),
            map: Map::new(),
            issues,
            ids,
            consumed_ways: BTreeSet::new(),
            consumed_relations: BTreeSet::new(),
            rule_lanes: BTreeMap::new(),
            crosswalks: BTreeMap::new(),
            lanes: BTreeMap::new(),
            junction_lanes: BTreeMap::new(),
        }
    }

    fn ir_id(&self, osm_id: i64) -> u64 {
        self.ids.get(&osm_id).copied().unwrap_or(osm_id as u64)
    }

    // ---------------------------------------------------------------------
    // Coordinates
    // ---------------------------------------------------------------------

    fn project(&mut self, options: &LoadOptions) -> Result<(), IoError> {
        let nodes = &self.osm.nodes;
        let local = |n: &osm::OsmNode| -> Option<(f64, f64)> {
            Some((
                n.tags.get("local_x")?.trim().parse().ok()?,
                n.tags.get("local_y")?.trim().parse().ok()?,
            ))
        };
        let ele = |n: &osm::OsmNode| -> f64 {
            n.tags
                .get("ele")
                .and_then(|v| v.trim().parse().ok())
                .unwrap_or(0.0)
        };
        let all_local = !nodes.is_empty() && nodes.values().all(|n| local(n).is_some());
        let latlon_degenerate = nodes.values().all(|n| {
            nodes
                .values()
                .next()
                .is_some_and(|f| f.lat == n.lat && f.lon == n.lon)
        });
        // Where coordinates come from, and the georeference of the result.
        let (use_local_tags, georef) = match options.projection {
            ProjectionChoice::LocalTags => (true, None),
            ProjectionChoice::Georeferenced(g) => (false, Some(g)),
            ProjectionChoice::Auto if all_local && latlon_degenerate => {
                self.issues.push(Issue::info(
                    codes::PROJECTION,
                    "coordinates taken from local_x/local_y (lat/lon are placeholders); the map \
                     has no georeference",
                ));
                (true, None)
            }
            ProjectionChoice::Auto => {
                let pairs: Vec<(f64, f64, f64, f64)> = if all_local {
                    nodes
                        .values()
                        .filter_map(|n| local(n).map(|(x, y)| (n.lat, n.lon, x, y)))
                        .collect()
                } else {
                    Vec::new()
                };
                match recover_georeference(&pairs, 0.05) {
                    Some(g) => {
                        self.issues.push(Issue::info(
                            codes::PROJECTION,
                            format!(
                                "coordinates taken from local_x/local_y; they are consistent with \
                                 a {:?} projection with origin ({:.9}, {:.9})",
                                g.projection, g.origin.lat, g.origin.lon
                            ),
                        ));
                        (true, Some(g))
                    }
                    None => {
                        let lat = nodes.values().map(|n| n.lat).fold(f64::INFINITY, f64::min);
                        let lon = nodes.values().map(|n| n.lon).fold(f64::INFINITY, f64::min);
                        let origin = if nodes.is_empty() {
                            GeoPoint::new(0.0, 0.0)
                        } else {
                            GeoPoint::new(lat, lon)
                        };
                        if all_local {
                            self.issues.push(Issue::info(
                                codes::PROJECTION,
                                "local_x/local_y tags do not match lat/lon under a UTM or \
                                 transverse Mercator projection and were ignored; load with the \
                                 LocalTags projection to use them",
                            ));
                        }
                        self.issues.push(Issue::info(
                            codes::PROJECTION,
                            format!(
                                "lat/lon projected with UTM relative to the south-west corner of \
                                 the data ({:.9}, {:.9})",
                                origin.lat, origin.lon
                            ),
                        ));
                        (
                            false,
                            Some(GeoReference {
                                projection: ProjectionKind::Utm,
                                origin,
                            }),
                        )
                    }
                }
            }
        };
        if use_local_tags {
            for n in nodes.values() {
                let (x, y) = local(n).ok_or_else(|| {
                    IoError::Format(format!("node {} has no local_x/local_y tags", n.id))
                })?;
                self.points.insert(n.id, Point3::new(x, y, ele(n)));
            }
        } else {
            let projector = LocalProjector::new(georef.expect("lat/lon need a georeference"));
            for n in nodes.values() {
                let p = projector.forward(GeoPoint {
                    lat: n.lat,
                    lon: n.lon,
                    alt: ele(n),
                });
                self.points.insert(n.id, p);
            }
        }
        self.map.metadata_mut().georeference = georef;
        Ok(())
    }

    // ---------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------

    fn way(&mut self, id: i64, context: &str) -> Option<&'a OsmWay> {
        let way = self.osm.ways.get(&id);
        if way.is_none() {
            self.issues.push(Issue::error(
                codes::MISSING_MEMBER,
                format!("{context} references way {id}, which does not exist"),
            ));
        }
        way
    }

    fn polyline(&mut self, way: &OsmWay) -> Polyline3 {
        let mut pts = Vec::with_capacity(way.nodes.len());
        for n in &way.nodes {
            match self.points.get(n) {
                Some(p) => pts.push(*p),
                None => self.issues.push(Issue::error(
                    codes::MISSING_NODE,
                    format!("way {} references node {n}, which does not exist", way.id),
                )),
            }
        }
        Polyline3::new(pts)
    }

    fn polygon(&mut self, way: &OsmWay) -> Polygon3 {
        Polygon3::new(self.polyline(way).points)
    }

    fn single_way(&mut self, rel: &OsmRelation, role: &str) -> Option<&'a OsmWay> {
        let ids: Vec<i64> = rel.members(MemberType::Way, role).collect();
        match ids.as_slice() {
            [id] => self.way(*id, &format!("relation {}", rel.id)),
            [] => None,
            [id, ..] => {
                self.issues.push(Issue::warning(
                    codes::UNSUPPORTED_MEMBER,
                    format!(
                        "relation {} has {} `{role}` members; only way {id} is used",
                        rel.id,
                        ids.len()
                    ),
                ));
                self.way(*id, &format!("relation {}", rel.id))
            }
        }
    }

    fn ensure_boundary(&mut self, way: &OsmWay) -> Option<BoundaryId> {
        let id = BoundaryId(self.ir_id(way.id));
        if self.map.boundary(id).is_some() {
            return Some(id);
        }
        let geometry = self.polyline(way);
        let ty = way.tags.get("type").map(String::as_str);
        let subtype = way.tags.get("subtype").map(String::as_str);
        let kind = boundary_kind_from_tags(ty, subtype);
        let mut attributes = prefixed(&way.tags, &["type", "subtype"]);
        let (canonical_type, canonical_subtype) = boundary_tags(kind);
        if let Some(t) = ty
            && t != canonical_type
        {
            set_attr(&mut attributes, "type", t);
        }
        if let Some(s) = subtype
            && Some(s) != canonical_subtype
        {
            set_attr(&mut attributes, "subtype", s);
        }
        let mut boundary = Boundary::new(id, kind, geometry);
        boundary.attributes = attributes;
        self.map.insert_boundary(boundary).ok()?;
        self.consumed_ways.insert(way.id);
        Some(id)
    }

    fn ensure_stop_line(&mut self, way: &OsmWay) -> Option<StopLineId> {
        let id = StopLineId(self.ir_id(way.id));
        if self.map.stop_line(id).is_some() {
            return Some(id);
        }
        let geometry = self.polyline(way);
        let mut attributes = prefixed(&way.tags, &["type"]);
        if let Some(t) = way.tags.get("type")
            && t != "stop_line"
        {
            set_attr(&mut attributes, "type", t);
        }
        self.map
            .insert_stop_line(StopLine {
                id,
                geometry,
                attributes,
            })
            .ok()?;
        self.consumed_ways.insert(way.id);
        Some(id)
    }

    fn ensure_signal(&mut self, way: &OsmWay) -> Option<SignalId> {
        let id = SignalId(self.ir_id(way.id));
        if self.map.traffic_signal(id).is_some() {
            return Some(id);
        }
        let geometry = self.polyline(way);
        let subtype = way.tags.get("subtype").map(String::as_str);
        let kind = signal_kind_from_subtype(subtype);
        let mut attributes = prefixed(&way.tags, &["type", "subtype", "height"]);
        if let Some(s) = subtype
            && s != signal_subtype(kind)
        {
            set_attr(&mut attributes, "subtype", s);
        }
        let height = match way.tags.get("height") {
            Some(h) => match h.trim().parse::<f64>() {
                Ok(v) if v.is_finite() && v > 0.0 => Some(v),
                _ => {
                    self.issues.push(Issue::warning(
                        codes::INVALID_TAG,
                        format!("traffic light way {} has an invalid height {h:?}", way.id),
                    ));
                    set_attr(&mut attributes, "height", h);
                    None
                }
            },
            None => None,
        };
        self.map
            .insert_traffic_signal(TrafficSignal {
                id,
                kind,
                geometry,
                height,
                bulbs: Vec::new(),
                attributes,
            })
            .ok()?;
        self.consumed_ways.insert(way.id);
        Some(id)
    }

    // ---------------------------------------------------------------------
    // Lanelets
    // ---------------------------------------------------------------------

    fn read_lanelets(&mut self) {
        let lanelets: Vec<&OsmRelation> = self
            .osm
            .relations
            .values()
            .filter(|r| r.tags.get("type").map(String::as_str) == Some("lanelet"))
            .collect();
        for rel in lanelets {
            self.consumed_relations.insert(rel.id);
            let left = self.single_way(rel, "left");
            let right = self.single_way(rel, "right");
            let (Some(left), Some(right)) = (left, right) else {
                self.issues.push(Issue::error(
                    codes::INVALID_PRIMITIVE,
                    format!(
                        "lanelet {} needs exactly one left and one right bound; skipped",
                        rel.id
                    ),
                ));
                continue;
            };
            let (lg, rg) = (self.polyline(left), self.polyline(right));
            if lg.is_empty() || rg.is_empty() {
                self.issues.push(Issue::error(
                    codes::INVALID_PRIMITIVE,
                    format!("lanelet {} has an empty bound; skipped", rel.id),
                ));
                continue;
            }
            let (left_rev, right_rev) = align(&lg, &rg);
            if rel.tags.get("subtype").map(String::as_str) == Some("crosswalk") {
                self.read_crosswalk(rel, (&lg, left_rev), (&rg, right_rev), left, right);
            } else {
                self.read_lane(rel, left, right, left_rev, right_rev, &lg);
            }
        }
    }

    fn read_crosswalk(
        &mut self,
        rel: &OsmRelation,
        (lg, left_rev): (&Polyline3, bool),
        (rg, right_rev): (&Polyline3, bool),
        left: &OsmWay,
        right: &OsmWay,
    ) {
        let id = CrosswalkId(self.ir_id(rel.id));
        let orient = |g: &Polyline3, rev: bool| if rev { g.reversed() } else { g.clone() };
        let mut attributes = prefixed(&rel.tags, &["type", "subtype"]);
        // Canonical crosswalk tags written back by the exporter.
        for (k, v) in [("one_way", "no"), ("participant:pedestrian", "yes")] {
            if rel.tags.get(k).map(String::as_str) == Some(v) {
                attributes.remove(&format!("{ATTRIBUTE_PREFIX}:{k}"));
            }
        }
        for (role, way) in [("left", left), ("right", right)] {
            if way.tags.get("type").map(String::as_str) != Some("pedestrian_marking")
                && let Some(t) = way.tags.get("type")
            {
                set_attr(&mut attributes, &format!("{role}_edge_type"), t);
            }
            self.consumed_ways.insert(way.id);
        }
        let crosswalk = Crosswalk {
            id,
            left_edge: orient(lg, left_rev),
            right_edge: orient(rg, right_rev),
            polygon: None,
            attributes,
        };
        if self.map.insert_crosswalk(crosswalk).is_ok() {
            self.crosswalks.insert(rel.id, id);
        }
        for re in rel.members(MemberType::Relation, "regulatory_element") {
            self.issues.push(Issue::warning(
                codes::UNSUPPORTED_MEMBER,
                format!(
                    "crosswalk lanelet {} references regulatory element {re}; rules applying to \
                     crosswalks are not represented",
                    rel.id
                ),
            ));
        }
    }

    fn read_lane(
        &mut self,
        rel: &OsmRelation,
        left: &OsmWay,
        right: &OsmWay,
        left_rev: bool,
        right_rev: bool,
        lg: &Polyline3,
    ) {
        let (Some(lb), Some(rb)) = (self.ensure_boundary(left), self.ensure_boundary(right)) else {
            return;
        };
        let id = LaneId(self.ir_id(rel.id));
        let mut lane = Lane::new(
            id,
            BoundaryRef {
                boundary: lb,
                reversed: left_rev,
            },
            BoundaryRef {
                boundary: rb,
                reversed: right_rev,
            },
        );
        let tags = &rel.tags;
        let mut attributes = prefixed(
            tags,
            &[
                "type",
                "subtype",
                "speed_limit",
                "one_way",
                "turn_direction",
                "intersection_area",
            ],
        );
        if let Some(st) = tags.get("subtype") {
            lane.kind = lane_kind_from_subtype(st);
            if st != lane_subtype(lane.kind) {
                set_attr(&mut attributes, "subtype", st);
            }
        }
        if let Some(v) = tags.get("speed_limit") {
            match parse_speed(v) {
                Some(s) => lane.speed_limit = Some(s),
                None => {
                    self.issues.push(
                        Issue::warning(
                            codes::INVALID_TAG,
                            format!("lanelet {} has an unparsable speed_limit {v:?}", rel.id),
                        )
                        .with_entity(id),
                    );
                    set_attr(&mut attributes, "speed_limit", v);
                }
            }
        }
        match tags.get("one_way").map(String::as_str) {
            None | Some("yes") => {}
            Some("no") => lane.one_way = false,
            Some(other) => set_attr(&mut attributes, "one_way", other),
        }
        if let Some(v) = tags.get("turn_direction") {
            match parse_turn(v) {
                Some(t) => lane.turn_direction = Some(t),
                None => set_attr(&mut attributes, "turn_direction", v),
            }
        }
        if let Some(v) = tags.get("intersection_area") {
            match v.trim().parse::<i64>() {
                Ok(area) => self.junction_lanes.entry(area).or_default().push(id),
                Err(_) => set_attr(&mut attributes, "intersection_area", v),
            }
        }
        lane.attributes = attributes;
        if let Some(c) = self.single_way(rel, "centerline") {
            let mut center = self.polyline(c);
            let oriented_left = if left_rev { lg.reversed() } else { lg.clone() };
            if let (Some(c0), Some(l0), Some(l1)) =
                (center.first(), oriented_left.first(), oriented_left.last())
                && c0.distance_2d(l1) < c0.distance_2d(l0)
            {
                center = center.reversed();
            }
            lane.centerline = Some(center);
            self.consumed_ways.insert(c.id);
        }
        for re in rel.members(MemberType::Relation, "regulatory_element") {
            self.rule_lanes.entry(re).or_default().push(id);
        }
        if self.map.insert_lane(lane).is_ok() {
            self.lanes.insert(rel.id, id);
        }
    }

    // ---------------------------------------------------------------------
    // Regulatory elements
    // ---------------------------------------------------------------------

    fn read_regulatory_elements(&mut self) {
        let rels: Vec<&OsmRelation> = self
            .osm
            .relations
            .values()
            .filter(|r| r.tags.get("type").map(String::as_str) == Some("regulatory_element"))
            .collect();
        for rel in rels {
            self.consumed_relations.insert(rel.id);
            let subtype = rel.tags.get("subtype").map(String::as_str).unwrap_or("");
            let (rule, used_roles): (Option<Rule>, &[&str]) = match subtype {
                "traffic_light" => (
                    self.traffic_light(rel),
                    &["refers", "ref_line", "light_bulbs"],
                ),
                "traffic_sign" => (self.traffic_sign(rel), &["refers", "ref_line"]),
                "road_marking" => (self.road_marking(rel), &["refers"]),
                "right_of_way" => (
                    self.right_of_way(rel),
                    &["right_of_way", "yield", "ref_line"],
                ),
                "crosswalk" => (
                    self.crosswalk_rule(rel),
                    &["refers", "ref_line", "crosswalk_polygon"],
                ),
                other => (
                    Some(Rule::Other {
                        kind: if other.is_empty() {
                            "unknown".into()
                        } else {
                            other.into()
                        },
                    }),
                    &[],
                ),
            };
            let Some(rule) = rule else {
                self.issues.push(Issue::error(
                    codes::INVALID_PRIMITIVE,
                    format!(
                        "regulatory element {} ({subtype}) is incomplete; skipped",
                        rel.id
                    ),
                ));
                continue;
            };
            let unused: BTreeSet<&str> = rel
                .members
                .iter()
                .map(|m| m.role.as_str())
                .filter(|r| !used_roles.contains(r))
                .collect();
            if !unused.is_empty() {
                self.issues.push(Issue::warning(
                    codes::UNSUPPORTED_MEMBER,
                    format!(
                        "regulatory element {} ({subtype}): members with roles {:?} are not \
                         represented and were dropped",
                        rel.id, unused
                    ),
                ));
            }
            let id = RegulatoryElementId(self.ir_id(rel.id));
            let mut lanes = self.rule_lanes.get(&rel.id).cloned().unwrap_or_default();
            lanes.sort();
            lanes.dedup();
            let skip: &[&str] = if matches!(rule, Rule::Other { .. }) {
                &["type"]
            } else {
                &["type", "subtype"]
            };
            let re = RegulatoryElement {
                id,
                rule,
                lanes,
                attributes: prefixed(&rel.tags, skip),
            };
            let _ = self.map.insert_regulatory_element(re);
        }
    }

    fn ways_with_role(&mut self, rel: &OsmRelation, role: &str) -> Vec<&'a OsmWay> {
        let ids: Vec<i64> = rel.members(MemberType::Way, role).collect();
        ids.into_iter()
            .filter_map(|id| self.way(id, &format!("regulatory element {}", rel.id)))
            .collect()
    }

    fn stop_line_member(&mut self, rel: &OsmRelation) -> Option<StopLineId> {
        let ways = self.ways_with_role(rel, "ref_line");
        if ways.len() > 1 {
            self.issues.push(Issue::warning(
                codes::UNSUPPORTED_MEMBER,
                format!(
                    "regulatory element {} has several ref_lines; only the first is used",
                    rel.id
                ),
            ));
        }
        ways.first().and_then(|w| self.ensure_stop_line(w))
    }

    fn traffic_light(&mut self, rel: &OsmRelation) -> Option<Rule> {
        let refers = self.ways_with_role(rel, "refers");
        let mut signals = Vec::new();
        for w in &refers {
            if let Some(s) = self.ensure_signal(w) {
                signals.push(s);
            }
        }
        if signals.is_empty() {
            return None;
        }
        let stop_line = self.stop_line_member(rel);
        for bulbs in self.ways_with_role(rel, "light_bulbs") {
            let target = bulbs
                .tags
                .get("traffic_light_id")
                .and_then(|v| v.trim().parse::<i64>().ok())
                .map(|w| SignalId(self.ir_id(w)))
                .filter(|s| signals.contains(s))
                .unwrap_or(signals[0]);
            let mut new_bulbs = Vec::new();
            for n in &bulbs.nodes {
                let (Some(node), Some(p)) = (self.osm.nodes.get(n), self.points.get(n)) else {
                    continue;
                };
                let Some(color) = node.tags.get("color").and_then(|c| parse_color(c)) else {
                    self.issues.push(Issue::warning(
                        codes::INVALID_TAG,
                        format!("light bulb node {n} has no valid color; skipped"),
                    ));
                    continue;
                };
                new_bulbs.push(SignalBulb {
                    position: *p,
                    color,
                    arrow: node.tags.get("arrow").and_then(|a| parse_arrow(a)),
                });
            }
            if let Some(sig) = self.map.traffic_signal_mut(target) {
                sig.bulbs.extend(new_bulbs);
            }
            self.consumed_ways.insert(bulbs.id);
        }
        Some(Rule::TrafficLight { signals, stop_line })
    }

    fn traffic_sign(&mut self, rel: &OsmRelation) -> Option<Rule> {
        let refers = self.ways_with_role(rel, "refers");
        let sign_way = refers.first().copied();
        if refers.len() > 1 {
            self.issues.push(Issue::warning(
                codes::UNSUPPORTED_MEMBER,
                format!(
                    "traffic sign {} refers to several signs; only the first is used",
                    rel.id
                ),
            ));
        }
        let sign_type = sign_way
            .and_then(|w| w.tags.get("subtype"))
            .or_else(|| rel.tags.get("sign_type"))
            .cloned()?;
        let sign = sign_way.map(|w| {
            self.consumed_ways.insert(w.id);
            self.polyline(w)
        });
        let stop_line = self.stop_line_member(rel);
        Some(Rule::TrafficSign {
            sign_type,
            sign,
            stop_line,
        })
    }

    fn road_marking(&mut self, rel: &OsmRelation) -> Option<Rule> {
        let refers = self.ways_with_role(rel, "refers");
        let way = refers
            .iter()
            .find(|w| w.tags.get("type").map(String::as_str) == Some("stop_line"))?;
        let stop_line = self.ensure_stop_line(way)?;
        Some(Rule::StopLine { stop_line })
    }

    fn lanes_of(&mut self, rel: &OsmRelation, role: &str) -> Vec<LaneId> {
        let mut out = Vec::new();
        for r in rel.members(MemberType::Relation, role) {
            match self.lanes.get(&r) {
                Some(l) => out.push(*l),
                None => self.issues.push(Issue::warning(
                    codes::MISSING_MEMBER,
                    format!(
                        "regulatory element {} references lanelet {r}, which was not loaded",
                        rel.id
                    ),
                )),
            }
        }
        out
    }

    fn right_of_way(&mut self, rel: &OsmRelation) -> Option<Rule> {
        let priority = self.lanes_of(rel, "right_of_way");
        let yielding = self.lanes_of(rel, "yield");
        let mut stop_lines = Vec::new();
        for w in self.ways_with_role(rel, "ref_line") {
            stop_lines.extend(self.ensure_stop_line(w));
        }
        Some(Rule::RightOfWay {
            priority,
            yielding,
            stop_lines,
        })
    }

    fn crosswalk_rule(&mut self, rel: &OsmRelation) -> Option<Rule> {
        let crosswalk = rel
            .members(MemberType::Relation, "refers")
            .find_map(|r| self.crosswalks.get(&r).copied())?;
        let mut stop_lines = Vec::new();
        for w in self.ways_with_role(rel, "ref_line") {
            stop_lines.extend(self.ensure_stop_line(w));
        }
        let polygons = self.ways_with_role(rel, "crosswalk_polygon");
        if let Some(w) = polygons.first() {
            let polygon = self.polygon(w);
            self.consumed_ways.insert(w.id);
            if let Some(c) = self.map.crosswalk_mut(crosswalk) {
                c.polygon = Some(polygon);
            }
        }
        Some(Rule::Crosswalk {
            crosswalk,
            stop_lines,
        })
    }

    // ---------------------------------------------------------------------
    // Remaining primitives
    // ---------------------------------------------------------------------

    fn read_remaining_ways(&mut self) {
        let mut skipped: BTreeMap<String, usize> = BTreeMap::new();
        let ways: Vec<&OsmWay> = self
            .osm
            .ways
            .values()
            .filter(|w| !self.consumed_ways.contains(&w.id))
            .collect();
        for way in ways {
            match way.tags.get("type").map(String::as_str) {
                Some("stop_line") => {
                    self.ensure_stop_line(way);
                }
                Some("traffic_light") => {
                    self.ensure_signal(way);
                }
                Some("intersection_area") => {}
                other => {
                    *skipped
                        .entry(other.unwrap_or("<untyped>").to_string())
                        .or_insert(0) += 1;
                }
            }
        }
        for (ty, count) in skipped {
            self.issues.push(Issue::warning(
                codes::UNSUPPORTED_WAY,
                format!("{count} way(s) of type {ty:?} are not used by any lanelet or supported rule and were skipped"),
            ));
        }
    }

    fn read_junctions(&mut self) {
        let areas: Vec<&OsmWay> = self
            .osm
            .ways
            .values()
            .filter(|w| w.tags.get("type").map(String::as_str) == Some("intersection_area"))
            .collect();
        let mut known = BTreeSet::new();
        for way in areas {
            known.insert(way.id);
            let outline = self.polygon(way);
            let mut lanes = self
                .junction_lanes
                .get(&way.id)
                .cloned()
                .unwrap_or_default();
            lanes.sort();
            let junction = Junction {
                id: JunctionId(self.ir_id(way.id)),
                name: way.tags.get("name").cloned(),
                lanes,
                outline: Some(outline),
                attributes: prefixed(&way.tags, &["type", "area", "name"]),
            };
            let _ = self.map.insert_junction(junction);
            self.consumed_ways.insert(way.id);
        }
        for (area, lanes) in &self.junction_lanes {
            if !known.contains(area) {
                self.issues.push(
                    Issue::warning(
                        codes::MISSING_MEMBER,
                        format!(
                            "lanelets reference intersection_area {area}, which does not exist"
                        ),
                    )
                    .with_entity(lanes[0]),
                );
            }
        }
    }

    fn report_unsupported_relations(&mut self) {
        let mut skipped: BTreeMap<String, usize> = BTreeMap::new();
        for rel in self.osm.relations.values() {
            if !self.consumed_relations.contains(&rel.id) {
                let ty = rel.tags.get("type").map_or("<untyped>", String::as_str);
                *skipped.entry(ty.to_string()).or_insert(0) += 1;
            }
        }
        for (ty, count) in skipped {
            self.issues.push(Issue::warning(
                codes::UNSUPPORTED_RELATION,
                format!("{count} relation(s) of type {ty:?} are not supported and were skipped"),
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn align_matches_lanelet2() {
        // Left bound drawn backwards, right bound forwards: travel is +x.
        let left = Polyline3::from_xy(&[[10.0, 1.0], [5.0, 1.0], [0.0, 1.0]]);
        let right = Polyline3::from_xy(&[[0.0, -1.0], [5.0, -1.0], [10.0, -1.0]]);
        assert_eq!(align(&left, &right), (true, false));
        // Both forwards.
        assert_eq!(align(&left.reversed(), &right), (false, false));
        // Both backwards: travel is -x, so the "left" line at y=1 would be
        // on the right; Lanelet2 inverts both and the lane runs +x.
        assert_eq!(align(&left, &right.reversed()), (true, true));
    }
}
