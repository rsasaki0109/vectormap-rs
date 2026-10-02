//! IR → Lanelet2 OSM.
//!
//! Output is deterministic: IDs are allocated in a fixed order (IR IDs are
//! reused when they are free), nodes are shared between all ways that pass
//! through the same position (so Lanelet2 can re-derive the topology), and
//! primitives and tags are written sorted.

use std::collections::{BTreeMap, BTreeSet};

use vectormap_core::{
    Attributes, Crosswalk, EntityRef, GeoPoint, Issue, Lane, Map, Point3, Polyline3,
    ProjectionKind, Rule, TrafficSignal,
};

use super::osm::{self, MemberType, OsmData, OsmMember, OsmNode, OsmRelation, OsmWay, Tags};
use super::tags::{
    arrow_str, boundary_kind_from_tags, boundary_tags, color_str, fmt_coord, fmt_decimal,
    lane_kind_from_subtype, lane_subtype, signal_kind_from_subtype, signal_subtype, turn_str,
};
use super::{ATTRIBUTE_PREFIX, FORMAT_PREFIXES, Profile, SaveOptions, codes};
use crate::autoware;
use crate::projection::LocalProjector;

/// Generator string written into the `<osm>` header.
pub const GENERATOR: &str = concat!("vectormap-rs ", env!("CARGO_PKG_VERSION"));

/// Serializes a map as Lanelet2 OSM XML. Returns the XML and export
/// warnings.
pub fn write_string(map: &Map, options: &SaveOptions) -> (String, Vec<Issue>) {
    let mut w = Writer::new(map, options);
    w.write_all();
    let issues = w.issues;
    (osm::write(&w.data, GENERATOR), issues)
}

/// A way waiting for node assignment.
struct PendingWay {
    id: i64,
    points: Vec<Point3>,
    tags: Tags,
    /// Per-point node tags (light bulbs); such nodes are never shared.
    node_tags: Option<Vec<Tags>>,
}

struct Writer<'a> {
    map: &'a Map,
    options: &'a SaveOptions,
    projector: Option<LocalProjector>,
    local_coordinates: bool,
    issues: Vec<Issue>,
    used: BTreeSet<i64>,
    next_free: i64,
    /// IR entity → OSM ID.
    osm_ids: BTreeMap<EntityRef, i64>,
    ways: Vec<PendingWay>,
    relations: Vec<OsmRelation>,
    /// Junction → intersection_area way.
    junction_ways: BTreeMap<vectormap_core::JunctionId, i64>,
    data: OsmData,
}

/// Tags from attributes: generic keys first, then `lanelet2:` keys (prefix
/// stripped). Typed tags inserted afterwards take precedence.
fn attribute_tags(attrs: &Attributes, skip: &[&str]) -> Tags {
    let mut tags = Tags::new();
    for (k, v) in attrs.generic(FORMAT_PREFIXES) {
        tags.insert(k.to_string(), v.to_string());
    }
    for (k, v) in attrs.with_prefix(ATTRIBUTE_PREFIX) {
        if !skip.contains(&k) {
            tags.insert(k.to_string(), v.to_string());
        }
    }
    tags
}

fn set(tags: &mut Tags, k: &str, v: impl Into<String>) {
    tags.insert(k.to_string(), v.into());
}

impl<'a> Writer<'a> {
    fn new(map: &'a Map, options: &'a SaveOptions) -> Self {
        let mut issues = Vec::new();
        let georef = options.georeference.or(map.metadata().georeference);
        if georef.is_none() {
            issues.push(Issue::warning(
                codes::NO_GEOREFERENCE,
                "the map has no georeference; lat/lon were written as 0 and the coordinates are \
                 only available in the local_x/local_y tags (Autoware `Local` projector)",
            ));
        }
        // Without a georeference the local tags are the only coordinates.
        let local_coordinates = georef.is_none() || options.local_coordinates.unwrap_or(true);
        Self {
            map,
            options,
            projector: georef.map(LocalProjector::new),
            local_coordinates,
            issues,
            used: BTreeSet::new(),
            next_free: 1,
            osm_ids: BTreeMap::new(),
            ways: Vec::new(),
            relations: Vec::new(),
            junction_ways: BTreeMap::new(),
            data: OsmData::default(),
        }
    }

    // ---------------------------------------------------------------------
    // IDs
    // ---------------------------------------------------------------------

    fn assign_ids(&mut self) {
        let map = self.map;
        // Entities that map 1:1 to OSM primitives, in a fixed order.
        let entities: Vec<EntityRef> = map
            .lanes()
            .map(|e| EntityRef::Lane(e.id))
            .chain(map.boundaries().map(|e| EntityRef::Boundary(e.id)))
            .chain(map.stop_lines().map(|e| EntityRef::StopLine(e.id)))
            .chain(
                map.traffic_signals()
                    .map(|e| EntityRef::TrafficSignal(e.id)),
            )
            .chain(map.crosswalks().map(|e| EntityRef::Crosswalk(e.id)))
            .chain(
                map.regulatory_elements()
                    .map(|e| EntityRef::RegulatoryElement(e.id)),
            )
            .chain(map.junctions().map(|e| EntityRef::Junction(e.id)))
            .collect();
        let max_raw = entities.iter().map(|e| e.raw()).max().unwrap_or(0);
        self.next_free = i64::try_from(max_raw).unwrap_or(i64::MAX / 2) + 1;
        let mut renumbered = Vec::new();
        for e in entities {
            let wanted = i64::try_from(e.raw()).ok().filter(|id| *id > 0);
            let id = match wanted {
                Some(id) if !self.used.contains(&id) => id,
                _ => {
                    renumbered.push(e);
                    self.fresh()
                }
            };
            self.used.insert(id);
            self.osm_ids.insert(e, id);
        }
        if !renumbered.is_empty() {
            let mut issue = Issue::info(
                codes::RENUMBERED,
                format!(
                    "{} entities share an ID with another entity and got new OSM IDs",
                    renumbered.len()
                ),
            )
            .with_entity(renumbered[0]);
            issue.related = renumbered[1..].to_vec();
            self.issues.push(issue);
        }
    }

    fn fresh(&mut self) -> i64 {
        while self.used.contains(&self.next_free) {
            self.next_free += 1;
        }
        let id = self.next_free;
        self.used.insert(id);
        self.next_free += 1;
        id
    }

    fn id_of(&self, e: impl Into<EntityRef>) -> Option<i64> {
        self.osm_ids.get(&e.into()).copied()
    }

    fn dangling(&mut self, owner: impl Into<EntityRef>, target: impl Into<EntityRef>) {
        let (owner, target) = (owner.into(), target.into());
        self.issues.push(
            Issue::warning(
                codes::DANGLING_REFERENCE,
                format!("{owner} references {target}, which does not exist; the reference was not written"),
            )
            .with_entity(owner)
            .with_related(target),
        );
    }

    // ---------------------------------------------------------------------
    // Top level
    // ---------------------------------------------------------------------

    fn write_all(&mut self) {
        self.assign_ids();
        self.write_boundaries();
        self.write_stop_lines();
        self.write_signals();
        self.write_junctions();
        self.write_lanes();
        self.write_crosswalks();
        self.write_rules();
        self.assign_nodes();
        for r in std::mem::take(&mut self.relations) {
            self.data.relations.insert(r.id, r);
        }
        if self.map.road_count() > 0 {
            let mut issue = Issue::info(
                codes::NOT_EXPORTED,
                format!(
                    "{} road(s) are not represented in Lanelet2 and were not written",
                    self.map.road_count()
                ),
            );
            issue.related = self.map.roads().map(|r| r.id.into()).collect();
            self.issues.push(issue);
        }
    }

    fn push_way(&mut self, id: i64, points: Vec<Point3>, tags: Tags) {
        self.ways.push(PendingWay {
            id,
            points,
            tags,
            node_tags: None,
        });
    }

    fn write_boundaries(&mut self) {
        for b in self.map.boundaries() {
            let id = self.id_of(b.id).expect("assigned");
            let mut tags = attribute_tags(&b.attributes, &["type", "subtype"]);
            // Non-canonical source tags win if they still describe the same kind.
            let attr_type = b.attributes.get_prefixed(ATTRIBUTE_PREFIX, "type");
            let attr_sub = b.attributes.get_prefixed(ATTRIBUTE_PREFIX, "subtype");
            let (ct, cs) = boundary_tags(b.kind);
            let ty = attr_type.unwrap_or(ct);
            let sub = attr_sub.or(cs);
            let (ty, sub) = if boundary_kind_from_tags(Some(ty), sub) == b.kind {
                (ty, sub)
            } else {
                (ct, cs)
            };
            set(&mut tags, "type", ty);
            match sub {
                Some(s) => set(&mut tags, "subtype", s),
                None => {
                    tags.remove("subtype");
                }
            }
            self.push_way(id, b.geometry.points.clone(), tags);
        }
    }

    fn write_stop_lines(&mut self) {
        for s in self.map.stop_lines() {
            let id = self.id_of(s.id).expect("assigned");
            let mut tags = attribute_tags(&s.attributes, &["type"]);
            set(&mut tags, "type", "stop_line");
            self.push_way(id, s.geometry.points.clone(), tags);
        }
    }

    fn signal_height(&mut self, s: &TrafficSignal) -> Option<f64> {
        match (s.height, self.options.profile) {
            (Some(h), _) => Some(h),
            (None, Profile::Autoware) => Some(autoware::DEFAULT_SIGNAL_HEIGHT),
            (None, Profile::Generic) => None,
        }
    }

    fn write_signals(&mut self) {
        for s in self.map.traffic_signals() {
            let id = self.id_of(s.id).expect("assigned");
            let mut tags = attribute_tags(&s.attributes, &["type", "subtype", "height"]);
            set(&mut tags, "type", "traffic_light");
            let subtype = s
                .attributes
                .get_prefixed(ATTRIBUTE_PREFIX, "subtype")
                .filter(|st| signal_kind_from_subtype(Some(st)) == s.kind)
                .unwrap_or(signal_subtype(s.kind));
            set(&mut tags, "subtype", subtype);
            if let Some(h) = self.signal_height(s) {
                set(&mut tags, "height", fmt_decimal(h));
            }
            self.push_way(id, s.geometry.points.clone(), tags);
        }
    }

    /// Light-bulb way of a signal (Autoware extension), if it has bulbs.
    fn light_bulbs_way(&mut self, s: &TrafficSignal) -> Option<i64> {
        if s.bulbs.is_empty() {
            return None;
        }
        let id = self.fresh();
        let mut tags = Tags::new();
        set(&mut tags, "type", "light_bulbs");
        set(&mut tags, "traffic_light_id", self.id_of(s.id)?.to_string());
        let node_tags = s
            .bulbs
            .iter()
            .map(|b| {
                let mut t = Tags::new();
                set(&mut t, "color", color_str(b.color));
                if let Some(a) = b.arrow {
                    set(&mut t, "arrow", arrow_str(a));
                }
                t
            })
            .collect();
        self.ways.push(PendingWay {
            id,
            points: s.bulbs.iter().map(|b| b.position).collect(),
            tags,
            node_tags: Some(node_tags),
        });
        Some(id)
    }

    fn write_junctions(&mut self) {
        for j in self.map.junctions() {
            let id = self.id_of(j.id).expect("assigned");
            let outline = match &j.outline {
                Some(o) => o.points.clone(),
                None => {
                    // Convex hull of the member lanes' boundaries.
                    let pts: Vec<Point3> = j
                        .lanes
                        .iter()
                        .filter_map(|l| self.map.lane_polygon(*l))
                        .flat_map(|p| p.points)
                        .collect();
                    let hull = convex_hull(&pts);
                    if hull.len() < 3 {
                        continue;
                    }
                    self.issues.push(
                        Issue::info(
                            codes::GEOMETRY_SYNTHESIZED,
                            format!("{} has no outline; its convex hull was written as intersection_area", j.id),
                        )
                        .with_entity(j.id),
                    );
                    hull
                }
            };
            let mut tags = attribute_tags(&j.attributes, &["type", "area", "name"]);
            if let Some(name) = &j.name {
                set(&mut tags, "name", name.clone());
            }
            set(&mut tags, "type", "intersection_area");
            set(&mut tags, "area", "yes");
            self.push_way(id, outline, tags);
            self.junction_ways.insert(j.id, id);
        }
    }

    // ---------------------------------------------------------------------
    // Lanelets
    // ---------------------------------------------------------------------

    fn rules_of(&self, lane: vectormap_core::LaneId) -> Vec<i64> {
        self.map
            .rules_for_lane(lane)
            .iter()
            .filter_map(|r| self.id_of(r.id))
            .collect()
    }

    fn lane_tags(&self, lane: &Lane) -> Tags {
        let mut tags = attribute_tags(
            &lane.attributes,
            &[
                "type",
                "subtype",
                "speed_limit",
                "one_way",
                "turn_direction",
                "intersection_area",
            ],
        );
        set(&mut tags, "type", "lanelet");
        let subtype = lane
            .attributes
            .get_prefixed(ATTRIBUTE_PREFIX, "subtype")
            .filter(|st| lane_kind_from_subtype(st) == lane.kind)
            .unwrap_or(lane_subtype(lane.kind));
        set(&mut tags, "subtype", subtype);
        set(
            &mut tags,
            "one_way",
            if lane.one_way { "yes" } else { "no" },
        );
        if let Some(s) = lane.speed_limit {
            set(&mut tags, "speed_limit", fmt_decimal(s.kmh()));
        } else if let Some(v) = lane
            .attributes
            .get_prefixed(ATTRIBUTE_PREFIX, "speed_limit")
        {
            set(&mut tags, "speed_limit", v);
        }
        if let Some(t) = lane.turn_direction {
            set(&mut tags, "turn_direction", turn_str(t));
        } else if let Some(v) = lane
            .attributes
            .get_prefixed(ATTRIBUTE_PREFIX, "turn_direction")
        {
            set(&mut tags, "turn_direction", v);
        }
        if let Some(j) = self.map.junction_of(lane.id)
            && let Some(w) = self.junction_ways.get(&j)
        {
            set(&mut tags, "intersection_area", w.to_string());
        }
        if self.options.profile == Profile::Autoware {
            autoware::apply_lanelet_defaults(&mut tags, lane.kind);
        }
        tags
    }

    fn write_lanes(&mut self) {
        for lane in self.map.lanes() {
            let id = self.id_of(lane.id).expect("assigned");
            let mut members = Vec::new();
            for (role, r) in [("left", lane.left), ("right", lane.right)] {
                match self
                    .id_of(r.boundary)
                    .filter(|_| self.map.boundary(r.boundary).is_some())
                {
                    Some(w) => members.push(OsmMember {
                        kind: MemberType::Way,
                        reference: w,
                        role: role.into(),
                    }),
                    None => self.dangling(lane.id, r.boundary),
                }
            }
            if members.len() < 2 {
                self.issues.push(
                    Issue::error(
                        codes::INVALID_PRIMITIVE,
                        format!("{} has a missing boundary and was not written", lane.id),
                    )
                    .with_entity(lane.id),
                );
                continue;
            }
            if let Some(c) = &lane.centerline {
                let cid = self.fresh();
                self.push_way(cid, c.points.clone(), Tags::new());
                members.push(OsmMember {
                    kind: MemberType::Way,
                    reference: cid,
                    role: "centerline".into(),
                });
            }
            for re in self.rules_of(lane.id) {
                members.push(OsmMember {
                    kind: MemberType::Relation,
                    reference: re,
                    role: "regulatory_element".into(),
                });
            }
            let tags = self.lane_tags(lane);
            self.relations.push(OsmRelation { id, members, tags });
        }
    }

    fn write_crosswalks(&mut self) {
        for c in self.map.crosswalks() {
            let id = self.id_of(c.id).expect("assigned");
            let mut members = Vec::new();
            for (role, edge) in [("left", &c.left_edge), ("right", &c.right_edge)] {
                let wid = self.fresh();
                let mut tags = Tags::new();
                let ty = c
                    .attributes
                    .get_prefixed(ATTRIBUTE_PREFIX, &format!("{role}_edge_type"))
                    .unwrap_or("pedestrian_marking");
                set(&mut tags, "type", ty);
                self.push_way(wid, edge.points.clone(), tags);
                members.push(OsmMember {
                    kind: MemberType::Way,
                    reference: wid,
                    role: role.into(),
                });
            }
            for re in self
                .map
                .regulatory_elements()
                .filter(|r| r.controlled_crosswalks.contains(&c.id))
            {
                members.push(OsmMember {
                    kind: MemberType::Relation,
                    reference: self.id_of(re.id).expect("assigned"),
                    role: "regulatory_element".into(),
                });
            }
            let mut tags = attribute_tags(
                &c.attributes,
                &["type", "subtype", "left_edge_type", "right_edge_type"],
            );
            set(&mut tags, "type", "lanelet");
            set(&mut tags, "subtype", "crosswalk");
            set(&mut tags, "one_way", "no");
            set(&mut tags, "participant:pedestrian", "yes");
            if self.options.profile == Profile::Autoware {
                autoware::apply_crosswalk_defaults(&mut tags);
            }
            self.relations.push(OsmRelation { id, members, tags });
        }
    }

    // ---------------------------------------------------------------------
    // Regulatory elements
    // ---------------------------------------------------------------------

    fn member_way(&mut self, owner: EntityRef, target: EntityRef, role: &str) -> Option<OsmMember> {
        if !self.map.contains(target) {
            self.dangling(owner, target);
            return None;
        }
        Some(OsmMember {
            kind: MemberType::Way,
            reference: self.id_of(target)?,
            role: role.into(),
        })
    }

    fn member_relation(
        &mut self,
        owner: EntityRef,
        target: EntityRef,
        role: &str,
    ) -> Option<OsmMember> {
        if !self.map.contains(target) {
            self.dangling(owner, target);
            return None;
        }
        Some(OsmMember {
            kind: MemberType::Relation,
            reference: self.id_of(target)?,
            role: role.into(),
        })
    }

    fn write_rules(&mut self) {
        for re in self.map.regulatory_elements() {
            let owner: EntityRef = re.id.into();
            let id = self.id_of(re.id).expect("assigned");
            let mut members: Vec<OsmMember> = Vec::new();
            let subtype: String = match &re.rule {
                Rule::TrafficLight { signals, stop_line } => {
                    for s in signals {
                        members.extend(self.member_way(owner, (*s).into(), "refers"));
                    }
                    if let Some(sl) = stop_line {
                        members.extend(self.member_way(owner, (*sl).into(), "ref_line"));
                    }
                    for s in signals {
                        if let Some(sig) = self.map.traffic_signal(*s)
                            && let Some(w) = self.light_bulbs_way(sig)
                        {
                            members.push(OsmMember {
                                kind: MemberType::Way,
                                reference: w,
                                role: "light_bulbs".into(),
                            });
                        }
                    }
                    "traffic_light".into()
                }
                Rule::TrafficSign {
                    sign_type,
                    sign,
                    stop_line,
                } => {
                    let geometry = match sign {
                        Some(g) if !g.is_empty() => g.clone(),
                        _ => {
                            let g = self.synthesize_sign(re, stop_line.as_ref());
                            self.issues.push(
                                Issue::info(
                                    codes::GEOMETRY_SYNTHESIZED,
                                    format!("{} has no sign geometry; a placeholder sign was written next to its stop line", re.id),
                                )
                                .with_entity(re.id),
                            );
                            g
                        }
                    };
                    let wid = self.fresh();
                    let mut tags = Tags::new();
                    set(&mut tags, "type", "traffic_sign");
                    set(&mut tags, "subtype", sign_type.clone());
                    self.push_way(wid, geometry.points, tags);
                    members.push(OsmMember {
                        kind: MemberType::Way,
                        reference: wid,
                        role: "refers".into(),
                    });
                    if let Some(sl) = stop_line {
                        members.extend(self.member_way(owner, (*sl).into(), "ref_line"));
                    }
                    "traffic_sign".into()
                }
                Rule::StopLine { stop_line } => {
                    members.extend(self.member_way(owner, (*stop_line).into(), "refers"));
                    "road_marking".into()
                }
                Rule::RightOfWay {
                    priority,
                    yielding,
                    stop_lines,
                } => {
                    for l in priority {
                        members.extend(self.member_relation(owner, (*l).into(), "right_of_way"));
                    }
                    for l in yielding {
                        members.extend(self.member_relation(owner, (*l).into(), "yield"));
                    }
                    for s in stop_lines {
                        members.extend(self.member_way(owner, (*s).into(), "ref_line"));
                    }
                    "right_of_way".into()
                }
                Rule::Crosswalk {
                    crosswalk,
                    stop_lines,
                } => {
                    members.extend(self.member_relation(owner, (*crosswalk).into(), "refers"));
                    for s in stop_lines {
                        members.extend(self.member_way(owner, (*s).into(), "ref_line"));
                    }
                    if let Some(c) = self.map.crosswalk(*crosswalk)
                        && let Some(w) = self.crosswalk_polygon(c)
                    {
                        members.push(OsmMember {
                            kind: MemberType::Way,
                            reference: w,
                            role: "crosswalk_polygon".into(),
                        });
                    }
                    "crosswalk".into()
                }
                Rule::Other { kind } => {
                    self.issues.push(
                        Issue::warning(
                            codes::UNSUPPORTED_MEMBER,
                            format!("{} ({kind}) is written without members; the IR does not model them", re.id),
                        )
                        .with_entity(re.id),
                    );
                    kind.clone()
                }
            };
            let mut tags = attribute_tags(&re.attributes, &["type", "subtype"]);
            set(&mut tags, "type", "regulatory_element");
            set(&mut tags, "subtype", subtype);
            self.relations.push(OsmRelation { id, members, tags });
        }
    }

    fn crosswalk_polygon(&mut self, c: &Crosswalk) -> Option<i64> {
        let polygon = c.polygon.as_ref()?;
        let id = self.fresh();
        let mut tags = Tags::new();
        set(&mut tags, "type", "crosswalk_polygon");
        set(&mut tags, "area", "yes");
        self.push_way(id, polygon.points.clone(), tags);
        Some(id)
    }

    /// A 0.6 m sign 0.5 m beyond the right end of the stop line (or of the
    /// first lane's end), 2 m above the ground.
    fn synthesize_sign(
        &self,
        re: &vectormap_core::RegulatoryElement,
        stop_line: Option<&vectormap_core::StopLineId>,
    ) -> Polyline3 {
        let line = stop_line
            .and_then(|s| self.map.stop_line(*s))
            .map(|s| s.geometry.clone())
            .or_else(|| {
                let lane = *re.lanes.first()?;
                let l = self
                    .map
                    .oriented_boundary(lane, vectormap_core::Side::Left)?;
                let r = self
                    .map
                    .oriented_boundary(lane, vectormap_core::Side::Right)?;
                Some(Polyline3::new(vec![l.last()?, r.last()?]))
            })
            .unwrap_or_else(|| Polyline3::from_xy(&[[0.0, 0.0], [1.0, 0.0]]));
        let (a, b) = (line.first().unwrap(), line.last().unwrap());
        let dir = (b.xy() - a.xy())
            .normalized()
            .unwrap_or(vectormap_core::Point2::new(1.0, 0.0));
        let p0 = b.xy() + dir * 0.5;
        let p1 = b.xy() + dir * 1.1;
        Polyline3::new(vec![p0.with_z(b.z + 2.0), p1.with_z(b.z + 2.0)])
    }

    // ---------------------------------------------------------------------
    // Nodes
    // ---------------------------------------------------------------------

    fn node_tags(&self, p: Point3) -> Tags {
        let mut tags = Tags::new();
        set(&mut tags, "ele", fmt_coord(p.z));
        if let Some(projector) = self
            .projector
            .filter(|p| p.georeference().projection == ProjectionKind::Mgrs)
        {
            if let Some(grid) = crate::projection::mgrs_grid(projector.inverse(p)) {
                set(&mut tags, "mgrs_code", grid);
            }
        }
        if self.local_coordinates {
            set(&mut tags, "local_x", fmt_coord(p.x));
            set(&mut tags, "local_y", fmt_coord(p.y));
        }
        tags
    }

    fn assign_nodes(&mut self) {
        let mut ways = std::mem::take(&mut self.ways);
        ways.sort_by_key(|w| w.id);
        let key = |p: Point3| {
            let q = |v: f64| (v * 1e6).round() as i64;
            (q(p.x), q(p.y), q(p.z))
        };
        let mut shared: BTreeMap<(i64, i64, i64), i64> = BTreeMap::new();
        for w in ways {
            let mut refs: Vec<i64> = Vec::with_capacity(w.points.len());
            for (i, &p) in w.points.iter().enumerate() {
                let node = match &w.node_tags {
                    Some(extra) => {
                        let id = self.fresh();
                        let mut tags = self.node_tags(p);
                        tags.extend(extra[i].clone());
                        self.push_node(id, p, tags);
                        id
                    }
                    None => match shared.get(&key(p)) {
                        Some(&id) => id,
                        None => {
                            let id = self.fresh();
                            let tags = self.node_tags(p);
                            self.push_node(id, p, tags);
                            shared.insert(key(p), id);
                            id
                        }
                    },
                };
                // Consecutive duplicates would create zero-length segments.
                if refs.last() != Some(&node) {
                    refs.push(node);
                }
            }
            self.data.ways.insert(
                w.id,
                OsmWay {
                    id: w.id,
                    nodes: refs,
                    tags: w.tags,
                },
            );
        }
    }

    fn push_node(&mut self, id: i64, p: Point3, tags: Tags) {
        let g = self
            .projector
            .map_or(GeoPoint::new(0.0, 0.0), |pr| pr.inverse(p));
        if let Some(projector) = self
            .projector
            .filter(|pr| pr.georeference().projection == ProjectionKind::Mgrs)
        {
            let grid = crate::projection::mgrs_grid(projector.georeference().origin);
            if (grid.is_none() || crate::projection::mgrs_grid(g) != grid)
                && !self.issues.iter().any(|i| i.code == codes::PROJECTION)
            {
                self.issues.push(Issue::error(
                    codes::PROJECTION,
                    "MGRS export contains coordinates outside the origin's supported grid square",
                ));
            }
        }
        self.data.nodes.insert(
            id,
            OsmNode {
                id,
                lat: g.lat,
                lon: g.lon,
                tags,
            },
        );
    }
}

/// Convex hull (XY) of a point set, counter-clockwise, z = mean z.
fn convex_hull(points: &[Point3]) -> Vec<Point3> {
    let mut pts: Vec<Point3> = points.iter().copied().filter(|p| p.is_finite()).collect();
    if pts.len() < 3 {
        return pts;
    }
    let z = pts.iter().map(|p| p.z).sum::<f64>() / pts.len() as f64;
    pts.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    pts.dedup_by(|a, b| a.x == b.x && a.y == b.y);
    let cross = |o: Point3, a: Point3, b: Point3| (a.xy() - o.xy()).cross(b.xy() - o.xy());
    let mut hull: Vec<Point3> = Vec::new();
    for pass in 0..2 {
        let start = hull.len();
        let iter: Box<dyn Iterator<Item = &Point3>> = if pass == 0 {
            Box::new(pts.iter())
        } else {
            Box::new(pts.iter().rev())
        };
        for &p in iter {
            while hull.len() >= start + 2
                && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0
            {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    hull.into_iter().map(|p| Point3::new(p.x, p.y, z)).collect()
}
