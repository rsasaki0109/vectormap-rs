//! # vectormap-validation
//!
//! Structured validation of [`vectormap_core::Map`]s.
//!
//! [`validate`] runs a fixed set of checks and returns a [`ValidationReport`]
//! of [`Issue`]s. Every issue has a stable machine-readable code (see
//! [`codes`]), a severity, the entity concerned and — where the repair is
//! unambiguous — a suggested [`Command`] in [`Issue::fix`], so that tools and
//! language models can fix problems automatically.
//!
//! ```
//! use vectormap_core::samples;
//! use vectormap_validation::{validate, ValidationOptions};
//!
//! let (map, _) = samples::intersection();
//! let report = validate(&map, &ValidationOptions::default());
//! assert!(!report.has_errors());
//! ```

use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use vectormap_core::{
    Command, EntityRef, Issue, LaneId, Map, Polyline3, Rule, Severity, Side,
    topology::reciprocal_side,
};

/// Issue codes emitted by the validator.
pub mod codes {
    use vectormap_core::IssueCode;

    pub use vectormap_core::diagnostics::codes::{DUPLICATE_ID, GEOMETRIC_GAP, ORPHAN_RULE};

    /// A reference to a lane that does not exist.
    pub const DANGLING_LANE_REFERENCE: IssueCode = IssueCode::new("dangling_lane_reference");
    /// A lane references a boundary that does not exist.
    pub const MISSING_BOUNDARY: IssueCode = IssueCode::new("missing_boundary");
    /// A rule references a stop line, signal or crosswalk that does not exist.
    pub const MISSING_REFERENCE: IssueCode = IssueCode::new("missing_reference");
    /// A predecessor / successor relation is only stored on one side.
    pub const ASYMMETRIC_LINK: IssueCode = IssueCode::new("asymmetric_link");
    /// A lane is its own successor.
    pub const SELF_LOOP: IssueCode = IssueCode::new("self_loop");
    /// A neighbor relation is missing, self-referencing or not reciprocal.
    pub const INVALID_NEIGHBOR: IssueCode = IssueCode::new("invalid_neighbor");
    /// A geometry has no points.
    pub const EMPTY_GEOMETRY: IssueCode = IssueCode::new("empty_geometry");
    /// A geometry has too few distinct points or repeated points.
    pub const DEGENERATE_GEOMETRY: IssueCode = IssueCode::new("degenerate_geometry");
    /// A coordinate is NaN or infinite.
    pub const NON_FINITE_COORDINATE: IssueCode = IssueCode::new("non_finite_coordinate");
    /// A lane uses the same boundary on both sides.
    pub const SAME_BOUNDARY_BOTH_SIDES: IssueCode = IssueCode::new("same_boundary_both_sides");
    /// The left and right boundaries of a lane point in opposite directions.
    pub const BOUNDARY_DIRECTION_MISMATCH: IssueCode =
        IssueCode::new("boundary_direction_mismatch");
    /// The left boundary lies to the right of the right boundary.
    pub const INVERTED_LANE: IssueCode = IssueCode::new("inverted_lane");
    /// A speed limit is not positive and finite.
    pub const INVALID_SPEED_LIMIT: IssueCode = IssueCode::new("invalid_speed_limit");
    /// A lane has neither predecessors nor successors.
    pub const ISOLATED_LANE: IssueCode = IssueCode::new("isolated_lane");
    /// The lane graph consists of several unconnected parts.
    pub const DISCONNECTED_TOPOLOGY: IssueCode = IssueCode::new("disconnected_topology");
    /// A boundary is not used by any lane.
    pub const UNUSED_BOUNDARY: IssueCode = IssueCode::new("unused_boundary");
    /// A stop line, signal or crosswalk is not referenced by any rule.
    pub const UNUSED_FEATURE: IssueCode = IssueCode::new("unused_feature");
    /// A lane belongs to several roads or junctions.
    pub const DUPLICATE_MEMBERSHIP: IssueCode = IssueCode::new("duplicate_membership");
    /// A rule is structurally incomplete (e.g. a traffic light without signals).
    pub const INCOMPLETE_RULE: IssueCode = IssueCode::new("incomplete_rule");
}

/// Options of [`validate`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ValidationOptions {
    /// Connected lanes whose end / start points are further apart than this
    /// are reported as `geometric_gap` (metres).
    pub gap_tolerance: f64,
    /// Run the connectivity checks (`isolated_lane`, `disconnected_topology`).
    pub check_connectivity: bool,
    /// Include informational issues.
    pub include_info: bool,
}

impl Default for ValidationOptions {
    fn default() -> Self {
        Self {
            gap_tolerance: 0.1,
            check_connectivity: true,
            include_info: true,
        }
    }
}

/// Number of issues per severity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct IssueCounts {
    /// Errors.
    pub errors: usize,
    /// Warnings.
    pub warnings: usize,
    /// Informational notes.
    pub infos: usize,
}

/// Result of [`validate`].
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct ValidationReport {
    /// Counts per severity.
    pub counts: IssueCounts,
    /// Issues, most severe first, then by code and entity.
    pub issues: Vec<Issue>,
}

impl ValidationReport {
    /// Builds a report from issues (sorting them deterministically).
    pub fn from_issues(mut issues: Vec<Issue>) -> Self {
        issues.sort_by(|a, b| {
            (Reverse(a.severity), &a.code, a.entity, &a.message).cmp(&(
                Reverse(b.severity),
                &b.code,
                b.entity,
                &b.message,
            ))
        });
        let mut counts = IssueCounts::default();
        for i in &issues {
            match i.severity {
                Severity::Error => counts.errors += 1,
                Severity::Warning => counts.warnings += 1,
                Severity::Info => counts.infos += 1,
            }
        }
        Self { counts, issues }
    }

    /// Adds more issues (e.g. from a loader) and re-sorts.
    pub fn merge(self, more: impl IntoIterator<Item = Issue>) -> Self {
        let mut issues = self.issues;
        issues.extend(more);
        Self::from_issues(issues)
    }

    /// `true` if at least one error was found.
    pub fn has_errors(&self) -> bool {
        self.counts.errors > 0
    }

    /// `true` if no issue at all was found.
    pub fn is_clean(&self) -> bool {
        self.issues.is_empty()
    }

    /// Issues with the given code.
    pub fn with_code<'a>(&'a self, code: &'a str) -> impl Iterator<Item = &'a Issue> + 'a {
        self.issues.iter().filter(move |i| i.code == code)
    }
}

/// Validates a map.
pub fn validate(map: &Map, options: &ValidationOptions) -> ValidationReport {
    let mut v = Validator {
        map,
        options,
        issues: Vec::new(),
    };
    v.check_geometry();
    v.check_lanes();
    v.check_topology();
    v.check_groups();
    v.check_rules();
    v.check_usage();
    v.check_id_collisions();
    if options.check_connectivity {
        v.check_connectivity();
    }
    let issues = if options.include_info {
        v.issues
    } else {
        v.issues
            .into_iter()
            .filter(|i| i.severity > Severity::Info)
            .collect()
    };
    ValidationReport::from_issues(issues)
}

struct Validator<'a> {
    map: &'a Map,
    options: &'a ValidationOptions,
    issues: Vec<Issue>,
}

impl Validator<'_> {
    fn push(&mut self, issue: Issue) {
        self.issues.push(issue);
    }

    fn polyline(&mut self, owner: EntityRef, what: &str, p: &Polyline3) {
        if p.is_empty() {
            self.push(
                Issue::error(
                    codes::EMPTY_GEOMETRY,
                    format!("{what} of {owner} has no points"),
                )
                .with_entity(owner),
            );
            return;
        }
        if p.points.iter().any(|q| !q.is_finite()) {
            self.push(
                Issue::error(
                    codes::NON_FINITE_COORDINATE,
                    format!("{what} of {owner} contains NaN or infinite coordinates"),
                )
                .with_entity(owner),
            );
            return;
        }
        if p.len() < 2 || p.length() <= 1e-9 {
            self.push(
                Issue::error(
                    codes::DEGENERATE_GEOMETRY,
                    format!("{what} of {owner} needs at least two distinct points"),
                )
                .with_entity(owner),
            );
            return;
        }
        let repeated = p
            .points
            .windows(2)
            .filter(|w| w[0].distance(w[1]) <= 1e-9)
            .count();
        if repeated > 0 {
            self.push(
                Issue::warning(
                    codes::DEGENERATE_GEOMETRY,
                    format!("{what} of {owner} has {repeated} repeated consecutive point(s)"),
                )
                .with_entity(owner),
            );
        }
    }

    fn check_geometry(&mut self) {
        let map = self.map;
        for b in map.boundaries() {
            self.polyline(b.id.into(), "geometry", &b.geometry);
        }
        for l in map.lanes() {
            if let Some(c) = &l.centerline {
                self.polyline(l.id.into(), "centerline", c);
            }
        }
        for s in map.stop_lines() {
            self.polyline(s.id.into(), "geometry", &s.geometry);
        }
        for s in map.traffic_signals() {
            self.polyline(s.id.into(), "geometry", &s.geometry);
        }
        for c in map.crosswalks() {
            self.polyline(c.id.into(), "left edge", &c.left_edge);
            self.polyline(c.id.into(), "right edge", &c.right_edge);
        }
        for j in map.junctions() {
            if let Some(o) = &j.outline {
                let owner: EntityRef = j.id.into();
                if o.points.is_empty() {
                    self.push(
                        Issue::error(
                            codes::EMPTY_GEOMETRY,
                            format!("outline of {owner} has no points"),
                        )
                        .with_entity(owner),
                    );
                } else if o.points.len() < 3 || o.area_2d() <= 1e-9 {
                    self.push(
                        Issue::error(
                            codes::DEGENERATE_GEOMETRY,
                            format!("outline of {owner} has no area"),
                        )
                        .with_entity(owner),
                    );
                }
            }
        }
    }

    fn check_lanes(&mut self) {
        let map = self.map;
        for lane in map.lanes() {
            let mut boundaries_ok = true;
            for side in [Side::Left, Side::Right] {
                let r = lane.boundary(side);
                if map.boundary(r.boundary).is_none() {
                    boundaries_ok = false;
                    self.push(
                        Issue::error(
                            codes::MISSING_BOUNDARY,
                            format!(
                                "{side:?} boundary {} of {} does not exist",
                                r.boundary, lane.id
                            ),
                        )
                        .with_entity(lane.id)
                        .with_related(r.boundary),
                    );
                }
            }
            if lane.left.boundary == lane.right.boundary {
                self.push(
                    Issue::error(
                        codes::SAME_BOUNDARY_BOTH_SIDES,
                        format!(
                            "{} uses {} as left and right boundary",
                            lane.id, lane.left.boundary
                        ),
                    )
                    .with_entity(lane.id),
                );
                boundaries_ok = false;
            }
            if let Some(s) = lane.speed_limit
                && !s.is_valid()
            {
                self.push(
                    Issue::error(
                        codes::INVALID_SPEED_LIMIT,
                        format!("{} has an invalid speed limit ({} km/h)", lane.id, s.kmh),
                    )
                    .with_entity(lane.id)
                    .with_fix(Command::SetSpeedLimit {
                        lanes: vec![lane.id],
                        kmh: None,
                    }),
                );
            }
            if boundaries_ok {
                self.check_lane_orientation(lane.id);
            }
        }
    }

    fn check_lane_orientation(&mut self, lane: LaneId) {
        let map = self.map;
        let (Some(l), Some(r)) = (
            map.oriented_boundary(lane, Side::Left),
            map.oriented_boundary(lane, Side::Right),
        ) else {
            return;
        };
        if !(l.is_valid() && r.is_valid())
            || l.points.iter().chain(&r.points).any(|p| !p.is_finite())
        {
            return;
        }
        let (ls, le, rs, re) = (
            l.first().unwrap(),
            l.last().unwrap(),
            r.first().unwrap(),
            r.last().unwrap(),
        );
        let aligned = ls.distance_2d(rs) + le.distance_2d(re);
        let crossed = ls.distance_2d(re) + le.distance_2d(rs);
        if crossed + 1e-9 < aligned {
            self.push(
                Issue::error(
                    codes::BOUNDARY_DIRECTION_MISMATCH,
                    format!("the left and right boundaries of {lane} point in opposite directions"),
                )
                .with_entity(lane),
            );
            return;
        }
        // The left boundary must be on the left of the direction of travel.
        let Some(center) = map.centerline(lane) else {
            return;
        };
        let (Some(lm), Some(rm)) = (l.point_at_fraction(0.5), r.point_at_fraction(0.5)) else {
            return;
        };
        let (Some(nl), Some(nr)) = (center.nearest_point(lm.xy()), center.nearest_point(rm.xy()))
        else {
            return;
        };
        if nl.lateral < nr.lateral {
            self.push(
                Issue::warning(
                    codes::INVERTED_LANE,
                    format!("the left boundary of {lane} lies to the right of its right boundary"),
                )
                .with_entity(lane),
            );
        }
    }

    fn check_topology(&mut self) {
        let map = self.map;
        let topo = map.topology();
        for (lane, links) in topo.iter() {
            if map.lane(lane).is_none() {
                self.push(
                    Issue::error(
                        codes::DANGLING_LANE_REFERENCE,
                        format!("topology has an entry for {lane}, which does not exist"),
                    )
                    .with_entity(lane),
                );
                continue;
            }
            for &s in &links.successors {
                if s == lane {
                    self.push(
                        Issue::error(codes::SELF_LOOP, format!("{lane} is its own successor"))
                            .with_entity(lane)
                            .with_fix(Command::DisconnectLanes {
                                from: lane,
                                to: lane,
                            }),
                    );
                    continue;
                }
                if map.lane(s).is_none() {
                    self.push(
                        Issue::error(
                            codes::DANGLING_LANE_REFERENCE,
                            format!("successor {s} of {lane} does not exist"),
                        )
                        .with_entity(lane)
                        .with_related(s),
                    );
                    continue;
                }
                if !topo.predecessors(s).contains(&lane) {
                    self.push(
                        Issue::error(
                            codes::ASYMMETRIC_LINK,
                            format!("{s} is a successor of {lane}, but {lane} is not a predecessor of {s}"),
                        )
                        .with_entity(lane)
                        .with_related(s)
                        .with_fix(Command::ConnectLanes { from: lane, to: s }),
                    );
                }
                if let Some(gap) = map.connection_gap(lane, s)
                    && gap > self.options.gap_tolerance
                {
                    self.push(
                        Issue::warning(
                            codes::GEOMETRIC_GAP,
                            format!(
                                "{s} starts {gap:.3} m away from the end of its predecessor {lane}"
                            ),
                        )
                        .with_entity(lane)
                        .with_related(s),
                    );
                }
            }
            for &p in &links.predecessors {
                if p == lane {
                    if !links.successors.contains(&lane) {
                        self.push(
                            Issue::error(
                                codes::SELF_LOOP,
                                format!("{lane} is its own predecessor"),
                            )
                            .with_entity(lane)
                            .with_fix(Command::DisconnectLanes {
                                from: lane,
                                to: lane,
                            }),
                        );
                    }
                    continue;
                }
                if map.lane(p).is_none() {
                    self.push(
                        Issue::error(
                            codes::DANGLING_LANE_REFERENCE,
                            format!("predecessor {p} of {lane} does not exist"),
                        )
                        .with_entity(lane)
                        .with_related(p),
                    );
                    continue;
                }
                if !topo.successors(p).contains(&lane) {
                    self.push(
                        Issue::error(
                            codes::ASYMMETRIC_LINK,
                            format!("{p} is a predecessor of {lane}, but {lane} is not a successor of {p}"),
                        )
                        .with_entity(lane)
                        .with_related(p)
                        .with_fix(Command::ConnectLanes { from: p, to: lane }),
                    );
                }
            }
            for side in [Side::Left, Side::Right] {
                let Some(n) = links.neighbor(side) else {
                    continue;
                };
                let clear = Command::SetNeighbor {
                    lane,
                    side,
                    neighbor: None,
                };
                if n.lane == lane {
                    self.push(
                        Issue::error(
                            codes::INVALID_NEIGHBOR,
                            format!("{lane} is its own {side:?} neighbor"),
                        )
                        .with_entity(lane)
                        .with_fix(clear),
                    );
                } else if map.lane(n.lane).is_none() {
                    self.push(
                        Issue::error(
                            codes::INVALID_NEIGHBOR,
                            format!("{side:?} neighbor {} of {lane} does not exist", n.lane),
                        )
                        .with_entity(lane)
                        .with_related(n.lane)
                        .with_fix(clear),
                    );
                } else {
                    let rside = reciprocal_side(side, n.direction);
                    let back = topo.neighbor(n.lane, rside);
                    if back.map(|b| (b.lane, b.direction)) != Some((lane, n.direction)) {
                        self.push(
                            Issue::error(
                                codes::INVALID_NEIGHBOR,
                                format!(
                                    "{} is the {side:?} neighbor of {lane}, but {lane} is not its \
                                     {rside:?} neighbor",
                                    n.lane
                                ),
                            )
                            .with_entity(lane)
                            .with_related(n.lane)
                            .with_fix(Command::SetNeighbor {
                                lane,
                                side,
                                neighbor: Some(n),
                            }),
                        );
                    }
                }
            }
        }
    }

    fn lane_refs(&mut self, owner: EntityRef, what: &str, lanes: &[LaneId]) {
        for &l in lanes {
            if self.map.lane(l).is_none() {
                self.push(
                    Issue::error(
                        codes::DANGLING_LANE_REFERENCE,
                        format!("{owner} references {l} ({what}), which does not exist"),
                    )
                    .with_entity(owner)
                    .with_related(l),
                );
            }
        }
    }

    fn check_groups(&mut self) {
        let map = self.map;
        let mut road_of: BTreeMap<LaneId, Vec<EntityRef>> = BTreeMap::new();
        let mut junction_of: BTreeMap<LaneId, Vec<EntityRef>> = BTreeMap::new();
        for r in map.roads() {
            self.lane_refs(r.id.into(), "member", &r.lanes);
            for &l in &r.lanes {
                road_of.entry(l).or_default().push(r.id.into());
            }
        }
        for j in map.junctions() {
            self.lane_refs(j.id.into(), "member", &j.lanes);
            for &l in &j.lanes {
                junction_of.entry(l).or_default().push(j.id.into());
            }
        }
        for (what, index) in [("road", road_of), ("junction", junction_of)] {
            for (lane, groups) in index {
                let mut unique = groups.clone();
                unique.dedup();
                if groups.len() > 1 {
                    let mut issue = Issue::warning(
                        codes::DUPLICATE_MEMBERSHIP,
                        format!("{lane} is listed {} times as a {what} member", groups.len()),
                    )
                    .with_entity(lane);
                    issue.related = unique;
                    self.push(issue);
                }
            }
        }
    }

    fn check_rules(&mut self) {
        let map = self.map;
        for re in map.regulatory_elements() {
            let owner: EntityRef = re.id.into();
            self.lane_refs(owner, "applies to", &re.lanes);
            self.lane_refs(owner, "rule", &re.rule.referenced_lanes());
            if re.lanes.is_empty() {
                self.push(
                    Issue::warning(
                        codes::ORPHAN_RULE,
                        format!("{owner} does not apply to any lane"),
                    )
                    .with_entity(owner)
                    .with_fix(Command::RemoveEntity { entity: owner }),
                );
            }
            for s in re.rule.stop_lines() {
                if map.stop_line(s).is_none() {
                    self.push(
                        Issue::error(
                            codes::MISSING_REFERENCE,
                            format!("{owner} references {s}, which does not exist"),
                        )
                        .with_entity(owner)
                        .with_related(s),
                    );
                }
            }
            for &s in re.rule.signals() {
                if map.traffic_signal(s).is_none() {
                    self.push(
                        Issue::error(
                            codes::MISSING_REFERENCE,
                            format!("{owner} references {s}, which does not exist"),
                        )
                        .with_entity(owner)
                        .with_related(s),
                    );
                }
            }
            if let Some(c) = re.rule.crosswalk()
                && map.crosswalk(c).is_none()
            {
                self.push(
                    Issue::error(
                        codes::MISSING_REFERENCE,
                        format!("{owner} references {c}, which does not exist"),
                    )
                    .with_entity(owner)
                    .with_related(c),
                );
            }
            match &re.rule {
                Rule::TrafficLight { signals, .. } if signals.is_empty() => {
                    self.push(
                        Issue::error(
                            codes::INCOMPLETE_RULE,
                            format!("traffic light {owner} has no signals"),
                        )
                        .with_entity(owner),
                    );
                }
                Rule::RightOfWay {
                    priority, yielding, ..
                } if priority.is_empty() || yielding.is_empty() => {
                    self.push(
                        Issue::warning(
                            codes::INCOMPLETE_RULE,
                            format!("right-of-way rule {owner} needs priority and yielding lanes"),
                        )
                        .with_entity(owner),
                    );
                }
                Rule::TrafficSign { sign: Some(g), .. } => {
                    self.polyline(owner, "sign geometry", g);
                }
                _ => {}
            }
        }
    }

    fn check_usage(&mut self) {
        if !self.options.include_info {
            return;
        }
        let map = self.map;
        let used: BTreeSet<_> = map
            .lanes()
            .flat_map(|l| [l.left.boundary, l.right.boundary])
            .collect();
        for b in map.boundaries() {
            if !used.contains(&b.id) {
                self.push(
                    Issue::info(
                        codes::UNUSED_BOUNDARY,
                        format!("{} is not used by any lane", b.id),
                    )
                    .with_entity(b.id)
                    .with_fix(Command::RemoveEntity {
                        entity: b.id.into(),
                    }),
                );
            }
        }
        let rules: Vec<&Rule> = map.regulatory_elements().map(|r| &r.rule).collect();
        for s in map.stop_lines() {
            if !rules.iter().any(|r| r.stop_lines().contains(&s.id)) {
                self.push(
                    Issue::info(
                        codes::UNUSED_FEATURE,
                        format!("{} is not referenced by any rule", s.id),
                    )
                    .with_entity(s.id),
                );
            }
        }
        for s in map.traffic_signals() {
            if !rules.iter().any(|r| r.signals().contains(&s.id)) {
                self.push(
                    Issue::info(
                        codes::UNUSED_FEATURE,
                        format!("{} is not referenced by any rule", s.id),
                    )
                    .with_entity(s.id),
                );
            }
        }
        for c in map.crosswalks() {
            if !rules.iter().any(|r| r.crosswalk() == Some(c.id)) {
                self.push(
                    Issue::info(
                        codes::UNUSED_FEATURE,
                        format!("{} is not referenced by any rule", c.id),
                    )
                    .with_entity(c.id),
                );
            }
        }
    }

    fn check_id_collisions(&mut self) {
        let mut by_raw: BTreeMap<u64, Vec<EntityRef>> = BTreeMap::new();
        for e in self.map.entity_refs() {
            by_raw.entry(e.raw()).or_default().push(e);
        }
        for (raw, refs) in by_raw {
            if refs.len() > 1 {
                let mut issue = Issue::warning(
                    codes::DUPLICATE_ID,
                    format!(
                        "id {raw} is used by {} entities of different kinds; exporters with a \
                         global ID space (Lanelet2) will renumber some of them",
                        refs.len()
                    ),
                )
                .with_entity(refs[0]);
                issue.related = refs[1..].to_vec();
                self.push(issue);
            }
        }
    }

    fn check_connectivity(&mut self) {
        let map = self.map;
        let topo = map.topology();
        let lanes: Vec<LaneId> = map.lanes().map(|l| l.id).collect();
        if lanes.len() < 2 {
            return;
        }
        for &l in &lanes {
            if topo.predecessors(l).is_empty() && topo.successors(l).is_empty() {
                self.push(
                    Issue::warning(
                        codes::ISOLATED_LANE,
                        format!("{l} has neither predecessors nor successors"),
                    )
                    .with_entity(l),
                );
            }
        }
        // Weakly connected components over links and neighbor relations.
        let index: BTreeMap<LaneId, usize> =
            lanes.iter().enumerate().map(|(i, l)| (*l, i)).collect();
        let mut parent: Vec<usize> = (0..lanes.len()).collect();
        fn find(parent: &mut [usize], mut x: usize) -> usize {
            while parent[x] != x {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            x
        }
        for (i, &l) in lanes.iter().enumerate() {
            let others = topo.successors(l).iter().copied().chain(
                [Side::Left, Side::Right]
                    .into_iter()
                    .filter_map(|s| topo.neighbor(l, s).map(|n| n.lane)),
            );
            for o in others {
                if let Some(&j) = index.get(&o) {
                    let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                    if a != b {
                        parent[a.max(b)] = a.min(b);
                    }
                }
            }
        }
        let mut components: BTreeMap<usize, Vec<LaneId>> = BTreeMap::new();
        for (i, &l) in lanes.iter().enumerate() {
            let root = find(&mut parent, i);
            components.entry(root).or_default().push(l);
        }
        if components.len() <= 1 {
            return;
        }
        let mut comps: Vec<Vec<LaneId>> = components.into_values().collect();
        // Largest first, then by smallest lane ID.
        comps.sort_by_key(|c| (Reverse(c.len()), c[0]));
        let total = comps.len();
        for comp in comps.iter().skip(1) {
            let mut issue = Issue::warning(
                codes::DISCONNECTED_TOPOLOGY,
                format!(
                    "the lane graph has {total} unconnected parts; this part has {} lane(s) and \
                     is not connected to the main part",
                    comp.len()
                ),
            )
            .with_entity(comp[0]);
            issue.related = comp.iter().skip(1).take(10).map(|&l| l.into()).collect();
            self.push(issue);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vectormap_core::samples;
    use vectormap_core::{MapDocument, Point3, SpeedLimit, StopLineId};

    fn codes_of(report: &ValidationReport) -> Vec<&str> {
        report.issues.iter().map(|i| i.code.as_str()).collect()
    }

    #[test]
    fn samples_are_valid() {
        for map in [
            samples::straight_road().0,
            samples::two_lane_road().0,
            samples::intersection().0,
        ] {
            let report = validate(&map, &ValidationOptions::default());
            assert!(report.is_clean(), "{:#?}", report.issues);
        }
    }

    /// Corrupts a sample map through its JSON document, which is how broken
    /// data reaches the library in practice.
    fn corrupt(f: impl FnOnce(&mut MapDocument)) -> Map {
        let (map, _) = samples::two_lane_road();
        let mut doc = map.to_document();
        f(&mut doc);
        doc.into_map().unwrap().0
    }

    #[test]
    fn detects_dangling_and_asymmetric_links() {
        let map = corrupt(|doc| {
            let first = doc.topology[0].lane;
            doc.topology[0].links.successors.push(LaneId(9999));
            // Remove the reverse half of an existing link.
            let succ = doc.topology[0].links.successors[0];
            for e in &mut doc.topology {
                if e.lane == succ {
                    e.links.predecessors.retain(|p| *p != first);
                }
            }
        });
        let report = validate(&map, &ValidationOptions::default());
        let codes = codes_of(&report);
        assert!(codes.contains(&"dangling_lane_reference"), "{codes:?}");
        assert!(codes.contains(&"asymmetric_link"), "{codes:?}");
        let fix = report
            .with_code("asymmetric_link")
            .next()
            .unwrap()
            .fix
            .clone()
            .unwrap();
        // Applying the suggested fix repairs the link.
        let mut fixed = map.clone();
        fixed.apply(&fix).unwrap();
        let after = validate(&fixed, &ValidationOptions::default());
        assert_eq!(after.with_code("asymmetric_link").count(), 0);
    }

    #[test]
    fn detects_missing_boundary_and_bad_geometry() {
        let map = corrupt(|doc| {
            doc.lanes[0].left.boundary = vectormap_core::BoundaryId(777);
            doc.boundaries[1].geometry.points.truncate(1);
            doc.boundaries[2].geometry.points.clear();
            doc.boundaries[3].geometry.points[0] = Point3::new(f64::NAN, 0.0, 0.0);
            let p0 = doc.boundaries[4].geometry.points[0];
            doc.boundaries[4].geometry.points.insert(0, p0);
        });
        let report = validate(&map, &ValidationOptions::default());
        let codes = codes_of(&report);
        for expected in [
            "missing_boundary",
            "degenerate_geometry",
            "empty_geometry",
            "non_finite_coordinate",
        ] {
            assert!(codes.contains(&expected), "{expected} not in {codes:?}");
        }
        assert!(report.has_errors());
    }

    #[test]
    fn detects_invalid_neighbors() {
        let map = corrupt(|doc| {
            let lane = doc.topology[0].lane;
            doc.topology[0].links.left = Some(vectormap_core::Neighbor::same(LaneId(4242)));
            doc.topology[1].links.right = Some(vectormap_core::Neighbor::same(lane));
        });
        let report = validate(&map, &ValidationOptions::default());
        assert!(
            report.with_code("invalid_neighbor").count() >= 2,
            "{:#?}",
            report.issues
        );
    }

    #[test]
    fn detects_disconnected_topology_and_isolated_lanes() {
        let map = corrupt(|doc| doc.topology.clear());
        let report = validate(&map, &ValidationOptions::default());
        assert_eq!(report.with_code("isolated_lane").count(), 4);
        assert_eq!(report.with_code("disconnected_topology").count(), 3);
    }

    #[test]
    fn detects_duplicate_ids_from_documents() {
        let (map, _) = samples::straight_road();
        let mut doc = map.to_document();
        let dup = doc.lanes[0].clone();
        doc.lanes.push(dup);
        let (map, load_issues) = doc.into_map().unwrap();
        let report = validate(&map, &ValidationOptions::default()).merge(load_issues);
        assert_eq!(report.with_code("duplicate_id").count(), 1);
        assert!(report.has_errors());
    }

    #[test]
    fn detects_rule_problems() {
        let map = corrupt(|doc| {
            doc.regulatory_elements
                .push(vectormap_core::RegulatoryElement {
                    id: vectormap_core::RegulatoryElementId(500),
                    rule: Rule::TrafficLight {
                        signals: vec![],
                        stop_line: Some(StopLineId(501)),
                    },
                    lanes: vec![],
                    attributes: Default::default(),
                });
            doc.lanes[0].speed_limit = Some(SpeedLimit::from_kmh(0.0));
        });
        let report = validate(&map, &ValidationOptions::default());
        let codes = codes_of(&report);
        for expected in [
            "missing_reference",
            "incomplete_rule",
            "orphan_rule",
            "invalid_speed_limit",
        ] {
            assert!(codes.contains(&expected), "{expected} not in {codes:?}");
        }
    }

    #[test]
    fn report_serializes_as_structured_data() {
        let map = corrupt(|doc| doc.lanes[0].left.boundary = vectormap_core::BoundaryId(777));
        let report = validate(&map, &ValidationOptions::default());
        let json = serde_json::to_value(&report).unwrap();
        let first = &json["issues"][0];
        assert_eq!(first["severity"], "error");
        assert_eq!(first["code"], "missing_boundary");
        assert_eq!(first["entity"]["kind"], "lane");
        assert!(json["counts"]["errors"].as_u64().unwrap() >= 1);
    }
}
