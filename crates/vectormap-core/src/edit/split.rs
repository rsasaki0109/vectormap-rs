//! Splitting lanes.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use serde::{Deserialize, Serialize};

use super::{ChangeSet, EditError, EditResult};
use crate::diagnostics::{Issue, codes};
use crate::entities::{Boundary, BoundaryRef, Rule, Side};
use crate::geometry::{Point2, Point3, Polyline3};
use crate::id::{BoundaryId, LaneId};
use crate::map::Map;
use crate::topology::{Neighbor, NeighborDirection};

/// Minimum length of each piece produced by a split (metres).
pub const MIN_PIECE_LENGTH: f64 = 0.1;

/// Where to split a lane, measured along its centreline.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SplitAt {
    /// Fraction of the lane length in `(0, 1)`.
    Fraction(f64),
    /// Distance from the lane start in metres.
    Station(f64),
}

/// Options of [`Map::split_lane`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SplitOptions {
    /// Also split all lanes reachable through neighbor relations at the
    /// same cross-section, so shared boundaries and neighbor relations stay
    /// consistent. Default: `true`.
    #[serde(default = "default_true")]
    pub include_neighbors: bool,
}

fn default_true() -> bool {
    true
}

impl Default for SplitOptions {
    fn default() -> Self {
        Self {
            include_neighbors: true,
        }
    }
}

struct LanePlan {
    id: LaneId,
    /// Station of the split on the lane's centreline.
    station: f64,
    /// Explicit centreline pieces, if the lane has one.
    centerline: Option<(Polyline3, Polyline3)>,
}

impl Map {
    /// Lanes reachable from `lane` through neighbor relations (including
    /// `lane`), in ID order.
    pub fn lateral_group(&self, lane: LaneId) -> Vec<LaneId> {
        let mut seen = BTreeSet::from([lane]);
        let mut queue = VecDeque::from([lane]);
        while let Some(l) = queue.pop_front() {
            for side in [Side::Left, Side::Right] {
                if let Some(n) = self.neighbor(l, side)
                    && self.lanes.contains_key(&n.lane)
                    && seen.insert(n.lane)
                {
                    queue.push_back(n.lane);
                }
            }
        }
        seen.into_iter().collect()
    }

    /// Splits a lane in two at the given position.
    ///
    /// The original lane keeps its ID and becomes the first piece; the
    /// returned ID is the second piece. With
    /// [`SplitOptions::include_neighbors`] (the default) every lane of the
    /// lateral group is split at the same cross-section, shared boundaries
    /// are split once, and neighbor relations are re-established between the
    /// corresponding pieces.
    ///
    /// Successors move to the second piece; rules with stop lines move to the
    /// piece that contains the stop line, other rules apply to both pieces.
    pub fn split_lane(
        &mut self,
        lane: LaneId,
        at: SplitAt,
        options: SplitOptions,
    ) -> EditResult<(LaneId, ChangeSet)> {
        // ---- validation ----
        self.require_lane(lane)?;
        let center = self
            .centerline(lane)
            .ok_or_else(|| EditError::geometry(format!("{lane} has no usable geometry")))?;
        let len = center.length();
        let station = match at {
            SplitAt::Fraction(f) => {
                if !(f > 0.0 && f < 1.0) {
                    return Err(EditError::invalid("at", "fraction must be in (0, 1)"));
                }
                f * len
            }
            SplitAt::Station(s) => s,
        };
        if !(station > MIN_PIECE_LENGTH && station < len - MIN_PIECE_LENGTH) {
            return Err(EditError::invalid(
                "at",
                format!(
                    "split position {station:.3} m must be inside (0, {len:.3}) with at least \
                     {MIN_PIECE_LENGTH} m on each side"
                ),
            ));
        }
        let split_point = center.point_at(station).expect("non-empty").xy();

        let group = if options.include_neighbors {
            self.lateral_group(lane)
        } else {
            vec![lane]
        };

        let mut plans = Vec::with_capacity(group.len());
        for &g in &group {
            let c = self
                .centerline(g)
                .ok_or_else(|| EditError::geometry(format!("{g} has no usable geometry")))?;
            let s = if g == lane {
                station
            } else {
                c.nearest_point(split_point).expect("non-empty").station
            };
            if !(s > MIN_PIECE_LENGTH && s < c.length() - MIN_PIECE_LENGTH) {
                return Err(EditError::invalid(
                    "at",
                    format!(
                        "neighbor {g} cannot be split at the same cross-section; \
                         use include_neighbors = false"
                    ),
                ));
            }
            let explicit = self.lanes[&g].centerline.as_ref().map(|c| {
                let s = c.nearest_point(split_point).expect("non-empty").station;
                c.split_at(s)
            });
            plans.push(LanePlan {
                id: g,
                station: s,
                centerline: explicit,
            });
        }

        // Boundary split stations, in each boundary's own orientation.
        let mut boundary_splits: BTreeMap<BoundaryId, f64> = BTreeMap::new();
        for &g in &group {
            for side in [Side::Left, Side::Right] {
                let b = self.lanes[&g].boundary(side).boundary;
                if boundary_splits.contains_key(&b) {
                    continue;
                }
                let geom = &self
                    .boundaries
                    .get(&b)
                    .ok_or(EditError::not_found(b))?
                    .geometry;
                let n = geom
                    .nearest_point(split_point)
                    .ok_or_else(|| EditError::geometry(format!("{b} is empty")))?;
                if !(n.station > 1e-6 && n.station < geom.length() - 1e-6) {
                    return Err(EditError::geometry(format!(
                        "{b} does not extend across the split position"
                    )));
                }
                boundary_splits.insert(b, n.station);
            }
        }

        // Snapshot of the state needed for re-linking.
        let mut neighbors: BTreeMap<(LaneId, Side), Neighbor> = BTreeMap::new();
        for &g in &group {
            for side in [Side::Left, Side::Right] {
                if let Some(n) = self.neighbor(g, side) {
                    neighbors.insert((g, side), n);
                }
            }
        }

        // ---- mutation ----
        let mut cs = ChangeSet::new();
        let group_set: BTreeSet<LaneId> = group.iter().copied().collect();

        // 1. Boundaries.
        let mut pieces: BTreeMap<BoundaryId, (BoundaryId, BoundaryId)> = BTreeMap::new();
        for (&b, &s) in &boundary_splits {
            let original = self.boundaries[&b].clone();
            let (first, second) = original.geometry.split_at(s);
            let exclusive = self
                .lanes_using_boundary(b)
                .iter()
                .all(|l| group_set.contains(l));
            let new_piece = |map: &mut Map, geometry: Polyline3| {
                let id = map.allocate_boundary_id();
                let mut nb = Boundary::new(id, original.kind, geometry);
                nb.attributes = original.attributes.clone();
                map.boundaries.insert(id, nb);
                id
            };
            if exclusive {
                self.boundaries.get_mut(&b).expect("exists").geometry = first;
                let second_id = new_piece(self, second);
                cs.modified(b);
                cs.created(second_id);
                pieces.insert(b, (b, second_id));
            } else {
                let first_id = new_piece(self, first);
                let second_id = new_piece(self, second);
                cs.created(first_id);
                cs.created(second_id);
                cs.warn(
                    Issue::warning(
                        codes::BOUNDARY_UNSHARED,
                        format!(
                            "{b} is also used by lanes outside the split group; the split lanes \
                             now use copies ({first_id}, {second_id})"
                        ),
                    )
                    .with_entity(b)
                    .with_related(first_id)
                    .with_related(second_id),
                );
                pieces.insert(b, (first_id, second_id));
            }
        }

        // 2. Lanes.
        let mut second_of: BTreeMap<LaneId, LaneId> = BTreeMap::new();
        for plan in &plans {
            let new_id = self.allocate_lane_id();
            let mut first = self.lanes[&plan.id].clone();
            let mut second = first.clone();
            second.id = new_id;
            for side in [Side::Left, Side::Right] {
                let r = first.boundary(side);
                let (p1, p2) = pieces[&r.boundary];
                let (a, b) = if r.reversed { (p2, p1) } else { (p1, p2) };
                *first.boundary_mut(side) = BoundaryRef {
                    boundary: a,
                    reversed: r.reversed,
                };
                *second.boundary_mut(side) = BoundaryRef {
                    boundary: b,
                    reversed: r.reversed,
                };
            }
            if let Some((c1, c2)) = &plan.centerline {
                first.centerline = Some(c1.clone());
                second.centerline = Some(c2.clone());
            }
            self.lanes.insert(plan.id, first);
            self.lanes.insert(new_id, second);
            second_of.insert(plan.id, new_id);
            cs.modified(plan.id);
            cs.created(new_id);
        }

        // 3. Longitudinal topology.
        for plan in &plans {
            let second = second_of[&plan.id];
            for succ in self.successors(plan.id).to_vec() {
                self.topology.remove_link(plan.id, succ);
                self.topology.add_link(second, succ);
                cs.modified(succ);
            }
            self.topology.add_link(plan.id, second);
        }

        // 4. Lateral topology.
        for (&(g, side), &n) in &neighbors {
            let g2 = second_of[&g];
            match second_of.get(&n.lane) {
                Some(&n2) => match n.direction {
                    NeighborDirection::Same => {
                        self.topology.set_neighbor(g, side, Some(n));
                        self.topology
                            .set_neighbor(g2, side, Some(Neighbor::same(n2)));
                    }
                    NeighborDirection::Opposite => {
                        self.topology
                            .set_neighbor(g, side, Some(Neighbor::opposite(n2)));
                        self.topology
                            .set_neighbor(g2, side, Some(Neighbor::opposite(n.lane)));
                    }
                },
                None => {
                    cs.warn(
                        Issue::warning(
                            codes::NEIGHBOR_DROPPED,
                            format!(
                                "{} was not split, so it stays the {side:?} neighbor of {g} only; \
                                 {g2} has no {side:?} neighbor",
                                n.lane
                            ),
                        )
                        .with_entity(g2)
                        .with_related(n.lane),
                    );
                }
            }
        }

        // 5. Roads and junctions.
        for road in self.roads.values_mut() {
            if insert_after(&mut road.lanes, &second_of) {
                cs.modified(road.id);
            }
        }
        for j in self.junctions.values_mut() {
            if insert_after(&mut j.lanes, &second_of) {
                cs.modified(j.id);
            }
        }

        // 6. Rules.
        let stations: BTreeMap<LaneId, f64> = plans.iter().map(|p| (p.id, p.station)).collect();
        let rule_ids: Vec<_> = self.regulatory_elements.keys().copied().collect();
        for re_id in rule_ids {
            let re = &self.regulatory_elements[&re_id];
            let anchor = self.rule_anchor(&re.rule);
            let mut lanes = re.lanes.clone();
            let mut changed = false;
            for (&g, &g2) in &second_of {
                let Some(pos) = lanes.iter().position(|l| *l == g) else {
                    continue;
                };
                match anchor {
                    Some(p) => {
                        // Decide by where the anchor projects on the original lane.
                        let on_second = self
                            .centerline_before_split(g, g2)
                            .and_then(|c| c.nearest_point(p))
                            .is_some_and(|n| n.station >= stations[&g]);
                        if on_second {
                            lanes[pos] = g2;
                            changed = true;
                        }
                    }
                    None => {
                        lanes.insert(pos + 1, g2);
                        changed = true;
                    }
                }
            }
            let re = self.regulatory_elements.get_mut(&re_id).expect("exists");
            if let Rule::RightOfWay {
                priority, yielding, ..
            } = &mut re.rule
            {
                changed |= insert_after(priority, &second_of);
                changed |= insert_after(yielding, &second_of);
            }
            if changed {
                re.lanes = lanes;
                cs.modified(re_id);
            }
        }

        let result = second_of[&lane];
        Ok((result, cs.finish()))
    }

    /// Centreline of the lane formed by `first` followed by `second`.
    fn centerline_before_split(&self, first: LaneId, second: LaneId) -> Option<Polyline3> {
        Some(self.centerline(first)?.concat(&self.centerline(second)?))
    }

    /// A representative location of a rule (its stop line or crosswalk).
    pub(crate) fn rule_anchor(&self, rule: &Rule) -> Option<Point2> {
        let mid = |p: &Polyline3| p.point_at_fraction(0.5).map(Point3::xy);
        if let Some(sl) = rule.stop_lines().first() {
            return self.stop_lines.get(sl).and_then(|s| mid(&s.geometry));
        }
        if let Some(cw) = rule.crosswalk() {
            let c = self.crosswalks.get(&cw)?;
            let a = mid(&c.left_edge)?;
            let b = mid(&c.right_edge)?;
            return Some(a.lerp(b, 0.5));
        }
        None
    }
}

/// Inserts `map[x]` right after every `x` of `list` that is a key of `map`.
fn insert_after(list: &mut Vec<LaneId>, map: &BTreeMap<LaneId, LaneId>) -> bool {
    let mut out = Vec::with_capacity(list.len());
    let mut changed = false;
    for &l in list.iter() {
        out.push(l);
        if let Some(&n) = map.get(&l)
            && !list.contains(&n)
        {
            out.push(n);
            changed = true;
        }
    }
    *list = out;
    changed
}
