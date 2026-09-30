//! Lane topology, stored separately from geometry.
//!
//! The [`Topology`] store keeps, for every lane, its predecessors, successors
//! and left/right neighbors. Both directions of every relation are stored so
//! that queries are O(1) and inconsistencies in hand-written data can be
//! detected by the validator. The editing API on [`Map`] keeps
//! both directions in sync.
//!
//! [`infer_topology`] derives a topology from geometry, which is how formats
//! with implicit topology (e.g. Lanelet2) are imported.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::diagnostics::{Issue, codes};
use crate::entities::Side;
use crate::geometry::Point3;
use crate::id::LaneId;
use crate::map::Map;

/// Whether a neighboring lane runs in the same or the opposite direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NeighborDirection {
    /// Same direction of travel.
    #[default]
    Same,
    /// Opposite direction of travel.
    Opposite,
}

fn is_same(d: &NeighborDirection) -> bool {
    *d == NeighborDirection::Same
}

/// A lateral neighbor of a lane.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Neighbor {
    /// The neighboring lane.
    pub lane: LaneId,
    /// Its direction relative to the lane.
    #[serde(default, skip_serializing_if = "is_same")]
    pub direction: NeighborDirection,
}

impl Neighbor {
    /// Neighbor with the same direction of travel.
    pub const fn same(lane: LaneId) -> Self {
        Self {
            lane,
            direction: NeighborDirection::Same,
        }
    }

    /// Neighbor with the opposite direction of travel.
    pub const fn opposite(lane: LaneId) -> Self {
        Self {
            lane,
            direction: NeighborDirection::Opposite,
        }
    }
}

/// On which side of `neighbor` the original lane appears, given that
/// `neighbor` is on `side` of it with `direction`.
pub const fn reciprocal_side(side: Side, direction: NeighborDirection) -> Side {
    match direction {
        NeighborDirection::Same => side.opposite(),
        NeighborDirection::Opposite => side,
    }
}

/// Links of a single lane.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct LaneLinks {
    /// Lanes leading into this lane (sorted, unique).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub predecessors: Vec<LaneId>,
    /// Lanes this lane leads into (sorted, unique).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub successors: Vec<LaneId>,
    /// Left neighbor.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub left: Option<Neighbor>,
    /// Right neighbor.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub right: Option<Neighbor>,
}

impl LaneLinks {
    /// `true` if the lane has no links at all.
    pub fn is_empty(&self) -> bool {
        self.predecessors.is_empty()
            && self.successors.is_empty()
            && self.left.is_none()
            && self.right.is_none()
    }

    /// Neighbor on `side`.
    pub const fn neighbor(&self, side: Side) -> Option<Neighbor> {
        match side {
            Side::Left => self.left,
            Side::Right => self.right,
        }
    }

    fn neighbor_mut(&mut self, side: Side) -> &mut Option<Neighbor> {
        match side {
            Side::Left => &mut self.left,
            Side::Right => &mut self.right,
        }
    }
}

fn insert_sorted(v: &mut Vec<LaneId>, id: LaneId) -> bool {
    match v.binary_search(&id) {
        Ok(_) => false,
        Err(pos) => {
            v.insert(pos, id);
            true
        }
    }
}

fn remove_sorted(v: &mut Vec<LaneId>, id: LaneId) -> bool {
    match v.iter().position(|x| *x == id) {
        Some(pos) => {
            v.remove(pos);
            true
        }
        None => false,
    }
}

/// Lane connectivity graph.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Topology {
    pub(crate) links: BTreeMap<LaneId, LaneLinks>,
}

impl Topology {
    /// Creates an empty topology.
    pub fn new() -> Self {
        Self::default()
    }

    /// Links of `lane`, if it has any.
    pub fn links(&self, lane: LaneId) -> Option<&LaneLinks> {
        self.links.get(&lane)
    }

    /// Iterates over all lanes that have links, in ID order.
    pub fn iter(&self) -> impl Iterator<Item = (LaneId, &LaneLinks)> {
        self.links.iter().map(|(k, v)| (*k, v))
    }

    /// `true` if no lane has any link.
    pub fn is_empty(&self) -> bool {
        self.links.values().all(LaneLinks::is_empty)
    }

    /// Successors of `lane`.
    pub fn successors(&self, lane: LaneId) -> &[LaneId] {
        self.links.get(&lane).map_or(&[], |l| &l.successors)
    }

    /// Predecessors of `lane`.
    pub fn predecessors(&self, lane: LaneId) -> &[LaneId] {
        self.links.get(&lane).map_or(&[], |l| &l.predecessors)
    }

    /// Neighbor of `lane` on `side`.
    pub fn neighbor(&self, lane: LaneId, side: Side) -> Option<Neighbor> {
        self.links.get(&lane).and_then(|l| l.neighbor(side))
    }

    fn entry(&mut self, lane: LaneId) -> &mut LaneLinks {
        self.links.entry(lane).or_default()
    }

    /// Adds the link `from → to` on both sides. Returns `true` if anything
    /// changed. Repairs half-present links.
    pub(crate) fn add_link(&mut self, from: LaneId, to: LaneId) -> bool {
        let a = insert_sorted(&mut self.entry(from).successors, to);
        let b = insert_sorted(&mut self.entry(to).predecessors, from);
        a || b
    }

    /// Removes the link `from → to` on both sides. Returns `true` if anything
    /// changed.
    pub(crate) fn remove_link(&mut self, from: LaneId, to: LaneId) -> bool {
        let a = self
            .links
            .get_mut(&from)
            .is_some_and(|l| remove_sorted(&mut l.successors, to));
        let b = self
            .links
            .get_mut(&to)
            .is_some_and(|l| remove_sorted(&mut l.predecessors, from));
        self.prune(from);
        self.prune(to);
        a || b
    }

    /// Sets (or clears) the neighbor of `lane` on `side`, keeping the
    /// reciprocal relation in sync. Returns the lanes whose links changed.
    pub(crate) fn set_neighbor(
        &mut self,
        lane: LaneId,
        side: Side,
        neighbor: Option<Neighbor>,
    ) -> Vec<LaneId> {
        let mut touched = Vec::new();
        let current = self.neighbor(lane, side);
        if current == neighbor {
            let consistent = match neighbor {
                None => true,
                Some(n) => {
                    self.neighbor(n.lane, reciprocal_side(side, n.direction))
                        == Some(Neighbor {
                            lane,
                            direction: n.direction,
                        })
                }
            };
            if consistent {
                return touched;
            }
        }
        // Clear the current neighbor on this side and its reciprocal.
        if current.is_some() {
            self.clear_slot(lane, side, &mut touched);
        }
        if let Some(n) = neighbor {
            let rside = reciprocal_side(side, n.direction);
            if self.neighbor(n.lane, rside).is_some() {
                self.clear_slot(n.lane, rside, &mut touched);
            }
            *self.entry(lane).neighbor_mut(side) = Some(n);
            *self.entry(n.lane).neighbor_mut(rside) = Some(Neighbor {
                lane,
                direction: n.direction,
            });
            touched.push(lane);
            touched.push(n.lane);
        }
        touched.sort();
        touched.dedup();
        touched
    }

    fn clear_slot(&mut self, lane: LaneId, side: Side, touched: &mut Vec<LaneId>) {
        let Some(old) = self
            .links
            .get_mut(&lane)
            .and_then(|l| l.neighbor_mut(side).take())
        else {
            return;
        };
        touched.push(lane);
        let rside = reciprocal_side(side, old.direction);
        if let Some(links) = self.links.get_mut(&old.lane) {
            let slot = links.neighbor_mut(rside);
            if slot.is_some_and(|n| n.lane == lane) {
                *slot = None;
                touched.push(old.lane);
            }
        }
        self.prune(lane);
        self.prune(old.lane);
    }

    /// Removes every link involving `lane`. Returns the other lanes whose
    /// links changed.
    pub(crate) fn remove_lane(&mut self, lane: LaneId) -> Vec<LaneId> {
        let Some(links) = self.links.remove(&lane) else {
            return Vec::new();
        };
        let mut touched = Vec::new();
        for p in &links.predecessors {
            if let Some(l) = self.links.get_mut(p) {
                remove_sorted(&mut l.successors, lane);
                touched.push(*p);
            }
        }
        for s in &links.successors {
            if let Some(l) = self.links.get_mut(s) {
                remove_sorted(&mut l.predecessors, lane);
                touched.push(*s);
            }
        }
        for side in [Side::Left, Side::Right] {
            if let Some(n) = links.neighbor(side) {
                let rside = reciprocal_side(side, n.direction);
                if let Some(l) = self.links.get_mut(&n.lane) {
                    let slot = l.neighbor_mut(rside);
                    if slot.is_some_and(|x| x.lane == lane) {
                        *slot = None;
                        touched.push(n.lane);
                    }
                }
            }
        }
        // Remove dangling back-references that are not reciprocated.
        for (id, l) in self.links.iter_mut() {
            let mut changed = remove_sorted(&mut l.successors, lane);
            changed |= remove_sorted(&mut l.predecessors, lane);
            for side in [Side::Left, Side::Right] {
                let slot = l.neighbor_mut(side);
                if slot.is_some_and(|n| n.lane == lane) {
                    *slot = None;
                    changed = true;
                }
            }
            if changed {
                touched.push(*id);
            }
        }
        touched.sort();
        touched.dedup();
        for id in &touched {
            self.prune(*id);
        }
        touched
    }

    /// Replaces the raw links of `lane` (used when importing documents).
    pub(crate) fn insert_raw(&mut self, lane: LaneId, links: LaneLinks) {
        if !links.is_empty() {
            self.links.insert(lane, links);
        }
    }

    fn prune(&mut self, lane: LaneId) {
        if self.links.get(&lane).is_some_and(LaneLinks::is_empty) {
            self.links.remove(&lane);
        }
    }
}

/// Options for [`infer_topology`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InferOptions {
    /// Maximum distance between end points that are considered identical
    /// (metres).
    pub tolerance: f64,
    /// Infer predecessor / successor links.
    pub links: bool,
    /// Infer left / right neighbors from shared boundaries.
    pub neighbors: bool,
}

impl Default for InferOptions {
    fn default() -> Self {
        Self {
            tolerance: 0.01,
            links: true,
            neighbors: true,
        }
    }
}

struct LaneEnds {
    start_left: Point3,
    start_right: Point3,
    end_left: Point3,
    end_right: Point3,
}

/// Derives lane topology from geometry.
///
/// - `a → b` if the end points of `a`'s left and right boundaries coincide
///   (within `tolerance`) with the start points of `b`'s boundaries.
/// - `a` and `b` are neighbors if they share a boundary: `a.left == b.right`
///   in the same orientation (same direction), or `a.left == b.left` in
///   opposite orientation (opposite direction), and symmetrically.
///
/// Ambiguities are resolved deterministically (smallest ID wins) and
/// reported as warnings.
pub fn infer_topology(map: &Map, options: InferOptions) -> (Topology, Vec<Issue>) {
    let mut topo = Topology::new();
    let mut issues = Vec::new();

    let ends: BTreeMap<LaneId, LaneEnds> = map
        .lanes()
        .filter_map(|lane| {
            let l = map.oriented_boundary(lane.id, Side::Left)?;
            let r = map.oriented_boundary(lane.id, Side::Right)?;
            Some((
                lane.id,
                LaneEnds {
                    start_left: l.first()?,
                    start_right: r.first()?,
                    end_left: l.last()?,
                    end_right: r.last()?,
                },
            ))
        })
        .collect();

    if options.links {
        let cell = options.tolerance.max(1e-6) * 2.0;
        let key = |p: Point3| ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64);
        let mut grid: BTreeMap<(i64, i64), Vec<LaneId>> = BTreeMap::new();
        for (id, e) in &ends {
            grid.entry(key(e.start_left)).or_default().push(*id);
        }
        for (a, ea) in &ends {
            let (cx, cy) = key(ea.end_left);
            let mut candidates: Vec<LaneId> = Vec::new();
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if let Some(ids) = grid.get(&(cx + dx, cy + dy)) {
                        candidates.extend(ids);
                    }
                }
            }
            candidates.sort();
            candidates.dedup();
            for b in candidates {
                if b == *a {
                    continue;
                }
                let eb = &ends[&b];
                if ea.end_left.distance(eb.start_left) <= options.tolerance
                    && ea.end_right.distance(eb.start_right) <= options.tolerance
                {
                    topo.add_link(*a, b);
                }
            }
        }
    }

    if options.neighbors {
        // boundary id -> [(lane, side, reversed)]
        let mut users: BTreeMap<_, Vec<(LaneId, Side, bool)>> = BTreeMap::new();
        for lane in map.lanes() {
            for side in [Side::Left, Side::Right] {
                let r = lane.boundary(side);
                users
                    .entry(r.boundary)
                    .or_default()
                    .push((lane.id, side, r.reversed));
            }
        }
        for lane in map.lanes() {
            for side in [Side::Left, Side::Right] {
                let r = lane.boundary(side);
                let mut candidates: Vec<Neighbor> = users[&r.boundary]
                    .iter()
                    .filter(|(other, _, _)| *other != lane.id)
                    .filter_map(|&(other, oside, orev)| {
                        if oside != side && orev == r.reversed {
                            Some(Neighbor::same(other))
                        } else if oside == side && orev != r.reversed {
                            Some(Neighbor::opposite(other))
                        } else {
                            None
                        }
                    })
                    .collect();
                candidates.sort_by_key(|n| n.lane);
                let Some(&chosen) = candidates.first() else {
                    continue;
                };
                if candidates.len() > 1 && lane.id < chosen.lane {
                    issues.push(
                        Issue::warning(
                            codes::AMBIGUOUS_TOPOLOGY,
                            format!(
                                "{} has {} candidate {:?} neighbors sharing {}; picked {}",
                                lane.id,
                                candidates.len(),
                                side,
                                r.boundary,
                                chosen.lane
                            ),
                        )
                        .with_entity(lane.id),
                    );
                }
                let rside = reciprocal_side(side, chosen.direction);
                let free_here = topo.neighbor(lane.id, side).is_none();
                let free_there = topo.neighbor(chosen.lane, rside).is_none();
                if free_here && free_there {
                    topo.set_neighbor(lane.id, side, Some(chosen));
                }
            }
        }
    }

    (topo, issues)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn links_are_symmetric() {
        let mut t = Topology::new();
        assert!(t.add_link(LaneId(1), LaneId(2)));
        assert!(!t.add_link(LaneId(1), LaneId(2)));
        assert_eq!(t.successors(LaneId(1)), &[LaneId(2)]);
        assert_eq!(t.predecessors(LaneId(2)), &[LaneId(1)]);
        assert!(t.remove_link(LaneId(1), LaneId(2)));
        assert!(t.is_empty());
        assert!(t.links.is_empty(), "empty entries are pruned");
    }

    #[test]
    fn neighbors_are_reciprocal() {
        let mut t = Topology::new();
        t.set_neighbor(LaneId(1), Side::Left, Some(Neighbor::same(LaneId(2))));
        assert_eq!(
            t.neighbor(LaneId(2), Side::Right),
            Some(Neighbor::same(LaneId(1)))
        );
        // Opposite neighbors see each other on the same side.
        t.set_neighbor(LaneId(2), Side::Left, Some(Neighbor::opposite(LaneId(3))));
        assert_eq!(
            t.neighbor(LaneId(3), Side::Left),
            Some(Neighbor::opposite(LaneId(2)))
        );
        // Replacing a neighbor clears the old reciprocal relation.
        let touched = t.set_neighbor(LaneId(1), Side::Left, Some(Neighbor::same(LaneId(4))));
        assert_eq!(touched, vec![LaneId(1), LaneId(2), LaneId(4)]);
        assert_eq!(t.neighbor(LaneId(2), Side::Right), None);
        assert_eq!(
            t.neighbor(LaneId(4), Side::Right),
            Some(Neighbor::same(LaneId(1)))
        );
        // Clearing.
        t.set_neighbor(LaneId(4), Side::Right, None);
        assert_eq!(t.neighbor(LaneId(1), Side::Left), None);
    }

    #[test]
    fn remove_lane_cleans_everything() {
        let mut t = Topology::new();
        t.add_link(LaneId(1), LaneId(2));
        t.add_link(LaneId(2), LaneId(3));
        t.set_neighbor(LaneId(2), Side::Left, Some(Neighbor::same(LaneId(5))));
        let touched = t.remove_lane(LaneId(2));
        assert_eq!(touched, vec![LaneId(1), LaneId(3), LaneId(5)]);
        assert!(t.is_empty());
    }
}
