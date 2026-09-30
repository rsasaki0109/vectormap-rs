//! Merging consecutive lanes.

use super::{ChangeSet, EditError, EditResult};
use crate::diagnostics::{Issue, codes};
use crate::entities::{Boundary, BoundaryRef, Rule, Side};
use crate::geometry::Polyline3;
use crate::id::LaneId;
use crate::map::Map;

enum SidePlan {
    /// Both lanes use the same boundary; nothing to do.
    Same,
    /// Extend the first lane's boundary in place and delete the second's.
    InPlace { merged: Polyline3 },
    /// Create a new boundary for the merged lane.
    New { merged: Polyline3 },
}

impl Map {
    /// Merges `second` into `first`. `second` must be the only successor of
    /// `first` and `first` the only predecessor of `second`.
    ///
    /// `first` keeps its ID, attributes and neighbors and takes over the
    /// successors of `second`; `second` is deleted. Boundaries used only by
    /// the two lanes are concatenated in place; shared boundaries are copied.
    pub fn merge_lanes(&mut self, first: LaneId, second: LaneId) -> EditResult<ChangeSet> {
        // ---- validation ----
        self.require_lane(first)?;
        self.require_lane(second)?;
        let not_mergeable = |reason: String| EditError::NotMergeable {
            first,
            second,
            reason,
        };
        if first == second {
            return Err(not_mergeable("a lane cannot be merged with itself".into()));
        }
        if self.successors(first) != [second] {
            return Err(not_mergeable(format!(
                "{second} must be the only successor of {first} (successors: {:?})",
                self.successors(first)
            )));
        }
        if self.predecessors(second) != [first] {
            return Err(not_mergeable(format!(
                "{first} must be the only predecessor of {second} (predecessors: {:?})",
                self.predecessors(second)
            )));
        }
        let (a, b) = (self.lanes[&first].clone(), self.lanes[&second].clone());
        let mut plans = Vec::new();
        for side in [Side::Left, Side::Right] {
            let (ra, rb) = (a.boundary(side), b.boundary(side));
            if ra.boundary == rb.boundary {
                plans.push((side, SidePlan::Same));
                continue;
            }
            let ga = self
                .oriented_boundary(first, side)
                .ok_or(EditError::not_found(ra.boundary))?;
            let gb = self
                .oriented_boundary(second, side)
                .ok_or(EditError::not_found(rb.boundary))?;
            let merged = ga.concat(&gb);
            let exclusive = self.lanes_using_boundary(ra.boundary) == [first]
                && self.lanes_using_boundary(rb.boundary) == [second];
            plans.push((
                side,
                if exclusive {
                    SidePlan::InPlace { merged }
                } else {
                    SidePlan::New { merged }
                },
            ));
        }

        // ---- mutation ----
        let mut cs = ChangeSet::new();
        if a.kind != b.kind
            || a.speed_limit != b.speed_limit
            || a.turn_direction != b.turn_direction
        {
            cs.warn(
                Issue::warning(
                    codes::ATTRIBUTE_CONFLICT,
                    format!(
                        "{first} and {second} differ in kind, speed limit or turn direction; \
                         the values of {first} were kept"
                    ),
                )
                .with_entity(first)
                .with_related(second),
            );
        }
        for side in [Side::Left, Side::Right] {
            if let Some(n) = self.neighbor(second, side)
                && self.neighbor(first, side).map(|m| m.lane) != Some(n.lane)
            {
                cs.warn(
                    Issue::warning(
                        codes::NEIGHBOR_DROPPED,
                        format!("{side:?} neighbor {} of {second} was dropped", n.lane),
                    )
                    .with_entity(first)
                    .with_related(n.lane),
                );
            }
        }

        let mut merged_lane = a.clone();
        for (side, plan) in plans {
            let (ra, rb) = (a.boundary(side), b.boundary(side));
            match plan {
                SidePlan::Same => {}
                SidePlan::InPlace { merged } => {
                    let geometry = if ra.reversed {
                        merged.reversed()
                    } else {
                        merged
                    };
                    self.boundaries
                        .get_mut(&ra.boundary)
                        .expect("exists")
                        .geometry = geometry;
                    cs.modified(ra.boundary);
                }
                SidePlan::New { merged } => {
                    let id = self.allocate_boundary_id();
                    let mut nb = Boundary::new(id, self.boundaries[&ra.boundary].kind, merged);
                    nb.attributes = self.boundaries[&ra.boundary].attributes.clone();
                    self.boundaries.insert(id, nb);
                    *merged_lane.boundary_mut(side) = BoundaryRef::forward(id);
                    cs.created(id);
                    cs.warn(
                        Issue::warning(
                            codes::BOUNDARY_UNSHARED,
                            format!(
                                "{} or {} is shared with other lanes; the merged lane uses a new \
                                 boundary {id}",
                                ra.boundary, rb.boundary
                            ),
                        )
                        .with_entity(id),
                    );
                }
            }
        }
        merged_lane.centerline = match (&a.centerline, &b.centerline) {
            (Some(ca), Some(cb)) => Some(ca.concat(cb)),
            _ => None,
        };
        self.lanes.insert(first, merged_lane);
        cs.modified(first);

        // Topology: take over successors of `second`, then drop `second`.
        let succs = self.successors(second).to_vec();
        for other in self.topology.remove_lane(second) {
            if other != first {
                cs.modified(other);
            }
        }
        for s in succs {
            self.topology.add_link(first, s);
            cs.modified(s);
        }
        self.lanes.remove(&second);
        cs.deleted(second);

        for road in self.roads.values_mut() {
            if road.lanes.contains(&second) {
                road.lanes.retain(|l| *l != second);
                cs.modified(road.id);
            }
        }
        for j in self.junctions.values_mut() {
            if j.lanes.contains(&second) {
                j.lanes.retain(|l| *l != second);
                cs.modified(j.id);
            }
        }
        for re in self.regulatory_elements.values_mut() {
            let mut changed = replace_lane(&mut re.lanes, second, first);
            if let Rule::RightOfWay {
                priority, yielding, ..
            } = &mut re.rule
            {
                changed |= replace_lane(priority, second, first);
                changed |= replace_lane(yielding, second, first);
            }
            if changed {
                cs.modified(re.id);
            }
        }

        // Delete boundaries that no longer have users.
        for r in [a.left, a.right, b.left, b.right] {
            if self.boundaries.contains_key(&r.boundary)
                && self.lanes_using_boundary(r.boundary).is_empty()
            {
                self.boundaries.remove(&r.boundary);
                cs.deleted(r.boundary);
            }
        }
        Ok(cs.finish())
    }
}

/// Replaces `from` by `to` in `list`, avoiding duplicates.
fn replace_lane(list: &mut Vec<LaneId>, from: LaneId, to: LaneId) -> bool {
    if !list.contains(&from) {
        return false;
    }
    if list.contains(&to) {
        list.retain(|l| *l != from);
    } else {
        for l in list.iter_mut() {
            if *l == from {
                *l = to;
            }
        }
    }
    true
}
