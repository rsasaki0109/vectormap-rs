//! Explicit equipment relationships; never infer control or synthesize geometry.
use super::{ChangeSet, EditError, EditResult};
use crate::{CrosswalkId, LaneId, Map, RegulatoryElementId, Rule, SignalKind, StopLineId};
use std::collections::BTreeSet;

fn unique<T: Ord + Copy>(values: &[T], name: &str) -> EditResult<Vec<T>> {
    let sorted: BTreeSet<_> = values.iter().copied().collect();
    if sorted.len() != values.len() {
        return Err(EditError::invalid(name, "contains duplicates"));
    }
    Ok(sorted.into_iter().collect())
}

impl Map {
    /// Update equipment associations without changing IDs, physical geometry,
    /// lamps, rule type or source attributes. Empty targets retain an unresolved
    /// rule; validation reports it as orphaned. All checks precede mutation.
    pub fn set_regulatory_links(
        &mut self,
        id: RegulatoryElementId,
        lanes: &[LaneId],
        controlled_crosswalks: &[CrosswalkId],
        stop_lines: &[StopLineId],
    ) -> EditResult<ChangeSet> {
        let mut edited = self
            .regulatory_element(id)
            .ok_or_else(|| EditError::not_found(id))?
            .clone();
        let lanes = unique(lanes, "lanes")?;
        let crosswalks = unique(controlled_crosswalks, "controlled_crosswalks")?;
        let stops = unique(stop_lines, "stop_lines")?;
        for &lane in &lanes {
            self.require_lane(lane)?;
        }
        for &crosswalk in &crosswalks {
            if self.crosswalk(crosswalk).is_none() {
                return Err(EditError::not_found(crosswalk));
            }
        }
        for &stop in &stops {
            if self.stop_line(stop).is_none() {
                return Err(EditError::not_found(stop));
            }
        }
        match &mut edited.rule {
            Rule::TrafficLight { signals, stop_line } => {
                if signals.is_empty() {
                    return Err(EditError::invalid("signals", "rule has no signals"));
                }
                let mut pedestrian = false;
                let mut vehicle = false;
                for &signal in signals.iter() {
                    let signal = self
                        .traffic_signal(signal)
                        .ok_or_else(|| EditError::not_found(signal))?;
                    match signal.kind {
                        SignalKind::Pedestrian => pedestrian = true,
                        SignalKind::Vehicle => vehicle = true,
                    }
                }
                if pedestrian && vehicle {
                    return Err(EditError::invalid(
                        "signals",
                        "mixed vehicle and pedestrian group",
                    ));
                }
                if pedestrian && (!lanes.is_empty() || !stops.is_empty()) {
                    return Err(EditError::invalid(
                        "targets",
                        "pedestrian lights control crosswalks, not vehicle lanes or stop lines",
                    ));
                }
                if vehicle && !crosswalks.is_empty() {
                    return Err(EditError::invalid(
                        "controlled_crosswalks",
                        "vehicle lights control lanes",
                    ));
                }
                if stops.len() > 1 {
                    return Err(EditError::invalid(
                        "stop_lines",
                        "traffic light accepts at most one",
                    ));
                }
                *stop_line = stops.first().copied();
            }
            Rule::Crosswalk { stop_lines, .. } => {
                if !crosswalks.is_empty() {
                    return Err(EditError::invalid(
                        "controlled_crosswalks",
                        "only pedestrian traffic lights control crosswalks",
                    ));
                }
                *stop_lines = stops;
            }
            Rule::StopLine { stop_line } => {
                if !crosswalks.is_empty() || stops != vec![*stop_line] {
                    return Err(EditError::invalid(
                        "targets",
                        "stop marking keeps its physical stop-line reference",
                    ));
                }
            }
            _ => {
                return Err(EditError::invalid(
                    "rule",
                    "only traffic-light, crosswalk and stop-marking associations can be edited",
                ));
            }
        }
        edited.lanes = lanes;
        edited.controlled_crosswalks = crosswalks;
        let mut changes = ChangeSet::new();
        if self.regulatory_element(id) != Some(&edited) {
            *self.regulatory_element_mut(id).expect("checked rule") = edited;
            changes.modified(id);
        }
        Ok(changes)
    }
}
