//! Integration tests of the IR: topology, editing operations and commands.

use vectormap_core::prelude::*;
use vectormap_core::samples;
use vectormap_core::{
    CrosswalkGeometry, InferOptions, LaneGeometry, StopLinePlacement, StopRule, infer_topology,
};

fn assert_topology_inferable(map: &Map) {
    let (inferred, issues) = infer_topology(map, InferOptions::default());
    assert!(issues.is_empty(), "{issues:?}");
    assert_eq!(
        &inferred,
        map.topology(),
        "topology must be derivable from geometry"
    );
}

#[test]
fn straight_road_topology() {
    let (map, [a, b, c]) = samples::straight_road();
    assert_eq!(map.successors(a), &[b]);
    assert_eq!(map.successors(b), &[c]);
    assert_eq!(map.predecessors(c), &[b]);
    assert!(map.predecessors(a).is_empty());
    assert!((map.lane_length(a).unwrap() - 20.0).abs() < 1e-9);
    assert_eq!(
        map.lane(a).unwrap().speed_limit,
        Some(SpeedLimit::from_kmh(50.0))
    );
    assert_topology_inferable(&map);
    let summary = map.summary();
    assert_eq!(summary.counts.lanes, 3);
    assert_eq!(summary.counts.boundaries, 6);
    assert_eq!(summary.topology.links, 2);
    assert!((summary.total_lane_length - 60.0).abs() < 1e-9);
}

#[test]
fn two_lane_road_neighbors() {
    let (map, [a, b, c, d]) = samples::two_lane_road();
    assert_eq!(map.right_neighbor(a), Some(c));
    assert_eq!(map.left_neighbor(c), Some(a));
    assert_eq!(map.right_neighbor(b), Some(d));
    assert_eq!(map.successors(a), &[b]);
    assert_eq!(map.successors(c), &[d]);
    assert_eq!(map.left_neighbor(a), None);
    assert_topology_inferable(&map);
}

#[test]
fn intersection_structure() {
    let (map, lanes) = samples::intersection();
    let s = map.summary();
    assert_eq!(s.counts.lanes, 20);
    assert_eq!(s.counts.junctions, 1);
    assert_eq!(s.counts.stop_lines, 4);
    assert_eq!(s.counts.traffic_signals, 4);
    assert_eq!(s.counts.crosswalks, 1);
    assert_eq!(s.rule_types.get("traffic_light"), Some(&4));
    assert_eq!(s.rule_types.get("crosswalk"), Some(&1));
    for &inc in &lanes.incoming {
        assert_eq!(map.successors(inc).len(), 3);
        let info = map.lane_info(inc).unwrap();
        assert_eq!(info.traffic_signals.len(), 1);
        assert_eq!(info.stop_lines.len(), 1);
    }
    // Coming from the east arm, the connector to the north arm is a right turn.
    let east_in = lanes.incoming[0];
    let north_out = lanes.outgoing[1];
    let right_turn = map
        .successors(east_in)
        .iter()
        .copied()
        .find(|c| map.successors(*c) == [north_out])
        .unwrap();
    assert_eq!(
        map.lane(right_turn).unwrap().turn_direction,
        Some(TurnDirection::Right)
    );
    // The crosswalk rule applies to both lanes of the north arm.
    let cw_rule = map
        .regulatory_elements()
        .find(|r| matches!(r.rule, Rule::Crosswalk { .. }))
        .unwrap();
    let mut expected = vec![lanes.incoming[1], lanes.outgoing[1]];
    expected.sort();
    assert_eq!(cw_rule.lanes, expected);
    // Opposite neighbors across the centre line.
    let n = map.neighbor(east_in, Side::Left).unwrap();
    assert_eq!(n, Neighbor::opposite(lanes.outgoing[0]));
    assert_topology_inferable(&map);
}

#[test]
fn json_round_trip_is_lossless() {
    for map in [
        samples::straight_road().0,
        samples::two_lane_road().0,
        samples::intersection().0,
    ] {
        let json = serde_json::to_string_pretty(&map).unwrap();
        let back: Map = serde_json::from_str(&json).unwrap();
        assert_eq!(back, map);
        assert_eq!(serde_json::to_string_pretty(&back).unwrap(), json);
    }
}

#[test]
fn building_is_deterministic() {
    let a = serde_json::to_string(&samples::intersection().0).unwrap();
    let b = serde_json::to_string(&samples::intersection().0).unwrap();
    assert_eq!(a, b);
}

#[test]
fn split_lane_with_neighbors() {
    let (mut map, [a, b, c, d]) = samples::two_lane_road();
    let (a2, cs) = map
        .split_lane(a, SplitAt::Fraction(0.5), SplitOptions::default())
        .unwrap();
    assert_eq!(
        cs.created_ids(EntityKind::Lane).len(),
        2,
        "both lanes are split"
    );
    assert!(cs.warnings.is_empty(), "{:?}", cs.warnings);
    let c2 = map.successors(c)[0];
    assert_eq!(map.successors(a), &[a2]);
    assert_eq!(map.successors(a2), &[b]);
    assert_eq!(map.successors(c2), &[d]);
    assert_eq!(map.right_neighbor(a2), Some(c2));
    assert_eq!(map.right_neighbor(a), Some(c));
    assert!((map.lane_length(a).unwrap() - 15.0).abs() < 1e-9);
    assert!((map.lane_length(a2).unwrap() - 15.0).abs() < 1e-9);
    // The shared middle boundary was split once and is still shared.
    assert_eq!(
        map.lane(a2).unwrap().right.boundary,
        map.lane(c2).unwrap().left.boundary
    );
    assert_eq!(map.road_of(a2), map.road_of(a));
    assert_topology_inferable(&map);
}

#[test]
fn split_single_lane_reports_unshared_boundary() {
    let (mut map, [a, _, c, _]) = samples::two_lane_road();
    let (a2, cs) = map
        .split_lane(
            a,
            SplitAt::Station(10.0),
            SplitOptions {
                include_neighbors: false,
            },
        )
        .unwrap();
    let codes: Vec<&str> = cs.warnings.iter().map(|w| w.code.as_str()).collect();
    assert!(codes.contains(&"boundary_unshared"), "{codes:?}");
    assert!(codes.contains(&"neighbor_dropped"), "{codes:?}");
    assert_eq!(map.right_neighbor(a), Some(c));
    assert_eq!(map.right_neighbor(a2), None);
}

#[test]
fn split_opposite_neighbors_pairs_the_right_pieces() {
    let (mut map, lanes) = samples::intersection();
    let inc = lanes.incoming[0];
    let out = lanes.outgoing[0];
    let (inc2, _) = map
        .split_lane(inc, SplitAt::Fraction(0.5), SplitOptions::default())
        .unwrap();
    let out2 = map.successors(out)[0];
    // First half of the incoming lane is next to the second half of the
    // outgoing lane and vice versa.
    assert_eq!(
        map.neighbor(inc, Side::Left),
        Some(Neighbor::opposite(out2))
    );
    assert_eq!(
        map.neighbor(inc2, Side::Left),
        Some(Neighbor::opposite(out))
    );
    // The traffic light (whose stop line is near the end) moved to the
    // second half.
    assert!(map.lane_info(inc2).unwrap().traffic_signals.len() == 1);
    assert!(map.lane_info(inc).unwrap().traffic_signals.is_empty());
    assert_topology_inferable(&map);
}

#[test]
fn failed_operations_leave_the_map_unchanged() {
    let (mut map, [a, b, _]) = samples::straight_road();
    let before = map.clone();
    assert!(matches!(
        map.split_lane(a, SplitAt::Fraction(1.5), SplitOptions::default()),
        Err(EditError::InvalidArgument { .. })
    ));
    assert!(map.connect(a, a).is_err());
    assert!(matches!(
        map.connect(a, LaneId(999)),
        Err(EditError::NotFound { .. })
    ));
    assert!(matches!(
        map.merge_lanes(b, a),
        Err(EditError::NotMergeable { .. })
    ));
    assert!(
        map.set_speed_limit(&[a], Some(SpeedLimit::from_kmh(-5.0)))
            .is_err()
    );
    assert_eq!(map, before);
}

#[test]
fn merge_lanes_concatenates_geometry() {
    let (mut map, [a, b, c]) = samples::straight_road();
    let cs = map.merge_lanes(a, b).unwrap();
    assert_eq!(
        cs.deleted
            .iter()
            .filter(|e| e.kind() == EntityKind::Lane)
            .count(),
        1
    );
    assert!(map.lane(b).is_none());
    assert_eq!(map.successors(a), &[c]);
    assert!((map.lane_length(a).unwrap() - 40.0).abs() < 1e-9);
    assert_eq!(map.boundary_count(), 4);
    assert_eq!(map.road(map.road_of(a).unwrap()).unwrap().lanes, vec![a, c]);
    assert_topology_inferable(&map);
}

#[test]
fn split_then_merge_restores_lengths() {
    let (mut map, [a, b, _]) = samples::straight_road();
    let (a2, _) = map
        .split_lane(a, SplitAt::Station(7.0), SplitOptions::default())
        .unwrap();
    map.merge_lanes(a, a2).unwrap();
    assert_eq!(map.successors(a), &[b]);
    assert!((map.lane_length(a).unwrap() - 20.0).abs() < 1e-9);
    assert_eq!(map.boundary_count(), 6);
}

#[test]
fn remove_lane_cleans_references() {
    let (mut map, [a, b, c]) = samples::straight_road();
    let cs = map.remove_lane(b).unwrap();
    assert!(map.successors(a).is_empty());
    assert!(map.predecessors(c).is_empty());
    assert_eq!(map.boundary_count(), 4);
    assert!(cs.modified.contains(&EntityRef::Lane(a)));
    assert!(cs.modified.contains(&EntityRef::Lane(c)));
    assert_eq!(cs.deleted.len(), 3, "lane and its two boundaries: {cs:?}");
}

#[test]
fn stop_line_signal_and_crosswalk() {
    let (mut map, [_, b, c]) = samples::straight_road();
    let (sl, cs) = map
        .add_stop_line(NewStopLine {
            lanes: vec![c],
            placement: StopLinePlacement::AtLaneEnd { offset: 0.0 },
            rule: StopRule::StopSign,
        })
        .unwrap();
    assert_eq!(cs.created.len(), 2);
    let geom = &map.stop_line(sl).unwrap().geometry;
    assert_eq!(geom.points[0], Point3::new(60.0, 1.75, 0.0));
    assert_eq!(geom.points[1], Point3::new(60.0, -1.75, 0.0));

    // A traffic light on the same lane reuses the stop line and replaces the
    // stop sign.
    let (sig, cs) = map
        .add_traffic_signal(NewTrafficSignal::for_lanes(vec![c]))
        .unwrap();
    let codes: Vec<&str> = cs.warnings.iter().map(|w| w.code.as_str()).collect();
    assert!(codes.contains(&"rule_replaced"));
    assert!(codes.contains(&"geometry_synthesized"));
    let info = map.lane_info(c).unwrap();
    assert_eq!(info.stop_lines, vec![sl]);
    assert_eq!(info.traffic_signals, vec![sig]);
    assert_eq!(info.rules.len(), 1);
    assert_eq!(map.traffic_signal(sig).unwrap().bulbs.len(), 3);

    let (cw, cs) = map
        .add_crosswalk(NewCrosswalk {
            geometry: CrosswalkGeometry::Across {
                lane: b,
                station: 10.0,
                width: 4.0,
                margin: 0.0,
            },
            crossing_lanes: None,
            stop_line_offset: Some(2.0),
        })
        .unwrap();
    assert!(cs.warnings.is_empty(), "{:?}", cs.warnings);
    let rule = map
        .regulatory_elements()
        .find(|r| r.rule.crosswalk() == Some(cw))
        .unwrap();
    assert_eq!(rule.lanes, vec![b]);
    let stop = rule.rule.stop_lines()[0];
    let x = map.stop_line(stop).unwrap().geometry.points[0].x;
    assert!(
        (x - 26.0).abs() < 1e-9,
        "stop line 2 m before the crosswalk, got {x}"
    );
    assert!((map.crosswalk(cw).unwrap().outline().area_2d() - 14.0).abs() < 1e-9);
}

#[test]
fn add_lane_beside_existing_lane() {
    let (mut map, [a, _, _]) = samples::straight_road();
    let (n, cs) = map
        .add_lane(NewLane::new(LaneGeometry::BesideLane {
            lane: a,
            side: Side::Left,
            width: 3.0,
            outer_kind: BoundaryKind::SOLID,
            shared_kind: Some(BoundaryKind::DASHED),
        }))
        .unwrap();
    assert_eq!(map.left_neighbor(a), Some(n));
    assert_eq!(map.right_neighbor(n), Some(a));
    assert_eq!(cs.created.len(), 2, "lane + outer boundary");
    let shared = map.lane(a).unwrap().left.boundary;
    assert_eq!(map.boundary(shared).unwrap().kind, BoundaryKind::DASHED);
    let outer = map.oriented_boundary(n, Side::Left).unwrap();
    assert!((outer.points[0].y - 4.75).abs() < 1e-9);
}

#[test]
fn commands_are_serializable_and_batches_are_atomic() {
    let (mut map, [a, b, c]) = samples::straight_road();
    let script = format!(
        r#"[
            {{"op": "set_speed_limit", "lanes": [{a}, {b}], "kmh": 30}},
            {{"op": "split_lane", "lane": {c}, "at": {{"fraction": 0.5}}}},
            {{"op": "add_stop_line", "lanes": [{c}], "placement": {{"at_lane_end": {{"offset": 1.0}}}}}}
        ]"#,
        a = a.0,
        b = b.0,
        c = c.0
    );
    let commands: Vec<Command> = serde_json::from_str(&script).unwrap();
    let results = map.apply_all(&commands).unwrap();
    assert_eq!(results.len(), 3);
    assert_eq!(
        map.lane(a).unwrap().speed_limit,
        Some(SpeedLimit::from_kmh(30.0))
    );
    assert_eq!(map.lane_count(), 4);

    // A failing batch leaves the map untouched.
    let before = map.clone();
    let bad: Vec<Command> = serde_json::from_str(&format!(
        r#"[{{"op": "remove_lane", "lane": {a}}}, {{"op": "connect_lanes", "from": {b}, "to": 424242}}]"#,
        a = a.0,
        b = b.0
    ))
    .unwrap();
    let err = map.apply_all(&bad).unwrap_err();
    assert_eq!(err.index, 1);
    assert_eq!(map, before);
    // Errors serialize with a machine-readable code.
    let json = serde_json::to_value(&err).unwrap();
    assert_eq!(json["error"]["code"], "not_found");

    // Commands round-trip through JSON.
    for cmd in &commands {
        let text = serde_json::to_string(cmd).unwrap();
        assert_eq!(&serde_json::from_str::<Command>(&text).unwrap(), cmd);
    }
}

#[test]
fn nearest_lane_query() {
    let (map, [_, b, _]) = samples::straight_road();
    let n = map.find_nearest_lane(Point2::new(25.0, 1.0)).unwrap();
    assert_eq!(n.lane, b);
    assert!(n.inside);
    assert!((n.station - 5.0).abs() < 1e-9);
    assert!((n.lateral - 1.0).abs() < 1e-9);
    assert_eq!(map.lanes_at(Point2::new(25.0, 1.0)), vec![b]);
    assert!(map.lanes_at(Point2::new(25.0, 10.0)).is_empty());
}

#[test]
fn set_attribute_and_remove_entities() {
    let (mut map, lanes) = samples::intersection();
    let lane = lanes.incoming[0];
    let cs = map
        .set_attribute(lane.into(), "note", Some("reviewed"))
        .unwrap();
    assert_eq!(cs.modified, vec![EntityRef::Lane(lane)]);
    assert_eq!(
        map.lane(lane).unwrap().attributes.get("note"),
        Some("reviewed")
    );

    // Removing a signal deletes its (now empty) traffic light rule.
    let sig = map.lane_info(lane).unwrap().traffic_signals[0];
    let cs = map.remove_entity(sig.into()).unwrap();
    assert_eq!(
        cs.deleted
            .iter()
            .filter(|e| e.kind() == EntityKind::RegulatoryElement)
            .count(),
        1
    );
    // Removing a boundary that is in use is rejected.
    let b = map.lane(lane).unwrap().left.boundary;
    assert!(matches!(
        map.remove_entity(b.into()),
        Err(EditError::InUse { .. })
    ));
}
