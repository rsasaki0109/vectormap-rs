//! Explicit control editing, atomic rejection and Lanelet2 preservation.
use vectormap::core::{SignalKind, StopLineChoice, samples};
use vectormap::io::{
    autoware, json,
    lanelet2::{self, LoadOptions, SaveOptions},
};
use vectormap::prelude::*;
use vectormap::validation::{ValidationOptions, validate};

fn fixture() -> (Map, RegulatoryElementId, CrosswalkId, LaneId, SignalId) {
    let (mut map, _) = samples::intersection();
    let lane = map.lanes().next().unwrap().id;
    let crossing = map.crosswalks().next().unwrap().id;
    let (signal, _) = map
        .add_traffic_signal(NewTrafficSignal {
            lanes: vec![lane],
            kind: SignalKind::Pedestrian,
            geometry: Some(Polyline3::new(vec![
                Point3::new(0., 0., 3.),
                Point3::new(1., 0., 3.),
            ])),
            height: Some(1.),
            stop_line: StopLineChoice::None,
            bulbs: Some(vec![]),
            group: None,
        })
        .unwrap();
    let rule = map
        .regulatory_elements()
        .find(|r| r.rule.signals().contains(&signal))
        .unwrap()
        .id;
    (map, rule, crossing, lane, signal)
}

#[test]
fn reviewed_crosswalk_control_keeps_physical_map_and_noop_exact() {
    let (mut map, rule, crossing, _, signal) = fixture();
    let before = map.clone();
    assert!(!json::to_string(&map).contains("controlled_crosswalks"));
    let changes = map
        .set_regulatory_links(rule, &[], &[crossing], &[])
        .unwrap();
    assert_eq!(changes.modified, vec![EntityRef::RegulatoryElement(rule)]);
    let re = map.regulatory_element(rule).unwrap();
    assert_eq!(re.controlled_crosswalks, vec![crossing]);
    assert!(re.lanes.is_empty());
    assert_eq!(map.traffic_signal(signal), before.traffic_signal(signal));
    assert_eq!(map.crosswalk(crossing), before.crosswalk(crossing));
    assert_eq!(
        map.boundaries().collect::<Vec<_>>(),
        before.boundaries().collect::<Vec<_>>()
    );
    assert_eq!(
        map.lanes().collect::<Vec<_>>(),
        before.lanes().collect::<Vec<_>>()
    );
    assert_eq!(map.topology(), before.topology());
    assert!(!validate(&map, &ValidationOptions::default()).has_errors());
    assert!(!autoware::check(&map).iter().any(|i| i.code
        == autoware::codes::TRAFFIC_LIGHT_WITHOUT_STOP_LINE
        && i.entity == Some(rule.into())));
    let edited = map.clone();
    assert!(
        map.set_regulatory_links(rule, &[], &[crossing], &[])
            .unwrap()
            .is_empty()
    );
    assert_eq!(map, edited);
    assert_eq!(json::from_str(&json::to_string(&map)).unwrap().map, map);
}

#[test]
fn pedestrian_control_roundtrips_as_membership_of_crosswalk_lanelet() {
    let (mut map, rule, crossing, lane, signal) = fixture();
    map.set_regulatory_links(rule, &[], &[crossing], &[])
        .unwrap();
    let (xml, issues) = lanelet2::write_string(&map, &SaveOptions::autoware());
    assert!(
        !issues.iter().any(|i| i.severity == Severity::Error),
        "{issues:?}"
    );
    let (osm, _) = lanelet2::parse_osm(&xml).unwrap();
    let c = osm
        .relations
        .values()
        .find(|r| r.id == crossing.0 as i64)
        .unwrap();
    assert!(
        c.members
            .iter()
            .any(|m| m.reference == rule.0 as i64 && m.role == "regulatory_element")
    );
    let l = osm
        .relations
        .values()
        .find(|r| r.id == lane.0 as i64)
        .unwrap();
    assert!(
        !l.members
            .iter()
            .any(|m| m.reference == rule.0 as i64 && m.role == "regulatory_element")
    );
    let loaded = lanelet2::read_str(&xml, &LoadOptions::default()).unwrap();
    assert!(
        !loaded
            .issues
            .iter()
            .any(|i| i.code == lanelet2::codes::UNSUPPORTED_MEMBER),
        "{:?}",
        loaded.issues
    );
    assert_eq!(
        loaded
            .map
            .regulatory_element(rule)
            .unwrap()
            .controlled_crosswalks,
        vec![crossing]
    );
    assert!(
        loaded
            .map
            .regulatory_element(rule)
            .unwrap()
            .lanes
            .is_empty()
    );
    assert_eq!(
        loaded.map.traffic_signal(signal).unwrap().kind,
        SignalKind::Pedestrian
    );
    assert_eq!(
        loaded.map.traffic_signal(signal).unwrap().bulbs,
        map.traffic_signal(signal).unwrap().bulbs
    );
    let (again, _) = lanelet2::write_string(&loaded.map, &SaveOptions::autoware());
    assert_eq!(
        lanelet2::read_str(&again, &LoadOptions::default())
            .unwrap()
            .map
            .regulatory_element(rule)
            .unwrap()
            .controlled_crosswalks,
        vec![crossing]
    );
}

#[test]
fn wrong_participant_missing_and_duplicate_targets_are_atomic() {
    let (mut map, rule, crossing, lane, signal) = fixture();
    let before = map.clone();
    for (lanes, crossings) in [
        (vec![lane], vec![crossing]),
        (vec![], vec![CrosswalkId(999999)]),
        (vec![], vec![crossing, crossing]),
    ] {
        assert!(
            map.set_regulatory_links(rule, &lanes, &crossings, &[])
                .is_err()
        );
        assert_eq!(map, before);
    }
    let commands = vec![
        Command::SetRegulatoryLinks {
            regulatory_element: rule,
            lanes: vec![],
            controlled_crosswalks: vec![crossing],
            stop_lines: vec![],
        },
        Command::SetRegulatoryLinks {
            regulatory_element: rule,
            lanes: vec![],
            controlled_crosswalks: vec![CrosswalkId(999999)],
            stop_lines: vec![],
        },
    ];
    let parsed: Vec<Command> =
        serde_json::from_str(&serde_json::to_string(&commands).unwrap()).unwrap();
    assert!(map.apply_all(&parsed).is_err());
    assert_eq!(map, before);
    map.traffic_signal_mut(signal).unwrap().kind = SignalKind::Vehicle;
    let before = map.clone();
    assert!(
        map.set_regulatory_links(rule, &[], &[crossing], &[])
            .is_err()
    );
    assert_eq!(map, before);
    assert!(autoware::check(&map).iter().any(|i| i.code
        == autoware::codes::TRAFFIC_LIGHT_WITHOUT_STOP_LINE
        && i.entity == Some(rule.into())));
}

#[test]
fn removing_controlled_crosswalk_detaches_reference_but_keeps_observed_head() {
    let (mut map, rule, crossing, _, signal) = fixture();
    map.set_regulatory_links(rule, &[], &[crossing], &[])
        .unwrap();
    let original = map.traffic_signal(signal).unwrap().clone();
    let changes = map.remove_entity(crossing.into()).unwrap();
    assert!(changes.modified.contains(&rule.into()));
    assert!(
        map.regulatory_element(rule)
            .unwrap()
            .controlled_crosswalks
            .is_empty()
    );
    assert_eq!(map.traffic_signal(signal), Some(&original));
    let report = validate(&map, &ValidationOptions::default());
    assert!(!report.has_errors());
    assert!(
        report
            .issues
            .iter()
            .any(|i| i.code == vectormap::validation::codes::ORPHAN_RULE
                && i.entity == Some(rule.into()))
    );
}

#[test]
fn imported_dangling_control_is_a_validation_error_not_silently_orphaned() {
    let (mut map, rule, _, _, _) = fixture();
    map.regulatory_element_mut(rule)
        .unwrap()
        .controlled_crosswalks = vec![CrosswalkId(999999)];
    let report = validate(&map, &ValidationOptions::default());
    assert!(report.issues.iter().any(|i| i.code
        == vectormap::validation::codes::MISSING_REFERENCE
        && i.entity == Some(rule.into())));
    assert!(autoware::check(&map).iter().any(|i| i.code
        == autoware::codes::TRAFFIC_LIGHT_WITHOUT_STOP_LINE
        && i.entity == Some(rule.into())));
}

#[test]
fn malformed_control_and_group_extension_are_rejected() {
    let (mut map, rule, crossing, lane, signal) = fixture();
    map.set_regulatory_links(rule, &[], &[crossing], &[])
        .unwrap();
    let before = map.clone();
    let mut spec = NewTrafficSignal::for_lanes(vec![lane]);
    spec.kind = SignalKind::Pedestrian;
    spec.group = Some(rule);
    assert!(map.add_traffic_signal(spec).is_err());
    assert_eq!(map, before);
    map.traffic_signal_mut(signal).unwrap().kind = SignalKind::Vehicle;
    assert!(
        validate(&map, &ValidationOptions::default())
            .issues
            .iter()
            .any(|i| i.code == vectormap::validation::codes::INCOMPLETE_RULE
                && i.entity == Some(rule.into()))
    );
    assert!(autoware::check(&map).iter().any(|i| i.entity == Some(rule.into()) && i.code.as_str().contains("without_stop_line")));
}
