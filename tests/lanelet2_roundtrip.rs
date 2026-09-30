//! Lanelet2 ⇄ IR integration tests on the three integration maps
//! (straight road, two-lane road, 4-way intersection) and a hand-written
//! JOSM-style fixture.

mod common;

use common::{assert_maps_close, fixture};
use vectormap::core::samples;
use vectormap::io::lanelet2::{self, LoadOptions, ProjectionChoice, SaveOptions};
use vectormap::io::{autoware, json};
use vectormap::prelude::*;
use vectormap::validation::{ValidationOptions, validate};

fn samples() -> Vec<(&'static str, Map)> {
    vec![
        ("straight_road", samples::straight_road().0),
        ("two_lane_road", samples::two_lane_road().0),
        ("intersection", samples::intersection().0),
    ]
}

fn read(osm: &str, projection: ProjectionChoice) -> vectormap::io::Loaded {
    lanelet2::read_str(
        osm,
        &LoadOptions {
            projection,
            ..Default::default()
        },
    )
    .unwrap()
}

/// The sample without the parts Lanelet2 cannot carry (roads, map name).
fn lanelet2_view(map: &Map) -> Map {
    let mut doc = map.to_document();
    doc.roads.clear();
    doc.metadata.name = None;
    doc.try_into_map().unwrap()
}

fn assert_valid(map: &Map, what: &str) {
    let report = validate(map, &ValidationOptions::default());
    assert!(!report.has_errors(), "{what}: {:#?}", report.issues);
}

#[test]
fn lanelet2_to_vectormap_to_lanelet2_to_vectormap() {
    for (name, sample) in samples() {
        let (osm1, issues) = lanelet2::write_string(&sample, &SaveOptions::default());
        let codes: Vec<&str> = issues.iter().map(|i| i.code.as_str()).collect();
        assert_eq!(
            codes,
            vec!["lanelet2.not_exported"],
            "{name}: only roads are dropped"
        );

        // Lanelet2 → VectorMap
        let ir1 = read(&osm1, ProjectionChoice::Auto);
        assert_valid(&ir1.map, name);
        // → Lanelet2 → VectorMap
        let (osm2, _) = lanelet2::write_string(&ir1.map, &SaveOptions::default());
        let ir2 = read(&osm2, ProjectionChoice::Auto);
        assert_valid(&ir2.map, name);

        // local_x/local_y make the round trip exact, and the georeference is
        // recovered from lat/lon.
        assert_eq!(ir1.map, ir2.map, "{name}");
        assert_eq!(
            ir1.map.topology(),
            sample.topology(),
            "{name}: topology survives"
        );
        let (g, g0) = (
            ir1.map.metadata().georeference.unwrap(),
            sample.metadata().georeference.unwrap(),
        );
        assert_eq!(g.projection, g0.projection);
        assert!((g.origin.lat - g0.origin.lat).abs() < 1e-9);
        assert!((g.origin.lon - g0.origin.lon).abs() < 1e-9);
        assert_maps_close(&ir1.map, &lanelet2_view(&sample), 1e-6);
    }
}

#[test]
fn round_trip_without_local_tags_uses_lat_lon() {
    for (name, sample) in samples() {
        let opts = SaveOptions {
            local_coordinates: Some(false),
            ..Default::default()
        };
        let (osm1, _) = lanelet2::write_string(&sample, &opts);
        assert!(!osm1.contains("local_x"));
        let ir1 = read(&osm1, ProjectionChoice::Auto);
        let (osm2, _) = lanelet2::write_string(&ir1.map, &opts);
        let ir2 = read(&osm2, ProjectionChoice::Auto);
        assert_maps_close(&ir1.map, &ir2.map, 1e-6);
        assert_eq!(ir1.map.topology(), ir2.map.topology(), "{name}");
    }
}

#[test]
fn round_trip_without_georeference() {
    let (sample, _) = samples::two_lane_road();
    let mut doc = sample.to_document();
    doc.metadata.georeference = None;
    let map = doc.try_into_map().unwrap();
    let (osm, warnings) = lanelet2::write_string(&map, &SaveOptions::default());
    assert_eq!(warnings[0].code, "lanelet2.no_georeference");
    let loaded = read(&osm, ProjectionChoice::Auto).map;
    assert_eq!(loaded.metadata().georeference, None);
    assert_eq!(loaded, lanelet2_view(&map));
}

#[test]
fn export_preserves_the_ir() {
    for (name, sample) in samples() {
        let (osm, _) = lanelet2::write_string(&sample, &SaveOptions::default());
        let georef = sample.metadata().georeference.unwrap();
        let loaded = read(&osm, ProjectionChoice::Georeferenced(georef));
        let unsupported: Vec<_> = loaded
            .issues
            .iter()
            .filter(|i| i.severity >= Severity::Warning)
            .collect();
        assert!(unsupported.is_empty(), "{name}: {unsupported:#?}");
        assert_maps_close(&loaded.map, &lanelet2_view(&sample), 1e-6);
    }
}

#[test]
fn autoware_profile_round_trip_is_exact() {
    for (name, sample) in samples() {
        let opts = SaveOptions::autoware();
        let (osm1, _) = lanelet2::write_string(&sample, &opts);
        let ir1 = read(&osm1, ProjectionChoice::LocalTags).map;
        let (osm2, _) = lanelet2::write_string(&ir1, &opts);
        let ir2 = read(&osm2, ProjectionChoice::LocalTags).map;
        assert_eq!(ir1, ir2, "{name}");
        let (osm3, _) = lanelet2::write_string(&ir2, &opts);
        assert_eq!(osm2, osm3, "{name}: byte-identical output");
    }
}

#[test]
fn export_is_deterministic() {
    for (name, sample) in samples() {
        let a = lanelet2::write_string(&sample, &SaveOptions::default()).0;
        let b = lanelet2::write_string(&sample.clone(), &SaveOptions::default()).0;
        assert_eq!(a, b, "{name}");
    }
}

#[test]
fn hand_written_josm_map() {
    let loaded =
        lanelet2::load_lanelet2(fixture("straight_road_josm.osm"), &LoadOptions::default())
            .unwrap();
    let map = &loaded.map;
    let codes: Vec<&str> = loaded.issues.iter().map(|i| i.code.as_str()).collect();
    assert!(codes.contains(&"lanelet2.unsupported_way"), "{codes:?}");
    assert!(
        codes.contains(&"lanelet2.unsupported_relation"),
        "{codes:?}"
    );
    assert_valid(map, "josm");

    assert_eq!(map.lane_count(), 3);
    // Negative IDs are mapped to fresh positive IDs, -1 first: -201 → A ...
    let lanes: Vec<&Lane> = map.lanes().collect();
    let (a, b, c) = (lanes[0].id, lanes[1].id, lanes[2].id);
    assert_eq!(map.successors(a), &[b]);
    assert_eq!(map.successors(b), &[c]);
    for l in [a, b, c] {
        assert!((map.lane_length(l).unwrap() - 20.0).abs() < 0.05, "{l}");
    }
    // B's left bound is drawn backwards in the file.
    assert!(map.lane(b).unwrap().left.reversed);
    assert!(!map.lane(a).unwrap().left.reversed);

    let la = map.lane(a).unwrap();
    assert_eq!(la.speed_limit, Some(SpeedLimit::from_kmh(50.0)));
    assert_eq!(la.attributes.get("lanelet2:location"), Some("urban"));
    assert_eq!(
        la.attributes.get("lanelet2:participant:vehicle"),
        Some("yes")
    );
    let lc = map.lane(c).unwrap();
    assert_eq!(lc.kind, LaneKind::Driving);
    assert_eq!(lc.attributes.get("lanelet2:subtype"), Some("highway"));
    assert_eq!(lc.speed_limit, Some(SpeedLimit::from_kmh(30.0)));

    // The middle curb is a low curbstone.
    let rb = map.boundary(map.lane(b).unwrap().right.boundary).unwrap();
    assert_eq!(rb.kind, BoundaryKind::Curb);
    assert_eq!(rb.attributes.get("lanelet2:subtype"), Some("low"));

    // Stop sign rule on C.
    let info = map.lane_info(c).unwrap();
    assert_eq!(info.stop_lines.len(), 1);
    let rule = map.regulatory_element(info.rules[0]).unwrap();
    match &rule.rule {
        Rule::TrafficSign {
            sign_type, sign, ..
        } => {
            assert_eq!(sign_type, "stop_sign");
            assert!((sign.as_ref().unwrap().points[0].z - 2.5).abs() < 1e-9);
        }
        other => panic!("unexpected rule {other:?}"),
    }

    // Export keeps the Lanelet2-specific tags and re-imports identically.
    let (osm, _) = lanelet2::write_string(map, &SaveOptions::default());
    for needle in [
        r#"<tag k="subtype" v="highway"/>"#,
        r#"<tag k="location" v="nonurban"/>"#,
        r#"<tag k="subtype" v="low"/>"#,
        r#"<tag k="speed_limit" v="50"/>"#,
    ] {
        assert!(osm.contains(needle), "missing {needle}");
    }
    let again = read(&osm, ProjectionChoice::Auto);
    assert_maps_close(&again.map, map, 1e-6);
}

#[test]
fn edit_save_reload() {
    let loaded =
        lanelet2::load_lanelet2(fixture("straight_road_josm.osm"), &LoadOptions::default())
            .unwrap();
    let mut map = loaded.map;
    let ids: Vec<LaneId> = map.lanes().map(|l| l.id).collect();
    let (a, b, c) = (ids[0], ids[1], ids[2]);

    let (b2, _) = map
        .split_lane(b, SplitAt::Fraction(0.5), SplitOptions::default())
        .unwrap();
    map.set_speed_limit(&[a, b, b2], Some(SpeedLimit::from_kmh(40.0)))
        .unwrap();
    let (cw, _) = map
        .add_crosswalk(NewCrosswalk {
            geometry: vectormap::core::CrosswalkGeometry::Across {
                lane: a,
                station: 10.0,
                width: 3.0,
                margin: 0.5,
            },
            crossing_lanes: None,
            stop_line_offset: Some(1.0),
        })
        .unwrap();

    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("edited.osm");
    lanelet2::save_lanelet2(&map, &path, &SaveOptions::default()).unwrap();
    let reloaded = lanelet2::load_lanelet2(&path, &LoadOptions::default())
        .unwrap()
        .map;
    assert_valid(&reloaded, "edited");

    assert_eq!(reloaded.successors(a), &[b]);
    assert_eq!(reloaded.successors(b), &[b2]);
    assert_eq!(reloaded.successors(b2), &[c]);
    assert_eq!(
        reloaded.lane(b2).unwrap().speed_limit,
        Some(SpeedLimit::from_kmh(40.0))
    );
    assert!(reloaded.crosswalk(cw).is_some());
    let a_info = reloaded.lane_info(a).unwrap();
    assert_eq!(a_info.stop_lines.len(), 1, "crosswalk stop line");
}

#[test]
fn autoware_map_directory() {
    let (map, lanes) = samples::intersection();
    let dir = tempfile::tempdir().unwrap();
    let issues = autoware::save(&map, dir.path()).unwrap();
    let serious: Vec<_> = issues
        .iter()
        .filter(|i| i.severity >= Severity::Warning)
        .collect();
    assert!(serious.is_empty(), "{serious:#?}");

    let yaml = std::fs::read_to_string(dir.path().join("map_projector_info.yaml")).unwrap();
    assert!(yaml.contains("projector_type: LocalCartesianUTM"));
    assert!(yaml.contains("latitude: 35.681236"));

    let osm = std::fs::read_to_string(dir.path().join("lanelet2_map.osm")).unwrap();
    let (data, _) = lanelet2::parse_osm(&osm).unwrap();
    for node in data.nodes.values() {
        for k in ["ele", "local_x", "local_y"] {
            assert!(node.tags.contains_key(k), "node {} lacks {k}", node.id);
        }
    }
    let lanelets: Vec<_> = data
        .relations
        .values()
        .filter(|r| r.tags.get("type").map(String::as_str) == Some("lanelet"))
        .collect();
    assert_eq!(lanelets.len(), 21, "20 lanes + 1 crosswalk");
    for ll in &lanelets {
        let t = &ll.tags;
        assert_eq!(t.get("location").map(String::as_str), Some("urban"));
        if t["subtype"] == "road" {
            assert!(t.contains_key("speed_limit"));
            assert_eq!(t["one_way"], "yes");
            assert_eq!(t["participant:vehicle"], "yes");
        } else {
            assert_eq!(t["subtype"], "crosswalk");
            assert_eq!(t["one_way"], "no");
            assert_eq!(t["participant:pedestrian"], "yes");
        }
    }
    let with_turn = lanelets
        .iter()
        .filter(|r| r.tags.contains_key("turn_direction"))
        .filter(|r| r.tags.contains_key("intersection_area"))
        .count();
    assert_eq!(with_turn, 12);

    let rules: Vec<_> = data
        .relations
        .values()
        .filter(|r| r.tags.get("type").map(String::as_str) == Some("regulatory_element"))
        .collect();
    let lights: Vec<_> = rules
        .iter()
        .filter(|r| r.tags["subtype"] == "traffic_light")
        .collect();
    assert_eq!(lights.len(), 4);
    for tl in lights {
        let roles: Vec<&str> = tl.members.iter().map(|m| m.role.as_str()).collect();
        assert_eq!(roles, vec!["refers", "ref_line", "light_bulbs"]);
        let bulbs = &data.ways[&tl.members[2].reference];
        assert_eq!(bulbs.tags["type"], "light_bulbs");
        assert_eq!(
            bulbs.tags["traffic_light_id"],
            tl.members[0].reference.to_string()
        );
        let light = &data.ways[&tl.members[0].reference];
        assert_eq!(light.tags["height"], "0.5");
        for n in &bulbs.nodes {
            assert!(data.nodes[n].tags.contains_key("color"));
        }
    }
    let crosswalk = rules
        .iter()
        .find(|r| r.tags["subtype"] == "crosswalk")
        .unwrap();
    assert_eq!(crosswalk.members[0].role, "refers");
    // The crossing road lanelets reference the crosswalk rule.
    let referencing = lanelets
        .iter()
        .filter(|l| l.members.iter().any(|m| m.reference == crosswalk.id))
        .count();
    assert_eq!(referencing, 2);

    // lat/lon are consistent with the projector info (LocalCartesianUTM).
    let reloaded = read(
        &osm,
        ProjectionChoice::Georeferenced(map.metadata().georeference.unwrap()),
    )
    .map;
    let p = reloaded.centerline(lanes.incoming[0]).unwrap().points[0];
    let q = map.centerline(lanes.incoming[0]).unwrap().points[0];
    assert!(p.distance(q) < 1e-6);
}

#[test]
fn json_and_lanelet2_agree() {
    let (map, _) = samples::intersection();
    let via_json = json::from_str(&json::to_string(&map)).unwrap().map;
    let (osm, _) = lanelet2::write_string(&via_json, &SaveOptions::default());
    let (osm_direct, _) = lanelet2::write_string(&map, &SaveOptions::default());
    assert_eq!(osm, osm_direct);
}

#[test]
fn invalid_lanelets_are_reported_not_fatal() {
    let osm = r#"<osm version="0.6">
      <node id="1" lat="49.0" lon="8.4"/>
      <node id="2" lat="49.0001" lon="8.4"/>
      <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="99"/><tag k="type" v="line_thin"/></way>
      <relation id="20"><member type="way" ref="10" role="left"/><tag k="type" v="lanelet"/></relation>
      <relation id="21"><member type="way" ref="10" role="left"/><member type="way" ref="11" role="right"/><tag k="type" v="lanelet"/></relation>
    </osm>"#;
    let loaded = read(osm, ProjectionChoice::Auto);
    assert_eq!(loaded.map.lane_count(), 0);
    let codes: Vec<&str> = loaded.issues.iter().map(|i| i.code.as_str()).collect();
    assert!(codes.contains(&"lanelet2.invalid_primitive"), "{codes:?}");
    assert!(codes.contains(&"lanelet2.missing_member"), "{codes:?}");
}

/// load → edit → save → reload on every integration map.
#[test]
fn edit_save_reload_all_samples() {
    let dir = tempfile::tempdir().unwrap();

    // Straight road: merge A and B.
    let (sample, [a, b, c]) = samples::straight_road();
    let mut map = reload_via_file(&sample, dir.path(), "straight");
    map.merge_lanes(a, b).unwrap();
    let map = reload_via_file(&map, dir.path(), "straight_edited");
    assert_eq!(map.lane_count(), 2);
    assert_eq!(map.successors(a), &[c]);

    // Two-lane road: split A together with its neighbor C.
    let (sample, [a, b, c, d]) = samples::two_lane_road();
    let mut map = reload_via_file(&sample, dir.path(), "two_lane");
    let (a2, _) = map
        .split_lane(a, SplitAt::Station(10.0), SplitOptions::default())
        .unwrap();
    let c2 = map.successors(c)[0];
    let expected = map.topology().clone();
    let map = reload_via_file(&map, dir.path(), "two_lane_edited");
    assert_eq!(map.topology(), &expected);
    assert_eq!(map.right_neighbor(a2), Some(c2));
    assert_eq!(map.successors(a2), &[b]);
    assert_eq!(map.successors(c2), &[d]);

    // Intersection: split an incoming lane (opposite neighbor), add a
    // crosswalk with stop lines on the east arm, slow down a turn.
    let (sample, lanes) = samples::intersection();
    let mut map = reload_via_file(&sample, dir.path(), "intersection");
    map.split_lane(
        lanes.incoming[2],
        SplitAt::Fraction(0.5),
        SplitOptions::default(),
    )
    .unwrap();
    let (cw, _) = map
        .add_crosswalk(NewCrosswalk {
            geometry: vectormap::core::CrosswalkGeometry::Across {
                lane: lanes.outgoing[0],
                station: 2.5,
                width: 4.0,
                margin: 0.5,
            },
            crossing_lanes: None,
            stop_line_offset: Some(1.0),
        })
        .unwrap();
    map.set_speed_limit(&[lanes.connectors[1]], Some(SpeedLimit::from_kmh(15.0)))
        .unwrap();
    let expected = map.topology().clone();
    let edited = map.clone();
    let map = reload_via_file(&map, dir.path(), "intersection_edited");
    assert_eq!(map.topology(), &expected);
    assert_eq!(map.lane_count(), 22);
    let rule = map
        .regulatory_elements()
        .find(|r| r.rule.crosswalk() == Some(cw))
        .unwrap();
    assert_eq!(rule.lanes.len(), 2);
    assert_eq!(rule.rule.stop_lines().len(), 2);
    assert_eq!(
        map.lane(lanes.connectors[1]).unwrap().speed_limit,
        Some(SpeedLimit::from_kmh(15.0))
    );
    assert_maps_close(&map, &lanelet2_view(&edited), 1e-6);
}

/// Saves as Lanelet2, loads it back and checks that the result is valid.
fn reload_via_file(map: &Map, dir: &std::path::Path, name: &str) -> Map {
    let path = dir.join(format!("{name}.osm"));
    lanelet2::save_lanelet2(map, &path, &SaveOptions::default()).unwrap();
    let loaded = lanelet2::load_lanelet2(&path, &LoadOptions::default()).unwrap();
    assert_valid(&loaded.map, name);
    loaded.map
}

/// A road built from a reference line, cut into pieces, with a turning
/// connector, keeps its topology through the Autoware profile: Lanelet2
/// re-derives successors and neighbors from the shared geometry.
#[test]
fn built_roads_keep_their_topology_in_autoware_lanelet2() {
    let mut map = Map::new();
    let mut east = NewRoad::new(
        Polyline3::from_xy(&[[-80.0, 0.0], [-40.0, 1.0], [0.0, 0.0]]),
        vec![
            RoadLane::new(3.5, LaneDirection::Forward),
            RoadLane::new(3.5, LaneDirection::Forward),
            RoadLane::new(3.25, LaneDirection::Backward),
        ],
    );
    east.segment_length = Some(25.0);
    east.resample = Some(2.0);
    east.speed_limit = Some(SpeedLimit::from_kmh(40.0));
    let (east, _) = map.build_road(east).unwrap();
    let mut north = NewRoad::new(
        Polyline3::from_xy(&[[15.0, 12.0], [15.0, 60.0]]),
        vec![RoadLane::new(3.5, LaneDirection::Forward)],
    );
    north.speed_limit = Some(SpeedLimit::from_kmh(30.0));
    let (north, _) = map.build_road(north).unwrap();
    let from = *east.lanes[1].last().unwrap();
    map.add_connector(NewConnector::new(from, north.lanes[0][0]))
        .unwrap();
    assert_valid(&map, "built");
    let problems: Vec<_> = autoware::check(&map)
        .into_iter()
        .filter(|i| i.severity != Severity::Info)
        .collect();
    assert!(problems.is_empty(), "{problems:#?}");

    let (osm, _) = lanelet2::write_string(&map, &SaveOptions::autoware());
    let back = read(&osm, ProjectionChoice::Auto).map;
    assert_eq!(back.topology(), map.topology());
    assert_eq!(back.lane_count(), map.lane_count());
    for lane in map.lanes() {
        let other = back.lane(lane.id).unwrap();
        assert_eq!(other.speed_limit, lane.speed_limit);
        assert_eq!(other.turn_direction, lane.turn_direction);
        let (a, b) = (
            map.centerline(lane.id).unwrap(),
            back.centerline(lane.id).unwrap(),
        );
        assert_eq!(a.points.len(), b.points.len());
        for (p, q) in a.points.iter().zip(&b.points) {
            assert!(p.distance(*q) < 1e-6, "{}: {p:?} != {q:?}", lane.id);
        }
    }
}
