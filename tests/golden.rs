//! Golden files: the Lanelet2 and JSON output for the integration maps is
//! committed in `tests/data` and must not change unintentionally.
//!
//! Regenerate with `UPDATE_GOLDEN=1 cargo test --test golden`.

mod common;

use common::fixture;
use vectormap::core::samples;
use vectormap::io::{json, lanelet2};

fn read_expected(name: &str, actual: &str) -> Option<String> {
    let path = fixture(name);
    if std::env::var_os("UPDATE_GOLDEN").is_some() {
        std::fs::write(&path, actual).unwrap();
        return None;
    }
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|_| panic!("{} is missing; run with UPDATE_GOLDEN=1", path.display()));
    Some(text.replace("\r\n", "\n"))
}

fn changed(name: &str) -> String {
    format!("{name} changed; review the diff and run with UPDATE_GOLDEN=1 if intended")
}

/// Byte-exact comparison (JSON IR: arithmetic only, identical everywhere).
fn check_exact(name: &str, actual: &str) {
    if let Some(expected) = read_expected(name, actual) {
        assert!(expected == actual, "{}", changed(name));
    }
}

/// OSM comparison: everything exact except lat/lon, which come from
/// transcendental functions whose last bit differs between platform math
/// libraries (compared to 1e-9 degrees, ~0.1 mm).
fn check_osm(name: &str, actual: &str) {
    let Some(expected) = read_expected(name, actual) else {
        return;
    };
    let (e, _) = lanelet2::parse_osm(&expected).unwrap();
    let (a, _) = lanelet2::parse_osm(actual).unwrap();
    assert_eq!(e.ways, a.ways, "{}", changed(name));
    assert_eq!(e.relations, a.relations, "{}", changed(name));
    assert_eq!(
        e.nodes.keys().collect::<Vec<_>>(),
        a.nodes.keys().collect::<Vec<_>>(),
        "{}",
        changed(name)
    );
    for (id, en) in &e.nodes {
        let an = &a.nodes[id];
        assert_eq!(en.tags, an.tags, "{}: node {id}", changed(name));
        assert!(
            (en.lat - an.lat).abs() < 1e-9 && (en.lon - an.lon).abs() < 1e-9,
            "{}: node {id} moved",
            changed(name)
        );
    }
}

#[test]
fn golden_outputs() {
    for (name, map) in [
        ("straight_road", samples::straight_road().0),
        ("two_lane_road", samples::two_lane_road().0),
        ("intersection", samples::intersection().0),
    ] {
        let (osm, _) = lanelet2::write_string(&map, &lanelet2::SaveOptions::default());
        check_osm(&format!("{name}.osm"), &osm);
        check_exact(&format!("{name}.json"), &json::to_string(&map));
    }
}

#[test]
fn golden_files_load_back() {
    for name in ["straight_road", "two_lane_road", "intersection"] {
        let from_osm =
            lanelet2::load_lanelet2(fixture(&format!("{name}.osm")), &Default::default())
                .unwrap()
                .map;
        let from_json = json::load(fixture(&format!("{name}.json"))).unwrap().map;
        assert_eq!(from_osm.lane_count(), from_json.lane_count(), "{name}");
        assert_eq!(from_osm.topology(), from_json.topology(), "{name}");
    }
}
