//! Golden files: the Lanelet2 and JSON output for the integration maps is
//! committed in `tests/data` and must not change unintentionally.
//!
//! Regenerate with `UPDATE_GOLDEN=1 cargo test --test golden`.

mod common;

use common::fixture;
use vectormap::core::samples;
use vectormap::io::{json, lanelet2};

fn check(name: &str, actual: &str) {
    let path = fixture(name);
    if std::env::var_os("UPDATE_GOLDEN").is_some() {
        std::fs::write(&path, actual).unwrap();
        return;
    }
    let expected = std::fs::read_to_string(&path)
        .unwrap_or_else(|_| panic!("{} is missing; run with UPDATE_GOLDEN=1", path.display()))
        .replace("\r\n", "\n");
    assert!(
        expected == actual,
        "{name} changed; review the diff and run with UPDATE_GOLDEN=1 if intended"
    );
}

#[test]
fn golden_outputs() {
    for (name, map) in [
        ("straight_road", samples::straight_road().0),
        ("two_lane_road", samples::two_lane_road().0),
        ("intersection", samples::intersection().0),
    ] {
        let (osm, _) = lanelet2::write_string(&map, &lanelet2::SaveOptions::default());
        check(&format!("{name}.osm"), &osm);
        check(&format!("{name}.json"), &json::to_string(&map));
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
