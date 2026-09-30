//! Build a small map with the editing API, validate it and export it.
//!
//! ```bash
//! cargo run --example quickstart
//! ```

use vectormap::core::{
    GeoPoint, GeoReference, LaneGeometry, ProjectionKind, StopLinePlacement, StopRule,
};
use vectormap::io::{json, lanelet2};
use vectormap::prelude::*;
use vectormap::validation::{ValidationOptions, validate};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut map = Map::new();
    map.metadata_mut().name = Some("quickstart".into());
    map.metadata_mut().georeference = Some(GeoReference {
        projection: ProjectionKind::Utm,
        origin: GeoPoint::new(35.681236, 139.767125),
    });

    // A 40 m lane from a centreline, and a second lane continuing it.
    let (a, _) = map.add_lane(
        NewLane::from_centerline(Polyline3::from_xy(&[[0.0, 0.0], [40.0, 0.0]]), 3.5)
            .with_speed_limit(SpeedLimit::from_kmh(40.0)),
    )?;
    let (b, _) = map.add_lane(
        NewLane::from_centerline(
            Polyline3::from_xy(&[[40.0, 0.0], [50.0, 0.0], [60.0, 5.0]]),
            3.5,
        )
        .with_speed_limit(SpeedLimit::from_kmh(30.0))
        .with_predecessors(vec![a]),
    )?;

    // A lane to the left of `a`, sharing its boundary (now dashed).
    let (c, changes) = map.add_lane(NewLane::new(LaneGeometry::BesideLane {
        lane: a,
        side: Side::Left,
        width: 3.5,
        outer_kind: BoundaryKind::SOLID,
        shared_kind: Some(BoundaryKind::DASHED),
    }))?;
    println!("add_lane → {}", json::to_pretty_string(&changes));

    // Split `a` (and its neighbor `c`) in the middle.
    let (a2, changes) = map.split_lane(a, SplitAt::Fraction(0.5), SplitOptions::default())?;
    println!("split_lane → created {:?}", changes.created);

    // A stop line with a stop sign at the end of the second half.
    let (stop, _) = map.add_stop_line(NewStopLine {
        lanes: vec![a2],
        placement: StopLinePlacement::AtLaneEnd { offset: 1.0 },
        rule: StopRule::StopSign,
    })?;

    println!("lanes: {a} → {a2} → {b}, neighbor {c}, stop line {stop}");
    println!("successors({a}) = {:?}", map.successors(a));
    println!("left_neighbor({a2}) = {:?}", map.left_neighbor(a2));

    let report = validate(&map, &ValidationOptions::default());
    println!(
        "validation: {} errors, {} warnings",
        report.counts.errors, report.counts.warnings
    );
    for issue in &report.issues {
        println!("  {issue}");
    }

    let out = std::env::temp_dir().join("vectormap_quickstart");
    std::fs::create_dir_all(&out)?;
    json::save(&map, out.join("map.json"))?;
    lanelet2::save_lanelet2(&map, out.join("map.osm"), &Default::default())?;
    println!("wrote {}", out.display());
    Ok(())
}
