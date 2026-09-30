//! Generate an Autoware-ready map directory from the intersection sample.
//!
//! ```bash
//! cargo run --example autoware_map                # writes examples/autoware/
//! cargo run --example autoware_map -- /tmp/map    # or any directory
//! ```
//!
//! The directory contains `lanelet2_map.osm` (Autoware profile) and
//! `map_projector_info.yaml`, the two files Autoware's map loader expects.

use vectormap::core::samples;
use vectormap::io::autoware;
use vectormap::validation::{ValidationOptions, validate};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dir = std::env::args()
        .nth(1)
        .unwrap_or_else(|| concat!(env!("CARGO_MANIFEST_DIR"), "/examples/autoware").to_string());
    let (map, _) = samples::intersection();

    let report = validate(&map, &ValidationOptions::default());
    assert!(!report.has_errors(), "{:#?}", report.issues);

    let issues = autoware::save(&map, &dir)?;
    for issue in &issues {
        println!("{issue}");
    }
    let summary = map.summary();
    println!(
        "wrote {dir}: {} lanes, {} traffic lights, {} stop lines, {} crosswalk(s)",
        summary.counts.lanes,
        summary.counts.traffic_signals,
        summary.counts.stop_lines,
        summary.counts.crosswalks
    );
    Ok(())
}
