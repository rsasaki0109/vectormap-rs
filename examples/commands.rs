//! Drive map edits with serializable high-level commands — the interface a
//! future MCP server exposes to language models.
//!
//! ```bash
//! cargo run --example commands
//! ```
//!
//! The model never touches OSM XML or internal data structures: it reads
//! summaries / lane details / validation reports and sends commands such as
//! `split_lane` or `add_traffic_signal`; the library applies them
//! deterministically and answers with a change set.

use vectormap::core::{Command, samples};
use vectormap::io::json::to_pretty_string;
use vectormap::validation::{ValidationOptions, validate};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (mut map, [a, b, c]) = samples::straight_road();

    // get_map_summary
    println!("summary: {}", to_pretty_string(&map.summary().counts));

    // What an agent would send, e.g. for
    // "split lane B, put a traffic light at the end of C and slow down A".
    let script = format!(
        r#"[
          {{"op": "split_lane", "lane": {b}, "at": {{"fraction": 0.5}}}},
          {{"op": "add_traffic_signal", "lanes": [{c}], "stop_line": {{"new": {{"at_lane_end": {{"offset": 2.0}}}}}}}},
          {{"op": "set_speed_limit", "lanes": [{a}], "kmh": 30}}
        ]"#,
        a = a.0,
        b = b.0,
        c = c.0
    );
    let commands: Vec<Command> = serde_json::from_str(&script)?;
    let changes = map.apply_all(&commands)?;
    for (cmd, cs) in commands.iter().zip(&changes) {
        println!("{} → {}", cmd.name(), to_pretty_string(cs));
    }

    // get_lane
    println!("lane {c}: {}", to_pretty_string(&map.lane_info(c).unwrap()));

    // validate_map
    let report = validate(&map, &ValidationOptions::default());
    println!("validation: {}", to_pretty_string(&report.counts));

    // A bad command is rejected with a machine-readable error and the map
    // stays unchanged.
    let bad: Vec<Command> =
        serde_json::from_str(r#"[{"op": "merge_lanes", "first": 1, "second": 2}]"#)?;
    let before = map.clone();
    if let Err(e) = map.apply_all(&bad) {
        println!("rejected: {}", to_pretty_string(&e));
    }
    assert_eq!(map, before);
    Ok(())
}
