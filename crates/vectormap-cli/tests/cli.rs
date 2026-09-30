//! End-to-end tests of the `vectormap` binary.

use std::path::Path;
use std::process::{Command, Output};

fn vectormap(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_vectormap"))
        .args(args)
        .output()
        .expect("binary runs")
}

fn stdout(o: &Output) -> String {
    String::from_utf8_lossy(&o.stdout).into_owned()
}

fn path(p: &Path) -> &str {
    p.to_str().unwrap()
}

#[test]
fn sample_info_validate_convert() {
    let dir = tempfile::tempdir().unwrap();
    let osm = dir.path().join("map.osm");
    let json = dir.path().join("map.json");
    let back = dir.path().join("back.osm");

    let o = vectormap(&["sample", "intersection", path(&osm)]);
    assert!(o.status.success(), "{o:?}");

    let o = vectormap(&["info", path(&osm)]);
    assert!(o.status.success());
    let text = stdout(&o);
    assert!(text.contains("lanes                20"), "{text}");
    assert!(
        text.contains("Rules: crosswalk 1, traffic_light 4"),
        "{text}"
    );

    let o = vectormap(&["info", "--json", path(&osm)]);
    let v: serde_json::Value = serde_json::from_str(&stdout(&o)).unwrap();
    assert_eq!(v["summary"]["counts"]["lanes"], 20);

    let o = vectormap(&["validate", "--autoware", path(&osm)]);
    assert!(o.status.success(), "{}", stdout(&o));
    assert!(stdout(&o).contains("0 error(s), 0 warning(s)"));

    let o = vectormap(&[
        "convert",
        "--from",
        "lanelet2",
        "--to",
        "json",
        path(&osm),
        path(&json),
    ]);
    assert!(o.status.success(), "{o:?}");
    let o = vectormap(&[
        "convert",
        "--from",
        "json",
        "--to",
        "lanelet2",
        path(&json),
        path(&back),
    ]);
    assert!(o.status.success(), "{o:?}");
    // lanelet2 → json → lanelet2 reproduces the file byte for byte.
    assert_eq!(
        std::fs::read_to_string(&osm).unwrap(),
        std::fs::read_to_string(&back).unwrap()
    );
}

#[test]
fn lanelet2_to_lanelet2_convert() {
    let dir = tempfile::tempdir().unwrap();
    let a = dir.path().join("a.osm");
    let b = dir.path().join("b.osm");
    assert!(
        vectormap(&["sample", "two-lane-road", path(&a)])
            .status
            .success()
    );
    let o = vectormap(&[
        "convert",
        "--from",
        "lanelet2",
        "--to",
        "lanelet2",
        path(&a),
        path(&b),
    ]);
    assert!(o.status.success(), "{o:?}");
    assert_eq!(
        std::fs::read_to_string(&a).unwrap(),
        std::fs::read_to_string(&b).unwrap()
    );
}

#[test]
fn autoware_convert_writes_projector_info() {
    let dir = tempfile::tempdir().unwrap();
    let json = dir.path().join("map.json");
    let osm = dir.path().join("lanelet2_map.osm");
    assert!(
        vectormap(&["sample", "intersection", path(&json)])
            .status
            .success()
    );
    let o = vectormap(&["convert", "--autoware", path(&json), path(&osm)]);
    assert!(o.status.success(), "{o:?}");
    let yaml = std::fs::read_to_string(dir.path().join("map_projector_info.yaml")).unwrap();
    assert!(yaml.contains("LocalCartesianUTM"));
    let text = std::fs::read_to_string(&osm).unwrap();
    assert!(text.contains(r#"<tag k="location" v="urban"/>"#));
}

#[test]
fn edit_applies_commands_atomically() {
    let dir = tempfile::tempdir().unwrap();
    let map = dir.path().join("map.json");
    let out = dir.path().join("out.osm");
    let ops = dir.path().join("ops.json");
    assert!(
        vectormap(&["sample", "straight-road", path(&map)])
            .status
            .success()
    );
    let doc: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&map).unwrap()).unwrap();
    let lanes: Vec<u64> = doc["lanes"]
        .as_array()
        .unwrap()
        .iter()
        .map(|l| l["id"].as_u64().unwrap())
        .collect();

    std::fs::write(
        &ops,
        format!(
            r#"[{{"op": "split_lane", "lane": {}, "at": {{"fraction": 0.5}}}},
                {{"op": "add_stop_line", "lanes": [{}]}}]"#,
            lanes[1], lanes[2]
        ),
    )
    .unwrap();
    let o = vectormap(&["edit", path(&map), path(&ops), "-o", path(&out)]);
    assert!(o.status.success(), "{o:?}");
    let changes: serde_json::Value = serde_json::from_str(&stdout(&o)).unwrap();
    assert_eq!(changes.as_array().unwrap().len(), 2);
    let o = vectormap(&["info", "--json", path(&out)]);
    let v: serde_json::Value = serde_json::from_str(&stdout(&o)).unwrap();
    assert_eq!(v["summary"]["counts"]["lanes"], 4);
    assert_eq!(v["summary"]["counts"]["stop_lines"], 1);

    // A failing batch writes nothing and reports a structured error.
    let out2 = dir.path().join("out2.osm");
    std::fs::write(&ops, r#"{"op": "remove_lane", "lane": 999999}"#).unwrap();
    let o = vectormap(&["edit", path(&map), path(&ops), "-o", path(&out2)]);
    assert_eq!(o.status.code(), Some(2));
    assert!(!out2.exists());
    let err: serde_json::Value = serde_json::from_str(&stdout(&o)).unwrap();
    assert_eq!(err["error"]["code"], "not_found");
}

#[test]
fn validate_reports_errors_with_exit_code() {
    let dir = tempfile::tempdir().unwrap();
    let map = dir.path().join("broken.json");
    std::fs::write(
        &map,
        r#"{"format": "vectormap-ir", "version": 1,
            "boundaries": [{"id": 1, "geometry": [[0,1,0],[10,1,0]]}],
            "lanes": [{"id": 2, "left": 1, "right": 3}],
            "topology": [{"lane": 2, "successors": [7]}]}"#,
    )
    .unwrap();
    let o = vectormap(&["validate", "--json", path(&map)]);
    assert_eq!(o.status.code(), Some(1));
    let report: serde_json::Value = serde_json::from_str(&stdout(&o)).unwrap();
    let codes: Vec<&str> = report["issues"]
        .as_array()
        .unwrap()
        .iter()
        .map(|i| i["code"].as_str().unwrap())
        .collect();
    assert!(codes.contains(&"missing_boundary"), "{codes:?}");
    assert!(codes.contains(&"dangling_lane_reference"), "{codes:?}");
}

#[test]
fn lane_and_nearest_queries() {
    let dir = tempfile::tempdir().unwrap();
    let osm = dir.path().join("map.osm");
    assert!(
        vectormap(&["sample", "straight-road", path(&osm)])
            .status
            .success()
    );
    let o = vectormap(&["nearest", path(&osm), "25", "-1"]);
    assert!(o.status.success(), "{o:?}");
    let n: serde_json::Value = serde_json::from_str(&stdout(&o)).unwrap();
    assert_eq!(n["inside"], true);
    let lane = n["lane"].as_u64().unwrap().to_string();
    let o = vectormap(&["lane", path(&osm), &lane]);
    let info: serde_json::Value = serde_json::from_str(&stdout(&o)).unwrap();
    assert_eq!(info["predecessors"].as_array().unwrap().len(), 1);
    assert_eq!(info["successors"].as_array().unwrap().len(), 1);
}

#[test]
fn unknown_format_is_a_usage_error() {
    let o = vectormap(&["info", "map.xodr"]);
    assert_eq!(o.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&o.stderr).contains("cannot determine the format"));
}

#[test]
fn mcp_server_over_stdio() {
    use std::io::Write;
    use std::process::Stdio;

    let dir = tempfile::tempdir().unwrap();
    let map = dir.path().join("map.osm");
    let out = dir.path().join("edited.osm");
    assert!(
        vectormap(&["sample", "straight-road", path(&map)])
            .status
            .success()
    );

    let mut child = Command::new(env!("CARGO_BIN_EXE_vectormap"))
        .args(["mcp", path(&map)])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let requests = [
        serde_json::json!({"jsonrpc": "2.0", "id": 1, "method": "initialize",
                           "params": {"protocolVersion": "2025-06-18", "capabilities": {},
                                      "clientInfo": {"name": "test", "version": "0"}}}),
        serde_json::json!({"jsonrpc": "2.0", "method": "notifications/initialized"}),
        serde_json::json!({"jsonrpc": "2.0", "id": 2, "method": "tools/call",
                           "params": {"name": "set_speed_limit", "arguments": {"lanes": [3], "kmh": 20}}}),
        serde_json::json!({"jsonrpc": "2.0", "id": 3, "method": "tools/call",
                           "params": {"name": "save_map", "arguments": {"path": out}}}),
    ];
    {
        let stdin = child.stdin.as_mut().unwrap();
        for r in &requests {
            writeln!(stdin, "{r}").unwrap();
        }
    }
    drop(child.stdin.take());
    let output = child.wait_with_output().unwrap();
    assert!(output.status.success(), "{output:?}");
    let replies: Vec<serde_json::Value> = String::from_utf8(output.stdout)
        .unwrap()
        .lines()
        .map(|l| serde_json::from_str(l).expect("stdout carries only JSON-RPC"))
        .collect();
    assert_eq!(replies.len(), 3);
    assert_eq!(replies[0]["result"]["protocolVersion"], "2025-06-18");
    assert_eq!(replies[1]["result"]["isError"], false);
    assert_eq!(replies[2]["result"]["isError"], false);
    let text = std::fs::read_to_string(&out).unwrap();
    assert!(text.contains(r#"<tag k="speed_limit" v="20"/>"#));
}
