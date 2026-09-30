//! MCP protocol and tool tests.

use serde_json::{Value, json};
use vectormap_mcp::{SUPPORTED_PROTOCOL_VERSIONS, Server, serve};

struct Client {
    server: Server,
    next_id: u64,
}

impl Client {
    fn new() -> Self {
        Self {
            server: Server::new(),
            next_id: 1,
        }
    }

    fn request(&mut self, method: &str, params: Value) -> Value {
        let id = self.next_id;
        self.next_id += 1;
        let reply = self
            .server
            .handle(json!({"jsonrpc": "2.0", "id": id, "method": method, "params": params}))
            .expect("requests get a reply");
        assert_eq!(reply["jsonrpc"], "2.0");
        assert_eq!(reply["id"], id);
        reply
    }

    /// Calls a tool and returns `(is_error, structured_content)`.
    fn call(&mut self, name: &str, args: Value) -> (bool, Value) {
        let reply = self.request("tools/call", json!({"name": name, "arguments": args}));
        let result = &reply["result"];
        assert!(result.is_object(), "{reply}");
        // The text content always mirrors the structured content.
        let text = result["content"][0]["text"].as_str().unwrap();
        let parsed: Value = serde_json::from_str(text).unwrap();
        assert_eq!(parsed, result["structuredContent"]);
        (
            result["isError"].as_bool().unwrap(),
            result["structuredContent"].clone(),
        )
    }

    fn ok(&mut self, name: &str, args: Value) -> Value {
        let (err, v) = self.call(name, args);
        assert!(!err, "{name} failed: {v}");
        v
    }

    fn err(&mut self, name: &str, args: Value) -> String {
        let (err, v) = self.call(name, args);
        assert!(err, "{name} should fail: {v}");
        v["error"]["code"].as_str().unwrap().to_string()
    }
}

#[test]
fn initialize_negotiates_the_protocol_version() {
    let mut c = Client::new();
    let r = c.request(
        "initialize",
        json!({"protocolVersion": "2025-03-26", "capabilities": {}}),
    );
    assert_eq!(r["result"]["protocolVersion"], "2025-03-26");
    assert_eq!(r["result"]["serverInfo"]["name"], "vectormap");
    assert!(r["result"]["capabilities"]["tools"].is_object());
    assert!(
        r["result"]["instructions"]
            .as_str()
            .unwrap()
            .contains("open_map")
    );
    let r = c.request("initialize", json!({"protocolVersion": "1999-01-01"}));
    assert_eq!(
        r["result"]["protocolVersion"],
        SUPPORTED_PROTOCOL_VERSIONS[0]
    );
}

#[test]
fn json_rpc_edge_cases() {
    let mut s = Server::new();
    // Notifications get no reply.
    assert!(
        s.handle(json!({"jsonrpc": "2.0", "method": "notifications/initialized"}))
            .is_none()
    );
    // Responses from the client are ignored.
    assert!(
        s.handle(json!({"jsonrpc": "2.0", "id": 9, "result": {}}))
            .is_none()
    );
    let r = s
        .handle(json!({"jsonrpc": "2.0", "id": 1, "method": "ping"}))
        .unwrap();
    assert_eq!(r["result"], json!({}));
    let r = s
        .handle(json!({"jsonrpc": "2.0", "id": 2, "method": "resources/list"}))
        .unwrap();
    assert_eq!(r["error"]["code"], -32601);
    let r = s
        .handle(
            json!({"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {"name": "nope"}}),
        )
        .unwrap();
    assert_eq!(r["error"]["code"], -32602);
    let r: Value = serde_json::from_str(&s.handle_line("{not json").unwrap()).unwrap();
    assert_eq!(r["error"]["code"], -32700);
    // Batches are answered with arrays.
    let r: Value = serde_json::from_str(
        &s.handle_line(
            r#"[{"jsonrpc":"2.0","id":1,"method":"ping"},{"jsonrpc":"2.0","method":"x"}]"#,
        )
        .unwrap(),
    )
    .unwrap();
    assert_eq!(r.as_array().unwrap().len(), 1);
}

#[test]
fn tools_are_listed_with_schemas() {
    let mut c = Client::new();
    let r = c.request("tools/list", json!({}));
    let tools = r["result"]["tools"].as_array().unwrap();
    let names: Vec<&str> = tools.iter().map(|t| t["name"].as_str().unwrap()).collect();
    for required in [
        "get_map_summary",
        "get_lane",
        "find_nearest_lane",
        "create_lane",
        "split_lane",
        "merge_lanes",
        "connect_lanes",
        "add_stop_line",
        "add_traffic_light",
        "add_crosswalk",
        "set_speed_limit",
        "validate_map",
        "export_lanelet2",
    ] {
        assert!(names.contains(&required), "missing {required}");
    }
    let mut unique = names.clone();
    unique.sort();
    unique.dedup();
    assert_eq!(unique.len(), names.len());
    for t in tools {
        assert_eq!(t["inputSchema"]["type"], "object", "{}", t["name"]);
        assert!(t["description"].as_str().unwrap().len() > 20);
        assert!(t["annotations"]["readOnlyHint"].is_boolean());
    }
}

#[test]
fn editing_without_a_map_is_a_tool_error() {
    let mut c = Client::new();
    assert_eq!(c.err("get_map_summary", json!({})), "no_map");
    assert_eq!(
        c.err("split_lane", json!({"lane": 1, "at": {"fraction": 0.5}})),
        "no_map"
    );
    assert_eq!(c.err("undo", json!({})), "nothing_to_undo");
}

#[test]
fn full_editing_workflow() {
    let mut c = Client::new();
    let s = c.ok("new_map", json!({"sample": "intersection"}));
    assert_eq!(s["summary"]["counts"]["lanes"], 20);

    // Discover lanes.
    let junction = c.ok("list_lanes", json!({"in_junction": true, "limit": 5}));
    assert_eq!(junction["total"], 12);
    assert_eq!(junction["lanes"].as_array().unwrap().len(), 5);
    let arms = c.ok("list_lanes", json!({"in_junction": false}));
    assert_eq!(arms["total"], 8);
    let incoming = arms["lanes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|l| l["successors"].as_array().unwrap().len() == 3)
        .unwrap()["id"]
        .as_u64()
        .unwrap();

    let lane = c.ok("get_lane", json!({"lane": incoming}));
    assert_eq!(lane["traffic_signals"].as_array().unwrap().len(), 1);
    assert!(lane["centerline"].as_array().unwrap().len() >= 2);

    // Nearest lane in local and geographic coordinates.
    let near = c.ok("find_nearest_lane", json!({"x": 20.0, "y": 1.5}));
    assert_eq!(near["nearest"]["inside"], true);
    let near_geo = c.ok(
        "find_nearest_lane",
        json!({"lat": 35.6812, "lon": 139.7672}),
    );
    assert!(near_geo["nearest"]["lane"].is_u64());
    assert_eq!(
        c.err("find_nearest_lane", json!({"x": 1.0})),
        "invalid_arguments"
    );

    // Split, then undo.
    let split = c.ok(
        "split_lane",
        json!({"lane": incoming, "at": {"fraction": 0.5}}),
    );
    assert!(
        split["summary"]
            .as_str()
            .unwrap()
            .starts_with("created: lane:")
    );
    assert_eq!(
        c.ok("get_map_summary", json!({}))["summary"]["counts"]["lanes"],
        22
    );
    c.ok("undo", json!({}));
    let s = c.ok("get_map_summary", json!({}));
    assert_eq!(s["summary"]["counts"]["lanes"], 20);
    assert_eq!(s["unsaved_changes"], true);

    // Crosswalk with stop lines and a speed limit change.
    let outgoing = arms["lanes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|l| l["predecessors"].as_array().unwrap().len() == 3)
        .unwrap()["id"]
        .as_u64()
        .unwrap();
    let cw = c.ok(
        "add_crosswalk",
        json!({"geometry": {"across": {"lane": outgoing, "station": 2.5}}, "stop_line_offset": 1.0}),
    );
    assert!(cw["summary"].as_str().unwrap().contains("crosswalk:"));
    c.ok("set_speed_limit", json!({"lanes": [incoming], "kmh": 30}));
    c.ok(
        "add_stop_line",
        json!({"lanes": [outgoing], "placement": {"at_lane_end": {"offset": 5}}}),
    );

    // Validation and Autoware export.
    let report = c.ok("validate_map", json!({"autoware": true}));
    assert_eq!(report["counts"]["errors"], 0, "{report}");
    let dir = tempfile::tempdir().unwrap();
    let osm = dir.path().join("out").join("lanelet2_map.osm");
    let exported = c.ok("export_lanelet2", json!({"path": osm, "autoware": true}));
    assert_eq!(exported["profile"], "autoware");
    assert!(osm.exists());
    assert!(
        dir.path()
            .join("out")
            .join("map_projector_info.yaml")
            .exists()
    );

    // Re-open the export and compare.
    let before = c.ok("get_map_summary", json!({}))["summary"]["counts"].clone();
    let opened = c.ok("open_map", json!({"path": osm}));
    let after = &opened["summary"]["counts"];
    assert_eq!(after["lanes"], before["lanes"]);
    assert_eq!(after["crosswalks"], before["crosswalks"]);
    assert_eq!(after["stop_lines"], before["stop_lines"]);
    assert_eq!(opened["unsaved_changes"], false);
}

#[test]
fn errors_are_structured() {
    let mut c = Client::new();
    c.ok("new_map", json!({"sample": "straight_road"}));
    assert_eq!(c.err("get_lane", json!({"lane": 999})), "not_found");
    assert_eq!(c.err("remove_lane", json!({"lane": 999})), "not_found");
    assert_eq!(
        c.err("split_lane", json!({"lane": 3, "at": {"fraction": 2.0}})),
        "invalid_argument"
    );
    assert_eq!(c.err("split_lane", json!({"lane": 3})), "invalid_arguments");
    assert_eq!(
        c.err("merge_lanes", json!({"first": 6, "second": 3})),
        "not_mergeable"
    );
    assert_eq!(
        c.err("get_entity", json!({"kind": "planet", "id": 1})),
        "invalid_arguments"
    );
    let (_, v) = c.call("merge_lanes", json!({"first": 6, "second": 3}));
    assert!(
        v["error"]["message"].as_str().unwrap().contains("[lane:9]"),
        "{v}"
    );
    // Failed edits do not create undo steps.
    assert_eq!(c.ok("get_map_summary", json!({}))["undo_steps"], 0);
}

#[test]
fn validation_fixes_can_be_applied() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("broken.json");
    std::fs::write(
        &path,
        r#"{"format": "vectormap-ir", "version": 1,
            "boundaries": [
              {"id": 1, "geometry": [[0,1.5],[10,1.5]]}, {"id": 2, "geometry": [[0,-1.5],[10,-1.5]]},
              {"id": 3, "geometry": [[10,1.5],[20,1.5]]}, {"id": 4, "geometry": [[10,-1.5],[20,-1.5]]}],
            "lanes": [{"id": 5, "left": 1, "right": 2}, {"id": 6, "left": 3, "right": 4}],
            "topology": [{"lane": 5, "successors": [6]}]}"#,
    )
    .unwrap();
    let mut c = Client::new();
    c.ok("open_map", json!({"path": path}));
    let report = c.ok("validate_map", json!({}));
    let issue = report["issues"]
        .as_array()
        .unwrap()
        .iter()
        .find(|i| i["code"] == "asymmetric_link")
        .unwrap()
        .clone();
    c.ok("apply_commands", json!({"commands": [issue["fix"]]}));
    let report = c.ok("validate_map", json!({}));
    assert_eq!(report["counts"]["errors"], 0, "{report}");

    // save_map writes back to the opened file.
    let saved = c.ok("save_map", json!({}));
    assert_eq!(saved["format"], "json");
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(text.contains("\"predecessors\": [5]"));
}

#[test]
fn stdio_transport() {
    let input = [
        r#"{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18"}}"#,
        r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#,
        "",
        r#"{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"new_map","arguments":{"sample":"two_lane_road"}}}"#,
        r#"{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"get_map_summary"}}"#,
    ]
    .join("\n");
    let mut out = Vec::new();
    serve(&mut Server::new(), input.as_bytes(), &mut out).unwrap();
    let lines: Vec<Value> = String::from_utf8(out)
        .unwrap()
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    assert_eq!(lines.len(), 3);
    assert_eq!(
        lines[2]["result"]["structuredContent"]["summary"]["counts"]["lanes"],
        4
    );
}

#[test]
fn building_a_map_from_scratch() {
    let mut c = Client::new();
    c.ok("new_map", json!({"name": "built"}));
    // A two-way road along a driven path (forward on the left), cut in two.
    let road = c.ok(
        "build_road",
        json!({
            "reference": [[0, 0, 0], [30, 0.5, 0], [60, 0, 0]],
            "lanes": [{"width": 3.5}, {"width": 3.5, "direction": "backward"}],
            "segment_length": 30,
            "speed_limit": {"kmh": 40}
        }),
    );
    let chains = road["lanes"].as_array().unwrap();
    assert_eq!(chains.len(), 2);
    assert!(chains.iter().all(|c| c.as_array().unwrap().len() == 2));
    let last = chains[0][1].as_u64().unwrap();
    // A second road going north from beyond the end, joined by a left turn.
    let north = c.ok(
        "build_road",
        json!({
            "reference": [[70, 10], [70, 50]],
            "lanes": [{"width": 3.5}],
            "speed_limit": {"kmh": 30}
        }),
    );
    let first_north = north["lanes"][0][0].as_u64().unwrap();
    let connector = c.ok("add_connector", json!({"from": last, "to": first_north}));
    let new_lane = connector["changes"]["created"]
        .as_array()
        .unwrap()
        .iter()
        .find(|e| e["kind"] == "lane")
        .unwrap()["id"]
        .as_u64()
        .unwrap();
    let lane = c.ok("get_lane", json!({"lane": new_lane}));
    assert_eq!(lane["lane"]["turn_direction"], "left", "{lane}");
    assert_eq!(
        c.err(
            "build_road",
            json!({"reference": [[0, 0], [1, 0]], "lanes": []})
        ),
        "invalid_argument"
    );

    let report = c.ok("validate_map", json!({"autoware": true}));
    assert_eq!(report["counts"]["errors"], 0, "{report}");
    assert_eq!(report["counts"]["warnings"], 0, "{report}");
    let dir = tempfile::tempdir().unwrap();
    let out = dir.path().join("lanelet2_map.osm");
    c.ok(
        "export_lanelet2",
        json!({"path": out.to_str().unwrap(), "autoware": true}),
    );
    assert!(dir.path().join("map_projector_info.yaml").exists());
    let reopened = c.ok("open_map", json!({"path": out.to_str().unwrap()}));
    assert_eq!(reopened["summary"]["counts"]["lanes"], 6);
}
