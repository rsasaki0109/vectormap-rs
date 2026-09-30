//! MCP tool definitions and their implementation.
//!
//! Editing tools are thin wrappers: their arguments are exactly the fields
//! of the corresponding [`Command`], so a call is turned into
//! `{"op": <op>, ...arguments}`, deserialized and applied atomically.

use std::path::PathBuf;

use serde_json::{Map as JsonMap, Value, json};
use vectormap_core::{
    Command, EntityKind, EntityRef, GeoPoint, GeoReference, LaneId, Map, Point2, ProjectionKind,
    Severity, samples,
};
use vectormap_io::lanelet2::{self, LoadOptions, ProjectionChoice, SaveOptions};
use vectormap_io::projection::LocalProjector;
use vectormap_io::{Format, autoware, json as irjson};
use vectormap_validation::{ValidationOptions, validate};

use crate::session::{Session, ToolError};

/// Static description of a tool.
pub struct ToolDef {
    /// Tool name.
    pub name: &'static str,
    /// Short human readable title.
    pub title: &'static str,
    /// What the tool does (shown to the model).
    pub description: &'static str,
    /// JSON Schema of the arguments.
    pub input_schema: Value,
    /// The tool does not modify the map or files.
    pub read_only: bool,
    /// The tool may discard data (e.g. remove entities, replace the map).
    pub destructive: bool,
}

impl ToolDef {
    /// The `tools/list` entry.
    pub fn to_json(&self) -> Value {
        json!({
            "name": self.name,
            "title": self.title,
            "description": self.description,
            "inputSchema": self.input_schema,
            "annotations": {
                "title": self.title,
                "readOnlyHint": self.read_only,
                "destructiveHint": self.destructive,
                "idempotentHint": self.read_only,
                "openWorldHint": false,
            },
        })
    }
}

/// Editing tools and the command `op` they map to.
const EDIT_TOOLS: &[(&str, &str)] = &[
    ("build_road", "build_road"),
    ("add_connector", "add_connector"),
    ("set_boundary_geometry", "set_boundary_geometry"),
    ("create_lane", "add_lane"),
    ("remove_lane", "remove_lane"),
    ("split_lane", "split_lane"),
    ("merge_lanes", "merge_lanes"),
    ("connect_lanes", "connect_lanes"),
    ("disconnect_lanes", "disconnect_lanes"),
    ("add_stop_line", "add_stop_line"),
    ("add_traffic_light", "add_traffic_signal"),
    ("add_crosswalk", "add_crosswalk"),
    ("set_speed_limit", "set_speed_limit"),
];

// ---------------------------------------------------------------------------
// Schema building blocks
// ---------------------------------------------------------------------------

fn object(properties: Value, required: &[&str]) -> Value {
    json!({
        "type": "object",
        "properties": properties,
        "required": required,
        "additionalProperties": false,
    })
}

fn id(description: &str) -> Value {
    json!({"type": "integer", "minimum": 1, "description": description})
}

fn ids(description: &str) -> Value {
    json!({"type": "array", "items": {"type": "integer", "minimum": 1}, "minItems": 1, "description": description})
}

fn point() -> Value {
    json!({
        "type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 3,
        "description": "[x, y] or [x, y, z] in metres, in the map's local frame"
    })
}

fn polyline(description: &str) -> Value {
    json!({"type": "array", "items": point(), "minItems": 2, "description": description})
}

fn boundary_kind() -> Value {
    json!({
        "type": "object",
        "description": "Boundary kind, e.g. {\"type\": \"lane_marking\", \"pattern\": \"dashed\"} or {\"type\": \"curb\"}",
        "properties": {
            "type": {"enum": ["virtual", "lane_marking", "curb", "road_edge", "other"]},
            "pattern": {"enum": ["solid", "dashed", "solid_solid", "solid_dashed", "dashed_solid"]},
            "weight": {"enum": ["thin", "thick"]}
        },
        "required": ["type"]
    })
}

fn stop_line_placement() -> Value {
    json!({
        "description": "Where to put the stop line (default: at the end of the first lane)",
        "oneOf": [
            object(json!({"at_lane_end": object(json!({"offset": {"type": "number", "minimum": 0, "description": "metres back from the lane end"}}), &[])}), &["at_lane_end"]),
            object(json!({"at_station": object(json!({"station": {"type": "number", "description": "metres from the lane start"}}), &["station"])}), &["at_station"]),
            object(json!({"geometry": object(json!({"geometry": polyline("explicit stop line")}), &["geometry"])}), &["geometry"])
        ]
    })
}

fn format_arg() -> Value {
    json!({"enum": ["lanelet2", "json"], "description": "file format (default: from the extension: .osm = lanelet2, .json = vectormap IR)"})
}

fn origin_arg() -> Value {
    json!({
        "type": "object",
        "description": "WGS84 origin of the local frame",
        "properties": {"lat": {"type": "number"}, "lon": {"type": "number"}, "alt": {"type": "number"}},
        "required": ["lat", "lon"]
    })
}

fn projection_arg() -> Value {
    json!({"enum": ["utm", "transverse_mercator", "mgrs"], "description": "projection used with origin (default utm); mgrs origin identifies a 100 km UTM grid square"})
}

// ---------------------------------------------------------------------------
// Definitions
// ---------------------------------------------------------------------------

/// All tools, in the order they are listed.
pub fn definitions() -> Vec<ToolDef> {
    vec![
        ToolDef {
            name: "open_map",
            title: "Open map",
            description: "Load a map file (Lanelet2 .osm or vectormap .json) into the session, replacing the current map. Returns a summary and load issues. Coordinates, georeference and topology are derived automatically.",
            input_schema: object(
                json!({
                    "path": {"type": "string", "description": "file to load"},
                    "format": format_arg(),
                    "origin": origin_arg(),
                    "projection": projection_arg(),
                    "local": {"type": "boolean", "description": "Lanelet2: take coordinates from local_x/local_y tags"}
                }),
                &["path"],
            ),
            read_only: false,
            destructive: true,
        },
        ToolDef {
            name: "new_map",
            title: "New map",
            description: "Start a new map in the session, replacing the current one: empty, or one of the built-in samples (straight_road, two_lane_road, intersection).",
            input_schema: object(
                json!({
                    "name": {"type": "string"},
                    "sample": {"enum": ["straight_road", "two_lane_road", "intersection"]},
                    "origin": origin_arg(),
                    "projection": projection_arg()
                }),
                &[],
            ),
            read_only: false,
            destructive: true,
        },
        ToolDef {
            name: "save_map",
            title: "Save map",
            description: "Write the current map to a file (default: the file it was opened from). Use format=json for the lossless vectormap IR, lanelet2 for OSM XML; autoware=true applies the Autoware profile and also writes map_projector_info.yaml next to the file.",
            input_schema: object(
                json!({
                    "path": {"type": "string"},
                    "format": format_arg(),
                    "autoware": {"type": "boolean"}
                }),
                &[],
            ),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "export_lanelet2",
            title: "Export Lanelet2",
            description: "Export the current map as Lanelet2 OSM XML. With autoware=true (recommended for Autoware) the Autoware profile is used, map_projector_info.yaml is written next to the file and Autoware compatibility issues are returned. Does not change where save_map writes.",
            input_schema: object(
                json!({
                    "path": {"type": "string", "description": "output .osm file, e.g. out/lanelet2_map.osm"},
                    "autoware": {"type": "boolean"}
                }),
                &["path"],
            ),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "undo",
            title: "Undo",
            description: "Revert the last successful editing call (up to 50 steps).",
            input_schema: object(json!({}), &[]),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "get_map_summary",
            title: "Map summary",
            description: "Entity counts, lane kinds, rule types, total lane length, bounding box (local metres), topology statistics, georeference, file and unsaved-changes state.",
            input_schema: object(json!({}), &[]),
            read_only: true,
            destructive: false,
        },
        ToolDef {
            name: "list_lanes",
            title: "List lanes",
            description: "Compact list of lanes with their topology (predecessors, successors, neighbors), length, speed limit, road/junction and turn direction. Filter by kind, junction membership or distance to a point; paginate with offset/limit.",
            input_schema: object(
                json!({
                    "kind": {"enum": ["driving", "shoulder", "bus", "bicycle", "walkway", "parking", "emergency", "other"]},
                    "in_junction": {"type": "boolean"},
                    "near": object(json!({"x": {"type": "number"}, "y": {"type": "number"}, "radius": {"type": "number", "minimum": 0}}), &["x", "y", "radius"]),
                    "offset": {"type": "integer", "minimum": 0},
                    "limit": {"type": "integer", "minimum": 1, "maximum": 500, "description": "default 50"}
                }),
                &[],
            ),
            read_only: true,
            destructive: false,
        },
        ToolDef {
            name: "get_lane",
            title: "Get lane",
            description: "Everything about one lane: kind, boundaries, speed limit, predecessors, successors, neighbors, road, junction, rules, stop lines, traffic signals, length and centreline points.",
            input_schema: object(json!({"lane": id("lane ID")}), &["lane"]),
            read_only: true,
            destructive: false,
        },
        ToolDef {
            name: "get_entity",
            title: "Get entity",
            description: "The full JSON of any entity (lane, boundary, road, junction, stop_line, traffic_signal, crosswalk, regulatory_element).",
            input_schema: object(
                json!({
                    "kind": {"enum": ["lane", "boundary", "road", "junction", "stop_line", "traffic_signal", "crosswalk", "regulatory_element"]},
                    "id": id("entity ID")
                }),
                &["kind", "id"],
            ),
            read_only: true,
            destructive: false,
        },
        ToolDef {
            name: "find_nearest_lane",
            title: "Find nearest lane",
            description: "The lane closest to a point, given in local metres (x, y) or, for georeferenced maps, in WGS84 (lat, lon). Returns the lane, distance, station along the lane, lateral offset (positive = left) and whether the point is inside the lane.",
            input_schema: json!({
                "type": "object",
                "properties": {
                    "x": {"type": "number"}, "y": {"type": "number"},
                    "lat": {"type": "number"}, "lon": {"type": "number"}
                },
                "additionalProperties": false
            }),
            read_only: true,
            destructive: false,
        },
        ToolDef {
            name: "validate_map",
            title: "Validate map",
            description: "Run structured validation. Every issue has severity, a stable code, message, entity and — when the repair is unambiguous — a `fix` command that can be passed to apply_commands. autoware=true adds Autoware convention checks.",
            input_schema: object(
                json!({
                    "autoware": {"type": "boolean"},
                    "include_info": {"type": "boolean", "description": "include informational issues (default false)"}
                }),
                &[],
            ),
            read_only: true,
            destructive: false,
        },
        ToolDef {
            name: "build_road",
            title: "Build road",
            description: "Build a whole road from a reference line (a road centre, a driven path, a surveyed edge): lanes side by side, left to right looking along the line, each forward (along it) or backward. Boundaries are shared, neighbor relations (same or opposite direction) and boundary kinds (solid edges, dashed lane lines, solid centre line) are set. Give lane widths (the road is centred on the line unless left_edge says where its left edge is), or explicit boundary lines from left to right (one more than lanes), e.g. lane markings found in a point cloud. segment_length cuts the road into connected pieces; resample thins a dense line. The result lists the new lanes; call list_lanes for details. For left-hand traffic (Japan, UK) put forward lanes on the left, for right-hand traffic on the right.",
            input_schema: object(
                json!({
                    "reference": polyline("reference line"),
                    "lanes": {
                        "type": "array", "minItems": 1,
                        "description": "lanes from left to right looking along the reference line",
                        "items": object(json!({
                            "width": {"type": "number", "exclusiveMinimum": 0, "description": "metres; required without boundaries"},
                            "direction": {"enum": ["forward", "backward"], "description": "default forward"},
                            "kind": {"enum": ["driving", "shoulder", "bus", "bicycle", "walkway", "parking", "emergency", "other"]}
                        }), &[])
                    },
                    "left_edge": {"type": "number", "description": "lateral position of the road's left edge from the reference line (metres, positive = left); default: centred"},
                    "boundaries": {"type": "array", "items": polyline("boundary line"), "description": "explicit boundary lines from left to right, one more than lanes; replaces widths and left_edge"},
                    "resample": {"type": "number", "exclusiveMinimum": 0, "description": "resample the lines at this spacing (metres) first"},
                    "segment_length": {"type": "number", "exclusiveMinimum": 0, "description": "cut into connected pieces of about this length (metres)"},
                    "speed_limit": object(json!({"kmh": {"type": "number", "exclusiveMinimum": 0}}), &["kmh"]),
                    "edge_kind": boundary_kind(),
                    "lane_line_kind": boundary_kind(),
                    "center_line_kind": boundary_kind(),
                    "name": {"type": "string", "description": "also create a named road entity (not exported to Lanelet2)"}
                }),
                &["reference", "lanes"],
            ),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "add_connector",
            title: "Add connector lane",
            description: "Join the end of lane `from` to the start of lane `to` with a new lane whose boundaries curve smoothly between them (as inside a junction), connected both ways. turn_direction (straight/left/right) is derived from the change of heading unless given; the speed limit defaults to the lower of the two lanes'; boundaries are virtual unless boundary_kind is given.",
            input_schema: object(
                json!({
                    "from": id("lane the connector starts from (at its end)"),
                    "to": id("lane the connector leads to (at its start)"),
                    "turn_direction": {"enum": ["straight", "left", "right"]},
                    "junction": id("junction to add the connector to"),
                    "speed_limit": object(json!({"kmh": {"type": "number", "exclusiveMinimum": 0}}), &["kmh"]),
                    "boundary_kind": boundary_kind()
                }),
                &["from", "to"],
            ),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "set_boundary_geometry",
            title: "Set boundary geometry",
            description: "Replace the line of a boundary, e.g. to snap it to an observed lane marking, in the boundary's own direction (see get_entity). Warns when a lane using it no longer meets its predecessors or successors.",
            input_schema: object(
                json!({
                    "boundary": id("boundary"),
                    "geometry": polyline("new line, in the boundary's own direction")
                }),
                &["boundary", "geometry"],
            ),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "create_lane",
            title: "Create lane",
            description: "Create a lane. geometry is one of: {\"centerline\": {\"centerline\": [[x,y],...], \"width\": 3.5}}; {\"beside_lane\": {\"lane\": ID, \"side\": \"left\"|\"right\", \"width\": 3.5}} (shares that lane's boundary and becomes its neighbor); {\"boundaries\": {\"left\": {\"existing\": BOUNDARY_ID} | {\"new\": {\"geometry\": [...]}}, \"right\": ...}}. Optionally connect predecessors/successors and set speed limit.",
            input_schema: object(
                json!({
                    "geometry": {
                        "oneOf": [
                            object(json!({"centerline": object(json!({
                                "centerline": polyline("centreline in the direction of travel"),
                                "width": {"type": "number", "exclusiveMinimum": 0},
                                "left_kind": boundary_kind(),
                                "right_kind": boundary_kind()
                            }), &["centerline", "width"])}), &["centerline"]),
                            object(json!({"beside_lane": object(json!({
                                "lane": id("existing lane"),
                                "side": {"enum": ["left", "right"]},
                                "width": {"type": "number", "exclusiveMinimum": 0},
                                "outer_kind": boundary_kind(),
                                "shared_kind": boundary_kind()
                            }), &["lane", "side", "width"])}), &["beside_lane"]),
                            object(json!({"boundaries": object(json!({
                                "left": {"type": "object", "description": "{\"existing\": BOUNDARY_ID} or {\"new\": {\"geometry\": [...], \"kind\": {...}}}"},
                                "right": {"type": "object", "description": "{\"existing\": BOUNDARY_ID} or {\"new\": {\"geometry\": [...], \"kind\": {...}}}"}
                            }), &["left", "right"])}), &["boundaries"])
                        ]
                    },
                    "kind": {"enum": ["driving", "shoulder", "bus", "bicycle", "walkway", "parking", "emergency", "other"]},
                    "speed_limit": object(json!({"kmh": {"type": "number", "exclusiveMinimum": 0}}), &["kmh"]),
                    "one_way": {"type": "boolean"},
                    "turn_direction": {"enum": ["straight", "left", "right"]},
                    "predecessors": {"type": "array", "items": {"type": "integer"}},
                    "successors": {"type": "array", "items": {"type": "integer"}},
                    "road": id("road to add the lane to"),
                    "junction": id("junction to add the lane to")
                }),
                &["geometry"],
            ),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "remove_lane",
            title: "Remove lane",
            description: "Remove a lane with its topology links, road/junction/rule memberships and boundaries no longer used by other lanes.",
            input_schema: object(json!({"lane": id("lane to remove")}), &["lane"]),
            read_only: false,
            destructive: true,
        },
        ToolDef {
            name: "split_lane",
            title: "Split lane",
            description: "Split a lane in two at a fraction of its length or a station in metres. The lane keeps its ID as the first piece; the second piece is created. By default its lateral neighbors are split at the same cross-section so shared boundaries and neighbor relations stay consistent.",
            input_schema: object(
                json!({
                    "lane": id("lane to split"),
                    "at": {
                        "oneOf": [
                            object(json!({"fraction": {"type": "number", "exclusiveMinimum": 0, "exclusiveMaximum": 1}}), &["fraction"]),
                            object(json!({"station": {"type": "number", "exclusiveMinimum": 0, "description": "metres from the lane start"}}), &["station"])
                        ]
                    },
                    "include_neighbors": {"type": "boolean", "description": "default true"}
                }),
                &["lane", "at"],
            ),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "merge_lanes",
            title: "Merge lanes",
            description: "Merge `second` into `first`. `second` must be the only successor of `first` and `first` the only predecessor of `second`.",
            input_schema: object(
                json!({"first": id("lane that is kept"), "second": id("its successor, merged into first")}),
                &["first", "second"],
            ),
            read_only: false,
            destructive: true,
        },
        ToolDef {
            name: "connect_lanes",
            title: "Connect lanes",
            description: "Make `to` a successor of `from`. Warns if the lanes do not meet geometrically (Lanelet2 needs shared end points to keep the link).",
            input_schema: object(
                json!({"from": id("predecessor"), "to": id("successor")}),
                &["from", "to"],
            ),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "disconnect_lanes",
            title: "Disconnect lanes",
            description: "Remove the successor link `from → to`.",
            input_schema: object(
                json!({"from": id("predecessor"), "to": id("successor")}),
                &["from", "to"],
            ),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "add_stop_line",
            title: "Add stop line",
            description: "Add a stop line across one or more lanes (the first lane is the placement reference) and the rule that makes vehicles stop: rule=stop_sign (default, Autoware's convention for plain stop lines), marking, or none (e.g. to attach a traffic light later).",
            input_schema: object(
                json!({
                    "lanes": ids("lanes the stop line spans and applies to"),
                    "placement": stop_line_placement(),
                    "rule": {"enum": ["stop_sign", "marking", "none"]}
                }),
                &["lanes"],
            ),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "add_traffic_light",
            title: "Add traffic light",
            description: "Add a traffic light controlling lanes. stop_line: \"auto\" (default: reuse the stop line of an existing stop rule on the first lane, otherwise create one at its end), {\"existing\": ID}, {\"new\": PLACEMENT} or \"none\". Without geometry the signal head is placed above the stop line (review the warning). Stop signs on the same stop line are replaced. group: add the head to an existing traffic-light rule.",
            input_schema: object(
                json!({
                    "lanes": ids("controlled lanes"),
                    "stop_line": {
                        "oneOf": [
                            {"enum": ["auto", "none"]},
                            object(json!({"existing": id("stop line ID")}), &["existing"]),
                            object(json!({"new": stop_line_placement()}), &["new"])
                        ]
                    },
                    "geometry": polyline("bottom edge of the signal housing, left to right as seen by the traffic, at its real elevation"),
                    "height": {"type": "number", "exclusiveMinimum": 0, "description": "housing height in metres (default 0.5)"},
                    "kind": {"enum": ["vehicle", "pedestrian"]},
                    "group": id("existing traffic-light regulatory element")
                }),
                &["lanes"],
            ),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "add_crosswalk",
            title: "Add crosswalk",
            description: "Add a crosswalk. geometry {\"across\": {\"lane\": ID, \"station\": metres, \"width\": 4, \"margin\": 0.5}} spans the lane and all its lateral neighbors; or {\"edges\": {\"left_edge\": [...], \"right_edge\": [...]}}. Crossing lanes get a crosswalk rule (detected from geometry unless crossing_lanes is given); stop_line_offset creates stop lines that many metres before the crosswalk.",
            input_schema: object(
                json!({
                    "geometry": {
                        "oneOf": [
                            object(json!({"across": object(json!({
                                "lane": id("reference lane"),
                                "station": {"type": "number", "minimum": 0},
                                "width": {"type": "number", "exclusiveMinimum": 0},
                                "margin": {"type": "number", "minimum": 0}
                            }), &["lane", "station"])}), &["across"]),
                            object(json!({"edges": object(json!({
                                "left_edge": polyline("left side in the walking direction"),
                                "right_edge": polyline("right side in the walking direction")
                            }), &["left_edge", "right_edge"])}), &["edges"])
                        ]
                    },
                    "crossing_lanes": {"type": "array", "items": {"type": "integer"}},
                    "stop_line_offset": {"type": "number", "minimum": 0}
                }),
                &["geometry"],
            ),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "set_speed_limit",
            title: "Set speed limit",
            description: "Set (kmh) or clear (kmh = null) the speed limit of lanes.",
            input_schema: object(
                json!({
                    "lanes": ids("lanes"),
                    "kmh": {"type": ["number", "null"], "exclusiveMinimum": 0}
                }),
                &["lanes", "kmh"],
            ),
            read_only: false,
            destructive: false,
        },
        ToolDef {
            name: "apply_commands",
            title: "Apply commands",
            description: "Apply a list of vectormap commands atomically (all or nothing). Each command is an object with an `op` field: build_road, add_connector, set_boundary_geometry, add_lane, remove_lane, remove_entity, connect_lanes, disconnect_lanes, set_neighbor, split_lane, merge_lanes, add_stop_line, add_traffic_signal, add_crosswalk, set_speed_limit, set_turn_direction, set_lane_kind, set_boundary_kind, set_attribute, add_road, add_junction. The `fix` of a validation issue can be passed as is.",
            input_schema: object(
                json!({
                    "commands": {
                        "type": "array",
                        "items": {"type": "object", "properties": {"op": {"type": "string"}}, "required": ["op"]},
                        "minItems": 1
                    }
                }),
                &["commands"],
            ),
            read_only: false,
            destructive: true,
        },
    ]
}

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------

/// Outcome of [`call`].
pub enum CallOutcome {
    /// The tool ran; `Ok` is a structured result, `Err` a tool error.
    Done(Result<Value, ToolError>),
    /// No such tool.
    UnknownTool,
}

/// Runs a tool.
pub fn call(session: &mut Session, name: &str, args: Value) -> CallOutcome {
    let args = match args {
        Value::Null => Value::Object(JsonMap::new()),
        Value::Object(_) => args,
        _ => {
            return CallOutcome::Done(Err(ToolError::new(
                "invalid_arguments",
                "arguments must be an object",
            )));
        }
    };
    if let Some((_, op)) = EDIT_TOOLS.iter().find(|(t, _)| *t == name) {
        return CallOutcome::Done(edit(session, op, args));
    }
    let result = match name {
        "open_map" => open_map(session, &args),
        "new_map" => new_map(session, &args),
        "save_map" => save_map(session, &args),
        "export_lanelet2" => export_lanelet2(session, &args),
        "undo" => session.undo().and_then(|()| summary(session)),
        "get_map_summary" => summary(session),
        "list_lanes" => list_lanes(session, &args),
        "get_lane" => get_lane(session, &args),
        "get_entity" => get_entity(session, &args),
        "find_nearest_lane" => find_nearest_lane(session, &args),
        "validate_map" => validate_map(session, &args),
        "apply_commands" => apply_commands(session, &args),
        _ => return CallOutcome::UnknownTool,
    };
    CallOutcome::Done(result)
}

fn invalid(message: impl Into<String>) -> ToolError {
    ToolError::new("invalid_arguments", message)
}

fn str_arg<'a>(args: &'a Value, key: &str) -> Result<Option<&'a str>, ToolError> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(s)) => Ok(Some(s)),
        Some(_) => Err(invalid(format!("`{key}` must be a string"))),
    }
}

fn bool_arg(args: &Value, key: &str) -> Result<bool, ToolError> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(false),
        Some(Value::Bool(b)) => Ok(*b),
        Some(_) => Err(invalid(format!("`{key}` must be a boolean"))),
    }
}

fn num_arg(args: &Value, key: &str) -> Result<Option<f64>, ToolError> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(v) => v
            .as_f64()
            .map(Some)
            .ok_or_else(|| invalid(format!("`{key}` must be a number"))),
    }
}

fn id_arg(args: &Value, key: &str) -> Result<u64, ToolError> {
    args.get(key)
        .and_then(Value::as_u64)
        .ok_or_else(|| invalid(format!("`{key}` must be a positive integer ID")))
}

fn format_of(path: &std::path::Path, args: &Value) -> Result<Format, ToolError> {
    match str_arg(args, "format")? {
        Some(f) => f.parse::<Format>().map_err(invalid),
        None => Format::from_path(path).ok_or_else(|| {
            invalid(format!(
                "cannot determine the format of {}; pass format",
                path.display()
            ))
        }),
    }
}

fn georeference_arg(args: &Value) -> Result<Option<GeoReference>, ToolError> {
    let Some(origin) = args.get("origin").filter(|v| !v.is_null()) else {
        return Ok(None);
    };
    let origin: GeoPoint = serde_json::from_value(origin.clone())
        .map_err(|e| invalid(format!("invalid `origin`: {e}")))?;
    let projection = match str_arg(args, "projection")? {
        None | Some("utm") => ProjectionKind::Utm,
        Some("transverse_mercator") => ProjectionKind::TransverseMercator,
        Some("mgrs") => ProjectionKind::Mgrs,
        Some(other) => return Err(invalid(format!("unknown projection {other:?}"))),
    };
    if projection == ProjectionKind::Mgrs && vectormap_io::projection::mgrs_grid(origin).is_none() {
        return Err(invalid(
            "MGRS origin must identify a non-polar UTM grid square",
        ));
    }
    Ok(Some(GeoReference { projection, origin }))
}

fn issues_json(issues: &[vectormap_core::Issue]) -> Value {
    serde_json::to_value(issues).unwrap_or(Value::Null)
}

// ---------------------------------------------------------------------------
// Session tools
// ---------------------------------------------------------------------------

fn summary(session: &Session) -> Result<Value, ToolError> {
    let map = session.map()?;
    Ok(json!({
        "file": session.source().map(|(p, f)| json!({"path": p, "format": f.name()})),
        "unsaved_changes": session.is_dirty(),
        "undo_steps": session.undo_depth(),
        "summary": map.summary(),
    }))
}

fn open_map(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let path = PathBuf::from(str_arg(args, "path")?.ok_or_else(|| invalid("`path` is required"))?);
    let format = format_of(&path, args)?;
    let loaded = match format {
        Format::Json => irjson::load(&path)?,
        Format::Lanelet2 => {
            let projection = if bool_arg(args, "local")? {
                ProjectionChoice::LocalTags
            } else if let Some(g) = georeference_arg(args)? {
                ProjectionChoice::Georeferenced(g)
            } else {
                ProjectionChoice::Auto
            };
            lanelet2::load_lanelet2(
                &path,
                &LoadOptions {
                    projection,
                    ..Default::default()
                },
            )?
        }
    };
    session.open(loaded.map, Some((path, format)));
    let mut result = summary(session)?;
    result["load_issues"] = issues_json(&loaded.issues);
    Ok(result)
}

fn new_map(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let mut map = match str_arg(args, "sample")? {
        None => Map::new(),
        Some("straight_road") => samples::straight_road().0,
        Some("two_lane_road") => samples::two_lane_road().0,
        Some("intersection") => samples::intersection().0,
        Some(other) => return Err(invalid(format!("unknown sample {other:?}"))),
    };
    if let Some(name) = str_arg(args, "name")? {
        map.metadata_mut().name = Some(name.to_string());
    }
    if let Some(g) = georeference_arg(args)? {
        map.metadata_mut().georeference = Some(g);
    }
    session.open(map, None);
    summary(session)
}

fn save_to(
    session: &mut Session,
    path: PathBuf,
    format: Format,
    autoware_profile: bool,
) -> Result<Vec<vectormap_core::Issue>, ToolError> {
    let map = session.map()?;
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)
            .map_err(|e| ToolError::new("io_error", format!("{}: {e}", parent.display())))?;
    }
    match format {
        Format::Json => {
            if autoware_profile {
                return Err(invalid("autoware only applies to Lanelet2 output"));
            }
            irjson::save(map, &path)?;
            Ok(Vec::new())
        }
        Format::Lanelet2 if autoware_profile => {
            let mut issues = autoware::check(map);
            issues.extend(lanelet2::save_lanelet2(
                map,
                &path,
                &SaveOptions::autoware(),
            )?);
            let yaml = path
                .parent()
                .unwrap_or(std::path::Path::new("."))
                .join(autoware::PROJECTOR_INFO_FILE_NAME);
            std::fs::write(
                &yaml,
                autoware::projector_info_yaml(map.metadata().georeference),
            )
            .map_err(|e| ToolError::new("io_error", format!("{}: {e}", yaml.display())))?;
            Ok(issues)
        }
        Format::Lanelet2 => Ok(lanelet2::save_lanelet2(
            map,
            &path,
            &SaveOptions::default(),
        )?),
    }
}

fn save_map(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    session.map()?;
    let (path, format) = match str_arg(args, "path")? {
        Some(p) => {
            let path = PathBuf::from(p);
            let format = format_of(&path, args)?;
            (path, format)
        }
        None => session
            .source()
            .cloned()
            .ok_or_else(|| invalid("the map was not opened from a file; pass `path`"))?,
    };
    let autoware_profile = bool_arg(args, "autoware")?;
    let issues = save_to(session, path.clone(), format, autoware_profile)?;
    session.saved((path.clone(), format));
    Ok(json!({"saved": path, "format": format.name(), "issues": issues_json(&issues)}))
}

fn export_lanelet2(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let path = PathBuf::from(str_arg(args, "path")?.ok_or_else(|| invalid("`path` is required"))?);
    let autoware_profile = bool_arg(args, "autoware")?;
    let issues = save_to(session, path.clone(), Format::Lanelet2, autoware_profile)?;
    let mut files = vec![path.clone()];
    if autoware_profile {
        files.push(
            path.parent()
                .unwrap_or(std::path::Path::new("."))
                .join(autoware::PROJECTOR_INFO_FILE_NAME),
        );
    }
    Ok(json!({
        "written": files,
        "profile": if autoware_profile { "autoware" } else { "lanelet2" },
        "issues": issues_json(&issues),
    }))
}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

fn lane_row(map: &Map, lane: &vectormap_core::Lane) -> Value {
    let topo = map.topology().links(lane.id).cloned().unwrap_or_default();
    json!({
        "id": lane.id,
        "kind": lane.kind,
        "length": map.lane_length(lane.id).map(|l| (l * 1000.0).round() / 1000.0),
        "speed_limit_kmh": lane.speed_limit.map(|s| s.kmh()),
        "turn_direction": lane.turn_direction,
        "predecessors": topo.predecessors,
        "successors": topo.successors,
        "left": topo.left,
        "right": topo.right,
        "road": map.road_of(lane.id),
        "junction": map.junction_of(lane.id),
    })
}

fn list_lanes(session: &Session, args: &Value) -> Result<Value, ToolError> {
    let map = session.map()?;
    let kind = str_arg(args, "kind")?;
    let in_junction = match args.get("in_junction") {
        None | Some(Value::Null) => None,
        Some(Value::Bool(b)) => Some(*b),
        Some(_) => return Err(invalid("`in_junction` must be a boolean")),
    };
    let near = match args.get("near").filter(|v| !v.is_null()) {
        None => None,
        Some(n) => Some((
            num_arg(n, "x")?.ok_or_else(|| invalid("near.x is required"))?,
            num_arg(n, "y")?.ok_or_else(|| invalid("near.y is required"))?,
            num_arg(n, "radius")?.ok_or_else(|| invalid("near.radius is required"))?,
        )),
    };
    let offset = args.get("offset").and_then(Value::as_u64).unwrap_or(0) as usize;
    let limit = args
        .get("limit")
        .and_then(Value::as_u64)
        .unwrap_or(50)
        .clamp(1, 500) as usize;
    let matching: Vec<&vectormap_core::Lane> = map
        .lanes()
        .filter(|l| kind.is_none_or(|k| l.kind.as_str() == k))
        .filter(|l| in_junction.is_none_or(|j| map.junction_of(l.id).is_some() == j))
        .filter(|l| {
            near.is_none_or(|(x, y, r)| {
                map.centerline(l.id)
                    .and_then(|c| c.distance_to(Point2::new(x, y)))
                    .is_some_and(|d| d <= r)
            })
        })
        .collect();
    let rows: Vec<Value> = matching
        .iter()
        .skip(offset)
        .take(limit)
        .map(|l| lane_row(map, l))
        .collect();
    Ok(json!({"total": matching.len(), "offset": offset, "lanes": rows}))
}

fn get_lane(session: &Session, args: &Value) -> Result<Value, ToolError> {
    let map = session.map()?;
    let lane = LaneId(id_arg(args, "lane")?);
    let info = map
        .lane_info(lane)
        .ok_or_else(|| ToolError::new("not_found", format!("{lane} does not exist")))?;
    let mut v = serde_json::to_value(&info).unwrap_or(Value::Null);
    if let Some(c) = map.centerline(lane) {
        let c = if c.len() > 50 {
            c.resample_count(50)
        } else {
            c
        };
        v["centerline"] = serde_json::to_value(&c).unwrap_or(Value::Null);
    }
    Ok(v)
}

fn get_entity(session: &Session, args: &Value) -> Result<Value, ToolError> {
    let map = session.map()?;
    let kind: EntityKind = serde_json::from_value(args.get("kind").cloned().unwrap_or(Value::Null))
        .map_err(|_| invalid("`kind` must be an entity kind such as \"lane\" or \"stop_line\""))?;
    let raw = id_arg(args, "id")?;
    let entity: EntityRef =
        serde_json::from_value(json!({"kind": kind, "id": raw})).expect("valid entity ref");
    let value = match entity {
        EntityRef::Lane(id) => map.lane(id).map(serde_json::to_value),
        EntityRef::Boundary(id) => map.boundary(id).map(serde_json::to_value),
        EntityRef::Road(id) => map.road(id).map(serde_json::to_value),
        EntityRef::Junction(id) => map.junction(id).map(serde_json::to_value),
        EntityRef::StopLine(id) => map.stop_line(id).map(serde_json::to_value),
        EntityRef::TrafficSignal(id) => map.traffic_signal(id).map(serde_json::to_value),
        EntityRef::Crosswalk(id) => map.crosswalk(id).map(serde_json::to_value),
        EntityRef::RegulatoryElement(id) => map.regulatory_element(id).map(serde_json::to_value),
    };
    match value {
        Some(Ok(v)) => Ok(v),
        _ => Err(ToolError::new(
            "not_found",
            format!("{entity} does not exist"),
        )),
    }
}

fn find_nearest_lane(session: &Session, args: &Value) -> Result<Value, ToolError> {
    let map = session.map()?;
    let point = match (
        num_arg(args, "x")?,
        num_arg(args, "y")?,
        num_arg(args, "lat")?,
        num_arg(args, "lon")?,
    ) {
        (Some(x), Some(y), None, None) => Point2::new(x, y),
        (None, None, Some(lat), Some(lon)) => {
            let g = map.metadata().georeference.ok_or_else(|| {
                invalid("the map has no georeference; pass local x/y instead of lat/lon")
            })?;
            LocalProjector::new(g).forward(GeoPoint::new(lat, lon)).xy()
        }
        _ => return Err(invalid("pass either x and y, or lat and lon")),
    };
    let nearest = map
        .find_nearest_lane(point)
        .ok_or_else(|| ToolError::new("not_found", "the map has no lanes"))?;
    Ok(json!({"query": [point.x, point.y], "nearest": nearest}))
}

fn validate_map(session: &Session, args: &Value) -> Result<Value, ToolError> {
    let map = session.map()?;
    let include_info = bool_arg(args, "include_info")?;
    let options = ValidationOptions {
        include_info,
        ..Default::default()
    };
    let mut extra = Vec::new();
    if bool_arg(args, "autoware")? {
        extra = autoware::check(map);
        if !include_info {
            extra.retain(|i| i.severity > Severity::Info);
        }
    }
    let report = validate(map, &options).merge(extra);
    Ok(serde_json::to_value(&report).unwrap_or(Value::Null))
}

// ---------------------------------------------------------------------------
// Editing
// ---------------------------------------------------------------------------

fn describe(changes: &[vectormap_core::ChangeSet]) -> String {
    let mut parts = Vec::new();
    let list = |refs: Vec<EntityRef>| {
        refs.iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join(", ")
    };
    let collect = |f: fn(&vectormap_core::ChangeSet) -> &Vec<EntityRef>| {
        let mut v: Vec<EntityRef> = changes.iter().flat_map(|c| f(c).iter().copied()).collect();
        v.sort();
        v.dedup();
        v
    };
    for (label, refs) in [
        ("created", collect(|c| &c.created)),
        ("modified", collect(|c| &c.modified)),
        ("deleted", collect(|c| &c.deleted)),
    ] {
        if !refs.is_empty() {
            parts.push(format!("{label}: {}", list(refs)));
        }
    }
    let warnings: usize = changes.iter().map(|c| c.warnings.len()).sum();
    if warnings > 0 {
        parts.push(format!("{warnings} warning(s)"));
    }
    if parts.is_empty() {
        "no changes".into()
    } else {
        parts.join("; ")
    }
}

fn apply(session: &mut Session, commands: Vec<Command>) -> Result<Value, ToolError> {
    let changes = session.apply(&commands)?;
    let summary = describe(&changes);
    let changes_json = if changes.len() == 1 {
        serde_json::to_value(&changes[0])
    } else {
        serde_json::to_value(&changes)
    }
    .unwrap_or(Value::Null);
    Ok(json!({"summary": summary, "changes": changes_json, "undo_steps": session.undo_depth()}))
}

fn edit(session: &mut Session, op: &str, mut args: Value) -> Result<Value, ToolError> {
    args.as_object_mut()
        .expect("checked")
        .insert("op".into(), Value::String(op.into()));
    let command: Command = serde_json::from_value(args)
        .map_err(|e| invalid(format!("invalid arguments for {op}: {e}")))?;
    let mut result = apply(session, vec![command])?;
    if op == "build_road" {
        result["lanes"] = road_chains(session.map()?, &result["changes"]);
    }
    Ok(result)
}

/// The lanes a `build_road` created, as one chain of pieces per lane of the
/// cross-section (left to right, each in its direction of travel).
fn road_chains(map: &Map, changes: &Value) -> Value {
    let created: Vec<LaneId> = changes["created"]
        .as_array()
        .into_iter()
        .flatten()
        .filter(|e| e["kind"] == "lane")
        .filter_map(|e| e["id"].as_u64().map(LaneId))
        .collect();
    // Lanes are numbered left to right before the road is cut, so the heads
    // (pieces without a predecessor in the road) come out in order.
    let chains: Vec<Vec<LaneId>> = created
        .iter()
        .filter(|l| !map.predecessors(**l).iter().any(|p| created.contains(p)))
        .map(|&head| {
            let mut chain = vec![head];
            while let [next] = map.successors(*chain.last().expect("non-empty")) {
                if !created.contains(next) || chain.contains(next) {
                    break;
                }
                chain.push(*next);
            }
            chain
        })
        .collect();
    serde_json::to_value(chains).unwrap_or(Value::Null)
}

fn apply_commands(session: &mut Session, args: &Value) -> Result<Value, ToolError> {
    let list = args
        .get("commands")
        .and_then(Value::as_array)
        .ok_or_else(|| invalid("`commands` must be an array"))?;
    let mut commands = Vec::with_capacity(list.len());
    for (i, c) in list.iter().enumerate() {
        commands.push(
            serde_json::from_value::<Command>(c.clone())
                .map_err(|e| invalid(format!("command #{i} is invalid: {e}")))?,
        );
    }
    apply(session, commands)
}
