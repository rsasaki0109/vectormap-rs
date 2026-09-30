# High-level commands (MCP-ready API)

`vectormap-rs` is designed so that an agent (Claude Code via MCP, a script,
an editor) never edits OSM XML or internal data structures. It reads
structured views of the map and sends **high-level commands**; the library
validates and applies them deterministically and answers with a **change
set** or a structured **error**.

```text
agent ── get_map_summary / get_lane / find_nearest_lane / validate_map ──▶ JSON views
agent ── {"op": "split_lane", ...} ──▶ Map::apply ──▶ ChangeSet | EditError
```

## Future MCP tools → library calls

| MCP tool | Call |
|---|---|
| `get_map_summary` | `map.summary()` → `MapSummary` |
| `get_lane` | `map.lane_info(id)` → `LaneInfo` |
| `find_nearest_lane` | `map.find_nearest_lane(point)` → `NearestLane` |
| `create_lane` | `Command::AddLane` |
| `split_lane` | `Command::SplitLane` |
| `merge_lanes` | `Command::MergeLanes` |
| `connect_lanes` | `Command::ConnectLanes` |
| `add_stop_line` | `Command::AddStopLine` |
| `add_traffic_light` | `Command::AddTrafficSignal` |
| `add_crosswalk` | `Command::AddCrosswalk` |
| `set_speed_limit` | `Command::SetSpeedLimit` |
| `validate_map` | `vectormap_validation::validate` → `ValidationReport` |
| `export_lanelet2` | `vectormap_io::lanelet2::save_lanelet2` / `autoware::save` |

Today the same flow is available from the CLI:

```bash
vectormap edit map.osm commands.json -o edited.osm   # prints the change sets
vectormap lane edited.osm 12                         # get_lane
vectormap nearest edited.osm 25.0 -1.5               # find_nearest_lane
vectormap validate --json edited.osm                 # validate_map
```

## Execution model

- `map.apply(&cmd)` runs one command. Every operation checks all inputs
  before mutating, so a rejected command leaves the map unchanged.
- `map.apply_all(&cmds)` runs a batch atomically: all succeed, or the map is
  unchanged and a `BatchError { index, op, error }` is returned.
- Results are deterministic: the same map and commands always produce the
  same IDs and geometry.

## Change sets

```json
{
  "created":  [{"kind": "lane", "id": 13}, {"kind": "boundary", "id": 11}],
  "modified": [{"kind": "lane", "id": 6}, {"kind": "road", "id": 10}],
  "deleted":  [],
  "warnings": [
    {"severity": "warning", "code": "boundary_unshared",
     "message": "boundary:4 is also used by lanes outside the split group; ...",
     "entity": {"kind": "boundary", "id": 4}}
  ]
}
```

Warning codes: `no_op`, `geometric_gap`, `boundary_unshared`,
`neighbor_dropped`, `orphan_rule`, `geometry_synthesized`, `rule_replaced`,
`attribute_conflict`, `placement_clamped`.

## Errors

```json
{"code": "not_found", "entity": {"kind": "lane", "id": 999}}
{"code": "invalid_argument", "name": "at", "reason": "fraction must be in (0, 1)"}
{"code": "not_mergeable", "first": 3, "second": 7, "reason": "lane:7 must be the only successor of lane:3 ..."}
```

Codes: `not_found`, `already_exists`, `invalid_argument`,
`invalid_geometry`, `in_use`, `not_mergeable`.

## Command reference

All commands are JSON objects with an `op` field.

### Lanes and topology

```json
{"op": "add_lane", "geometry": {"centerline": {"centerline": [[0,0,0],[30,0,0]], "width": 3.5}},
 "speed_limit": {"kmh": 40}, "predecessors": [3]}

{"op": "add_lane", "geometry": {"beside_lane": {"lane": 3, "side": "left", "width": 3.5,
                                                "shared_kind": {"type": "lane_marking", "pattern": "dashed"}}}}

{"op": "add_lane", "geometry": {"boundaries": {"left": {"existing": 5},
                                               "right": {"new": {"geometry": [[0,-3.5,0],[30,-3.5,0]]}}}}}

{"op": "remove_lane", "lane": 3}
{"op": "connect_lanes", "from": 3, "to": 6}
{"op": "disconnect_lanes", "from": 3, "to": 6}
{"op": "set_neighbor", "lane": 3, "side": "left", "neighbor": {"lane": 8, "direction": "opposite"}}
{"op": "split_lane", "lane": 6, "at": {"fraction": 0.5}, "include_neighbors": true}
{"op": "split_lane", "lane": 6, "at": {"station": 12.0}}
{"op": "merge_lanes", "first": 6, "second": 13}
```

`split_lane` splits the whole lateral group by default (neighbors are split
at the same cross-section, shared boundaries stay shared). `merge_lanes`
requires `second` to be the only successor of `first` and vice versa.

### Traffic features

```json
{"op": "add_stop_line", "lanes": [9], "placement": {"at_lane_end": {"offset": 2.0}}, "rule": "stop_sign"}
{"op": "add_stop_line", "lanes": [9, 10], "placement": {"at_station": {"station": 15.0}}, "rule": "marking"}

{"op": "add_traffic_signal", "lanes": [9]}
{"op": "add_traffic_signal", "lanes": [9], "stop_line": {"existing": 14},
 "geometry": [[-8,1.2,5],[-8,2.4,5]], "height": 0.5}
{"op": "add_traffic_signal", "lanes": [12], "group": 16}

{"op": "add_crosswalk", "geometry": {"across": {"lane": 6, "station": 10.0, "width": 4.0, "margin": 0.5}},
 "stop_line_offset": 1.5}
```

- `add_stop_line` spans all listed lanes; `rule` is `stop_sign` (default),
  `marking` or `none`.
- `add_traffic_signal` defaults: reuse the stop line of an existing stop
  rule on the first lane or create one at its end; synthesize the signal
  above the stop line (warning `geometry_synthesized`) with standard bulbs.
  Stop-sign / marking rules on the same stop line are replaced
  (`rule_replaced`). `group` adds the head to an existing signal group.
- `add_crosswalk` spans the lane and its lateral neighbors; crossing lanes
  are detected from geometry unless `crossing_lanes` is given; with
  `stop_line_offset` a stop line is created on each crossing lane.

### Attributes and grouping

```json
{"op": "set_speed_limit", "lanes": [3, 6], "kmh": 30}
{"op": "set_speed_limit", "lanes": [3], "kmh": null}
{"op": "set_turn_direction", "lane": 21, "turn_direction": "left"}
{"op": "set_lane_kind", "lane": 3, "kind": "bus"}
{"op": "set_boundary_kind", "boundary": 5, "kind": {"type": "curb"}}
{"op": "set_attribute", "entity": {"kind": "lane", "id": 3}, "key": "note", "value": "reviewed"}
{"op": "add_road", "name": "Main Street", "lanes": [3, 6, 9]}
{"op": "add_junction", "name": "Central", "lanes": [21, 22], "outline": [[-7,-7,0],[7,-7,0],[7,7,0],[-7,7,0]]}
{"op": "remove_entity", "entity": {"kind": "traffic_signal", "id": 15}}
```

## Repairing validation issues

Validation issues may carry a `fix` command:

```json
{"severity": "error", "code": "asymmetric_link",
 "message": "lane:6 is a successor of lane:3, but lane:3 is not a predecessor of lane:6",
 "entity": {"kind": "lane", "id": 3}, "related": [{"kind": "lane", "id": 6}],
 "fix": {"op": "connect_lanes", "from": 3, "to": 6}}
```

Applying `fix` resolves the issue; an agent can loop *validate → apply
fixes → validate*.
