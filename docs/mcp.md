# MCP server

`vectormap mcp` runs a [Model Context Protocol](https://modelcontextprotocol.io)
server on stdio. It lets Claude Code (or any MCP client) inspect and edit a
vector map through high-level tools:

> "split lane 12 in the middle", "add a stop line at this intersection",
> "connect these two lanes", "export it as Lanelet2 for Autoware"

The model never edits OSM XML. Each tool call becomes a deterministic
operation of `vectormap-core`, answered with a structured change set or
error.

## Setup with Claude Code

```bash
cargo install --path crates/vectormap-cli          # installs `vectormap`

claude mcp add vectormap -- vectormap mcp          # start with no map open
claude mcp add vectormap -- vectormap mcp /path/to/lanelet2_map.osm
```

Or share it with a project via `.mcp.json`:

```json
{
  "mcpServers": {
    "vectormap": { "command": "vectormap", "args": ["mcp"] }
  }
}
```

`vectormap mcp` accepts the usual input options (`--from`, `--origin LAT,LON`,
`--projection`, `--local`) for the map given on the command line. Logs go to
stderr; stdout carries only JSON-RPC.

## Tools

| Tool | Kind | Purpose |
|---|---|---|
| `open_map` | session | load a `.osm` (Lanelet2) or `.json` (IR) file |
| `new_map` | session | empty map or a sample (`straight_road`, `two_lane_road`, `intersection`) |
| `save_map` | session | save to the opened file or a new path (`format`, `autoware`) |
| `export_lanelet2` | session | write Lanelet2 (`autoware: true` → Autoware profile + `map_projector_info.yaml`) |
| `undo` | session | revert the last edit (50 steps) |
| `get_map_summary` | read-only | counts, lane kinds, rules, bounding box, topology stats, file state |
| `list_lanes` | read-only | lanes with topology; filter by kind / junction / distance, paginate |
| `get_lane` | read-only | everything about one lane, including its centreline |
| `get_entity` | read-only | raw JSON of any entity |
| `find_nearest_lane` | read-only | nearest lane to `x, y` (local metres) or `lat, lon` |
| `validate_map` | read-only | structured issues with fix commands (`autoware: true` for Autoware checks) |
| `build_road` | edit | a whole road along a reference line: lanes left to right, both directions, shared boundaries, neighbors, optional pieces |
| `add_connector` | edit | a smooth lane from the end of one lane to the start of another (turn direction derived) |
| `set_boundary_geometry` | edit | replace a boundary's line, e.g. snap it to an observed marking |
| `create_lane` | edit | from a centreline + width, beside an existing lane, or from boundaries |
| `remove_lane` | edit | remove a lane and its references |
| `split_lane` | edit | split at a fraction / station (neighbors split too) |
| `merge_lanes` | edit | merge a lane with its only successor |
| `connect_lanes` / `disconnect_lanes` | edit | successor links |
| `add_stop_line` | edit | stop line + stop sign / marking rule |
| `add_traffic_light` | edit | signal head + traffic-light rule (+ stop line) |
| `add_crosswalk` | edit | crosswalk + crosswalk rule (+ stop lines) |
| `set_speed_limit` | edit | set or clear speed limits |
| `apply_commands` | edit | any list of [commands](commands.md), atomically |

The arguments of the editing tools are exactly the fields of the
corresponding [command](commands.md) (`add_traffic_light` ⇄
`add_traffic_signal`, `create_lane` ⇄ `add_lane`).

## Results

Successful calls return `structuredContent` (and the same JSON as text):

```json
{
  "summary": "created: lane:13, boundary:11, boundary:12; modified: lane:6, lane:9, boundary:4, boundary:5",
  "changes": {"created": [...], "modified": [...], "warnings": [...]},
  "undo_steps": 1
}
```

Failures are tool results with `isError: true`:

```json
{"error": {"code": "not_mergeable",
           "message": "command #0 (merge_lanes) failed: cannot merge lane:6 and lane:3: ...",
           "details": {...}}}
```

Codes: `no_map`, `invalid_arguments`, `io_error`, `nothing_to_undo`,
`not_found`, and the editing errors `invalid_argument`, `invalid_geometry`,
`already_exists`, `in_use`, `not_mergeable`. A failed edit never changes the
map.

## Building a map from scratch

1. `new_map {"name": "site"}`
2. `build_road {"reference": [[0,0],[120,0]], "lanes": [{"width": 3.5}, {"width": 3.5, "direction": "backward"}], "segment_length": 40, "speed_limit": {"kmh": 40}}`
   → the new lanes, one chain per lane of the cross-section
3. `build_road` again for the crossing road, then `add_connector {"from": ..., "to": ...}`
   for each turn through the junction
4. `add_stop_line`, `add_traffic_light`, `add_crosswalk` as needed
5. `validate_map {"autoware": true}`, then `export_lanelet2 {"path": "out/lanelet2_map.osm", "autoware": true}`

Reference lines can come from anywhere: a driven trajectory, a road centre
drawn over aerial imagery, or lane markings extracted from a point cloud
(pass those as `boundaries`).

## Typical session

1. `open_map {"path": "lanelet2_map.osm"}`
2. `find_nearest_lane {"x": 25.0, "y": -1.2}` → lane 6
3. `split_lane {"lane": 6, "at": {"fraction": 0.5}}`
4. `add_traffic_light {"lanes": [9]}` → warning `geometry_synthesized`
5. `validate_map {"autoware": true}` → apply any `fix` via `apply_commands`
6. `export_lanelet2 {"path": "out/lanelet2_map.osm", "autoware": true}`

## Protocol notes

- JSON-RPC 2.0 over stdio, newline-delimited; batches are accepted.
- Supported protocol versions: `2025-11-25`, `2025-06-18`, `2025-03-26`,
  `2024-11-05` (the client's version is echoed when supported).
- Capabilities: `tools` only. Tool annotations mark read-only and
  destructive tools so clients can auto-approve queries.
- One map per server process; files are read and written relative to the
  server's working directory.
