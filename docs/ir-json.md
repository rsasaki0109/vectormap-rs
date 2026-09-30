# vectormap IR JSON (`vectormap-ir`, version 1)

The IR serializes to a single JSON document. It is lossless (`Map → JSON →
Map` is the identity), deterministic (entities sorted by ID, fixed field
order) and meant to be read and written by humans and language models.

```bash
vectormap convert map.osm map.json          # Lanelet2 → IR JSON
vectormap sample intersection demo.json     # a sample map
```

## Top level

```json
{
  "format": "vectormap-ir",
  "version": 1,
  "metadata": { ... },
  "lanes": [ ... ],
  "boundaries": [ ... ],
  "roads": [ ... ],
  "junctions": [ ... ],
  "stop_lines": [ ... ],
  "traffic_signals": [ ... ],
  "crosswalks": [ ... ],
  "regulatory_elements": [ ... ],
  "topology": [ ... ],
  "next_id": 42
}
```

- Empty arrays and default values are omitted.
- `next_id` appears only when entities were deleted, so that ID allocation
  continues identically after a save / load cycle.
- Units: metres, local Cartesian frame (x east, y north, z up); speed limits
  in km/h.
- Points are `[x, y, z]` arrays; polylines and polygons are arrays of points
  (polygons are implicitly closed, the first point is not repeated).
- IDs are plain integers. Every entity kind has its own ID type in Rust; in
  JSON, references are unambiguous from the field name.

## metadata

```json
"metadata": {
  "name": "intersection",
  "georeference": {
    "projection": "utm",
    "origin": { "lat": 35.681236, "lon": 139.767125, "alt": 0.0 }
  },
  "attributes": { "source": "survey-2026-09" }
}
```

`projection` is `utm` (UTM zone of the origin, coordinates relative to the
origin — Lanelet2 `UtmProjector`, Autoware `LocalCartesianUTM`) or
`transverse_mercator` (central meridian through the origin, Autoware
`TransverseMercator`). Without a georeference the map lives in a purely
local frame.

## boundaries

```json
{ "id": 1, "kind": {"type": "lane_marking", "pattern": "dashed", "weight": "thin"},
  "geometry": [[0.0, 0.0, 0.0], [30.0, 0.0, 0.0]] }
```

`kind.type`: `virtual`, `lane_marking` (`pattern`: `solid`, `dashed`,
`solid_solid`, `solid_dashed`, `dashed_solid`; `weight`: `thin`, `thick`),
`curb`, `road_edge`, `other`.

## lanes

```json
{
  "id": 3,
  "kind": "driving",
  "left": {"boundary": 2, "reversed": true},
  "right": 1,
  "speed_limit": {"kmh": 40.0},
  "turn_direction": "left",
  "one_way": false,
  "centerline": [[...], [...]],
  "attributes": {"lanelet2:location": "urban"}
}
```

- `kind`: `driving` (default), `shoulder`, `bus`, `bicycle`, `walkway`,
  `parking`, `emergency`, `other`.
- `left` / `right`: a boundary ID, or `{"boundary": id, "reversed": true}`
  when the lane runs against the boundary's direction (e.g. the incoming
  lane of a two-way road sharing its centre line).
- `centerline` is optional; it is derived from the boundaries when absent.
- `one_way` defaults to `true`.

## roads and junctions

```json
{ "id": 10, "name": "Main Street", "lanes": [3, 6, 9] }
{ "id": 60, "name": "Central Junction", "lanes": [21, 24, 27],
  "outline": [[-7, -7, 0], [7, -7, 0], [7, 7, 0], [-7, 7, 0]] }
```

## stop_lines, traffic_signals, crosswalks

```json
{ "id": 61, "geometry": [[27.0, 3.5, 0.0], [27.0, 0.0, 0.0]] }

{ "id": 62, "kind": "vehicle", "height": 0.5,
  "geometry": [[-8.0, 1.15, 5.0], [-8.0, 2.35, 5.0]],
  "bulbs": [ {"position": [-8.0, 1.35, 5.25], "color": "green"},
             {"position": [-8.0, 1.75, 5.25], "color": "yellow"},
             {"position": [-8.0, 2.15, 5.25], "color": "red", "arrow": "left"} ] }

{ "id": 80, "left_edge": [[...], [...]], "right_edge": [[...], [...]],
  "polygon": [[...], [...], [...], [...]] }
```

A crosswalk's edges are its sides relative to the pedestrian walking
direction.

## regulatory_elements

A regulatory element is a **rule** applying to a set of lanes:

```json
{ "id": 63, "lanes": [4],
  "rule": {"type": "traffic_light", "signals": [62], "stop_line": 61} }
```

| `rule.type` | Fields |
|---|---|
| `traffic_light` | `signals`, `stop_line?` |
| `traffic_sign` | `sign_type` (e.g. `stop_sign`), `sign?` (polyline), `stop_line?` |
| `stop_line` | `stop_line` (a marking without sign or light) |
| `right_of_way` | `priority`, `yielding` (lane IDs), `stop_lines` |
| `crosswalk` | `crosswalk`, `stop_lines` |
| `other` | `kind` (source-specific, e.g. `detection_area`) |

## topology

```json
"topology": [
  { "lane": 3, "successors": [6], "right": {"lane": 5} },
  { "lane": 4, "predecessors": [9], "left": {"lane": 7, "direction": "opposite"} }
]
```

Both directions of every relation are stored (`a.successors ∋ b` ⇔
`b.predecessors ∋ a`; `a.left = b (same)` ⇔ `b.right = a (same)`;
`a.left = b (opposite)` ⇔ `b.left = a (opposite)`). The editing API keeps
them in sync; `vectormap validate` reports hand-written inconsistencies
(`asymmetric_link`, `invalid_neighbor`) together with a suggested fix.

## attributes

Every entity has an optional `attributes` object of string pairs. Keys
prefixed with a format name (`lanelet2:`, `opendrive:`, ...) hold source
tags without a typed counterpart and are written back by the matching
exporter; un-prefixed keys are generic and written by every exporter.

## Loading rules

- Unknown `format` → error; `version` newer than supported → error.
- A duplicated ID does not fail the load: the first definition wins and a
  `duplicate_id` error issue is reported (`vectormap validate` shows it).
- References are not checked while loading; run the validator.
