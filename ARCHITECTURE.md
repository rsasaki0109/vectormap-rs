# vectormap-rs Architecture

This document records the design of `vectormap-rs`: what problem each crate
solves, the shape of the intermediate representation (IR), and the reasoning
behind the non-obvious decisions. It is written for contributors and for
anyone building tools (CLI, MCP servers, editors) on top of the library.

## 1. Goals and non-goals

**Goals**

- A **format-independent vector map core** for autonomous driving and
  robotics: lanes, boundaries, roads, junctions, stop lines, traffic signals,
  crosswalks and regulatory elements.
- **Strongly typed**: every entity kind has its own ID newtype; a `LaneId`
  can never be passed where a `BoundaryId` is expected.
- **Deterministic**: the same input and the same sequence of operations always
  produce byte-identical output (ordered collections, stable ID allocation,
  stable serialization order).
- **Validation friendly**: problems are reported as structured
  `Issue { severity, code, message, entity, related, fix }` records, never as
  free-form strings only.
- **AI / MCP friendly**: all mutations go through high-level, serializable
  operations (`Command`) that return a `ChangeSet`
  (`created / modified / deleted / warnings`). A language model never edits
  OSM XML or internal data structures directly.

**Non-goals**

- Re-implementing Lanelet2 in Rust. Lanelet2 is *one* adapter.
- Being a routing engine or a planner. Topology queries are provided; a full
  routing graph is on the roadmap.
- Being a geometry engine. Only the handful of operations a vector map needs
  are implemented.

## 2. Survey summary (2026-09)

| Area | Findings | Consequence |
|---|---|---|
| Lanelet2 | OSM XML: nodes (lat/lon + `ele`), ways (linestrings, `area=yes` polygons), relations (`type=lanelet`, `type=regulatory_element`, `type=multipolygon`). Topology is **implicit**: successors share end/start points, neighbors share a boundary linestring. The loader aligns bound directions itself. | The Lanelet2 adapter must *infer* topology from geometry and must share OSM nodes/ways on export so that Lanelet2 can re-derive it. |
| Autoware | Road lanelets: `subtype=road`, `location=urban`, `one_way=yes`, `speed_limit` (bare km/h), `turn_direction` on intersection lanelets, optional `intersection_area`. Traffic lights: `traffic_light` way with `height`, `light_bulbs` way (`traffic_light_id`, node `color`/`arrow`), regulatory element with `refers` / `ref_line` / `light_bulbs`. Stop line without a light: `traffic_sign` regulatory element (`refers` a `stop_sign` way, `ref_line` a `stop_line` way). `road_marking` for guide stop lines. Crosswalk lanelet (`participant:pedestrian=yes`, `one_way=no`) plus a `crosswalk` regulatory element (`refers`, `ref_line`, `crosswalk_polygon`) referenced by the crossing road lanelets. `map_projector_info.yaml` selects `MGRS`/`LocalCartesianUTM`/`TransverseMercator`/`LocalCartesian`/`Local`; `local_x`/`local_y` are only read for `Local`, so lat/lon must always be correct. | Autoware conventions live in `vectormap-io::autoware` (export profile, compatibility lint, projector info). Core stays neutral. |
| Rust Lanelet2 crates | Nothing mature on crates.io (a few very young GitHub reimplementations). | Write our own adapter; keep it a thin mapping layer. |
| Rust OpenDRIVE crates | `opendrive` (parser/writer, 1.7), `libopendrive` (young). | Candidates for the future OpenDRIVE adapter. |
| Geometry crates | `geo` is excellent but 2D-only and heavy; `glam`/`nalgebra` are vector-math libraries; `rstar` is the standard R*-tree. | Implement the small 3D polyline toolkit ourselves (~500 lines, no deps). Add `rstar` when spatial indexing becomes necessary. |
| XML crates | `roxmltree` (read-only DOM, tiny, no deps), `quick-xml` (streaming read/write). | Read with `roxmltree`; write with a small hand-written deterministic writer (OSM XML is a trivial subset). |
| Projection | No mature pure-Rust MGRS; `utm` crate is stale. | Implement Krüger-series transverse Mercator (UTM / TM) in `vectormap-io::projection`. MGRS is on the roadmap. |

## 3. Workspace layout

```text
vectormap-rs/
├── Cargo.toml                 # workspace + thin `vectormap` facade crate
├── src/lib.rs                 # facade: re-exports core / io / validation
├── crates/
│   ├── vectormap-core/        # IR, geometry, topology, editing, commands
│   ├── vectormap-validation/  # structured map validation
│   ├── vectormap-io/          # JSON IR, Lanelet2 adapter, Autoware profile
│   ├── vectormap-mcp/         # MCP server: tools over JSON-RPC / stdio, undo
│   └── vectormap-cli/         # `vectormap` binary (incl. `vectormap mcp`)
├── examples/                  # runnable examples (build maps, Autoware export)
├── tests/                     # cross-crate integration tests + fixture maps
└── docs/                      # format notes (IR JSON, Lanelet2 mapping, Autoware)
```

Dependency graph (arrows = "depends on"):

```text
vectormap-cli ──> vectormap-mcp ──> vectormap-io ──> vectormap-core
      │                  │
      └──────────────────┴──> vectormap-validation ──> vectormap-core
```

`vectormap-io` does **not** depend on `vectormap-validation`; the shared
diagnostic vocabulary (`Issue`, `Severity`, `IssueCode`) lives in core so that
importers, editing operations and validators all speak the same language.

External dependencies are deliberately few: `serde`, `serde_json`,
`thiserror`, `roxmltree`, and `clap`/`anyhow` in the CLI only.

## 4. The IR (`vectormap-core`)

### 4.1 Identifiers

```rust
LaneId, BoundaryId, RoadId, JunctionId, StopLineId,
SignalId, CrosswalkId, RegulatoryElementId   // newtypes over u64
```

- Serialized as plain integers (readable JSON).
- `EntityRef` is the tagged union used in diagnostics and change sets:
  `{"kind": "lane", "id": 12}`.
- `Map` allocates new IDs from a **single counter** shared by all kinds.
  IDs are unique per kind by construction and, for newly created entities,
  also unique across kinds, so exporters whose ID space is global
  (OSM/Lanelet2) can reuse IR IDs without collisions. Imported IDs are kept.

### 4.2 Entities

Physical features and rules are separated:

| Entity | Content |
|---|---|
| `Boundary` | `Polyline3` + `BoundaryKind` (`virtual`, `lane_marking{pattern, weight}`, `curb`, `road_edge`, `other`). Shared by adjacent lanes. |
| `Lane` | `kind` (`driving`, `shoulder`, `bus`, `bicycle`, `walkway`, `parking`, `emergency`, `other`), `left`/`right` `BoundaryRef { boundary, reversed }`, optional explicit `centerline`, `speed_limit`, `one_way`, `turn_direction`. |
| `Road` | Named group of lanes (ordered). |
| `Junction` | Group of connecting lanes plus an optional outline polygon. |
| `StopLine` | `Polyline3`. Which lanes it governs is expressed by a rule. |
| `TrafficSignal` | Physical signal head: `Polyline3` (bottom edge, left → right), `height`, `kind` (`vehicle`/`pedestrian`), `bulbs` (`position`, `color`, `arrow`). |
| `Crosswalk` | Pedestrian crossing: left/right edges (walking direction) and optional explicit polygon. |
| `RegulatoryElement` | A **rule** applying to a set of lanes: `traffic_light{signals, stop_line}`, `traffic_sign{sign_type, sign, stop_line}`, `stop_line{stop_line}`, `right_of_way{priority, yielding, stop_line}`, `crosswalk{crosswalk, stop_lines}`, `other{kind}`. |

Rationale for rules as entities: the same stop line may be referenced by a
light and by a sign; Autoware uses the traffic-light regulatory element ID as
the *signal group* ID; OpenDRIVE also separates signals from controllers.
Keeping rules explicit makes all three mappings lossless.

`BoundaryRef::reversed` exists because a boundary shared by two lanes of
opposite direction is traversed backwards by one of them.

### 4.3 Geometry vs. topology

Geometry lives on entities (`Boundary::geometry`, `StopLine::geometry`, ...).
Topology lives in a separate store, `map.topology()`:

```rust
struct LaneLinks {
    predecessors: Vec<LaneId>,     // sorted, unique
    successors:   Vec<LaneId>,
    left:  Option<Neighbor>,       // Neighbor { lane, direction: same | opposite }
    right: Option<Neighbor>,
}
```

- Both directions of a link are stored (O(1) queries, and hand-written JSON
  can be checked by the validator for predecessor/successor inconsistency).
  Editing APIs maintain the symmetry.
- `topology::infer_topology(&map, tolerance)` derives links from geometry
  (shared end points ⇒ succession, shared boundary ⇒ adjacency). Importers of
  formats with implicit topology (Lanelet2) use it; the IR itself is explicit.
- Query API: `successors`, `predecessors`, `left_neighbor`, `right_neighbor`,
  `neighbor(lane, side)`.

### 4.4 Attributes (extension mechanism)

Every entity has `attributes: BTreeMap<String, String>`. Importers store tags
they do not map to typed fields under a format prefix
(`lanelet2:location = urban`). Exporters write back keys carrying their own
prefix and ignore other prefixes; un-prefixed keys are generic and written by
every exporter. Canonical values (e.g. `subtype=road` for a driving lane) are
*not* duplicated into attributes, so the IR stays clean while non-canonical
values (`subtype=highway`) survive a round trip.

### 4.5 Geometry toolkit

`Point2`, `Point3`, `Polyline2`, `Polyline3`, `Polygon2`, `Polygon3`,
`BoundingBox`. Operations: length, distance, nearest point (with station and
segment index), bounding box, interpolation at a station, resampling, slicing
and splitting, offsetting (to build lanes from a centerline + width), point in
polygon and segment intersection. Points serialize as `[x, y, z]` arrays.
Units: metres in a local Cartesian frame; `metadata.georeference` records how
the frame relates to WGS84.

### 4.6 Editing API and change sets

All edits are methods on `Map` returning `Result<ChangeSet, EditError>`
(creators also return the new ID):

```text
add_lane  remove_lane  connect  disconnect  set_neighbor
split_lane  merge_lanes
add_stop_line  add_traffic_signal  add_crosswalk
set_speed_limit  set_turn_direction  set_attribute  add_road  add_junction
```

- Every operation validates its inputs **before** mutating, so a failed
  operation leaves the map untouched.
- `ChangeSet { created, modified, deleted, warnings }` lists `EntityRef`s and
  structured warnings (e.g. "boundary 7 was shared with lane 3 and was
  duplicated").
- `split_lane` splits the whole lateral group (the lane and its neighbors) by
  default so that shared boundaries and neighbor links stay consistent.
- High-level placement helpers compute geometry deterministically: a stop line
  "at the end of lane 12", a signal above a stop line, a crosswalk "across
  lane 5 at s = 20 m, 4 m wide" (crossing lanes are detected automatically).

### 4.7 Commands (MCP-ready)

`Command` is a serde-tagged enum mirroring the editing API
(`{"op": "split_lane", "lane": 12, "at": {"fraction": 0.5}}`).
`map.apply(&cmd)` executes one; `map.apply_all(&cmds)` executes a batch
atomically (all or nothing). Together with the query API
(`summary`, `lane_info`, `find_nearest_lane`) this is exactly the surface the
MCP server (`vectormap-mcp`, see `docs/mcp.md`) exposes. The server is a
thin, dependency-free JSON-RPC loop: editing tools add `op` to their
arguments and deserialize them as a `Command`, so the MCP layer contains no
editing logic of its own. It keeps one map per session with a bounded undo
history.

| MCP tool | Library call |
|---|---|
| `get_map_summary` | `map.summary()` |
| `get_lane` | `map.lane_info(id)` |
| `find_nearest_lane` | `map.find_nearest_lane(point)` |
| `create_lane`, `split_lane`, `merge_lanes`, `connect_lanes`, `add_stop_line`, `add_traffic_light`, `add_crosswalk`, `set_speed_limit` | `map.apply(&Command::...)` |
| `validate_map` | `vectormap_validation::validate(&map, &opts)` |
| `export_lanelet2` | `vectormap_io::lanelet2::save(...)` |

Issues may carry a `fix: Option<Command>` so an agent can repair problems by
applying the suggested command.

### 4.8 Serialization

The IR serializes to JSON (`format: "vectormap-ir"`, `version: 1`) with
entities as arrays sorted by ID, points as `[x, y, z]`, empty/default fields
omitted. The format is documented in `docs/ir-json.md`.

## 5. Validation (`vectormap-validation`)

`validate(&map, &ValidationOptions) -> ValidationReport`. 22 checks grouped
as follows (full table with severities and fixes in `docs/validation.md`):

| Group | Codes |
|---|---|
| References | `dangling_lane_reference`, `missing_boundary`, `missing_reference`, `duplicate_membership` |
| Topology | `asymmetric_link`, `self_loop`, `invalid_neighbor`, `geometric_gap`, `isolated_lane`, `disconnected_topology` |
| Geometry | `empty_geometry`, `degenerate_geometry`, `non_finite_coordinate`, `same_boundary_both_sides`, `boundary_direction_mismatch`, `inverted_lane` |
| Semantics | `invalid_speed_limit`, `incomplete_rule`, `orphan_rule` |
| IDs / usage | `duplicate_id`, `unused_boundary`, `unused_feature` |

Same-kind duplicate IDs cannot exist inside a `Map`; they are detected by
the loaders (JSON documents store entities as arrays, OSM files are parsed
element by element) and reported with the same `duplicate_id` code.
`ValidationReport::merge` combines loader, validator and Autoware issues.

Autoware-specific checks (`autoware.*`) live in `vectormap-io::autoware`.

## 6. Formats (`vectormap-io`)

- `json`: lossless IR serialization.
- `lanelet2`: OSM XML adapter (`load_lanelet2`, `save_lanelet2`).
  Import: parse OSM with `roxmltree` → obtain coordinates → classify
  primitives → build IR (bound orientation via a faithful port of Lanelet2's
  `geometry::align`) → `infer_topology`. Export: allocate OSM IDs (reuse IR
  IDs when free) → share nodes between ways at identical positions so that
  Lanelet2 can re-derive topology → write nodes/ways/relations sorted by ID
  with sorted tags. Unsupported primitives are reported as structured
  `lanelet2.unsupported_*` issues rather than silently dropped.
- Coordinates: nodes are written with lat/lon *and* `local_x`/`local_y`.
  On import, local tags that are consistent with lat/lon under UTM or
  transverse Mercator are used directly and the georeference is recovered
  from them; this makes IR → Lanelet2 → IR exact (byte-identical re-export)
  and loads Autoware MGRS maps in their MGRS grid frame. Files with lat/lon
  only are projected with UTM relative to the south-west corner of the data.
- `autoware`: an export *profile* for the Lanelet2 writer (required tag
  defaults, `local_x`/`local_y`, `ele` on every node), a compatibility lint
  (`autoware.missing_speed_limit`, `autoware.missing_turn_direction`, ...), and
  `map_projector_info.yaml` generation.
- `projection`: WGS84 ⇄ local metric frame (UTM relative to an origin, as
  Lanelet2's `UtmProjector` / Autoware `LocalCartesianUTM`, and transverse
  Mercator centred on the origin, as Autoware `TransverseMercator`).

### Lanelet2 ⇄ IR mapping (summary)

| Lanelet2 | IR |
|---|---|
| `lanelet` (road, highway, bus_lane, …) | `Lane` |
| `lanelet` `subtype=crosswalk` | `Crosswalk` |
| way used as lanelet bound | `Boundary` |
| way `type=stop_line` | `StopLine` |
| way `type=traffic_light` (+ `light_bulbs`) | `TrafficSignal` |
| RE `traffic_light` | `RegulatoryElement::traffic_light` |
| RE `traffic_sign` | `RegulatoryElement::traffic_sign` |
| RE `road_marking` | `RegulatoryElement::stop_line` |
| RE `right_of_way` | `RegulatoryElement::right_of_way` |
| RE `crosswalk` (+ `crosswalk_polygon`) | `RegulatoryElement::crosswalk` (+ `Crosswalk.polygon`) |
| way `type=intersection_area` + lanelet tag | `Junction` |
| implicit shared nodes / bounds | `Topology` (inferred) |
| anything else | attributes (tags) or `lanelet2.unsupported_*` issue |

## 7. Determinism rules

1. All collections are `BTreeMap`/sorted `Vec`; no `HashMap` iteration leaks
   into output.
2. ID allocation is a monotonic counter; imports keep source IDs.
3. Floating point values are written with Rust's shortest round-trip
   formatting; speed limits are formatted with fixed rounding.
4. Operations with several candidate results (e.g. ambiguous neighbors) pick
   the smallest ID and emit a warning.

## 8. Implementation phases

1. Workspace → core data model → geometry → topology → JSON → validation → CLI.
2. Lanelet2 parser → exporter → round-trip tests.
3. Autoware profile → example Autoware map → documentation.
