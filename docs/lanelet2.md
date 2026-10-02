# Lanelet2 adapter

`vectormap-io::lanelet2` reads and writes Lanelet2 maps in OSM XML. Lanelet2
is an *external format*: the adapter maps its primitives to IR entities and
back; nothing in `vectormap-core` depends on it.

```text
Lanelet2 OSM ──read_str / load_lanelet2──▶ IR ──write_string / save_lanelet2──▶ Lanelet2 OSM
```

## Mapping

| Lanelet2 | IR |
|---|---|
| relation `type=lanelet` | `Lane` (`subtype` → `LaneKind`) |
| relation `type=lanelet`, `subtype=crosswalk` | `Crosswalk` (bounds → edges) |
| way used as lanelet `left` / `right` | `Boundary` (`type`/`subtype` → `BoundaryKind`) |
| lanelet `centerline` member | `Lane.centerline` |
| lanelet `speed_limit` (`30`, `30 km/h`, `20 mph`, `10 m/s`) | `Lane.speed_limit` |
| lanelet `one_way`, `turn_direction` | `Lane.one_way`, `Lane.turn_direction` |
| way `type=stop_line` | `StopLine` |
| way `type=traffic_light` (`height`, `subtype`) | `TrafficSignal` |
| way `type=light_bulbs` (node `color` / `arrow`, `traffic_light_id`) | `TrafficSignal.bulbs` |
| RE `subtype=traffic_light` (`refers`, `ref_line`, `light_bulbs`) | `Rule::TrafficLight` |
| RE `subtype=traffic_sign` (`refers` sign way, `ref_line`) | `Rule::TrafficSign` |
| RE `subtype=road_marking` (`refers` stop line) | `Rule::StopLine` |
| RE `subtype=right_of_way` (`right_of_way`, `yield`, `ref_line`) | `Rule::RightOfWay` |
| RE `subtype=crosswalk` (`refers`, `ref_line`, `crosswalk_polygon`) | `Rule::Crosswalk`, `Crosswalk.polygon` |
| other RE subtypes | `Rule::Other { kind }` (members dropped, reported) |
| way `type=intersection_area` + lanelet tag `intersection_area=<id>` | `Junction` (outline + member lanes) |
| lanelets referencing an RE with role `regulatory_element` | `RegulatoryElement.lanes` |
| shared nodes / shared bounds | `Topology` (inferred) |
| all other tags | `lanelet2:*` attributes |

Canonical values (e.g. `subtype=road` for a driving lane, `type=line_thin
subtype=dashed` for a thin dashed marking) are not stored as attributes;
non-canonical ones are (`lanelet2:subtype = highway`,
`lanelet2:subtype = low` for a low curbstone) and are written back on export
as long as they still describe the entity's typed kind.

## Orientation

A lanelet's direction of travel is defined purely by which way has the
`left` and which the `right` role. The reader replicates Lanelet2's
`geometry::align` exactly (a bound is inverted when the middle point of the
other bound lies on its wrong side) and records the result in
`BoundaryRef::reversed`. The writer therefore never needs an inversion
marker; ways are written in their stored order.

## Topology

Lanelet2 has no explicit topology. After reading, `infer_topology` derives:

- `a → b` when the end points of `a`'s bounds coincide with the start points
  of `b`'s bounds (1 cm tolerance);
- neighbors when two lanelets share a bound: same direction when one uses it
  as `left` and the other as `right` in the same orientation, opposite
  direction when both use it on the same side in opposite orientation.

The writer shares nodes between all ways that pass through the same
position (1 µm grid), so Lanelet2 (and Autoware) re-derive exactly the IR
topology. `connect` / `set_neighbor` warn when an explicit IR relation cannot
be represented this way.

## Coordinates

Nodes carry `lat`/`lon`, `ele` and — by default — `local_x`/`local_y`.

On load (`ProjectionChoice::Auto`):

1. all nodes have `local_x`/`local_y` and lat/lon are placeholders (Autoware
   `Local` maps) → local tags, no georeference;
2. `mgrs_code` node tags identify one UTM-based 100 km square → lat/lon
   projected within that square, preserving MGRS on export. The tags are
   checked against lat/lon; invalid tags, polar UPS grids and multiple grid
   squares are rejected;
3. all nodes have local tags consistent with lat/lon under UTM or transverse
   Mercator → local tags plus the recovered georeference;
4. otherwise → lat/lon projected with UTM relative to the south-west corner
   of the data (recorded as the map's georeference).

`ProjectionChoice::Georeferenced(..)` and `ProjectionChoice::LocalTags`
force a behaviour (`vectormap --origin LAT,LON [--projection utm|tm|mgrs]`,
`vectormap --local`).

On save, lat/lon are computed from the map's georeference. Maps without a
georeference get lat/lon = 0 and local tags only (Autoware `Local`).

## IDs

- Reading: positive OSM IDs are kept as IR IDs; negative (JOSM) IDs are
  mapped to fresh IDs above the largest positive one.
- Writing: IR IDs are reused as OSM IDs when they are unique across all
  written primitives; clashes get fresh IDs (reported as
  `lanelet2.renumbered`). Synthesized ways (bulbs, crosswalk edges, signs)
  and nodes get fresh IDs.

## What is not (yet) supported

Reported as structured issues rather than dropped silently:

- `lanelet2.unsupported_way`: ways not used by any lanelet or supported
  rule (e.g. parking spaces, detection areas, road markings);
- `lanelet2.unsupported_relation`: `multipolygon` areas and other relation
  types;
- `lanelet2.unsupported_member`: members of rules the IR does not model
  (e.g. `cancels`, `detection_area` polygons).

## Issue codes

| Code | Meaning |
|---|---|
| `duplicate_id` | an OSM ID appears twice (first definition kept) |
| `lanelet2.missing_node` / `lanelet2.missing_member` | dangling references in the file |
| `lanelet2.invalid_primitive` | a lanelet / RE is structurally invalid and was skipped |
| `lanelet2.invalid_tag` | an unparsable tag value (kept as attribute) |
| `lanelet2.projection` | how coordinates were obtained (info) |
| `lanelet2.no_georeference` | export without georeference (lat/lon = 0) |
| `lanelet2.renumbered` | entities that could not keep their ID |
| `lanelet2.geometry_synthesized` | placeholder geometry written (e.g. a stop sign) |
| `lanelet2.dangling_reference` | export skipped a reference to a missing entity |

### Reviewed pedestrian signal control

`RegulatoryElement.controlled_crosswalks` contains the pedestrian lanelets controlled
by a traffic-light rule. It is distinct from vehicle `lanes` and from the physical
crosswalk referred to by a crossing/yield rule. Legacy JSON defaults to an empty
list and omits that empty field. Lanelet2 writes each controlled crosswalk lanelet's
`regulatory_element` membership and restores it on import, following the
[Autoware format](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/docs/lanelet2_format_extension.md).

Use `set_regulatory_links` (Rust edit, JSON command or MCP tool) for explicit review.
It validates all targets before mutation, rejects mixed vehicle/pedestrian control,
and changes no physical geometry, lamps or source attributes. Empty targets retain
an unresolved rule and produce an orphan warning. Removing a controlled crosswalk
detaches its control references while retaining the observed signal head.
A correctly associated pedestrian rule has no vehicle stop-line warning; unresolved
or invalid control remains reported. These relationships are operator choices, not
inferred legal control or lamp states.
