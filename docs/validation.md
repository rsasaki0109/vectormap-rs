# Validation

`vectormap_validation::validate(&map, &ValidationOptions)` returns a
`ValidationReport`:

```json
{
  "counts": {"errors": 1, "warnings": 1, "infos": 0},
  "issues": [
    {"severity": "error", "code": "missing_boundary",
     "message": "Right boundary boundary:3 of lane:2 does not exist",
     "entity": {"kind": "lane", "id": 2}, "related": [{"kind": "boundary", "id": 3}]},
    {"severity": "warning", "code": "geometric_gap", "...": "..."}
  ]
}
```

Issues are sorted by severity (errors first), code and entity. Where a
repair is unambiguous, `fix` holds a [command](commands.md).

```bash
vectormap validate map.osm                 # text, exit status 1 on errors
vectormap validate --json map.osm          # structured report
vectormap validate --autoware map.osm      # + Autoware conventions
vectormap validate --deny-warnings map.osm # exit status 1 on warnings too
```

## Checks

| Code | Severity | Detects | `fix` |
|---|---|---|---|
| `dangling_lane_reference` | error | topology, road, junction or rule references a missing lane | — |
| `missing_boundary` | error | lane references a missing boundary | — |
| `missing_reference` | error | rule references a missing stop line / signal / crosswalk | — |
| `asymmetric_link` | error | predecessor / successor stored on one side only | `connect_lanes` |
| `self_loop` | error | lane is its own successor | `disconnect_lanes` |
| `invalid_neighbor` | error | neighbor missing, itself, or not reciprocal | `set_neighbor` |
| `empty_geometry` | error | polyline / polygon without points | — |
| `degenerate_geometry` | error / warning | < 2 distinct points, no area / repeated points | — |
| `non_finite_coordinate` | error | NaN or infinite coordinate | — |
| `same_boundary_both_sides` | error | lane uses one boundary for left and right | — |
| `boundary_direction_mismatch` | error | left and right boundaries point in opposite directions | — |
| `invalid_speed_limit` | error | speed limit ≤ 0 or not finite | `set_speed_limit` (clear) |
| `incomplete_rule` | error / warning | traffic light without signals, right-of-way without lanes | — |
| `duplicate_id` | error / warning | same ID twice in the source (from loaders); same number used by different kinds | — |
| `inverted_lane` | warning | left boundary lies to the right of the right boundary | — |
| `geometric_gap` | warning | successor does not start where its predecessor ends | — |
| `isolated_lane` | warning | lane without predecessors and successors | — |
| `disconnected_topology` | warning | lane graph has several unconnected parts (one issue per extra part) | — |
| `orphan_rule` | warning | rule applies to no lane | `remove_entity` |
| `duplicate_membership` | warning | lane listed several times in roads / junctions | — |
| `unused_boundary` | info | boundary used by no lane | `remove_entity` |
| `unused_feature` | info | stop line / signal / crosswalk referenced by no rule | — |

Loaders add their own issues (`duplicate_id`, `lanelet2.*`), and
`vectormap_io::autoware::check` adds `autoware.*` issues; `ValidationReport::merge`
combines them into one report.
