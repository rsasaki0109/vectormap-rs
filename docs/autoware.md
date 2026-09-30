# Autoware compatibility

Autoware consumes Lanelet2 maps with additional conventions. They are
implemented in `vectormap-io::autoware` and in the Lanelet2 writer's
`Profile::Autoware`; `vectormap-core` stays Autoware-agnostic.

```bash
vectormap convert --autoware map.json out/lanelet2_map.osm   # + out/map_projector_info.yaml
vectormap validate --autoware out/lanelet2_map.osm
cargo run --example autoware_map                              # examples/autoware/
```

## Conventions implemented

Based on the Autoware vector map requirements, the `autoware_lanelet2_extension`
format documentation and the `autoware_map_loader` /
`autoware_map_projection_loader` sources (surveyed 2026-09).

| Topic | Convention | vectormap-rs |
|---|---|---|
| Road lanelets | `subtype=road`, `location=urban`, `one_way=yes`, `speed_limit` (km/h, bare number), `participant:vehicle=yes` | written; `location` / `participant:vehicle` added by the profile if absent; `autoware.missing_speed_limit`, `autoware.bidirectional_lane` |
| Intersections | `turn_direction=straight/left/right` on intersection lanelets, optional `intersection_area` polygon | `Lane.turn_direction`; `Junction` ⇄ `intersection_area`; `autoware.missing_turn_direction` |
| Traffic lights | `traffic_light` way (bottom edge, left → right, real height) with `height`; RE `traffic_light` with `refers`, `ref_line`, `light_bulbs` | `TrafficSignal` + `Rule::TrafficLight`; `height` defaults to 0.5 m in the profile (`autoware.missing_signal_height`) |
| Light bulbs | `light_bulbs` way with `traffic_light_id`; nodes with `color` (and `arrow`) | `TrafficSignal.bulbs` |
| Stop lines | `stop_line` way; stop signs as RE `traffic_sign` (`refers` a `stop_sign` way, `ref_line`); `road_marking` RE for guide stop lines | `Rule::TrafficSign { sign_type: "stop_sign" }` (default of `add_stop_line`), `Rule::StopLine` ⇄ `road_marking` (`autoware.stop_line_as_road_marking` info) |
| Crosswalks | lanelet `subtype=crosswalk`, `participant:pedestrian=yes`, `one_way=no`; RE `crosswalk` with `refers`, `ref_line`, `crosswalk_polygon`, referenced by the crossing road lanelets | `Crosswalk` + `Rule::Crosswalk` |
| Coordinates | every node has `ele`; lat/lon must be correct (all projectors except `Local` read them); `local_x`/`local_y` only read by `Local` | always written; lat/lon from the georeference; `local_x`/`local_y` written |
| Projection | `map_projector_info.yaml`: `projector_type` (`MGRS`, `LocalCartesianUTM`, `LocalCartesian`, `TransverseMercator`, `Local`), `vertical_datum`, `map_origin` | `utm` → `LocalCartesianUTM`, `transverse_mercator` → `TransverseMercator`, none → `Local` |

## Compatibility check

`autoware::check(&map)` returns `Issue`s with `autoware.*` codes:

| Code | Severity |
|---|---|
| `autoware.missing_speed_limit` | warning |
| `autoware.missing_turn_direction` | warning |
| `autoware.bidirectional_lane` | warning |
| `autoware.traffic_light_without_stop_line` | warning |
| `autoware.unmodeled_rule` | warning |
| `autoware.crosswalk_without_lanes` | warning |
| `autoware.missing_signal_height` | info |
| `autoware.unsupported_lane_kind` | info |
| `autoware.stop_line_as_road_marking` | info |
| `autoware.local_projector` | info |

## Known gaps

- MGRS output (`projector_type: MGRS`) is not generated; UTM / TM origins
  are used instead. MGRS maps can be *read*: their local frame is recovered.
- Detection areas, no-stopping areas, speed bumps, virtual traffic lights,
  bus stops, parking lots and `MetaInfo` are not modelled yet; they are
  reported on import (`lanelet2.unsupported_*`).
- Pedestrian traffic lights attached to crosswalk lanelets are not modelled.
- No Autoware integration test (loading the map in an Autoware container)
  yet — see the roadmap.
